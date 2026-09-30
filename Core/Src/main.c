/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32H753ZI Ethernet UDP Test
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "lwip.h"
#include "gpio.h"

#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"

#include <string.h>

/* Private variables ---------------------------------------------------------*/

static struct udp_pcb *udp_pcb = NULL;
static ip_addr_t destination_ip;

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MPU_Config(void);

/* USER CODE BEGIN 0 */

/*
 * ---------------------------------------------------------------------------
 * What goes on the wire
 * ---------------------------------------------------------------------------
 */

/*
 * Payload length. The whole point of this project is that exactly this many
 * bytes reach the PC, so it is a single named constant that everything else
 * (the buffer, pbuf_alloc, memcpy) is derived from.
 */
#define UDP_PAYLOAD_LEN     50U

/* Destination UDP port on the PC. */
#define UDP_DEST_PORT       5000U

/*
 * Datagrams per second, roughly. 1000 ms gives one frame a second, which is
 * easy to follow in Wireshark.
 */
#define UDP_SEND_PERIOD_MS  1000U

/*
 * The 50 payload bytes.
 *
 * Declared as [UDP_PAYLOAD_LEN + 1] so the string literal's terminating NUL
 * has somewhere to live; only the first UDP_PAYLOAD_LEN bytes are copied into
 * the packet, so the NUL never goes on the wire. The _Static_assert below
 * fails the build if the literal is ever edited to the wrong length.
 */
static const char udp_payload[UDP_PAYLOAD_LEN + 1] =
    "12345678901234567890123456789012345678901234567890";

_Static_assert(sizeof(udp_payload) == UDP_PAYLOAD_LEN + 1U,
               "udp_payload must be exactly UDP_PAYLOAD_LEN characters");

/*
 * Count of datagrams the stack accepted. Handy as a debugger watch
 * expression: if this stays at 0 the send path is stuck (usually ARP has not
 * resolved yet), if it climbs at one per second the board is transmitting.
 */
static uint32_t udp_frames_sent;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MPU Configuration */
    MPU_Config();

    /* MCU Configuration */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize GPIO */
    MX_GPIO_Init();

    /* Initialize LwIP */
    MX_LWIP_Init();

    /*
     * LD1 green LED on PB0.
     * LED ON = application started.
     */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    /*
     * Create UDP control block.
     */
    udp_pcb = udp_new();

    if (udp_pcb == NULL)
    {
        Error_Handler();
    }

    /*
     * Destination PC IP address.
     *
     * The PC has to be on the same 192.168.10.0/24 subnet as the board
     * (192.168.10.2, set in LWIP/App/lwip.c) and has to answer ARP, or
     * udp_sendto() has no destination MAC to put in the frame.
     */
    IP_ADDR4(&destination_ip, 192, 168, 10, 1);

    /*
     * Main loop.
     *
     * Deliberately free of HAL_Delay(): this is a NO_SYS = 1 build, so the
     * loop itself is the LwIP thread. Blocking here would stop ARP replies,
     * LwIP timers and the PHY link poll from being serviced. Timing is done
     * against HAL_GetTick() instead, so MX_LWIP_Process() runs continuously.
     */
    {
        uint32_t last_send_tick = HAL_GetTick();

        while (1)
        {
            /*
             * Service LwIP on every pass. With no tcpip thread and no ETH
             * interrupt, this call is the only thing that
             *   - pulls received frames out of the DMA, including the ARP
             *     reply from the PC, without which udp_sendto() has no
             *     destination MAC and nothing ever leaves the board,
             *   - runs the LwIP timers, and
             *   - polls the LAN8742 link state every 100 ms.
             */
            MX_LWIP_Process();

            if ((HAL_GetTick() - last_send_tick) < UDP_SEND_PERIOD_MS)
            {
                continue;
            }

            last_send_tick = HAL_GetTick();

            /*
             * Toggle LD1 (green, PB0) once per datagram, so a blinking LED
             * means frames are being built and handed to the driver.
             */
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

            /*
             * Build and send one 50-byte UDP datagram.
             */
            {
                struct pbuf *p;

                /*
                 * PBUF_TRANSPORT reserves the UDP + IP + Ethernet headers
                 * ahead of the payload pointer; PBUF_RAM allocates one
                 * contiguous block, which is what low_level_output() expects.
                 */
                p = pbuf_alloc(PBUF_TRANSPORT, UDP_PAYLOAD_LEN, PBUF_RAM);

                if (p != NULL)
                {
                    memcpy(p->payload, udp_payload, UDP_PAYLOAD_LEN);

                    /*
                     * Send UDP packet.
                     *
                     * STM32 IP : 192.168.10.2   (set in LWIP/App/lwip.c)
                     * PC IP    : 192.168.10.1   (destination_ip above)
                     * UDP port : 5000
                     *
                     * The first attempt typically does not return ERR_OK:
                     * LwIP has to ARP for 192.168.10.1 before it knows the
                     * PC's MAC address. It queues the request and the reply is
                     * picked up by MX_LWIP_Process(), so the send starts
                     * succeeding within a second or two of link up.
                     */
                    if (udp_sendto(udp_pcb, p, &destination_ip,
                                   UDP_DEST_PORT) == ERR_OK)
                    {
                        udp_frames_sent++;
                    }

                    pbuf_free(p);
                }
            }
        }
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /*
     * Configure power supply.
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /*
     * Voltage scaling.
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    /*
     * Use HSI for initial Ethernet test.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * Configure clocks.
     */
    RCC_ClkInitStruct.ClockType =
          RCC_CLOCKTYPE_HCLK
        | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1
        | RCC_CLOCKTYPE_PCLK2
        | RCC_CLOCKTYPE_D3PCLK1
        | RCC_CLOCKTYPE_D1PCLK1;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief MPU Configuration
  * @retval None
  */
static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /*
     * Disable MPU before configuration.
     */
    HAL_MPU_Disable();

    /*
     * Region 0: background, no access. Everything below is punched through
     * by the explicit regions that follow.
     */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /*
     * Region 1: domain D2 SRAM, 0x30000000 + 256 KB, as non-cacheable.
     *
     * This is where ethernetif.c puts the ETH DMA descriptors, the Rx buffer
     * pool and the Tx staging buffer, and where lwipopts.h puts the LwIP heap
     * (LWIP_RAM_HEAP_POINTER = 0x30004000).
     *
     * TEX = 1, C = 0, B = 0 selects Normal non-cacheable memory. The D-cache
     * is not enabled in this project, so this region is not load-bearing yet,
     * but without it the moment someone adds SCB_EnableDCache() the CPU would
     * cache lines the ETH DMA is writing behind its back and frames would be
     * silently corrupted. Configuring it now keeps the driver correct either
     * way.
     */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.BaseAddress = 0x30000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /*
     * Enable MPU.
     */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief Error Handler
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
        /*
         * Fast LED blinking indicates an error.
         */
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

        HAL_Delay(100);
    }
}

#ifdef USE_FULL_ASSERT

/**
  * @brief Reports the name of the source file and line number.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /*
     * User may add debugging output here.
     */
}

#endif
