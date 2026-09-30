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
     * Fedora PC IP address.
     */
    IP_ADDR4(&destination_ip, 192, 168, 10, 1);

    /*
     * Main loop.
     */
    while (1)
    {
        /*
         * Toggle LD1 every second.
         */
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

        /*
         * Exactly 50 bytes.
         */
        const char message[50] =
            "12345678901234567890123456789012345678901234567890";

        struct pbuf *p;

        /*
         * Allocate 50-byte UDP payload.
         */
        p = pbuf_alloc(PBUF_TRANSPORT, 50, PBUF_RAM);

        if (p != NULL)
        {
            /*
             * Copy exactly 50 bytes.
             */
            memcpy(p->payload, message, 50);

            /*
             * Send UDP packet.
             *
             * STM32 IP : 192.168.10.2
             * PC IP    : 192.168.10.1
             * UDP port : 5000
             */
            udp_sendto(
                udp_pcb,
                p,
                &destination_ip,
                5000
            );

            /*
             * Free packet buffer.
             */
            pbuf_free(p);
        }

        /*
         * Wait 1 second.
         */
        HAL_Delay(1000);
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
     * Configure default memory region.
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
