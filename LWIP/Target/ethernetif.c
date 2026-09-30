/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : ethernetif.c
  * Description        : LwIP network interface driver for the STM32H753ZI ETH
  *                      MAC + LAN8742 RMII PHY.
  *
  *                      NO_SYS = 1 (polling) build. Every function here runs in
  *                      thread context, driven from MX_LWIP_Process() in the
  *                      main loop, so no locking is needed.
  *
  *                      Written against the STM32Cube FW_H7 V1.13.0 ETH HAL that
  *                      ships in this repository. The facts that shaped this
  *                      file, all read out of that HAL:
  *
  *                      - there is no HAL_ETH_WriteData() and no
  *                        HAL_ETH_BuildRxDescriptors();
  *                      - ETH_DMATxDescListInit() / ETH_DMARxDescListInit(),
  *                        called from HAL_ETH_Init(), zero both descriptor rings
  *                        and populate heth->TxDescList / heth->RxDescList;
  *                      - HAL_ETH_Start() sets RxBuildDescCnt = ETH_RX_DESC_CNT
  *                        and calls ETH_UpdateDescriptor(), which pulls the
  *                        first ETH_RX_DESC_CNT buffers out of the Rx pool
  *                        through rxAllocateCallback. The callbacks therefore
  *                        have to be registered BEFORE HAL_ETH_Start();
  *                      - USE_HAL_ETH_REGISTER_CALLBACKS is 0 in
  *                        stm32h7xx_hal_conf.h, but rxAllocateCallback /
  *                        rxLinkCallback / txFreeCallback are declared
  *                        unconditionally in ETH_HandleTypeDef, so
  *                        HAL_ETH_RegisterRxAllocateCallback() and
  *                        HAL_ETH_RegisterRxLinkCallback() are the right way in
  *                        and do compile.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/etharp.h"
#include "ethernetif.h"
#include "lan8742.h"

#include <string.h>

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */

/*
 * ---------------------------------------------------------------------------
 * Tunables
 * ---------------------------------------------------------------------------
 */

/*
 * Rx buffers handed to the ETH DMA. Each one is ETH_RX_BUFFER_SIZE bytes
 * (1536, from lwipopts.h), so the pool costs
 * ETH_RX_BUFFER_CNT * ETH_RX_BUFFER_SIZE bytes of domain D2 SRAM --
 * 12 KB with the default 8 x 1536.
 *
 * HAL_ETH_Start() immediately claims ETH_RX_DESC_CNT (4) of them, so this must
 * stay above 4.
 */
#define ETH_RX_BUFFER_CNT       8U

/*
 * Staging buffer that an outgoing frame is linearised into before it is given
 * to the DMA. See low_level_output(). Sized for a standard Ethernet frame.
 */
#define ETH_TX_BUFFER_SIZE      1524U

/*
 * How many times low_level_output() retries when the Tx ring is momentarily
 * full, before giving up and letting LwIP drop the packet.
 */
#define ETH_TX_RETRY_COUNT      10U

/*
 * The MAC address comes from ETH_MAC_ADDR0..5 in Core/Inc/stm32h7xx_hal_conf.h
 * (02:00:00:00:00:00 as CubeMX generates it). That first byte has bit 1 set and
 * bit 0 clear, so it is a locally administered unicast address -- ours to
 * choose, not an IEEE-assigned OUI. Change it there if several of these boards
 * share one LAN.
 */

/*
 * Interface name reported as "st0" by the LwIP netif. This LwIP tree does not
 * define IFNAME0/IFNAME1 itself.
 */
#define IFNAME0                 's'
#define IFNAME1                 't'

/* Prototypes; the definitions follow. The bottom-of-file CubeMX USER CODE 4
   block re-declares low_level_init() to keep the definition checked. */
static err_t low_level_output(struct netif *netif, struct pbuf *p);
err_t low_level_init(struct netif *netif);

/* USER CODE END 0 */

/* Private types -------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Private variables ---------------------------------------------------------*/

/*
 * Ethernet handle. Defined here, exported to lwip.c through lwip.h.
 */
ETH_HandleTypeDef heth;

/*
 * LAN8742 PHY driver object plus the MDIO glue that binds it to heth.
 */
static lan8742_Object_t LAN8742;
static lan8742_IOCtx_t  LAN8742_IOCtx;

/*
 * ---------------------------------------------------------------------------
 * DMA descriptors, Rx buffer pool and Tx staging buffer.
 *
 * All four arrays are placed in domain D2 SRAM (0x30000000) by the
 * .RxDecripSection / .TxDecripSection / .Rx_PoolSection output sections that
 * STM32H753ZITX_FLASH.ld puts in RAM_D2, and MPU_Config() in main.c marks
 * 0x30000000..0x3007FFFF as non-cacheable. Both properties matter:
 *
 *   - the ETH DMA is an AHB master in domain D2. It reaches D1/D2/D3 SRAM, but
 *     never DTCM (0x20000000) or AXI SRAM (0x24000000), which is where this
 *     project's .bss would otherwise put these arrays;
 *   - if the CPU caches a descriptor or an Rx buffer, the DMA and the CPU
 *     disagree about its contents and frames are silently corrupted.
 * ---------------------------------------------------------------------------
 */
__attribute__((aligned(4)))
__attribute__((section(".RxDecripSection")))
static ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT];

__attribute__((aligned(4)))
__attribute__((section(".TxDecripSection")))
static ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT];

__attribute__((aligned(32)))
__attribute__((section(".Rx_PoolSection")))
static uint8_t Rx_Buff[ETH_RX_BUFFER_CNT][ETH_RX_BUFFER_SIZE];

__attribute__((aligned(4)))
__attribute__((section(".Rx_PoolSection")))
static uint8_t Tx_Buff[ETH_TX_BUFFER_SIZE];

/*
 * Free list of Rx pool buffers, threaded through ETH_BufferTypeDef::next.
 * low_level_rx_allocate() pops from it, low_level_rx_free() pushes back.
 */
__attribute__((aligned(4)))
static ETH_BufferTypeDef RxBuffer[ETH_RX_BUFFER_CNT];

static ETH_BufferTypeDef *RxPoolFreeList;

/*
 * 0 once the Rx pool has run dry, 1 whenever a buffer comes back. Reported
 * through ethernet_link_status_updated().
 */
static uint8_t RxAllocStatus;

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN 3 */

static void low_level_rx_allocate(uint8_t **buff);
static void low_level_rx_link(void **pStart, void **pEnd, uint8_t *buff,
                              uint16_t len);
static void low_level_rx_free(struct pbuf *p);
static void low_level_rx_pool_init(void);

static void ethernet_phy_init(void);

static int32_t ETH_PHY_IO_Init(void);
static int32_t ETH_PHY_IO_DeInit(void);
static int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr,
                                  uint32_t *pRegVal);
static int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr,
                                   uint32_t RegVal);
static int32_t ETH_PHY_IO_GetTick(void);

static int32_t LAN8742_IO_Register(void);

/* USER CODE END 3 */

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> )
*******************************************************************************/

/**
 * @brief  Configure the hardware and fill in the netif. Called once by
 *         netif_add() from MX_LWIP_Init().
 *
 * @param  netif the lwIP network interface structure for this ethernetif
 * @retval ERR_OK on success, ERR_IF if the MAC refuses to start
 */
err_t ethernetif_init(struct netif *netif)
{
  /*
   * heth.Init.MACAddr has to stay valid for the lifetime of the interface, so
   * the address array is static rather than on the stack.
   */
  static uint8_t macAddress[6];

  macAddress[0] = ETH_MAC_ADDR0;
  macAddress[1] = ETH_MAC_ADDR1;
  macAddress[2] = ETH_MAC_ADDR2;
  macAddress[3] = ETH_MAC_ADDR3;
  macAddress[4] = ETH_MAC_ADDR4;
  macAddress[5] = ETH_MAC_ADDR5;

  LWIP_ASSERT("netif != NULL", (netif != NULL));

  /* Initialize the snmp variables and counters inside the struct netif. */
  NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, 100000000);

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;

  /*
   * etharp_output() is used directly to save a function call; the alternative
   * is to declare your own and pass netif->output to netif_add().
   */
  netif->output     = etharp_output;
  netif->linkoutput = low_level_output;
#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */

  if (low_level_init(netif) != ERR_OK)
  {
    return ERR_IF;
  }

  /* Maximum transfer unit. No jumbo frames. */
  netif->mtu = 1500;

  /* Hardware address */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;
  netif->hwaddr[0]  = macAddress[0];
  netif->hwaddr[1]  = macAddress[1];
  netif->hwaddr[2]  = macAddress[2];
  netif->hwaddr[3]  = macAddress[3];
  netif->hwaddr[4]  = macAddress[4];
  netif->hwaddr[5]  = macAddress[5];

  /* Accept broadcast and our own unicast address */
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

  return ERR_OK;
}

/**
 * @brief  Thread every Rx pool buffer onto the free list. Must run before
 *         HAL_ETH_Start(), because that call already claims ETH_RX_DESC_CNT
 *         buffers through low_level_rx_allocate().
 */
static void low_level_rx_pool_init(void)
{
  uint32_t i;

  for (i = 0; i < ETH_RX_BUFFER_CNT; i++)
  {
    RxBuffer[i].buffer = &Rx_Buff[i][0];
    RxBuffer[i].len    = ETH_RX_BUFFER_SIZE;
    RxBuffer[i].next   = ((i + 1U) < ETH_RX_BUFFER_CNT) ? &RxBuffer[i + 1U]
                                                        : NULL;
  }

  RxPoolFreeList  = &RxBuffer[0];
  RxAllocStatus   = 1U;
}

/**
 * @brief  Configure the ETH MAC and DMA, then bring the MAC up.
 *
 * @param  netif the lwIP network interface structure for this ethernetif
 * @retval ERR_OK on success, ERR_IF on failure
 */
err_t low_level_init(struct netif *netif)
{
  uint8_t macAddress[6] = {ETH_MAC_ADDR0, ETH_MAC_ADDR1, ETH_MAC_ADDR2,
                           ETH_MAC_ADDR3, ETH_MAC_ADDR4, ETH_MAC_ADDR5};
  ETH_MACConfigTypeDef macconf;

  LWIP_UNUSED_ARG(netif);

  /* The Rx pool has to be ready before HAL_ETH_Start() drains it */
  low_level_rx_pool_init();

  heth.Instance            = ETH;
  heth.Init.MACAddr        = macAddress;
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc         = DMATxDscrTab;
  heth.Init.RxDesc         = DMARxDscrTab;
  heth.Init.RxBuffLen      = ETH_RX_BUFFER_SIZE;

  /* ETH MSP init (GPIO, clocks) lives in Core/Src/stm32h7xx_hal_msp.c.
     HAL_ETH_Init() calls it, and it zeroes both descriptor rings. */
  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    return ERR_IF;
  }

  /*
   * This HAL version has no HAL_ETH_GetMACDefaultConfig(), so the MAC
   * configuration is written out in full. The values mirror the
   * macDefaultConf that ETH_MACDMAConfig() applies internally, with the
   * exceptions STMicroelectronics makes for LwIP: the CRC is stripped from
   * received frames and received frames with a bad checksum are not dropped
   * here, because LwIP wants to see them.
   */
  memset(&macconf, 0, sizeof(macconf));

  macconf.SourceAddrControl               = ETH_SOURCEADDRESS_REPLACE_ADDR0;
  macconf.ChecksumOffload                 = ENABLE;
  macconf.InterPacketGapVal               = ETH_INTERPACKETGAP_96BIT;
  macconf.GiantPacketSizeLimitControl     = ENABLE;
  macconf.Support2KPacket                 = DISABLE;
  macconf.CRCStripTypePacket              = ENABLE;
  macconf.AutomaticPadCRCStrip            = DISABLE;
  macconf.Watchdog                        = DISABLE;
  macconf.Jabber                          = DISABLE;
  macconf.JumboPacket                     = DISABLE;
  macconf.Speed                           = ETH_SPEED_100M;
  macconf.DuplexMode                      = ETH_FULLDUPLEX_MODE;
  macconf.LoopbackMode                    = DISABLE;
  macconf.CarrierSenseBeforeTransmit      = DISABLE;
  macconf.ReceiveOwn                      = ENABLE;
  macconf.CarrierSenseDuringTransmit      = DISABLE;
  macconf.RetryTransmission               = DISABLE;
  macconf.BackOffLimit                    = ETH_BACKOFFLIMIT_4;
  macconf.DeferralCheck                   = DISABLE;
  macconf.PreambleLength                  = ETH_PREAMBLELENGTH_7;
  macconf.UnicastSlowProtocolPacketDetect = DISABLE;
  macconf.SlowProtocolDetect              = DISABLE;
  macconf.CRCCheckingRxPackets            = DISABLE;
  macconf.GiantPacketSizeLimit            = 0x618U;
  macconf.ExtendedInterPacketGap          = DISABLE;
  macconf.ExtendedInterPacketGapVal       = 0x0U;
  macconf.ProgrammableWatchdog            = DISABLE;
  macconf.WatchdogTimeout                 = ETH_MACWTR_WTO_2KB;
  macconf.PauseTime                       = 0x0U;
  macconf.ZeroQuantaPause                 = DISABLE;
  macconf.PauseLowThreshold               = ETH_PAUSELOWTHRESHOLD_MINUS_4;
  macconf.TransmitFlowControl             = DISABLE;
  macconf.UnicastPausePacketDetect        = DISABLE;
  macconf.ReceiveFlowControl              = DISABLE;
  macconf.TransmitQueueMode               = ETH_TRANSMITSTOREFORWARD;
  macconf.ReceiveQueueMode                = ETH_RECEIVESTOREFORWARD;
  macconf.DropTCPIPChecksumErrorPacket    = DISABLE;
  macconf.ForwardRxErrorPacket            = DISABLE;
  macconf.ForwardRxUndersizedGoodPacket   = DISABLE;

  if (HAL_ETH_SetMACConfig(&heth, &macconf) != HAL_OK)
  {
    return ERR_IF;
  }

  /*
   * Register the Rx callbacks BEFORE HAL_ETH_Start(). HAL_ETH_Start() sets
   * RxBuildDescCnt = ETH_RX_DESC_CNT and calls ETH_UpdateDescriptor(), which
   * fills the first four descriptors by calling rxAllocateCallback. Leaving
   * the weak HAL_ETH_RxAllocateCallback() in place would leave every buffer
   * NULL and the DMA would never own a descriptor.
   */
  if (HAL_ETH_RegisterRxAllocateCallback(&heth, low_level_rx_allocate) != HAL_OK)
  {
    return ERR_IF;
  }

  if (HAL_ETH_RegisterRxLinkCallback(&heth, low_level_rx_link) != HAL_OK)
  {
    return ERR_IF;
  }

  /*
   * Polling mode: HAL_ETH_Start() rather than HAL_ETH_Start_IT(). No ETH DMA
   * interrupt is unmasked, so nothing preempts the LwIP thread and the
   * NO_SYS = 1 build stays free of locking. ETH_IRQHandler() is still wired up
   * in stm32h7xx_it.c in case you switch to HAL_ETH_Start_IT() later.
   */
  if (HAL_ETH_Start(&heth) != HAL_OK)
  {
    return ERR_IF;
  }

  /* Bring the PHY up and match the MAC to what it negotiated */
  ethernet_phy_init();

  return ERR_OK;
}

/**
 * @brief  Push one frame out through the ETH DMA.
 *
 *         The pbuf handed in by LwIP is a chain living in the LwIP heap. The
 *         DMA needs one contiguous, DMA-reachable block, so the chain is
 *         linearised into Tx_Buff (domain D2 SRAM, non-cacheable) and
 *         submitted as a single ETH_BufferTypeDef. That works for a chain of
 *         any length and keeps the DMA off the LwIP heap.
 *
 * @param  netif the lwIP network interface structure for this ethernetif
 * @param  p     the MAC packet to send (IP packet including MAC header)
 * @retval ERR_OK if the frame was queued to the DMA, ERR_MEM/ERR_IF otherwise
 */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  ETH_TxPacketConfigTypeDef TxConfig;
  ETH_BufferTypeDef         TxBuffer;
  struct pbuf              *q;
  uint32_t                  offset = 0;
  uint32_t                  retries;
  err_t                     err = ERR_MEM;

  LWIP_UNUSED_ARG(netif);

  if ((p == NULL) || (p->tot_len > ETH_TX_BUFFER_SIZE))
  {
    MIB2_STATS_NETIF_INC(netif, ifoutdiscards);
    return ERR_MEM;
  }

  /* Flatten the pbuf chain into the DMA-reachable staging buffer */
  for (q = p; q != NULL; q = q->next)
  {
    memcpy(&Tx_Buff[offset], q->payload, q->len);
    offset += q->len;
  }

  TxBuffer.buffer = Tx_Buff;
  TxBuffer.len    = offset;
  TxBuffer.next   = NULL;

  memset(&TxConfig, 0, sizeof(TxConfig));
  TxConfig.Attributes   = ETH_TX_PACKETS_FEATURES_CSUM |
                          ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl   = ETH_CRC_PAD_INSERT;
  TxConfig.Length       = offset;
  TxConfig.TxBuffer     = &TxBuffer;
  TxConfig.pData        = Tx_Buff;

  /*
   * Hand back any descriptor the DMA has finished with, then queue this frame.
   * HAL_ETH_Transmit_IT() returns HAL_ERROR when the next descriptor is still
   * DMA-owned, which just means the line is busy -- retry a bounded number of
   * times rather than spinning forever.
   */
  for (retries = 0; retries < ETH_TX_RETRY_COUNT; retries++)
  {
    (void)HAL_ETH_ReleaseTxPacket(&heth);

    if (HAL_ETH_Transmit_IT(&heth, &TxConfig) == HAL_OK)
    {
      MIB2_STATS_NETIF_ADD(netif, ifoutoctets, (u32_t)p->tot_len);
      return ERR_OK;
    }

    if ((HAL_ETH_GetError(&heth) & HAL_ETH_ERROR_PARAM) != 0U)
    {
      /* A parameter is wrong; retrying will not help */
      err = ERR_IF;
      break;
    }

    err = ERR_MEM;
  }

  MIB2_STATS_NETIF_INC(netif, ifoutdiscards);

  return err;
}

/**
 * @brief  Return one Rx buffer to the pool. Installed as the
 *         custom_free_function of the pbufs built by low_level_rx_link(), so
 *         it runs when LwIP drops its last reference to the frame.
 *
 * @param  p the pbuf whose payload was a buffer from Rx_Buff[]
 */
static void low_level_rx_free(struct pbuf *p)
{
  uint32_t i;

  /* Recover the pool slot from the buffer address */
  for (i = 0; i < ETH_RX_BUFFER_CNT; i++)
  {
    if (p->payload == (void *)&Rx_Buff[i][0])
    {
      RxBuffer[i].next = RxPoolFreeList;
      RxPoolFreeList   = &RxBuffer[i];
      break;
    }
  }

  RxAllocStatus = 1U;
}

/**
 * @brief  Hand a free buffer from the Rx pool to the HAL. Called from
 *         ETH_UpdateDescriptor() inside HAL_ETH_ReadData() and from
 *         HAL_ETH_Start().
 *
 * @param  buff receives the address of a buffer the DMA may write into, or
 *              NULL when the pool is empty
 */
static void low_level_rx_allocate(uint8_t **buff)
{
  if (RxPoolFreeList != NULL)
  {
    *buff = RxPoolFreeList->buffer;

    RxPoolFreeList = RxPoolFreeList->next;
    RxAllocStatus = 1U;
  }
  else
  {
    /* Pool exhausted -- the DMA will drop the incoming frame */
    *buff = NULL;
    RxAllocStatus = 0U;
  }
}

/**
 * @brief  Chain one received DMA buffer onto the pbuf being assembled.
 *
 *         The buffer is wrapped in a custom pbuf (LWIP_SUPPORT_CUSTOM_PBUF is
 *         enabled in lwipopts.h) so no copy is needed; low_level_rx_free()
 *         marks it reusable once the stack is done with it.
 *
 * @param  pStart head of the pbuf chain being built
 * @param  pEnd   tail of the pbuf chain being built
 * @param  buff   the buffer the DMA just filled
 * @param  len    number of valid bytes in buff
 */
static void low_level_rx_link(void **pStart, void **pEnd, uint8_t *buff,
                              uint16_t len)
{
  static struct pbuf_custom pbuf_custom[ETH_RX_BUFFER_CNT];
  struct pbuf **start = (struct pbuf **)pStart;
  struct pbuf **end   = (struct pbuf **)pEnd;
  struct pbuf  *p;
  uint32_t      i;

  /* Find the pool slot this buffer came from */
  for (i = 0; i < ETH_RX_BUFFER_CNT; i++)
  {
    if (&Rx_Buff[i][0] == buff)
    {
      break;
    }
  }

  if (i == ETH_RX_BUFFER_CNT)
  {
    /* Not one of ours -- the buffer would never come back, so drop it */
    RxAllocStatus = 0U;
    return;
  }

  pbuf_custom[i].custom_free_function = low_level_rx_free;

  p = pbuf_alloced_custom(PBUF_RAW, len, PBUF_REF, &pbuf_custom[i],
                          buff, ETH_RX_BUFFER_SIZE);

  if (p == NULL)
  {
    RxAllocStatus = 0U;
    return;
  }

  if (*end != NULL)
  {
    pbuf_cat(*end, p);
  }
  else
  {
    *start = p;
  }

  *end = p;
}

/**
 * @brief  Pull the frames waiting in the DMA and push them up the LwIP stack.
 *         Called from MX_LWIP_Process() on every pass of the main loop.
 *
 * @param  netif the lwIP network interface structure for this ethernetif
 */
void ethernetif_input(struct netif *netif)
{
  struct pbuf *p;
  err_t        result;

  /* Drain every frame that is waiting, not just the first one */
  while (HAL_ETH_ReadData(&heth, (void **)&p) == HAL_OK)
  {
    if (p == NULL)
    {
      continue;
    }

    result = netif->input(p, netif);

    if (result != ERR_OK)
    {
      MIB2_STATS_NETIF_INC(netif, ifinerrors);
      pbuf_free(p);
    }
  }
}

/*******************************************************************************
                       LL Driver Utils (LAN8742 PHY)
*******************************************************************************/

/**
 * @brief  Bind the LAN8742 BSP driver to the ETH MDIO bus.
 *
 *         LAN8742_GetLinkState() dereferences pObj->IO.ReadReg without checking
 *         that it is set, so this has to run before the first link state read.
 *
 * @retval LAN8742_STATUS_OK on success, LAN8742_STATUS_ERROR on failure
 */
static int32_t LAN8742_IO_Register(void)
{
  if (LAN8742.Is_Initialized == 0U)
  {
    LAN8742_IOCtx.Init     = ETH_PHY_IO_Init;
    LAN8742_IOCtx.DeInit   = ETH_PHY_IO_DeInit;
    LAN8742_IOCtx.ReadReg  = ETH_PHY_IO_ReadReg;
    LAN8742_IOCtx.WriteReg = ETH_PHY_IO_WriteReg;
    LAN8742_IOCtx.GetTick  = ETH_PHY_IO_GetTick;

    if (LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx) != LAN8742_STATUS_OK)
    {
      return LAN8742_STATUS_ERROR;
    }
  }

  return LAN8742_STATUS_OK;
}

static int32_t ETH_PHY_IO_Init(void)
{
  return (HAL_ETH_Start(&heth) == HAL_OK) ? LAN8742_STATUS_OK
                                          : LAN8742_STATUS_ERROR;
}

static int32_t ETH_PHY_IO_DeInit(void)
{
  return (HAL_ETH_Stop(&heth) == HAL_OK) ? LAN8742_STATUS_OK
                                         : LAN8742_STATUS_ERROR;
}

static int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr,
                                  uint32_t *pRegVal)
{
  return (HAL_ETH_ReadPHYRegister(&heth, DevAddr, RegAddr, pRegVal) == HAL_OK)
             ? LAN8742_STATUS_OK
             : LAN8742_STATUS_ERROR;
}

static int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr,
                                   uint32_t RegVal)
{
  return (HAL_ETH_WritePHYRegister(&heth, DevAddr, RegAddr, RegVal) == HAL_OK)
             ? LAN8742_STATUS_OK
             : LAN8742_STATUS_ERROR;
}

static int32_t ETH_PHY_IO_GetTick(void)
{
  return (int32_t)HAL_GetTick();
}

/**
 * @brief  Read the PHY once and set the MAC speed and duplex to match.
 *
 *         Only the link state is read; the PHY is not reset here, so a board
 *         that has already negotiated keeps its link through the LwIP init.
 *         If the link is down at this point the MAC keeps the 100 Mbit/s full
 *         duplex defaults and ethernet_link_check_state() fixes it up when the
 *         cable is plugged in.
 */
static void ethernet_phy_init(void)
{
  ETH_MACConfigTypeDef macconf;
  int32_t              linkstate;

  if (LAN8742_IO_Register() != LAN8742_STATUS_OK)
  {
    return;
  }

  linkstate = LAN8742_GetLinkState(&LAN8742);

  if (linkstate <= LAN8742_STATUS_LINK_DOWN)
  {
    return;
  }

  if (HAL_ETH_GetMACConfig(&heth, &macconf) != HAL_OK)
  {
    return;
  }

  switch (linkstate)
  {
    case LAN8742_STATUS_100MBITS_FULLDUPLEX:
      macconf.DuplexMode = ETH_FULLDUPLEX_MODE;
      macconf.Speed      = ETH_SPEED_100M;
      break;

    case LAN8742_STATUS_100MBITS_HALFDUPLEX:
      macconf.DuplexMode = ETH_HALFDUPLEX_MODE;
      macconf.Speed      = ETH_SPEED_100M;
      break;

    case LAN8742_STATUS_10MBITS_FULLDUPLEX:
      macconf.DuplexMode = ETH_FULLDUPLEX_MODE;
      macconf.Speed      = ETH_SPEED_10M;
      break;

    case LAN8742_STATUS_10MBITS_HALFDUPLEX:
      macconf.DuplexMode = ETH_HALFDUPLEX_MODE;
      macconf.Speed      = ETH_SPEED_10M;
      break;

    default:
      /* Autonegotiation still running: leave the defaults in place */
      return;
  }

  (void)HAL_ETH_SetMACConfig(&heth, &macconf);
}

/**
 * @brief  Ask the PHY for its current link state and tell LwIP when it
 *         changes. Called from Ethernet_Link_Periodic_Handle() every 100 ms.
 *
 *         The MAC speed/duplex follow-up lives here rather than in
 *         ethernet_link_status_updated(), because lwip.c defines that callback
 *         as a file-static function with an empty body.
 *
 * @param  netif the lwIP network interface structure for this ethernetif
 */
void ethernet_link_check_state(struct netif *netif)
{
  int32_t linkstate;

  if (LAN8742_IO_Register() != LAN8742_STATUS_OK)
  {
    return;
  }

  linkstate = LAN8742_GetLinkState(&LAN8742);

  if (linkstate == LAN8742_STATUS_LINK_DOWN)
  {
    if (netif_is_link_up(netif))
    {
      netif_set_link_down(netif);
    }
  }
  else if (linkstate > LAN8742_STATUS_LINK_DOWN)
  {
    if (!netif_is_link_up(netif))
    {
      netif_set_link_up(netif);
    }

    /* Cable is up: keep the MAC speed and duplex matched to the negotiation */
    ethernet_phy_init();

    /*
     * Buffers return to the Rx pool as LwIP frees the pbufs built from them,
     * so a pool that ran dry refills itself; just clear the flag.
     */
    RxAllocStatus = 1U;
  }
}

/* USER CODE BEGIN 4 */

/*
 * Prototypes for the driver entry points that are defined above but are not in
 * ethernetif.h, so the compiler checks the definitions against them.
 */
err_t low_level_init(struct netif *netif);

/* USER CODE END 4 */
