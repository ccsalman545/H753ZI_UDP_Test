# STM32H753ZI → PC: 50-byte UDP over Ethernet, visible in Wireshark

The board sends one **50-byte** UDP datagram per second to `192.168.10.1:5000`
over its RMII Ethernet port (LAN8742 PHY). This document covers what the
firmware does, how to set up the PC, and what to expect in Wireshark.

---

## 1. Wire it up

| | |
|---|---|
| Board | Nucleo-H753ZI (STM32H753ZITx) |
| PHY | LAN8742, RMII |
| Link | LAN cable, board RJ45 → PC NIC (or a switch) |

RMII pin map (from `H753ZI_UDP_Test.ioc`):

| Signal | Pin | Signal | Pin | Signal | Pin |
|---|---|---|---|---|---|
| REF_CLK | PA1 | MDIO | PA2 | CRS_DV | PA7 |
| TX_EN | PB11 | TXD0 | PB12 | TXD1 | PB13 |
| MDC | PC1 | RXD0 | PC4 | RXD1 | PC5 |

> **50 MHz RMII reference clock.** `PA1` (REF_CLK) must be driven at 50 MHz.
> On the LAN8742 this normally comes from the PHY's `nINT/REFCLKO` pin, which
> only outputs 50 MHz when the PHY is strapped for it (a 25 MHz crystal on the
> PHY, with the REFCLKO strap enabled). If nothing drives PA1 at 50 MHz the
> MAC transmits nothing and you will see no packets at all — this is the
> single most common reason a working LwIP build still puts nothing on the
> wire. Check it on a scope if you see no ARP and no UDP.

## 2. Configure the PC

The board is `192.168.10.2/24` with no gateway. Put the PC on the same subnet
with a **static** address:

```
IP address   192.168.10.1
Netmask      255.255.255.0
Gateway      (leave blank)
```

Fedora/GNOME: Settings → Network → the wired adapter → IPv4 → Manual.
Windows: `ncpa.cpl` → adapter → Properties → IPv4.

A static address is required — the board does not run a DHCP server, so a PC
left on DHCP will not get an address on that subnet and will not answer ARP.

Confirm reachability from the PC before touching Wireshark:

```
ping 192.168.10.2
```

If ping works, ARP and the receive path are proven end to end.

## 3. Capture in Wireshark

Select the wired interface and use this display filter:

```
udp.port == 5000 && ip.src == 192.168.10.2
```

You should see one packet per second:

```
Ethernet II  src 02:00:00:00:00:00   dst <your PC's MAC>   type 0x0800
IPv4         192.168.10.2 → 192.168.10.1   proto UDP (17)  len 78
UDP          src 49152 → dst 5000          len 58
Data         50 bytes
             "12345678901234567890123456789012345678901234567890"
```

The frame is 92 bytes on the wire (14 Ethernet + 20 IP + 8 UDP + 50 payload),
plus the 4-byte FCS the MAC appends.

The source MAC is `02:00:00:00:00:00`, from `ETH_MAC_ADDR0..5` in
`Core/Inc/stm32h7xx_hal_conf.h`. The UDP source port is an ephemeral port
LwIP picks; the destination port is fixed at 5000.

### Changing what is sent

All of it is in `Core/Src/main.c`:

```c
#define UDP_PAYLOAD_LEN     50U      /* bytes on the wire */
#define UDP_DEST_PORT       5000U
#define UDP_SEND_PERIOD_MS  1000U    /* 1 datagram/second */

static const char udp_payload[UDP_PAYLOAD_LEN + 1] =
    "12345678901234567890123456789012345678901234567890";
```

A `_Static_assert` fails the build if the literal is ever edited to the wrong
length, so the 50 bytes cannot silently drift. The destination IP is set in
`main()`; the board's own IP is in `LWIP/App/lwip.c`.

## 4. Build and flash

Open `H753ZI_UDP_Test.ioc` in STM32CubeIDE and build the `Debug`
configuration, then flash over ST-Link. Toolchain: GNU Tools for STM32
(12.3.rel1), `arm-none-eabi-gcc`.

## 5. How it is put together

`LWIP/Target/ethernetif.c` is the LwIP ↔ ETH MAC driver. It was empty in this
repository, so it is written from scratch against the ETH HAL that ships here
(STM32Cube FW_H7 V1.13.0). Three things about that HAL version shape the
driver, and are worth knowing if you edit it:

- There is **no `HAL_ETH_WriteData()` and no `HAL_ETH_BuildRxDescriptors()`**.
  Descriptor rings are handed to the HAL through `ETH_HandleTypeDef::Init`, and
  Rx buffers are recycled through the registered allocate/link callbacks.
- **`HAL_ETH_Start()` drains the Rx pool immediately** — it sets
  `RxBuildDescCnt = ETH_RX_DESC_CNT` and calls `ETH_UpdateDescriptor()`, which
  calls `rxAllocateCallback`. The callbacks therefore have to be registered
  *before* `HAL_ETH_Start()`, or every descriptor is armed with a NULL buffer
  and the DMA never receives anything.
- `USE_HAL_ETH_REGISTER_CALLBACKS` is `0` in `stm32h7xx_hal_conf.h`, but
  `rxAllocateCallback` / `rxLinkCallback` are declared unconditionally in
  `ETH_HandleTypeDef`, so `HAL_ETH_RegisterRxAllocateCallback()` is still the
  correct entry point.

### Polling, not interrupts

`lwipopts.h` has `NO_SYS = 1`, so there is no `tcpip` thread. The main loop in
`Core/Src/main.c` *is* the LwIP thread, and `MX_LWIP_Process()` is what drives
it: it pulls received frames out of the DMA (including the ARP reply without
which `udp_sendto()` has no destination MAC), runs the LwIP timers, and polls
the LAN8742 link state every 100 ms.

Because of that, the loop must not block. Timing uses `HAL_GetTick()` rather
than `HAL_Delay()`, so `MX_LWIP_Process()` keeps running between datagrams.

### Memory placement

The ETH DMA is an AHB master in domain D2. It reaches D1/D2/D3 SRAM but not
DTCM (`0x20000000`) or AXI SRAM (`0x24000000`), which is where this project's
`.bss` lives. The descriptors, the Rx pool and the Tx staging buffer are
therefore placed in D2 SRAM via the `.RxDecripSection` / `.TxDecripSection` /
`.Rx_PoolSection` sections that `STM32H753ZITX_FLASH.ld` maps into an
`ETH_RAM_D2` region at `0x30000000`.

That region is bounded at 16 KB so the linker raises *"section will not fit in
region"* if the driver outgrows it, rather than silently running into the LwIP
heap, which `lwipopts.h` pins at `0x30004000`. Current usage is 14016 bytes,
leaving 2368 bytes of headroom.

`MPU_Config()` in `main.c` also marks `0x30000000 + 256 KB` non-cacheable. The
D-cache is not enabled in this project so it is not load-bearing yet, but
without it the first `SCB_EnableDCache()` would have the CPU caching lines the
DMA writes behind its back.

### Hardware checksums

`lwipopts.h` sets `CHECKSUM_BY_HARDWARE = 1` and disables the software
generators (`CHECKSUM_GEN_IP`, `CHECKSUM_GEN_UDP`, ...). The driver sets
`ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC` in the Tx descriptor, so the MAC
fills in the IP and UDP checksums. Wireshark will show them as valid.

If you ever set `CHECKSUM_BY_HARDWARE` to `0`, re-enable the software
generators in `lwipopts.h` as well — otherwise the checksums go out as zero.

## 6. Troubleshooting

| Symptom | Likely cause |
|---|---|
| Nothing at all in Wireshark | No 50 MHz on PA1 — see the note in section 1 |
| ARP requests visible, no UDP | PC is not on `192.168.10.0/24`, so no ARP reply |
| ARP works, UDP appears briefly then stops | Rx buffer pool exhausted; raise `ETH_RX_BUFFER_CNT` in `ethernetif.c` |
| `udp_sendto()` never returns `ERR_OK` | Expected for the first second or two — LwIP is still resolving ARP |
| LED on PB0 not blinking | Code never reached the main loop; check `Error_Handler()` |
