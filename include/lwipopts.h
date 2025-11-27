/**
 * @file    lwipopts.h
 * @brief   lwIP configuration for Pico W with MQTT (SIMPLE POLL MODE)
 */

#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

/* NO_SYS==1: Use lightweight API (no threads) */
#define NO_SYS                          1
#define LWIP_SOCKET                     0
#define LWIP_NETCONN                    0

/* Memory settings */
#define MEM_LIBC_MALLOC                 0
#define MEMP_MEM_MALLOC                 1
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        4000
#define MEMP_NUM_TCP_SEG                32
#define MEMP_NUM_ALTCP_PCB              MEMP_NUM_TCP_PCB
#define PBUF_POOL_SIZE                  24

/* Protocol features */
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1
#define LWIP_RAW                        1
#define LWIP_DHCP                       1
#define LWIP_IPV4                       1
#define LWIP_TCP                        1
#define LWIP_UDP                        1
#define LWIP_DNS                        1
#define LWIP_TCP_KEEPALIVE              1

/* TCP settings */
#define TCP_MSS                         1460
#define TCP_WND                         (8 * TCP_MSS)
#define TCP_SND_BUF                     (8 * TCP_MSS)
#define TCP_SND_QUEUELEN                ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))

/* Network interface settings */
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_NETIF_HOSTNAME             1
#define LWIP_NETIF_TX_SINGLE_PBUF       1

/* DHCP settings */
#define DHCP_DOES_ARP_CHECK             0
#define LWIP_DHCP_DOES_ACD_CHECK        0

/* MQTT - THE IMPORTANT PART! */
#define LWIP_MQTT                       1

/* Stats */
#define MEM_STATS                       0
#define SYS_STATS                       0
#define MEMP_STATS                      0
#define LINK_STATS                      0

/* Checksum */
#define LWIP_CHKSUM_ALGORITHM           3

/* Disable unused features */
#define LWIP_IGMP                       0
#define LWIP_AUTOIP                     0
#define LWIP_SNMP                       0

/* Debug (all off for release) */
#define LWIP_DEBUG                      0

#endif /* _LWIPOPTS_H */