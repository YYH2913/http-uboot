// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Based on Linux airoha_eth.c majorly rewritten
 * and simplified for U-Boot usage for single TX/RX ring.
 *
 * Copyright (c) 2024 AIROHA Inc
 * Author: Lorenzo Bianconi <lorenzo@kernel.org>
 *         Christian Marangi <ansuelsmth@gmail.org>
 */

#include <env.h>
#include <command.h>
#include <dm.h>
#include <dm/devres.h>
#include <dm/lists.h>
#include <mapmem.h>
#include <malloc.h>
#include <net.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <asm-generic/gpio.h>
#include <asm/cache.h>
#include <miiphy.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/mdio.h>
#include <linux/mii.h>
#include <linux/iopoll.h>
#include <linux/time.h>
#include <vsprintf.h>
#include <asm/arch/scu-regmap.h>

#ifndef MDIO_USXGMII_LINK
#define MDIO_USXGMII_LINK BIT(15)
#endif

#define AIROHA_MAX_NUM_GDM_PORTS 4
#define AIROHA_MAX_NUM_QDMA 2
#define AIROHA_MAX_NUM_RSTS 3
#define AIROHA_MAX_NUM_XSI_RSTS 6
#define AIROHA_PEER_FPORT_CACHE_SIZE 8

#define AIROHA_MAX_PACKET_SIZE 2048
#define AIROHA_RX_BUF_SIZE AIROHA_MAX_PACKET_SIZE
#define AIROHA_NUM_TX_RING 8
#define AIROHA_NUM_RX_RING 1
#define AIROHA_NUM_TX_IRQ 1
#define HW_DSCP_NUM 32
#define IRQ_QUEUE_LEN 1
#define TX_DSCP_NUM 16
#define RX_DSCP_NUM PKTBUFSRX
#define PSE_QUEUE_RSV_PAGES 64

/* SCU */
#define SCU_SHARE_FEMEM_SEL 0x958
#define SCU_WAN_CONF 0x70
#define SCU_WAN_SEL GENMASK(7, 0)
#define SCU_WAN_SEL_USXGMII FIELD_PREP(SCU_WAN_SEL, 0x12)
#define SCU_SSR3 0x94
#define SCU_ETH_XSI_SEL GENMASK(14, 13)
#define SCU_ETH_XSI_USXGMII FIELD_PREP(SCU_ETH_XSI_SEL, 0x1)
#define SCU_GPIO_2ND_I2C_MODE 0x214
#define SCU_GPIO_MDC_IO_MASTER_MODE BIT(14)
#define SCU_GPIO_I2C_MASTER_MODE BIT(13)
#define SCU_FORCE_GPIO_EN 0x228
#define SCU_FORCE_GPIO1_EN BIT(1)
#define SCU_FORCE_GPIO2_EN BIT(2)
#define SCU_GPIO_PON_MODE 0x21c
#define SCU_GPIO_SGMII_MDIO_MODE BIT(13)

/* GPIO SYSCTL */
#define GPIO_SYSCTL_BASE 0x1fbf0200
#define GPIO_REG_DATA 0x0004
#define GPIO_REG_OE 0x0014
#define GPIO_REG_CTRL 0x0000
#define GPIO_REG_CTRL1 0x0020
#define GPIO_REG_CTRL2 0x0060
#define GPIO_REG_CTRL3 0x0064
#define GPIO_REG_DATA1 0x0070
#define GPIO_REG_OE1 0x0078

/* ETH PCS (XR1710G gdm4 / lan1) */
#define ETH_PCS_XFI_MAC_BASE 0x1fa09000
#define ETH_PCS_MULTI_SGMII_BASE 0x1fa74000
#define ETH_PCS_USXGMII_BASE 0x1fa75900
#define ETH_PCS_HSGMII_RATE_ADP_BASE 0x1fa76000
#define ETH_PCS_XFI_ANA_BASE 0x1fa7a000
#define ETH_PCS_XFI_PMA_BASE 0x1fa7b000

#define PCS_XFI_GIB_CFG 0x0000
#define PCS_XFI_RX_FRAG_LEN GENMASK(26, 22)
#define PCS_XFI_TX_FRAG_LEN GENMASK(21, 17)
#define PCS_XFI_IPG_NUM GENMASK(15, 10)
#define PCS_XFI_TX_FC_EN BIT(5)
#define PCS_XFI_RX_FC_EN BIT(4)
#define PCS_XFI_RXMPI_STOP BIT(3)
#define PCS_XFI_RXMBI_STOP BIT(2)
#define PCS_XFI_TXMPI_STOP BIT(1)
#define PCS_XFI_TXMBI_STOP BIT(0)
#define PCS_XFI_LOGIC_RST 0x0010
#define PCS_XFI_MAC_LOGIC_RST BIT(0)
#define PCS_XFI_MACADDRH 0x0060
#define PCS_XFI_MACADDRH_MASK GENMASK(15, 0)
#define PCS_XFI_MACADDRL 0x0064
#define PCS_XFI_MACADDRL_MASK GENMASK(31, 0)
#define PCS_XFI_CNT_CLR 0x0100
#define PCS_XFI_GLB_CNT_CLR BIT(0)

#define PCS_MULTI_SGMII_MSG_RX_CTRL_0 0x0100
#define PCS_HSGMII_XFI_SEL BIT(28)

#define PCS_USXGMII_PCS_CTROL_1 0x0000
#define PCS_USXGMII_SPEED_SEL_H BIT(13)
#define PCS_USXGMII_PCS_STUS_1 0x0004
#define PCS_USXGMII_PCS_RX_LINK_STATUS BIT(2)
#define PCS_USXGMII_BASE_R_10GB_T_PCS_STUS_1 0x0030
#define PCS_USXGMII_RX_LINK_STUS BIT(12)
#define PCS_USXGMII_PCS_CTRL_0 0x02c0
#define PCS_USXGMII_T_TYPE_T_INT_EN BIT(24)
#define PCS_USXGMII_T_TYPE_D_INT_EN BIT(16)
#define PCS_USXGMII_T_TYPE_C_INT_EN BIT(8)
#define PCS_USXGMII_T_TYPE_S_INT_EN BIT(0)
#define PCS_USXGMII_PCS_CTRL_1 0x02c4
#define PCS_USXGMII_R_TYPE_C_INT_EN BIT(24)
#define PCS_USXGMII_R_TYPE_S_INT_EN BIT(16)
#define PCS_USXGMII_TXPCS_FSM_ENC_ERR_INT_EN BIT(8)
#define PCS_USXGMII_T_TYPE_E_INT_EN BIT(0)
#define PCS_USXGMII_PCS_CTRL_2 0x02c8
#define PCS_USXGMII_RPCS_FSM_DEC_ERR_INT_EN BIT(24)
#define PCS_USXGMII_R_TYPE_E_INT_EN BIT(16)
#define PCS_USXGMII_R_TYPE_T_INT_EN BIT(8)
#define PCS_USXGMII_R_TYPE_D_INT_EN BIT(0)
#define PCS_USXGMII_PCS_CTRL_3 0x02cc
#define PCS_USXGMII_FAIL_SYNC_XOR_ST_INT_EN BIT(24)
#define PCS_USXGMII_RX_BLOCK_LOCK_ST_INT_EN BIT(16)
#define PCS_USXGMII_LINK_UP_ST_INT_EN BIT(8)
#define PCS_USXGMII_HI_BER_ST_INT_EN BIT(0)
#define PCS_USXGMII_PCS_INT_STA_2 0x02d8
#define PCS_USXGMII_RPCS_FSM_DEC_ERR_INT BIT(24)
#define PCS_USXGMII_R_TYPE_E_INT BIT(16)
#define PCS_USXGMII_R_TYPE_T_INT BIT(8)
#define PCS_USXGMII_R_TYPE_D_INT BIT(0)
#define PCS_USXGMII_PCS_INT_STA_3 0x02dc
#define PCS_USXGMII_FAIL_SYNC_XOR_ST_INT BIT(24)
#define PCS_USXGMII_RX_BLOCK_LOCK_ST_INT BIT(16)
#define PCS_USXGMII_LINK_UP_ST_INT BIT(8)
#define PCS_USXGMII_HI_BER_ST_INT BIT(0)
#define PCS_USXGMII_PCS_CTRL_4 0x02e0
#define PCS_USXGMII_LINK_DOWN_ST_INT_EN BIT(0)
#define PCS_USXGMII_PCS_INT_STA_4 0x02e4
#define PCS_USXGMII_LINK_DOWN_ST_INT BIT(0)
#define PCS_USXGMII_PCS_AN_CONTROL_0 0x02f8
#define PCS_USXGMII_AN_RESTART BIT(8)
#define PCS_USXGMII_AN_ENABLE BIT(0)
#define PCS_USXGMII_PCS_AN_STATS_0 0x0310
#define PCS_USXGMII_PCS_AN_STATS_2 0x0318
#define PCS_USXGMII_PCS_AN_CONTROL_6 0x031c
#define PCS_USXGMII_TOG_PCS_AUTONEG_STS BIT(0)
#define PCS_USXGMII_PCS_AN_CONTROL_7 0x0320
#define PCS_USXGMII_RATE_UPDATE_MODE BIT(12)
#define PCS_USXGMII_MODE GENMASK(10, 8)
#define PCS_USXGMII_MODE_10000 FIELD_PREP(PCS_USXGMII_MODE, 0x0)
#define PCS_USXGMII_MODE_5000 FIELD_PREP(PCS_USXGMII_MODE, 0x1)
#define PCS_USXGMII_MODE_2500 FIELD_PREP(PCS_USXGMII_MODE, 0x2)
#define PCS_USXGMII_MODE_1000 FIELD_PREP(PCS_USXGMII_MODE, 0x3)
#define PCS_USXGMII_MODE_100 FIELD_PREP(PCS_USXGMII_MODE, 0x4)

#define PCS_HSGMII_RATE_ADAPT_CTRL_0 0x0000
#define PCS_HSGMII_RATE_ADAPT_RX_BYPASS BIT(27)
#define PCS_HSGMII_RATE_ADAPT_TX_BYPASS BIT(26)
#define PCS_HSGMII_RATE_ADAPT_RX_EN BIT(4)
#define PCS_HSGMII_RATE_ADAPT_TX_EN BIT(0)
#define PCS_HSGMII_RATE_ADAPT_CTRL_11 0x002c
#define PCS_HSGMII_FORCE_RATE_MODE_EN BIT(8)
#define PCS_HSGMII_FORCE_RATE_MODE GENMASK(7, 4)
#define PCS_HSGMII_FORCE_RATE_MODE_10000 \
	FIELD_PREP(PCS_HSGMII_FORCE_RATE_MODE, 0x8)
#define PCS_HSGMII_FORCE_RATE_MODE_5000 \
	FIELD_PREP(PCS_HSGMII_FORCE_RATE_MODE, 0x1)
#define PCS_HSGMII_FORCE_RATE_MODE_2500 \
	FIELD_PREP(PCS_HSGMII_FORCE_RATE_MODE, 0x2)
#define PCS_HSGMII_FORCE_RATE_MODE_1000 \
	FIELD_PREP(PCS_HSGMII_FORCE_RATE_MODE, 0x4)
#define PCS_HSGMII_FORCE_RATE_MODE_100 \
	FIELD_PREP(PCS_HSGMII_FORCE_RATE_MODE, 0x6)

#define PCS_PMA_DIG_RESERVE_0 0x0360
#define PCS_TRIGGER_RX_SIGDET_SCAN GENMASK(17, 16)
#define PCS_PMA_DIG_RO_RESERVE_2 0x0380
#define PCS_RX_SIGDET BIT(8)
#define PCS_PMA_RX_FREQDET 0x0530
#define PCS_PMA_FBCK_LOCK BIT(0)

/* SWITCH */
#define SWITCH_CFC 0x04
#define SWITCH_CPU_PMAP GENMASK(7, 0)
#define SWITCH_AGC 0x0c
#define SWITCH_LOCAL_EN BIT(7)
#define SWITCH_MFC 0x10
#define SWITCH_BC_FFP GENMASK(31, 24)
#define SWITCH_UNM_FFP GENMASK(23, 16)
#define SWITCH_UNU_FFP GENMASK(15, 8)
#define SWITCH_PCR(_n) (0x2004 + ((_n) * 0x100))
#define SWITCH_PORT_VLAN_MASK GENMASK(1, 0)
#define SWITCH_PORT_FALLBACK_MODE 1
#define SWITCH_PORT_MATRIX GENMASK(23, 16)
#define SWITCH_PVC(_n) (0x2010 + ((_n) * 0x100))
#define SWITCH_STAG_VPID GENMASK(31, 16)
#define SWITCH_VLAN_ATTR GENMASK(7, 6)
#define SWITCH_VLAN_ATTR_USER 0
#define SWITCH_PORT_SPEC_TAG BIT(5)
#define SWITCH_PMCR(_n) 0x3000 + ((_n) * 0x100)
#define SWITCH_IPG_CFG GENMASK(19, 18)
#define SWITCH_IPG_CFG_NORMAL FIELD_PREP(SWITCH_IPG_CFG, 0x0)
#define SWITCH_IPG_CFG_SHORT FIELD_PREP(SWITCH_IPG_CFG, 0x1)
#define SWITCH_IPG_CFG_SHRINK FIELD_PREP(SWITCH_IPG_CFG, 0x2)
#define SWITCH_MAC_MODE BIT(16)
#define SWITCH_FORCE_MODE BIT(15)
#define SWITCH_MAC_TX_EN BIT(14)
#define SWITCH_MAC_RX_EN BIT(13)
#define SWITCH_BKOFF_EN BIT(9)
#define SWITCH_BKPR_EN BIT(8)
#define SWITCH_FORCE_RX_FC BIT(5)
#define SWITCH_FORCE_TX_FC BIT(4)
#define SWITCH_FORCE_SPD GENMASK(3, 2)
#define SWITCH_FORCE_SPD_10 FIELD_PREP(SWITCH_FORCE_SPD, 0x0)
#define SWITCH_FORCE_SPD_100 FIELD_PREP(SWITCH_FORCE_SPD, 0x1)
#define SWITCH_FORCE_SPD_1000 FIELD_PREP(SWITCH_FORCE_SPD, 0x2)
#define SWITCH_FORCE_DPX BIT(1)
#define SWITCH_FORCE_LNK BIT(0)
#define SWITCH_SMACCR0 0x30e4
#define SMACCR0_MAC2 GENMASK(31, 24)
#define SMACCR0_MAC3 GENMASK(23, 16)
#define SMACCR0_MAC4 GENMASK(15, 8)
#define SMACCR0_MAC5 GENMASK(7, 0)
#define SWITCH_SMACCR1 0x30e8
#define SMACCR1_MAC0 GENMASK(15, 8)
#define SMACCR1_MAC1 GENMASK(7, 0)
#define SWITCH_PHY_POLL 0x7018
#define SWITCH_PHY_AP_EN GENMASK(30, 24)
#define SWITCH_EEE_POLL_EN GENMASK(22, 16)
#define SWITCH_PHY_PRE_EN BIT(15)
#define SWITCH_PHY_END_ADDR GENMASK(12, 8)
#define SWITCH_PHY_ST_ADDR GENMASK(4, 0)
#define SWITCH_CPORT_SPTAG_CFG 0x7c10
#define SWITCH_SW2FE_STAG_EN BIT(1)
#define SWITCH_FE2SW_STAG_EN BIT(0)

/* FE */
#define PSE_BASE 0x0100
#define CSR_IFC_BASE 0x0200
#define CDM1_BASE 0x0400
#define GDM1_BASE 0x0500
#define PPE1_BASE 0x0c00

#define CDM2_BASE 0x1400
#define GDM2_BASE 0x1500

#define GDM3_BASE 0x1100
#define GDM4_BASE 0x2500

#define GDM_BASE(_n)             \
	((_n) == 4 ? GDM4_BASE : \
	 (_n) == 3 ? GDM3_BASE : \
	 (_n) == 2 ? GDM2_BASE : \
		     GDM1_BASE)

#define REG_FE_DMA_GLO_CFG 0x0000
#define FE_DMA_GLO_L2_SPACE_MASK GENMASK(7, 4)
#define FE_DMA_GLO_PG_SZ_MASK BIT(3)

#define REG_FE_RST_GLO_CFG 0x0004
#define FE_RST_GDM4_MBI_ARB_MASK BIT(3)
#define FE_RST_GDM3_MBI_ARB_MASK BIT(2)
#define FE_RST_CORE_MASK BIT(0)

#define REG_FE_WAN_MAC_H 0x0030
#define REG_FE_LAN_MAC_H 0x0040
#define REG_FE_MAC_LMIN(_n) ((_n) + 0x04)
#define REG_FE_MAC_LMAX(_n) ((_n) + 0x08)

#define REG_FE_CDM1_OQ_MAP0 0x0050
#define REG_FE_CDM1_OQ_MAP1 0x0054
#define REG_FE_CDM1_OQ_MAP2 0x0058
#define REG_FE_CDM1_OQ_MAP3 0x005c

#define REG_FE_WAN_PORT 0x0024
#define WAN1_EN_MASK BIT(16)
#define WAN1_MASK GENMASK(12, 8)
#define WAN0_MASK GENMASK(4, 0)

#define REG_FE_PCE_CFG 0x0070
#define PCE_DPI_EN_MASK BIT(2)
#define PCE_KA_EN_MASK BIT(1)
#define PCE_MC_EN_MASK BIT(0)

#define REG_FE_PSE_QUEUE_CFG_WR 0x0080
#define PSE_CFG_PORT_ID_MASK GENMASK(27, 24)
#define PSE_CFG_QUEUE_ID_MASK GENMASK(20, 16)
#define PSE_CFG_WR_EN_MASK BIT(8)
#define PSE_CFG_OQRSV_SEL_MASK BIT(0)

#define REG_FE_PSE_QUEUE_CFG_VAL 0x0084
#define PSE_CFG_OQ_RSV_MASK GENMASK(13, 0)

#define PSE_FQ_CFG 0x008c
#define PSE_FQ_LIMIT_MASK GENMASK(14, 0)

#define REG_FE_PSE_BUF_SET 0x0090
#define PSE_SHARE_USED_LTHD_MASK GENMASK(31, 16)
#define PSE_ALLRSV_MASK GENMASK(14, 0)

#define REG_PSE_SHARE_USED_THD 0x0094
#define PSE_SHARE_USED_MTHD_MASK GENMASK(31, 16)
#define PSE_SHARE_USED_HTHD_MASK GENMASK(15, 0)

#define REG_GDM_MISC_CFG 0x0148
#define GDM2_RDM_ACK_WAIT_PREF_MASK BIT(9)
#define GDM2_CHN_VLD_MODE_MASK BIT(5)

#define REG_FE_CSR_IFC_CFG CSR_IFC_BASE
#define FE_IFC_EN_MASK BIT(0)
#define REG_FE_VIP_PORT_EN 0x01f0
#define REG_FE_IFC_PORT_EN 0x01f4

#define REG_PSE_IQ_REV1 (PSE_BASE + 0x08)
#define PSE_IQ_RES1_P2_MASK GENMASK(23, 16)

#define REG_PSE_IQ_REV2 (PSE_BASE + 0x0c)
#define PSE_IQ_RES2_P5_MASK GENMASK(15, 8)
#define PSE_IQ_RES2_P4_MASK GENMASK(7, 0)

#define REG_CDM1_FWD_CFG (CDM1_BASE + 0x08)
#define REG_CDM1_VLAN_CTRL CDM1_BASE
#define CDM1_VLAN_MASK GENMASK(31, 16)
#define CDM1_VIP_QSEL_MASK GENMASK(24, 20)
#define REG_CDM1_CRSN_QSEL(_n) (CDM1_BASE + 0x10 + ((_n) << 2))
#define CDM1_CRSN_QSEL_REASON_MASK(_n) \
	GENMASK(4 + (((_n) % 4) << 3), (((_n) % 4) << 3))

#define REG_CDM2_FWD_CFG (CDM2_BASE + 0x08)
#define CDM2_OAM_QSEL_MASK GENMASK(31, 27)
#define CDM2_VIP_QSEL_MASK GENMASK(24, 20)
#define REG_CDM2_CRSN_QSEL(_n) (CDM2_BASE + 0x10 + ((_n) << 2))
#define CDM2_CRSN_QSEL_REASON_MASK(_n) \
	GENMASK(4 + (((_n) % 4) << 3), (((_n) % 4) << 3))

#define REG_GDM_FWD_CFG(_n) GDM_BASE(_n)
#define GDM_PAD_EN BIT(28)
#define GDM_DROP_CRC_ERR BIT(23)
#define GDM_IP4_CKSUM BIT(22)
#define GDM_TCP_CKSUM BIT(21)
#define GDM_UDP_CKSUM BIT(20)
#define GDM_STRIP_CRC BIT(16)
#define GDM_UCFQ_MASK GENMASK(15, 12)
#define GDM_BCFQ_MASK GENMASK(11, 8)
#define GDM_MCFQ_MASK GENMASK(7, 4)
#define GDM_OCFQ_MASK GENMASK(3, 0)

#define REG_GDM_INGRESS_CFG(_n) (GDM_BASE(_n) + 0x10)
#define GDM_STAG_EN_MASK BIT(0)

#define REG_GDM_LEN_CFG(_n) (GDM_BASE(_n) + 0x14)
#define GDM_SHORT_LEN_MASK GENMASK(13, 0)
#define GDM_LONG_LEN_MASK GENMASK(29, 16)
#define REG_GDM_LPBK_CFG(_n) (GDM_BASE(_n) + 0x1c)
#define LPBK_CHAN_MASK GENMASK(8, 4)
#define LPBK_MODE_MASK GENMASK(3, 1)
#define LBK_GAP_MODE_MASK BIT(3)
#define LBK_LEN_MODE_MASK BIT(2)
#define LBK_CHAN_MODE_MASK BIT(1)
#define LPBK_EN_MASK BIT(0)
#define REG_GDM_TXCHN_EN(_n) (GDM_BASE(_n) + 0x24)
#define REG_GDM_RXCHN_EN(_n) (GDM_BASE(_n) + 0x28)

#define PPE2_BASE 0x1c00
#define REG_PPE_DFT_CPORT_BASE(_n) (((_n) ? PPE2_BASE : PPE1_BASE) + 0x248)
#define REG_PPE_DFT_CPORT(_m, _n) \
	(REG_PPE_DFT_CPORT_BASE(_m) + (((_n) / 8) << 2))
#define DFT_CPORT_MASK(_n) \
	GENMASK(3 + (((_n) % 8) << 2), (((_n) % 8) << 2))
#define REG_FE_CPORT_CFG (GDM1_BASE + 0x40)
#define FE_CPORT_PAD BIT(26)
#define FE_CPORT_PORT_XFC_MASK BIT(25)
#define FE_CPORT_QUEUE_XFC_MASK BIT(24)

#define REG_GDMA4_TMBI_FRAG 0x2028
#define GDMA4_SGMII1_TX_WEIGHT GENMASK(31, 26)
#define GDMA4_SGMII1_TX_FRAG_SIZE GENMASK(25, 16)
#define GDMA4_SGMII0_TX_WEIGHT GENMASK(15, 10)
#define GDMA4_SGMII0_TX_FRAG_SIZE GENMASK(9, 0)

#define REG_GDMA4_RMBI_FRAG 0x202c
#define GDMA4_SGMII1_RX_WEIGHT GENMASK(31, 26)
#define GDMA4_SGMII1_RX_FRAG_SIZE GENMASK(25, 16)
#define GDMA4_SGMII0_RX_WEIGHT GENMASK(15, 10)
#define GDMA4_SGMII0_RX_FRAG_SIZE GENMASK(9, 0)

#define REG_GDM2_CHN_RLS (GDM2_BASE + 0x20)
#define MBI_RX_AGE_SEL_MASK GENMASK(26, 25)
#define MBI_TX_AGE_SEL_MASK GENMASK(18, 17)
#define REG_GDM3_FWD_CFG GDM3_BASE
#define GDM3_PAD_EN_MASK BIT(28)
#define REG_GDM4_FWD_CFG GDM4_BASE
#define GDM4_PAD_EN_MASK BIT(28)
#define REG_GDM4_SRC_PORT_SET (GDM4_BASE + 0x23c)
#define REG_GDM4_SRC_PORT_SET_LEGACY (GDM4_BASE + 0x33c)
#define GDM4_SPORT_OFF2_MASK GENMASK(19, 16)
#define GDM4_SPORT_OFF1_MASK GENMASK(15, 12)
#define GDM4_SPORT_OFF0_MASK GENMASK(11, 8)

#define REG_SP_DFT_CPORT(_n) (0x20e0 + ((_n) << 2))
#define SP_CPORT_DFT_MASK GENMASK(2, 0)
#define SP_CPORT_MASK(_n) GENMASK(3 + ((_n) << 2), ((_n) << 2))

#define REG_SRC_PORT_FC_MAP6 0x2298
#define FC_ID_OF_SRC_PORT24_MASK GENMASK(4, 0)

#define REG_FE_DBG_GDM4_TX_OK 0x2604
#define REG_FE_DBG_GDM4_RX_BASE 0x2648
#define REG_FE_GDM_MIB_CLEAR(_n) (GDM_BASE(_n) + 0xf0)
#define FE_GDM_MIB_RX_CLEAR_MASK BIT(1)
#define FE_GDM_MIB_TX_CLEAR_MASK BIT(0)
#define REG_FE_GDM_TX_OK_PKT_CNT_L(_n) (GDM_BASE(_n) + 0x104)
#define REG_FE_GDM_TX_ETH_PKT_CNT_L(_n) (GDM_BASE(_n) + 0x110)
#define REG_FE_GDM_TX_ETH_DROP_CNT(_n) (GDM_BASE(_n) + 0x118)
#define REG_FE_GDM_RX_OK_PKT_CNT_L(_n) (GDM_BASE(_n) + 0x148)
#define REG_FE_GDM_RX_ETH_DROP_CNT(_n) (GDM_BASE(_n) + 0x168)
#define REG_FE_GDM_TX_OK_PKT_CNT_H(_n) (GDM_BASE(_n) + 0x280)
#define REG_FE_GDM_TX_ETH_PKT_CNT_H(_n) (GDM_BASE(_n) + 0x288)
#define REG_FE_GDM_RX_OK_PKT_CNT_H(_n) (GDM_BASE(_n) + 0x290)
#define REG_QDMA_DBG_TX_BASE 0x0108
#define REG_QDMA_DBG_RX_BASE 0x0208

/* QDMA */
#define REG_QDMA_GLOBAL_CFG 0x0004
#define GLOBAL_CFG_RX_2B_OFFSET_MASK BIT(31)
#define GLOBAL_CFG_DMA_PREFERENCE_MASK GENMASK(30, 29)
#define GLOBAL_CFG_CPU_TXR_RR_MASK BIT(28)
#define GLOBAL_CFG_DSCP_BYTE_SWAP_MASK BIT(27)
#define GLOBAL_CFG_PAYLOAD_BYTE_SWAP_MASK BIT(26)
#define GLOBAL_CFG_MULTICAST_MODIFY_FP_MASK BIT(25)
#define GLOBAL_CFG_OAM_MODIFY_MASK BIT(24)
#define GLOBAL_CFG_RESET_MASK BIT(23)
#define GLOBAL_CFG_RESET_DONE_MASK BIT(22)
#define GLOBAL_CFG_MULTICAST_EN_MASK BIT(21)
#define GLOBAL_CFG_IRQ1_EN_MASK BIT(20)
#define GLOBAL_CFG_IRQ0_EN_MASK BIT(19)
#define GLOBAL_CFG_LOOPCNT_EN_MASK BIT(18)
#define GLOBAL_CFG_RD_BYPASS_WR_MASK BIT(17)
#define GLOBAL_CFG_QDMA_LOOPBACK_MASK BIT(16)
#define GLOBAL_CFG_LPBK_RXQ_SEL_MASK GENMASK(13, 8)
#define GLOBAL_CFG_CHECK_DONE_MASK BIT(7)
#define GLOBAL_CFG_TX_WB_DONE_MASK BIT(6)
#define GLOBAL_CFG_MAX_ISSUE_NUM_MASK GENMASK(5, 4)
#define GLOBAL_CFG_RX_DMA_BUSY_MASK BIT(3)
#define GLOBAL_CFG_RX_DMA_EN_MASK BIT(2)
#define GLOBAL_CFG_TX_DMA_BUSY_MASK BIT(1)
#define GLOBAL_CFG_TX_DMA_EN_MASK BIT(0)

#define REG_FWD_DSCP_BASE 0x0010
#define REG_FWD_BUF_BASE 0x0014

#define REG_HW_FWD_DSCP_CFG 0x0018
#define HW_FWD_DSCP_PAYLOAD_SIZE_MASK GENMASK(29, 28)
#define HW_FWD_DSCP_SCATTER_LEN_MASK GENMASK(17, 16)
#define HW_FWD_DSCP_MIN_SCATTER_LEN_MASK GENMASK(15, 0)

#define REG_INT_STATUS(_n)      \
	(((_n) == 4) ? 0x0730 : \
	 ((_n) == 3) ? 0x0724 : \
	 ((_n) == 2) ? 0x0720 : \
	 ((_n) == 1) ? 0x0024 : \
		       0x0020)

#define REG_TX_IRQ_BASE(_n) ((_n) ? 0x0048 : 0x0050)

#define REG_TX_IRQ_CFG(_n) ((_n) ? 0x004c : 0x0054)
#define TX_IRQ_THR_MASK GENMASK(27, 16)
#define TX_IRQ_DEPTH_MASK GENMASK(11, 0)

#define REG_IRQ_CLEAR_LEN(_n) ((_n) ? 0x0064 : 0x0058)
#define IRQ_CLEAR_LEN_MASK GENMASK(7, 0)

#define REG_TX_RING_BASE(_n) \
	(((_n) < 8) ? 0x0100 + ((_n) << 5) : 0x0b00 + (((_n) - 8) << 5))

#define REG_TX_CPU_IDX(_n) \
	(((_n) < 8) ? 0x0108 + ((_n) << 5) : 0x0b08 + (((_n) - 8) << 5))

#define TX_RING_CPU_IDX_MASK GENMASK(15, 0)

#define REG_TX_DMA_IDX(_n) \
	(((_n) < 8) ? 0x010c + ((_n) << 5) : 0x0b0c + (((_n) - 8) << 5))

#define TX_RING_DMA_IDX_MASK GENMASK(15, 0)

#define IRQ_RING_IDX_MASK GENMASK(20, 16)
#define IRQ_DESC_IDX_MASK GENMASK(15, 0)

#define REG_RX_RING_BASE(_n) \
	(((_n) < 16) ? 0x0200 + ((_n) << 5) : 0x0e00 + (((_n) - 16) << 5))

#define REG_RX_RING_SIZE(_n) \
	(((_n) < 16) ? 0x0204 + ((_n) << 5) : 0x0e04 + (((_n) - 16) << 5))

#define RX_RING_THR_MASK GENMASK(31, 16)
#define RX_RING_SIZE_MASK GENMASK(15, 0)

#define REG_RX_CPU_IDX(_n) \
	(((_n) < 16) ? 0x0208 + ((_n) << 5) : 0x0e08 + (((_n) - 16) << 5))

#define RX_RING_CPU_IDX_MASK GENMASK(15, 0)

#define REG_RX_DMA_IDX(_n) \
	(((_n) < 16) ? 0x020c + ((_n) << 5) : 0x0e0c + (((_n) - 16) << 5))

#define REG_RX_DELAY_INT_IDX(_n) \
	(((_n) < 16) ? 0x0210 + ((_n) << 5) : 0x0e10 + (((_n) - 16) << 5))

#define RX_DELAY_INT_MASK GENMASK(15, 0)

#define RX_RING_DMA_IDX_MASK GENMASK(15, 0)

#define REG_LMGR_INIT_CFG 0x1000
#define LMGR_INIT_START BIT(31)
#define LMGR_SRAM_MODE_MASK BIT(30)
#define HW_FWD_PKTSIZE_OVERHEAD_MASK GENMASK(27, 20)
#define HW_FWD_DESC_NUM_MASK GENMASK(16, 0)

/* CTRL */
#define QDMA_DESC_DONE_MASK BIT(31)
#define QDMA_DESC_DROP_MASK BIT(30) /* tx: drop - rx: overflow */
#define QDMA_DESC_MORE_MASK BIT(29) /* more SG elements */
#define QDMA_DESC_DEI_MASK BIT(25)
#define QDMA_DESC_NO_DROP_MASK BIT(24)
#define QDMA_DESC_LEN_MASK GENMASK(15, 0)
/* DATA */
#define QDMA_DESC_NEXT_ID_MASK GENMASK(15, 0)
/* TX MSG0 */
#define QDMA_ETH_TXMSG_MIC_IDX_MASK BIT(30)
#define QDMA_ETH_TXMSG_SP_TAG_MASK GENMASK(29, 14)
#define QDMA_ETH_TXMSG_ICO_MASK BIT(13)
#define QDMA_ETH_TXMSG_UCO_MASK BIT(12)
#define QDMA_ETH_TXMSG_TCO_MASK BIT(11)
#define QDMA_ETH_TXMSG_TSO_MASK BIT(10)
#define QDMA_ETH_TXMSG_FAST_MASK BIT(9)
#define QDMA_ETH_TXMSG_OAM_MASK BIT(8)
#define QDMA_ETH_TXMSG_CHAN_MASK GENMASK(7, 3)
#define QDMA_ETH_TXMSG_QUEUE_MASK GENMASK(2, 0)
/* TX MSG1 */
#define QDMA_ETH_TXMSG_NO_DROP BIT(31)
#define QDMA_ETH_TXMSG_METER_MASK GENMASK(30, 24) /* 0x7f no meters */
#define QDMA_ETH_TXMSG_FPORT_MASK GENMASK(23, 20)
#define QDMA_ETH_TXMSG_NBOQ_MASK GENMASK(19, 15)
#define QDMA_ETH_TXMSG_HWF_MASK BIT(14)
#define QDMA_ETH_TXMSG_HOP_MASK BIT(13)
#define QDMA_ETH_TXMSG_PTP_MASK BIT(12)
#define QDMA_ETH_TXMSG_ACNT_G1_MASK GENMASK(10, 6) /* 0x1f do not count */
#define QDMA_ETH_TXMSG_ACNT_G0_MASK GENMASK(5, 0) /* 0x3f do not count */

/* RX MSG1 */
#define QDMA_ETH_RXMSG_DEI_MASK BIT(31)
#define QDMA_ETH_RXMSG_IP6_MASK BIT(30)
#define QDMA_ETH_RXMSG_IP4_MASK BIT(29)
#define QDMA_ETH_RXMSG_IP4F_MASK BIT(28)
#define QDMA_ETH_RXMSG_L4_VALID_MASK BIT(27)
#define QDMA_ETH_RXMSG_L4F_MASK BIT(26)
#define QDMA_ETH_RXMSG_SPORT_MASK GENMASK(25, 21)
#define QDMA_ETH_RXMSG_CRSN_MASK GENMASK(20, 16)
#define QDMA_ETH_RXMSG_PPE_ENTRY_MASK GENMASK(15, 0)

struct airoha_qdma_desc {
	__le32 rsv;
	__le32 ctrl;
	__le32 addr;
	__le32 data;
	__le32 msg0;
	__le32 msg1;
	__le32 msg2;
	__le32 msg3;
};

struct airoha_qdma_fwd_desc {
	__le32 addr;
	__le32 ctrl0;
	__le32 ctrl1;
	__le32 ctrl2;
	__le32 msg0;
	__le32 msg1;
	__le32 rsv0;
	__le32 rsv1;
};

struct airoha_queue {
	struct airoha_qdma_desc *desc;
	uchar *rx_buf;
	u16 head;
	bool pending;

	int ndesc;
};

struct airoha_tx_irq_queue {
	struct airoha_qdma *qdma;

	int size;
	u32 *q;
};

struct airoha_qdma {
	struct airoha_eth *eth;
	void __iomem *regs;

	struct airoha_tx_irq_queue q_tx_irq[AIROHA_NUM_TX_IRQ];

	struct airoha_queue q_tx[AIROHA_NUM_TX_RING];
	struct airoha_queue q_rx[AIROHA_NUM_RX_RING];

	/* descriptor and packet buffers for qdma hw forward */
	struct {
		void *desc;
		void *q;
	} hfwd;
};

struct airoha_gdm_port {
	struct airoha_qdma *qdma;
	int id;
};

struct airoha_peer_fport {
	u8 mac[ARP_HLEN];
	u8 fport;
	bool valid;
};

struct airoha_eth {
	void __iomem *fe_regs;
	void __iomem *switch_regs;
	void __iomem *eth_pcs_xfi_mac;
	void __iomem *eth_pcs_multi_sgmii;
	void __iomem *eth_pcs_usxgmii;
	void __iomem *eth_pcs_hsgmii_rate_adp;
	void __iomem *eth_pcs_xfi_ana;
	void __iomem *eth_pcs_xfi_pma;

	struct regmap *scu_regmap;
	struct regmap *chip_scu_regmap;
	struct udevice *mdio_dev;
	struct udevice *switch_mdio_dev;
	struct reset_ctl_bulk rsts;
	struct reset_ctl_bulk xsi_rsts;
	struct reset_ctl switch_rst;
	bool has_switch_rst;
	u8 tx_fport;
	bool gdm4_pcs_ready;
	bool gdm4_link_up;
	bool rtl8261_init_done;
	bool rtl8261_phy5_pnswap_tx;
	bool rtl8261_phy5_pnswap_rx;
	bool rtl8261_phy8_pnswap_tx;
	bool rtl8261_phy8_pnswap_rx;
	bool last_rx_valid;
	u8 last_rx_qdma;
	u8 last_rx_sport;
	u8 last_rx_crsn;
	u16 last_rx_index;
	u16 last_rx_ppe_entry;
	u16 last_rx_len;
	u32 last_rx_ctrl;
	u32 last_rx_msg1;
	u8 peer_fport_next;
	struct airoha_peer_fport peer_fport[AIROHA_PEER_FPORT_CACHE_SIZE];

	struct airoha_qdma qdma[AIROHA_MAX_NUM_QDMA];
	struct airoha_gdm_port *ports[AIROHA_MAX_NUM_GDM_PORTS];
};

struct airoha_eth_soc_data {
	int num_xsi_rsts;
	const char *const *xsi_rsts_names;
	const char *switch_compatible;
};

static bool airoha_rtl8261_is_phy_addr(int phy_addr);

enum {
	FE_PSE_PORT_CDM1,
	FE_PSE_PORT_GDM1,
	FE_PSE_PORT_GDM2,
	FE_PSE_PORT_GDM3,
	FE_PSE_PORT_PPE1,
	FE_PSE_PORT_CDM2,
	FE_PSE_PORT_CDM3,
	FE_PSE_PORT_CDM4,
	FE_PSE_PORT_PPE2,
	FE_PSE_PORT_GDM4,
	FE_PSE_PORT_CDM5,
};

enum {
	HSGMII_LAN_7581_PCIE0_SRCPORT = 0x16,
	HSGMII_LAN_7581_PCIE1_SRCPORT,
	HSGMII_LAN_7581_ETH_SRCPORT,
	HSGMII_LAN_7581_USB_SRCPORT,
};

enum {
	CDM_CRSN_QSEL_Q0 = 0,
	CDM_CRSN_QSEL_Q1 = 1,
	CDM_CRSN_QSEL_Q6 = 6,
};

enum {
	CRSN_08 = 0x8,
	CRSN_21 = 0x15,
	CRSN_22 = 0x16,
	CRSN_24 = 0x18,
	CRSN_25 = 0x19,
};

#define XSI_ETH_VIP_PORT_MASK BIT(24)

#define RTL8261_PHY5_ADDR 5
#define RTL8261_PHY8_ADDR 8
#define RTL8261_PHY_ID2 MII_PHYSID2
#define RTL8261_PHY_ID2_EXPECT 0xcaf3
#define RTL8261_MMD_VEND1 30
#define RTL8261_MMD_VEND2 31
#define RTL8261_PHY_CHIP_ID 0x0104
#define RTL8261_PHY_EXT_RESET 0x0145
#define RTL8261_PHY_SDS_DATA 0x0141
#define RTL8261_PHY_SDS_RD_WR 0x0142
#define RTL8261_PHY_SDS_CMD 0x0143
#define RTL8261_PHY_SDS_BUSY BIT(15)
#define RTL8261_PHY_SDS_CMD_READ(page, reg) \
	(0x8000 | (((page) & 0x3f) | (((reg) & 0x1f) << 6)))
#define RTL8261_PHY_SDS_CMD_WRITE(page, reg) \
	(0x8800 | (((page) & 0x3f) | (((reg) & 0x1f) << 6)))
#define RTL8261_PHY_SDS_XR1710G_MODE 0x88c6
#define RTL8261_PHY_10G_CTRL 0x0000
#define RTL8261_PHY_10G_AN_DIS BIT(11)
#define RTL8261_PMA_STATUS1 0x0001
#define RTL8261_PMA_MGBT_STATUS 0x0081
#define RTL8261_PMA_MGBT_PAIR_SWAP 0x0082
#define RTL8261_PCS_STATUS1 0x0001
#define RTL8261_PCS_BASE_R_STATUS1 0x0020
#define RTL8261_PCS_BASE_R_STATUS2 0x0021
#define RTL8261_AN_STATUS1 0x0001
#define RTL8261_AN_MGBT_CTRL1 0x0020
#define RTL8261_AN_MGBT_STATUS1 0x0021
#define RTL8261_PHY_SPEED_STATUS 0xa434
#define RTL8261_PHY_SERDES_OP_CTRL 0x7587
#define RTL8261_PHY_SERDES_OP_ADDR 0x7588
#define RTL8261_PHY_SERDES_OP_DATA 0x7589
#define RTL8261_PHY_SERDES_OPTION_5G 0x6973
#define RTL8261_PHY_SERDES_OPTION_2P5G 0x6974
#define RTL8261_PHY_SERDES_OPTION_1G 0x6975
#define RTL8261_PHY_SERDES_OPTION_100M 0x6976
#define RTL8261_PHY_SERDES_OPTION_10M 0x6977
#define RTL8261_PHY_SPEED_MASK 0x0630
#define RTL8261_PHY_SPEED_10M 0x0000
#define RTL8261_PHY_SPEED_100M 0x0010
#define RTL8261_PHY_SPEED_1000M 0x0020
#define RTL8261_PHY_SPEED_2500M 0x0210
#define RTL8261_PHY_SPEED_5000M 0x0220
#define RTL8261_PHY_SPEED_10000M 0x0200
#define RTL8261_SERDES_SPEED_USXGMII 0x0
#define RTL8261BE_CHIP_ID_MASK 0xffc0
#define RTL8261BE_CHIP_ID 0x1140
#define RTL8261_SERDES_GLOBAL_CFG 0x00c1
#define RTL8261_SERDES_HSO_INV BIT(7)
#define RTL8261_SERDES_HSI_INV BIT(6)

#define RTK_PATCH_CMP_W 0
#define RTK_PATCH_CMP_WC 1
#define RTK_PATCH_CMP_SWC 2
#define RTK_PATCH_CMP_WS 3

#define RTK_PATCH_OP_PHY 0
#define RTK_PATCH_OP_PHYOCP 1
#define RTK_PATCH_OP_TOP 2
#define RTK_PATCH_OP_TOPOCP 3
#define RTK_PATCH_OP_PSDS0 4
#define RTK_PATCH_OP_PSDS1 5
#define RTK_PATCH_OP_MSDS 6
#define RTK_PATCH_OP_MAC 7
#define RTK_PATCH_OP_DELAY_MS 200

typedef struct rtk_hwpatch_s {
	u8 patch_op;
	u8 portmask;
	u16 pagemmd;
	u16 addr;
	u8 msb;
	u8 lsb;
	u16 data;
	u8 compare_op;
	u16 sram_p;
	u16 sram_rr;
	u16 sram_rw;
	u16 sram_a;
} rtk_hwpatch_t;

#include "rtl8261be_patch_table.inc"

static const char *const en7523_xsi_rsts_names[] = {
	"hsi0-mac",
	"hsi1-mac",
	"hsi-mac",
};

static const char *const en7581_xsi_rsts_names[] = {
	"hsi0-mac",
	"hsi1-mac",
	"hsi-mac",
	"xfp-mac",
};

static u32 airoha_rr(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static void airoha_wr(void __iomem *base, u32 offset, u32 val)
{
	writel(val, base + offset);
}

static u32 airoha_rmw(void __iomem *base, u32 offset, u32 mask, u32 val)
{
	val |= (airoha_rr(base, offset) & ~mask);
	airoha_wr(base, offset, val);

	return val;
}

static void airoha_eth_gdm4_set_speed_mode(struct airoha_eth *eth, u32 mode,
					   u32 rate_adapt);
static void airoha_eth_gdm4_set_frag_size(struct airoha_eth *eth, u16 speed);
static void airoha_eth_gdm4_select_speed_mode(struct airoha_eth *eth);
static void airoha_eth_gdm4_sync_rtl8261(struct airoha_eth *eth);
static void airoha_eth_gdm4_cdr_reset(struct airoha_eth *eth);
static void airoha_eth_gdm4_phya_init(struct airoha_eth *eth);
static void airoha_eth_gdm4_diag(struct airoha_eth *eth, const char *tag);
static bool airoha_gdm4_loopback_enabled(struct airoha_eth *eth);
static bool airoha_rtl8261_host_update_enabled(void);
static int airoha_usxgmii_link_up(struct airoha_eth *eth);
static int airoha_gdm4_pcs_link_up(struct airoha_eth *eth);
static int airoha_eth_gdm4_have_rx_signal(struct airoha_eth *eth);
static bool airoha_qdma_ready(struct airoha_qdma *qdma);
static int arht_eth_write_hwaddr(struct udevice *dev);
static int airoha_rtl8261_speed_status_get(struct airoha_eth *eth, int phy_addr,
					   u16 *status);
static int airoha_rtl8261_link_state_get(struct airoha_eth *eth, int phy_addr,
					 bool *link_up, bool *full_duplex,
					 u16 *speed_status, u32 *mode,
					 u32 *rate_adapt);
static int airoha_rtl8261_apply_board_cfg(struct airoha_eth *eth, int phy_addr);
static int airoha_rtl8261_serdes_option_set(struct airoha_eth *eth, int phy_addr,
					    u16 option);
static int airoha_rtl8261_serdes_autoneg_set(struct airoha_eth *eth,
					     int phy_addr, bool enable);
static int airoha_rtl8261_serdes_mode_update(struct airoha_eth *eth,
					     int phy_addr);
static void airoha_eth_gdm4_apply_runtime_phy_cfg(struct airoha_eth *eth);

static void airoha_clrsetbits_le32(uintptr_t addr, u32 clear, u32 set)
{
	u32 val = readl((void __iomem *)addr);

	val &= ~clear;
	val |= set;
	writel(val, (void __iomem *)addr);
}

static uintptr_t airoha_gpio_data_reg(u32 gpio)
{
	return GPIO_SYSCTL_BASE + (gpio < 32 ? GPIO_REG_DATA : GPIO_REG_DATA1);
}

static uintptr_t airoha_gpio_oe_reg(u32 gpio)
{
	return GPIO_SYSCTL_BASE + (gpio < 32 ? GPIO_REG_OE : GPIO_REG_OE1);
}

static uintptr_t airoha_gpio_dir_reg(u32 gpio)
{
	static const u16 dir_regs[] = {
		GPIO_REG_CTRL,
		GPIO_REG_CTRL1,
		GPIO_REG_CTRL2,
		GPIO_REG_CTRL3,
	};

	return GPIO_SYSCTL_BASE + dir_regs[gpio / 16];
}

static void airoha_gpio_direction_output(u32 gpio)
{
	u32 bank_bit = BIT(gpio % 32);
	u32 dir_bit = BIT(2 * (gpio % 16));

	airoha_clrsetbits_le32(airoha_gpio_oe_reg(gpio), 0, bank_bit);
	airoha_clrsetbits_le32(airoha_gpio_dir_reg(gpio), 0, dir_bit);
}

static void airoha_gpio_set_active_low(u32 gpio, int active)
{
	u32 bit = BIT(gpio % 32);

	airoha_clrsetbits_le32(airoha_gpio_data_reg(gpio), bit,
			       active ? 0 : bit);
}

static void airoha_delay_us(u32 delay_us)
{
	while (delay_us >= 1000) {
		mdelay(1);
		delay_us -= 1000;
	}

	if (delay_us)
		udelay(delay_us);
}

static void airoha_eth_pcs_pre_config(struct airoha_eth *eth)
{
	airoha_rmw(eth->eth_pcs_xfi_mac, PCS_XFI_GIB_CFG, 0,
		   PCS_XFI_RXMPI_STOP | PCS_XFI_RXMBI_STOP |
			   PCS_XFI_TXMPI_STOP | PCS_XFI_TXMBI_STOP);
	airoha_rmw(eth->eth_pcs_xfi_mac, PCS_XFI_LOGIC_RST,
		   PCS_XFI_MAC_LOGIC_RST, 0);
	airoha_rmw(eth->eth_pcs_xfi_mac, PCS_XFI_LOGIC_RST, 0,
		   PCS_XFI_MAC_LOGIC_RST);
	udelay(1500);
	airoha_rmw(eth->eth_pcs_xfi_mac, PCS_XFI_CNT_CLR, 0,
		   PCS_XFI_GLB_CNT_CLR);
}

static void airoha_eth_pcs_init_usxgmii(struct airoha_eth *eth)
{
	airoha_rmw(eth->eth_pcs_multi_sgmii, PCS_MULTI_SGMII_MSG_RX_CTRL_0, 0,
		   PCS_HSGMII_XFI_SEL);
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_CTROL_1,
		   PCS_USXGMII_SPEED_SEL_H, 0);
	airoha_wr(eth->eth_pcs_usxgmii, 0x22c, 0);

	airoha_rmw(eth->eth_pcs_hsgmii_rate_adp, PCS_HSGMII_RATE_ADAPT_CTRL_0,
		   PCS_HSGMII_RATE_ADAPT_RX_BYPASS |
			   PCS_HSGMII_RATE_ADAPT_TX_BYPASS |
			   PCS_HSGMII_RATE_ADAPT_RX_EN |
			   PCS_HSGMII_RATE_ADAPT_TX_EN,
		   PCS_HSGMII_RATE_ADAPT_RX_EN | PCS_HSGMII_RATE_ADAPT_TX_EN);

	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_AN_CONTROL_0, 0,
		   PCS_USXGMII_AN_ENABLE);
	/*
	 * mdio_dev is not ready during the first PCS bring-up, so the helper
	 * falls back to 10G here and re-applies the actual PHY speed later.
	 */
	airoha_eth_gdm4_select_speed_mode(eth);
}

static void airoha_eth_pcs_interrupt_init_usxgmii(struct airoha_eth *eth)
{
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_CTRL_0,
		   PCS_USXGMII_T_TYPE_T_INT_EN | PCS_USXGMII_T_TYPE_D_INT_EN |
			   PCS_USXGMII_T_TYPE_C_INT_EN |
			   PCS_USXGMII_T_TYPE_S_INT_EN,
		   0);
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_CTRL_1,
		   PCS_USXGMII_R_TYPE_C_INT_EN | PCS_USXGMII_R_TYPE_S_INT_EN |
			   PCS_USXGMII_TXPCS_FSM_ENC_ERR_INT_EN |
			   PCS_USXGMII_T_TYPE_E_INT_EN,
		   0);
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_CTRL_2,
		   PCS_USXGMII_RPCS_FSM_DEC_ERR_INT_EN |
			   PCS_USXGMII_R_TYPE_E_INT_EN |
			   PCS_USXGMII_R_TYPE_T_INT_EN |
			   PCS_USXGMII_R_TYPE_D_INT_EN,
		   0);
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_CTRL_3,
		   PCS_USXGMII_FAIL_SYNC_XOR_ST_INT_EN |
			   PCS_USXGMII_RX_BLOCK_LOCK_ST_INT_EN |
			   PCS_USXGMII_LINK_UP_ST_INT_EN |
			   PCS_USXGMII_HI_BER_ST_INT_EN,
		   0);
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_CTRL_4,
		   PCS_USXGMII_LINK_DOWN_ST_INT_EN, 0);
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_INT_STA_2, 0,
		   PCS_USXGMII_RPCS_FSM_DEC_ERR_INT | PCS_USXGMII_R_TYPE_E_INT |
			   PCS_USXGMII_R_TYPE_T_INT | PCS_USXGMII_R_TYPE_D_INT);
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_INT_STA_3, 0,
		   PCS_USXGMII_FAIL_SYNC_XOR_ST_INT |
			   PCS_USXGMII_RX_BLOCK_LOCK_ST_INT |
			   PCS_USXGMII_LINK_UP_ST_INT |
			   PCS_USXGMII_HI_BER_ST_INT);
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_INT_STA_4, 0,
		   PCS_USXGMII_LINK_DOWN_ST_INT);
}

static void airoha_eth_gdm4_restart_an(struct airoha_eth *eth)
{
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_AN_CONTROL_0, 0,
		   PCS_USXGMII_AN_RESTART);
	udelay(3);
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_AN_CONTROL_0,
		   PCS_USXGMII_AN_RESTART, 0);
}

static void airoha_eth_pcs_post_config(struct airoha_eth *eth)
{
	airoha_rmw(eth->eth_pcs_xfi_mac, PCS_XFI_GIB_CFG, PCS_XFI_RX_FRAG_LEN,
		   FIELD_PREP(PCS_XFI_RX_FRAG_LEN, 31));
	airoha_rmw(eth->eth_pcs_xfi_mac, PCS_XFI_GIB_CFG, PCS_XFI_TX_FRAG_LEN,
		   FIELD_PREP(PCS_XFI_TX_FRAG_LEN, 31));
	airoha_rmw(eth->eth_pcs_xfi_mac, PCS_XFI_GIB_CFG, PCS_XFI_IPG_NUM,
		   FIELD_PREP(PCS_XFI_IPG_NUM, 10));
	airoha_rmw(eth->eth_pcs_xfi_mac, PCS_XFI_GIB_CFG, 0,
		   PCS_XFI_TX_FC_EN | PCS_XFI_RX_FC_EN);
	airoha_rmw(eth->eth_pcs_xfi_mac, PCS_XFI_GIB_CFG,
		   PCS_XFI_RXMPI_STOP | PCS_XFI_RXMBI_STOP |
			   PCS_XFI_TXMPI_STOP | PCS_XFI_TXMBI_STOP,
		   0);
}

static void airoha_eth_gdm4_link_up_config(struct airoha_eth *eth)
{
	if (!eth->eth_pcs_xfi_mac || !eth->eth_pcs_usxgmii ||
	    !eth->eth_pcs_hsgmii_rate_adp)
		return;

	/*
	 * Match the upstream Airoha PCS speed-change fix for USXGMII/10GBASE-R:
	 * rerun the full PHYA bring-up on link-up instead of only poking TXPCS.
	 * The lighter sequence can leave CDR/FreqDet state stale after retrain.
	 */
	airoha_eth_pcs_pre_config(eth);
	airoha_eth_gdm4_phya_init(eth);
	airoha_eth_pcs_init_usxgmii(eth);
	airoha_eth_pcs_interrupt_init_usxgmii(eth);
	airoha_eth_pcs_post_config(eth);

	airoha_eth_gdm4_sync_rtl8261(eth);
	airoha_eth_gdm4_restart_an(eth);

	airoha_rmw(eth->eth_pcs_xfi_mac, PCS_XFI_GIB_CFG,
		   PCS_XFI_RXMPI_STOP | PCS_XFI_RXMBI_STOP |
			   PCS_XFI_TXMPI_STOP | PCS_XFI_TXMBI_STOP,
		   0);
}

static void airoha_eth_gdm4_cdr_reset(struct airoha_eth *eth)
{
	void __iomem *pma = eth->eth_pcs_xfi_pma;

	if (!pma)
		return;

	/* Force LPF reset low. */
	airoha_rmw(pma, 0x818, BIT(24) | BIT(16) | BIT(8) | BIT(0),
		   BIT(24) | BIT(8));
	udelay(1000);

	/* Bring LPF reset high, then enable lock-to-data, then release force. */
	airoha_rmw(pma, 0x818, 0, BIT(16));
	udelay(200);
	airoha_rmw(pma, 0x818, 0, BIT(0));
	airoha_rmw(pma, 0x818, BIT(24) | BIT(8), 0);
}

static void airoha_eth_gdm4_phya_init(struct airoha_eth *eth)
{
	void __iomem *ana = eth->eth_pcs_xfi_ana;
	void __iomem *pma = eth->eth_pcs_xfi_pma;

	if (!ana || !pma)
		return;

	/* Enable analog common lane. */
	airoha_rmw(ana, 0x0, 0, BIT(0));

	/* JCPLL bring-up for 10.3125G USXGMII. */
	udelay(300);
	airoha_rmw(ana, 0x48, GENMASK(15, 8),
		   FIELD_PREP(GENMASK(15, 8), BIT(5)));
	airoha_rmw(ana, 0x1c, GENMASK(2, 0), FIELD_PREP(GENMASK(2, 0), 0x4));
	airoha_rmw(ana, 0x1c, 0, BIT(8));
	airoha_rmw(pma, 0x828, BIT(24) | BIT(16), BIT(24));
	airoha_rmw(ana, 0x1c, GENMASK(25, 24) | BIT(16), 0);
	airoha_rmw(ana, 0x20,
		   BIT(24) | GENMASK(17, 16) | GENMASK(9, 8) | BIT(0),
		   FIELD_PREP(GENMASK(17, 16), 0x3));
	airoha_rmw(ana, 0x24, BIT(0), 0);
	airoha_rmw(ana, 0x38, GENMASK(31, 16) | GENMASK(15, 0), 0);
	airoha_rmw(ana, 0x34, GENMASK(23, 8) | BIT(0), 0);
	airoha_rmw(ana, 0x30,
		   BIT(17) | BIT(16) | GENMASK(10, 8) | GENMASK(2, 0),
		   FIELD_PREP(GENMASK(10, 8), 0x0) |
			   FIELD_PREP(GENMASK(2, 0), 0x0));
	airoha_rmw(ana, 0x4, GENMASK(29, 24) | GENMASK(21, 16) | BIT(8),
		   FIELD_PREP(GENMASK(29, 24), 0x0) |
			   FIELD_PREP(GENMASK(21, 16), 0x18));
	airoha_rmw(ana, 0x8,
		   GENMASK(28, 24) | GENMASK(20, 16) | GENMASK(12, 8) |
			   GENMASK(4, 0),
		   FIELD_PREP(GENMASK(28, 24), 0x0) |
			   FIELD_PREP(GENMASK(20, 16), 0x10) |
			   FIELD_PREP(GENMASK(12, 8), 0x1f) |
			   FIELD_PREP(GENMASK(4, 0), BIT(3) | BIT(1)));
	airoha_rmw(ana, 0xc, GENMASK(4, 0), 0);
	airoha_rmw(ana, 0x2c, GENMASK(26, 24) | BIT(16) | GENMASK(9, 8),
		   FIELD_PREP(GENMASK(26, 24), 0x4) | BIT(16) |
			   FIELD_PREP(GENMASK(9, 8), 0x1));
	airoha_rmw(ana, 0x30, GENMASK(10, 8) | GENMASK(5, 3) | GENMASK(2, 0),
		   FIELD_PREP(GENMASK(10, 8), 0x0) |
			   FIELD_PREP(GENMASK(5, 3), 0x3) |
			   FIELD_PREP(GENMASK(2, 0), 0x3));
	airoha_rmw(pma, 0x800, GENMASK(30, 0),
		   FIELD_PREP(GENMASK(30, 0), 0x25800000));
	airoha_rmw(pma, 0x79c, 0, BIT(16));
	airoha_rmw(ana, 0x14, BIT(24) | GENMASK(1, 0), 0);
	airoha_rmw(ana, 0x2c, GENMASK(1, 0), 0);
	airoha_rmw(ana, 0x10, GENMASK(17, 16) | GENMASK(9, 8) | GENMASK(1, 0),
		   FIELD_PREP(GENMASK(17, 16), 0x0) |
			   FIELD_PREP(GENMASK(9, 8), 0x3) |
			   FIELD_PREP(GENMASK(1, 0), 0x0));
	airoha_rmw(ana, 0xc, GENMASK(26, 24) | GENMASK(23, 16) | BIT(8),
		   FIELD_PREP(GENMASK(26, 24), 0x2) |
			   FIELD_PREP(GENMASK(23, 16), 0xe4));
	airoha_rmw(ana, 0x48, GENMASK(20, 16),
		   FIELD_PREP(GENMASK(20, 16), 0xf));
	airoha_rmw(ana, 0x24, GENMASK(28, 24) | GENMASK(18, 16) | BIT(8),
		   FIELD_PREP(GENMASK(28, 24), 0x5) |
			   FIELD_PREP(GENMASK(18, 16), 0x1) | BIT(8));
	airoha_rmw(ana, 0x28, GENMASK(26, 24) | BIT(16),
		   FIELD_PREP(GENMASK(26, 24), 0x1) | BIT(16));
	airoha_rmw(pma, 0x828, 0, BIT(16));
	airoha_rmw(pma, 0x828, 0, BIT(8) | BIT(0));

	udelay(300);

	/* TXPLL bring-up for 10G. */
	airoha_rmw(ana, 0x84, GENMASK(25, 24) | GENMASK(17, 16),
		   FIELD_PREP(GENMASK(25, 24), 0x1) |
			   FIELD_PREP(GENMASK(17, 16), 0x1));
	airoha_rmw(ana, 0x64,
		   BIT(24) | GENMASK(18, 16) | GENMASK(9, 8) | BIT(0),
		   BIT(24) | FIELD_PREP(GENMASK(18, 16), 0x4) | BIT(0));
	airoha_rmw(pma, 0x854, BIT(24) | BIT(16), BIT(24));
	airoha_rmw(ana, 0x68,
		   GENMASK(25, 24) | BIT(16) | GENMASK(9, 8) | BIT(0),
		   FIELD_PREP(GENMASK(25, 24), 0x0));
	airoha_rmw(ana, 0x6c, BIT(16) | BIT(8) | GENMASK(1, 0),
		   BIT(16) | FIELD_PREP(GENMASK(1, 0), 0x3));
	airoha_rmw(ana, 0x80, GENMASK(31, 16) | GENMASK(15, 0), 0);
	airoha_rmw(ana, 0x7c, BIT(16) | BIT(8) | BIT(0), 0);
	airoha_rmw(ana, 0x84, GENMASK(15, 0), 0);
	airoha_rmw(ana, 0x50,
		   GENMASK(28, 24) | GENMASK(20, 16) | GENMASK(13, 8) |
			   GENMASK(5, 0),
		   FIELD_PREP(GENMASK(28, 24), 0x1f) |
			   FIELD_PREP(GENMASK(20, 16), 0x5) |
			   FIELD_PREP(GENMASK(13, 8), 0x0) |
			   FIELD_PREP(GENMASK(5, 0), 0xf));
	airoha_rmw(ana, 0x54,
		   BIT(24) | GENMASK(20, 16) | GENMASK(12, 8) | GENMASK(4, 0),
		   FIELD_PREP(GENMASK(20, 16), BIT(4) | BIT(3)) |
			   FIELD_PREP(GENMASK(12, 8),
				      BIT(3) | BIT(1) | BIT(0)) |
			   FIELD_PREP(GENMASK(4, 0), BIT(1)));
	airoha_rmw(ana, 0x74, GENMASK(25, 24),
		   FIELD_PREP(GENMASK(25, 24), BIT(0)));
	airoha_rmw(ana, 0x78,
		   GENMASK(29, 27) | GENMASK(26, 24) | GENMASK(18, 16) |
			   GENMASK(10, 8) | BIT(0),
		   FIELD_PREP(GENMASK(29, 27), 0x0) |
			   FIELD_PREP(GENMASK(26, 24), 0x4) |
			   FIELD_PREP(GENMASK(18, 16), 0x4) |
			   FIELD_PREP(GENMASK(10, 8), 0x7) | BIT(0));
	airoha_rmw(pma, 0x798, GENMASK(30, 0),
		   FIELD_PREP(GENMASK(30, 0), BIT(27) | BIT(22)));
	airoha_rmw(pma, 0x794, 0, BIT(24));
	airoha_rmw(ana, 0x58,
		   GENMASK(25, 24) | GENMASK(17, 16) | GENMASK(10, 8) |
			   GENMASK(7, 0),
		   FIELD_PREP(GENMASK(25, 24), 0x3) |
			   FIELD_PREP(GENMASK(17, 16), 0x0) |
			   FIELD_PREP(GENMASK(10, 8), 0x4) |
			   FIELD_PREP(GENMASK(7, 0), 0xe4));
	airoha_rmw(ana, 0x5c, GENMASK(1, 0), FIELD_PREP(GENMASK(1, 0), 0x1));
	airoha_rmw(ana, 0x54, BIT(24), 0);
	airoha_rmw(ana, 0x5c, GENMASK(17, 16) | BIT(8), 0);
	airoha_rmw(ana, 0x74, GENMASK(17, 16), 0);
	airoha_rmw(ana, 0x94, GENMASK(4, 0), FIELD_PREP(GENMASK(4, 0), 0xf));
	airoha_rmw(ana, 0x70, GENMASK(12, 8) | GENMASK(2, 0),
		   FIELD_PREP(GENMASK(12, 8), BIT(3) | BIT(1) | BIT(0)) |
			   FIELD_PREP(GENMASK(2, 0), 0x3));
	airoha_rmw(ana, 0x74, GENMASK(10, 8) | BIT(0), BIT(0));
	airoha_rmw(ana, 0x6c, 0, BIT(24));
	airoha_rmw(pma, 0x854, 0, BIT(16));
	airoha_rmw(pma, 0x854, 0, BIT(8) | BIT(0));

	udelay(300);

	/* TX path setup. */
	airoha_rmw(pma, 0x580, GENMASK(1, 0),
		   FIELD_PREP(GENMASK(1, 0), BIT(1)));
	airoha_rmw(ana, 0xc4, 0, BIT(24) | BIT(0));
	udelay(1);
	airoha_rmw(pma, 0x874, 0, BIT(24) | BIT(16));
	airoha_rmw(pma, 0x77c,
		   BIT(24) | GENMASK(19, 16) | BIT(8) | GENMASK(2, 0),
		   BIT(24) | FIELD_PREP(GENMASK(19, 16), BIT(2) | BIT(0)) |
			   FIELD_PREP(GENMASK(2, 0), 0x0));
	airoha_rmw(pma, 0x784, BIT(8) | GENMASK(1, 0),
		   BIT(8) | FIELD_PREP(GENMASK(1, 0), BIT(1)));
	airoha_rmw(pma, 0x778,
		   BIT(24) | GENMASK(20, 16) | BIT(8) | GENMASK(5, 0),
		   BIT(24) | FIELD_PREP(GENMASK(20, 16), 0x1) | BIT(8) |
			   FIELD_PREP(GENMASK(5, 0), 0x1));
	airoha_rmw(pma, 0x780,
		   BIT(24) | GENMASK(20, 16) | BIT(8) | GENMASK(5, 0),
		   BIT(8) | FIELD_PREP(GENMASK(5, 0), 0xb));
	airoha_rmw(pma, 0x260, 0, BIT(8) | BIT(0));

	/* RX path setup. */
	airoha_rmw(pma, 0x374, GENMASK(1, 0),
		   FIELD_PREP(GENMASK(1, 0), BIT(1)));
	airoha_rmw(pma, 0x184, GENMASK(26, 16) | GENMASK(10, 0),
		   FIELD_PREP(GENMASK(26, 16), 0x400) |
			   FIELD_PREP(GENMASK(10, 0), 0x3ff));
	airoha_rmw(ana, 0x148, 0, BIT(24) | BIT(16) | BIT(8) | BIT(0));
	airoha_rmw(ana, 0x144, 0, BIT(24));
	airoha_rmw(ana, 0x11c, 0, BIT(0));
	airoha_rmw(pma, 0x4, 0, BIT(0));
	airoha_rmw(ana, 0x13c, GENMASK(19, 8),
		   FIELD_PREP(GENMASK(19, 8), BIT(9)));
	airoha_rmw(ana, 0x120, GENMASK(17, 8),
		   FIELD_PREP(GENMASK(17, 8), 0x3ff));
	airoha_rmw(pma, 0x320, BIT(24), 0);
	airoha_rmw(pma, 0x48c, BIT(0), 0);
	airoha_rmw(ana, 0xdc, BIT(8) | BIT(0), 0);
	airoha_rmw(pma, 0x80c, BIT(24) | BIT(16), BIT(24));
	airoha_rmw(pma, 0x814, BIT(24) | BIT(16), BIT(24) | BIT(16));
	airoha_rmw(ana, 0x10c, GENMASK(28, 24) | BIT(2) | BIT(1) | BIT(0),
		   FIELD_PREP(GENMASK(28, 24), 0x0) | BIT(2));
	airoha_rmw(pma, 0x88c, BIT(8) | GENMASK(1, 0), 0);
	airoha_rmw(pma, 0x768, BIT(24) | GENMASK(19, 16), 0);
	airoha_rmw(pma, 0x390, GENMASK(15, 0), FIELD_PREP(GENMASK(15, 0), 0x1));
	airoha_rmw(pma, 0x394, GENMASK(31, 16) | GENMASK(15, 0),
		   FIELD_PREP(GENMASK(31, 16), 0xffff));
	airoha_rmw(pma, 0x39c, GENMASK(11, 8), FIELD_PREP(GENMASK(11, 8), 0x1));
	airoha_rmw(ana, 0xd4,
		   GENMASK(26, 24) | GENMASK(22, 20) | GENMASK(19, 18),
		   FIELD_PREP(GENMASK(26, 24), BIT(2)) |
			   FIELD_PREP(GENMASK(22, 20), BIT(2)));
	airoha_rmw(pma, 0x100, GENMASK(31, 16) | GENMASK(15, 0),
		   FIELD_PREP(GENMASK(31, 16), 0xa) |
			   FIELD_PREP(GENMASK(15, 0), 0x5));
	airoha_rmw(pma, 0x8c, GENMASK(23, 8) | GENMASK(2, 0),
		   FIELD_PREP(GENMASK(23, 8), 0x1) |
			   FIELD_PREP(GENMASK(2, 0), 0x1));
	airoha_rmw(pma, 0x104, GENMASK(15, 0), FIELD_PREP(GENMASK(15, 0), 0x2));
	airoha_rmw(pma, 0x90, GENMASK(31, 16) | GENMASK(15, 0),
		   FIELD_PREP(GENMASK(31, 16), 0x32) |
			   FIELD_PREP(GENMASK(15, 0), 0x2));
	airoha_rmw(pma, 0x9c, GENMASK(31, 16) | GENMASK(15, 0),
		   FIELD_PREP(GENMASK(31, 16), 0x32) |
			   FIELD_PREP(GENMASK(15, 0), 0x2));
	airoha_rmw(pma, 0x94, GENMASK(31, 16) | GENMASK(15, 0),
		   FIELD_PREP(GENMASK(31, 16), 0x32) |
			   FIELD_PREP(GENMASK(15, 0), 0x2));
	airoha_rmw(pma, 0x98, GENMASK(31, 16) | GENMASK(15, 0),
		   FIELD_PREP(GENMASK(31, 16), 0x32) |
			   FIELD_PREP(GENMASK(15, 0), 0x2));
	airoha_rmw(pma, 0x76c, BIT(24) | GENMASK(17, 16),
		   BIT(24) | FIELD_PREP(GENMASK(17, 16), 0x0));
	airoha_rmw(ana, 0xdc, BIT(8), 0);
	airoha_rmw(ana, 0xe8, GENMASK(26, 8) | GENMASK(1, 0),
		   FIELD_PREP(GENMASK(26, 8), 0x20000) |
			   FIELD_PREP(GENMASK(1, 0), 0x0));
	airoha_rmw(ana, 0xf8,
		   GENMASK(26, 24) | GENMASK(19, 16) | GENMASK(11, 8) |
			   GENMASK(6, 0),
		   FIELD_PREP(GENMASK(26, 24), 0x4) |
			   FIELD_PREP(GENMASK(19, 16), 0x1) |
			   FIELD_PREP(GENMASK(11, 8), 0x8) |
			   FIELD_PREP(GENMASK(6, 0), BIT(3)));
	airoha_rmw(ana, 0xfc,
		   GENMASK(25, 24) | GENMASK(20, 16) | GENMASK(10, 8) |
			   GENMASK(2, 0),
		   FIELD_PREP(GENMASK(25, 24), 0x0) |
			   FIELD_PREP(GENMASK(20, 16), BIT(3)) |
			   FIELD_PREP(GENMASK(10, 8), 0x6) |
			   FIELD_PREP(GENMASK(2, 0), 0x6));
	airoha_rmw(pma, 0x120, GENMASK(17, 16) | GENMASK(12, 8) | GENMASK(1, 0),
		   FIELD_PREP(GENMASK(17, 16), 0x0) |
			   FIELD_PREP(GENMASK(12, 8), 0x1) |
			   FIELD_PREP(GENMASK(1, 0), 0x3));
	airoha_rmw(pma, 0x88, BIT(8) | BIT(0), BIT(0));
	airoha_rmw(pma, 0x38c, GENMASK(1, 0), FIELD_PREP(GENMASK(1, 0), 0x1));
	airoha_rmw(pma, 0x0, 0, BIT(24));
	udelay(600);
	airoha_rmw(pma, 0x33c, BIT(0), 0);
	airoha_rmw(pma, 0x330, 0, BIT(0));
	airoha_rmw(ana, 0x118, BIT(24) | BIT(16) | BIT(8) | BIT(0),
		   BIT(24) | BIT(16) | BIT(8));
	airoha_rmw(ana, 0x10c, BIT(19) | GENMASK(18, 16),
		   FIELD_PREP(GENMASK(18, 16), BIT(2) | BIT(1) | BIT(0)));
	airoha_rmw(pma, 0x824, BIT(24) | BIT(16) | BIT(8) | BIT(0),
		   BIT(24) | BIT(8));
	airoha_rmw(pma, 0x81c, BIT(24) | BIT(16) | BIT(8) | BIT(0), BIT(8));
	airoha_rmw(pma, 0x894, BIT(24) | BIT(16) | BIT(8) | BIT(0), BIT(8));
	airoha_rmw(pma, 0x84c, BIT(24) | BIT(16) | BIT(8) | BIT(0), BIT(24));
	airoha_rmw(pma, 0x34c, 0, BIT(24) | BIT(16) | BIT(8) | BIT(0));
	airoha_rmw(ana, 0x114, GENMASK(20, 16) | GENMASK(9, 8),
		   FIELD_PREP(GENMASK(20, 16), BIT(1)) |
			   FIELD_PREP(GENMASK(9, 8), BIT(1)));
	airoha_rmw(ana, 0x110, GENMASK(25, 24),
		   FIELD_PREP(GENMASK(25, 24), BIT(1) | BIT(0)));
	airoha_rmw(pma, 0x350, BIT(0), 0);
	airoha_rmw(ana, 0xd8, BIT(24) | BIT(16) | GENMASK(9, 8) | GENMASK(7, 0),
		   BIT(16) | FIELD_PREP(GENMASK(9, 8), BIT(1)) |
			   FIELD_PREP(GENMASK(7, 0), BIT(6) | BIT(1)));
	airoha_rmw(ana, 0xcc, BIT(24) | BIT(16), BIT(24));
	udelay(200);
	airoha_rmw(pma, 0x824, 0, BIT(16) | BIT(0));
	airoha_rmw(pma, 0x81c, 0, BIT(0));
	airoha_rmw(pma, 0x894, 0, BIT(0));
	airoha_rmw(pma, 0x84c, 0, BIT(16));
	airoha_rmw(pma, 0x34c, 0, BIT(24) | BIT(16) | BIT(8) | BIT(0));
	airoha_rmw(pma, 0x350, 0, BIT(0));

	udelay(200);
	airoha_eth_gdm4_cdr_reset(eth);

	/* Global reset pulse, as in Linux PHYA bring-up. */
	airoha_rmw(pma, 0x460,
		   BIT(11) | BIT(10) | BIT(9) | BIT(8) | BIT(7) | BIT(6) |
			   BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
		   BIT(5));
	udelay(200);
	airoha_rmw(pma, 0x460, 0,
		   BIT(11) | BIT(10) | BIT(9) | BIT(8) | BIT(7) | BIT(6) |
			   BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0));
	udelay(7000);
	airoha_eth_gdm4_cdr_reset(eth);
}

static void airoha_eth_gdm4_pcs_init(struct airoha_eth *eth)
{
	if (eth->gdm4_pcs_ready || !eth->eth_pcs_xfi_mac ||
	    !eth->eth_pcs_multi_sgmii || !eth->eth_pcs_usxgmii ||
	    !eth->eth_pcs_hsgmii_rate_adp || !eth->eth_pcs_xfi_ana ||
	    !eth->eth_pcs_xfi_pma)
		return;

	airoha_eth_pcs_pre_config(eth);
	airoha_eth_gdm4_phya_init(eth);
	airoha_eth_pcs_init_usxgmii(eth);
	airoha_eth_pcs_interrupt_init_usxgmii(eth);
	airoha_eth_pcs_post_config(eth);
	airoha_eth_gdm4_restart_an(eth);
	eth->gdm4_pcs_ready = true;
}

#define airoha_fe_rr(eth, offset) airoha_rr((eth)->fe_regs, (offset))
#define airoha_fe_wr(eth, offset, val) \
	airoha_wr((eth)->fe_regs, (offset), (val))
#define airoha_fe_rmw(eth, offset, mask, val) \
	airoha_rmw((eth)->fe_regs, (offset), (mask), (val))
#define airoha_fe_set(eth, offset, val) \
	airoha_rmw((eth)->fe_regs, (offset), 0, (val))
#define airoha_fe_clear(eth, offset, val) \
	airoha_rmw((eth)->fe_regs, (offset), (val), 0)

#define airoha_qdma_rr(qdma, offset) airoha_rr((qdma)->regs, (offset))
#define airoha_qdma_wr(qdma, offset, val) \
	airoha_wr((qdma)->regs, (offset), (val))
#define airoha_qdma_rmw(qdma, offset, mask, val) \
	airoha_rmw((qdma)->regs, (offset), (mask), (val))
#define airoha_qdma_set(qdma, offset, val) \
	airoha_rmw((qdma)->regs, (offset), 0, (val))
#define airoha_qdma_clear(qdma, offset, val) \
	airoha_rmw((qdma)->regs, (offset), (val), 0)

#define airoha_switch_wr(eth, offset, val) \
	airoha_wr((eth)->switch_regs, (offset), (val))
#define airoha_switch_rr(eth, offset) airoha_rr((eth)->switch_regs, (offset))
#define airoha_switch_rmw(eth, offset, mask, val) \
	airoha_rmw((eth)->switch_regs, (offset), (mask), (val))

static void airoha_set_gdm_port_fwd_cfg(struct airoha_eth *eth, u32 addr,
					u32 val)
{
	airoha_fe_rmw(eth, addr, GDM_OCFQ_MASK, FIELD_PREP(GDM_OCFQ_MASK, val));
	airoha_fe_rmw(eth, addr, GDM_MCFQ_MASK, FIELD_PREP(GDM_MCFQ_MASK, val));
	airoha_fe_rmw(eth, addr, GDM_BCFQ_MASK, FIELD_PREP(GDM_BCFQ_MASK, val));
	airoha_fe_rmw(eth, addr, GDM_UCFQ_MASK, FIELD_PREP(GDM_UCFQ_MASK, val));
}

static void airoha_fe_crsn_qsel_init(struct airoha_eth *eth)
{
	/*
	 * Linux spreads FE exception traffic across multiple RX rings
	 * (notably ring1/ring6/ring15). This U-Boot port only services a
	 * single RX ring, so collapse every recovery-relevant reason onto
	 * ring0 instead of keeping the upstream multi-ring layout.
	 */
	airoha_fe_rmw(eth, REG_CDM1_CRSN_QSEL(CRSN_22 >> 2),
		      CDM1_CRSN_QSEL_REASON_MASK(CRSN_22),
		      FIELD_PREP(CDM1_CRSN_QSEL_REASON_MASK(CRSN_22),
				 CDM_CRSN_QSEL_Q0));
	airoha_fe_rmw(eth, REG_CDM1_CRSN_QSEL(CRSN_08 >> 2),
		      CDM1_CRSN_QSEL_REASON_MASK(CRSN_08),
		      FIELD_PREP(CDM1_CRSN_QSEL_REASON_MASK(CRSN_08),
				 CDM_CRSN_QSEL_Q0));
	airoha_fe_rmw(eth, REG_CDM1_CRSN_QSEL(CRSN_21 >> 2),
		      CDM1_CRSN_QSEL_REASON_MASK(CRSN_21),
		      FIELD_PREP(CDM1_CRSN_QSEL_REASON_MASK(CRSN_21),
				 CDM_CRSN_QSEL_Q0));
	airoha_fe_rmw(eth, REG_CDM1_CRSN_QSEL(CRSN_24 >> 2),
		      CDM1_CRSN_QSEL_REASON_MASK(CRSN_24),
		      FIELD_PREP(CDM1_CRSN_QSEL_REASON_MASK(CRSN_24),
				 CDM_CRSN_QSEL_Q0));
	airoha_fe_rmw(eth, REG_CDM1_CRSN_QSEL(CRSN_25 >> 2),
		      CDM1_CRSN_QSEL_REASON_MASK(CRSN_25),
		      FIELD_PREP(CDM1_CRSN_QSEL_REASON_MASK(CRSN_25),
				 CDM_CRSN_QSEL_Q0));

	airoha_fe_rmw(eth, REG_CDM2_CRSN_QSEL(CRSN_08 >> 2),
		      CDM2_CRSN_QSEL_REASON_MASK(CRSN_08),
		      FIELD_PREP(CDM2_CRSN_QSEL_REASON_MASK(CRSN_08),
				 CDM_CRSN_QSEL_Q0));
	airoha_fe_rmw(eth, REG_CDM2_CRSN_QSEL(CRSN_21 >> 2),
		      CDM2_CRSN_QSEL_REASON_MASK(CRSN_21),
		      FIELD_PREP(CDM2_CRSN_QSEL_REASON_MASK(CRSN_21),
				 CDM_CRSN_QSEL_Q0));
	airoha_fe_rmw(eth, REG_CDM2_CRSN_QSEL(CRSN_22 >> 2),
		      CDM2_CRSN_QSEL_REASON_MASK(CRSN_22),
		      FIELD_PREP(CDM2_CRSN_QSEL_REASON_MASK(CRSN_22),
				 CDM_CRSN_QSEL_Q0));
	airoha_fe_rmw(eth, REG_CDM2_CRSN_QSEL(CRSN_24 >> 2),
		      CDM2_CRSN_QSEL_REASON_MASK(CRSN_24),
		      FIELD_PREP(CDM2_CRSN_QSEL_REASON_MASK(CRSN_24),
				 CDM_CRSN_QSEL_Q0));
	airoha_fe_rmw(eth, REG_CDM2_CRSN_QSEL(CRSN_25 >> 2),
		      CDM2_CRSN_QSEL_REASON_MASK(CRSN_25),
		      FIELD_PREP(CDM2_CRSN_QSEL_REASON_MASK(CRSN_25),
				 CDM_CRSN_QSEL_Q0));
}

static bool airoha_recovery_port_is_10g(void)
{
	const char *port = env_get("recovery_port");

	return port && !strcmp(port, "10g");
}

static bool airoha_recovery_dual_service_enabled(void)
{
	const char *port = env_get("recovery_port");

	return !port || !*port || !strcmp(port, "auto") ||
	       !strcmp(port, "both") || !strcmp(port, "dual");
}

static bool airoha_recovery_accept_gdm4_rx(void)
{
	return airoha_recovery_port_is_10g() ||
	       airoha_recovery_dual_service_enabled();
}

static bool airoha_eth_is_broadcast_addr(const u8 *addr)
{
	int i;

	for (i = 0; i < ARP_HLEN; i++) {
		if (addr[i] != 0xff)
			return false;
	}

	return true;
}

static bool airoha_eth_is_multicast_addr(const u8 *addr)
{
	return !!(addr[0] & 0x01);
}

static u8 airoha_rx_sport_to_fport(u8 sport)
{
	if (sport == 0x18)
		return 4;

	if (sport == 1 || sport == 2 || sport == 4)
		return sport;

	return 0;
}

static void airoha_peer_fport_learn(struct airoha_eth *eth, const u8 *mac,
				      u8 fport)
{
	int i, slot = -1;

	if (!fport || airoha_eth_is_multicast_addr(mac))
		return;

	for (i = 0; i < AIROHA_PEER_FPORT_CACHE_SIZE; i++) {
		if (!eth->peer_fport[i].valid) {
			if (slot < 0)
				slot = i;
			continue;
		}

		if (!memcmp(eth->peer_fport[i].mac, mac, ARP_HLEN)) {
			slot = i;
			break;
		}
	}

	if (slot < 0) {
		slot = eth->peer_fport_next;
		eth->peer_fport_next =
			(eth->peer_fport_next + 1) % AIROHA_PEER_FPORT_CACHE_SIZE;
	}

	memcpy(eth->peer_fport[slot].mac, mac, ARP_HLEN);
	eth->peer_fport[slot].fport = fport;
	eth->peer_fport[slot].valid = true;
}

static u8 airoha_peer_fport_lookup(struct airoha_eth *eth, const u8 *mac)
{
	int i;

	if (airoha_eth_is_multicast_addr(mac))
		return 0;

	for (i = 0; i < AIROHA_PEER_FPORT_CACHE_SIZE; i++) {
		if (!eth->peer_fport[i].valid)
			continue;

		if (!memcmp(eth->peer_fport[i].mac, mac, ARP_HLEN))
			return eth->peer_fport[i].fport;
	}

	return 0;
}

static bool airoha_rtl8261_host_update_enabled(void)
{
	const char *host_update = env_get("rtl8261_host_update");

	if (!host_update)
		return false;

	if (!strcmp(host_update, "1") || !strcmp(host_update, "on") ||
	    !strcmp(host_update, "enable"))
		return true;

	return false;
}

static int airoha_env_override_bool(const char *name, bool *value)
{
	const char *s = env_get(name);

	if (!s)
		return 0;

	if (!strcmp(s, "1") || !strcmp(s, "on") || !strcmp(s, "enable") ||
	    !strcmp(s, "true")) {
		*value = true;
		return 1;
	}

	if (!strcmp(s, "0") || !strcmp(s, "off") || !strcmp(s, "disable") ||
	    !strcmp(s, "false")) {
		*value = false;
		return 1;
	}

	return 0;
}

static bool airoha_gdm4_use_cdm2(struct airoha_eth *eth)
{
	const char *use_cdm2 = env_get("gdm4_cdm2");

	(void)eth;

	/*
	 * Linux can bind GDM4 to CDM2/QDMA1, but this recovery driver still
	 * only programs the reduced CDM1/QDMA0 RX path end-to-end. Do not
	 * implicitly flip 10G recovery to CDM2 unless explicitly requested,
	 * otherwise TX moves to QDMA1 while RX keeps depending on the CDM1
	 * recovery setup and packets never make it back to the CPU.
	 */
	if (use_cdm2) {
		if (!strcmp(use_cdm2, "0") || !strcmp(use_cdm2, "off") ||
		    !strcmp(use_cdm2, "disable"))
			return false;
		if (!strcmp(use_cdm2, "1") || !strcmp(use_cdm2, "on") ||
		    !strcmp(use_cdm2, "enable"))
			return true;
	}

	/*
	 * Once 10G recovery is using the upstream-like GDM2 loopback path,
	 * keep the FE/QDMA side aligned with that topology and default to the
	 * CDM2/QDMA1 datapath as well.
	 */
	if (airoha_gdm4_loopback_enabled(eth))
		return true;

	return false;
}

static bool airoha_gdm4_loopback_enabled(struct airoha_eth *eth)
{
	const char *loopback = env_get("gdm4_loopback");

	if (loopback) {
		if (!strcmp(loopback, "0") || !strcmp(loopback, "off") ||
		    !strcmp(loopback, "disable"))
			return false;
		if (!strcmp(loopback, "1") || !strcmp(loopback, "on") ||
		    !strcmp(loopback, "enable"))
			return true;
	}

	/*
	 * The full Linux driver enables the GDM2->GDM4 loopback topology for
	 * the external serdes netdev, but U-Boot recovery is still a much
	 * smaller datapath and already receives 10G ingress on the simpler
	 * CDM1/QDMA0 route. Keep loopback disabled by default and require an
	 * explicit gdm4_loopback=1 opt-in for experimentation.
	 */
	return false;
}

static void airoha_gdm4_update_loopback(struct airoha_eth *eth)
{
	if (airoha_gdm4_loopback_enabled(eth)) {
		/*
		 * Mirror Linux GDM4/XSI bring-up on EN7581: enable GDM2
		 * loopback and forward that traffic to GDM4, otherwise the
		 * external serdes port never reaches the FE ingress path.
		 */
		airoha_set_gdm_port_fwd_cfg(eth, REG_GDM_FWD_CFG(2),
					    FE_PSE_PORT_GDM4);
		airoha_fe_clear(eth, REG_GDM_FWD_CFG(2), GDM_STRIP_CRC);
		airoha_fe_wr(eth, REG_GDM_TXCHN_EN(2), 0xffffffff);
		airoha_fe_wr(eth, REG_GDM_RXCHN_EN(2), 0xffff);
		airoha_fe_rmw(eth, REG_GDM_LPBK_CFG(2),
			      LPBK_CHAN_MASK | LPBK_MODE_MASK |
				      LBK_GAP_MODE_MASK | LBK_LEN_MODE_MASK |
				      LBK_CHAN_MODE_MASK | LPBK_EN_MASK,
			      FIELD_PREP(LPBK_CHAN_MASK, 0) |
				      LBK_GAP_MODE_MASK | LBK_LEN_MODE_MASK |
				      LBK_CHAN_MODE_MASK | LPBK_EN_MASK);
		airoha_fe_rmw(eth, REG_GDM_LEN_CFG(2),
			      GDM_SHORT_LEN_MASK | GDM_LONG_LEN_MASK,
			      FIELD_PREP(GDM_SHORT_LEN_MASK, 60) |
				      FIELD_PREP(GDM_LONG_LEN_MASK,
						 AIROHA_MAX_PACKET_SIZE));
		airoha_fe_clear(eth, REG_FE_VIP_PORT_EN, BIT(2));
		airoha_fe_clear(eth, REG_FE_IFC_PORT_EN, BIT(2));
	} else {
		airoha_fe_wr(eth, REG_GDM_TXCHN_EN(2), 0);
		airoha_fe_wr(eth, REG_GDM_RXCHN_EN(2), 0);
		airoha_fe_clear(eth, REG_GDM_LPBK_CFG(2),
				LPBK_CHAN_MASK | LPBK_MODE_MASK |
					LBK_GAP_MODE_MASK | LBK_LEN_MODE_MASK |
					LBK_CHAN_MODE_MASK | LPBK_EN_MASK);
		airoha_set_gdm_port_fwd_cfg(eth, REG_GDM_FWD_CFG(2),
					    FE_PSE_PORT_PPE1);
	}
}

static void airoha_gdm4_update_cpu_path(struct airoha_eth *eth)
{
	u32 src_port = HSGMII_LAN_7581_ETH_SRCPORT;
	u32 src_port_slot = src_port & SP_CPORT_DFT_MASK;
	u32 src_port_mask = SP_CPORT_MASK(src_port_slot);
	bool use_cdm2 = airoha_gdm4_use_cdm2(eth);
	u32 sp_cport = use_cdm2 ? FE_PSE_PORT_CDM2 : FE_PSE_PORT_CDM1;
	bool loopback = airoha_gdm4_loopback_enabled(eth);

	airoha_gdm4_update_loopback(eth);

	if (loopback) {
		/*
		 * Follow the upstream EN7581 GDM4 external-serdes topology:
		 * GDM2 loopback feeds GDM4, GDM4 ingress still goes through PPE1
		 * to the active CPU port, while source-port 24 is steered to
		 * CDM2 for the outbound serdes path.
		 *
		 * GDM4 is FE/PSE port 9, so its PPE default CPU port entry lives
		 * in the second default-cport register rather than alongside the
		 * legacy logical port ids.
		 */
		airoha_set_gdm_port_fwd_cfg(eth, REG_GDM_FWD_CFG(4),
					    FE_PSE_PORT_PPE1);
		airoha_fe_rmw(eth,
			      REG_PPE_DFT_CPORT(0, FE_PSE_PORT_GDM4),
			      DFT_CPORT_MASK(FE_PSE_PORT_GDM4),
			      FIELD_PREP(DFT_CPORT_MASK(FE_PSE_PORT_GDM4),
					 sp_cport));
		airoha_fe_rmw(eth,
			      REG_PPE_DFT_CPORT(1, FE_PSE_PORT_GDM4),
			      DFT_CPORT_MASK(FE_PSE_PORT_GDM4),
			      FIELD_PREP(DFT_CPORT_MASK(FE_PSE_PORT_GDM4),
					 sp_cport));
		airoha_fe_rmw(eth, REG_FE_WAN_PORT,
			      WAN1_EN_MASK | WAN1_MASK | WAN0_MASK,
			      FIELD_PREP(WAN0_MASK, src_port));
		airoha_fe_rmw(eth, REG_SP_DFT_CPORT(src_port >> 3), src_port_mask,
			      FE_PSE_PORT_CDM2 << (src_port_slot << 2));
		airoha_fe_rmw(eth, REG_SRC_PORT_FC_MAP6,
			      FC_ID_OF_SRC_PORT24_MASK,
			      FIELD_PREP(FC_ID_OF_SRC_PORT24_MASK, 2));
		return;
	}

	/*
	 * Bypass PPE entirely for recovery: route GDM4 RX straight to the
	 * CDM port that feeds the active QDMA instance.  This eliminates any
	 * PPE back-pressure or mis-routing that can stall gdm4_rx counters.
	 * PPE DFT_CPORT is still programmed for completeness.
	 */
	airoha_set_gdm_port_fwd_cfg(eth, REG_GDM_FWD_CFG(4), sp_cport);
	airoha_fe_rmw(eth, REG_PPE_DFT_CPORT(0, FE_PSE_PORT_GDM4),
		      DFT_CPORT_MASK(FE_PSE_PORT_GDM4),
		      FIELD_PREP(DFT_CPORT_MASK(FE_PSE_PORT_GDM4), sp_cport));
	airoha_fe_rmw(eth, REG_PPE_DFT_CPORT(1, FE_PSE_PORT_GDM4),
		      DFT_CPORT_MASK(FE_PSE_PORT_GDM4),
		      FIELD_PREP(DFT_CPORT_MASK(FE_PSE_PORT_GDM4), sp_cport));
	airoha_fe_rmw(eth, REG_SP_DFT_CPORT(src_port >> 3), src_port_mask,
		      sp_cport << (src_port_slot << 2));
}

static void airoha_set_xsi_eth_port(struct airoha_eth *eth, bool enable)
{
	if (enable) {
		airoha_fe_set(eth, REG_FE_VIP_PORT_EN, XSI_ETH_VIP_PORT_MASK);
		airoha_fe_set(eth, REG_FE_IFC_PORT_EN, XSI_ETH_VIP_PORT_MASK);
		airoha_fe_set(eth, REG_GDM_INGRESS_CFG(4), GDM_STAG_EN_MASK);
	} else {
		airoha_fe_clear(eth, REG_FE_VIP_PORT_EN, XSI_ETH_VIP_PORT_MASK);
		airoha_fe_clear(eth, REG_FE_IFC_PORT_EN, XSI_ETH_VIP_PORT_MASK);
		airoha_fe_clear(eth, REG_GDM_INGRESS_CFG(4), GDM_STAG_EN_MASK);
	}
}

static inline dma_addr_t dma_map_unaligned(void *vaddr, size_t len,
					   enum dma_data_direction dir)
{
	uintptr_t start, end;

	start = ALIGN_DOWN((uintptr_t)vaddr, ARCH_DMA_MINALIGN);
	end = ALIGN((uintptr_t)(vaddr + len), ARCH_DMA_MINALIGN);

	return dma_map_single((void *)start, end - start, dir);
}

static inline void dma_unmap_unaligned(dma_addr_t addr, size_t len,
				       enum dma_data_direction dir)
{
	uintptr_t start, end;

	start = ALIGN_DOWN((uintptr_t)addr, ARCH_DMA_MINALIGN);
	end = ALIGN((uintptr_t)(addr + len), ARCH_DMA_MINALIGN);
	dma_unmap_single(start, end - start, dir);
}

static void airoha_fe_maccr_init(struct airoha_eth *eth)
{
	int p;

	for (p = 1; p <= AIROHA_MAX_NUM_GDM_PORTS; p++) {
		u32 val = GDM_PAD_EN;

		/*
		 * Disable any kind of CRC drop or offload.
		 * Enable padding of short TX packets to 60 bytes. Match the
		 * Linux GDM3/4 bring-up and strip CRC on RX for the external
		 * ports so packets are accepted by the reduced recovery path.
		 */
		if (p >= 3)
			val |= GDM_STRIP_CRC;

		airoha_fe_wr(eth, REG_GDM_FWD_CFG(p), val);
	}

	airoha_fe_rmw(eth, REG_CDM1_VLAN_CTRL, CDM1_VLAN_MASK,
		      FIELD_PREP(CDM1_VLAN_MASK, 0x8100));
	airoha_fe_set(eth, REG_FE_CPORT_CFG, FE_CPORT_PAD);
}

static int airoha_mdio_read_lstatus(struct airoha_eth *eth, int addr, int devad,
				    int reg)
{
	struct udevice *mdio_dev;
	int val;

	mdio_dev = airoha_rtl8261_is_phy_addr(addr) ? eth->mdio_dev :
		   (eth->switch_mdio_dev ? eth->switch_mdio_dev : eth->mdio_dev);
	if (!mdio_dev)
		return -ENODEV;

	/*
	 * Airoha MDIO controllers can retain stale read data across failed /
	 * unsupported transactions. Flush the bus state with a harmless write
	 * to an unused/read-only register before each real read so subsequent
	 * Clause 45 TOP/VEND1 accesses do not inherit an old value.
	 */
	if (!ofnode_device_is_compatible(dev_ofnode(mdio_dev),
					 "airoha,arht-mdio"))
		dm_mdio_write(mdio_dev, 0, MDIO_DEVAD_NONE, MII_BMSR, 0x1);
	val = dm_mdio_read(mdio_dev, addr, devad, reg);
	if (val < 0)
		return val;

	/* Link state is latched; a second read returns the current state. */
	if (!ofnode_device_is_compatible(dev_ofnode(mdio_dev),
					 "airoha,arht-mdio"))
		dm_mdio_write(mdio_dev, 0, MDIO_DEVAD_NONE, MII_BMSR, 0x1);
	return dm_mdio_read(mdio_dev, addr, devad, reg);
}

static int airoha_mdio_c45_read(struct airoha_eth *eth, int addr, int devad,
				int reg)
{
	struct udevice *mdio_dev;

	mdio_dev = airoha_rtl8261_is_phy_addr(addr) ? eth->mdio_dev :
		   (eth->switch_mdio_dev ? eth->switch_mdio_dev : eth->mdio_dev);
	if (!mdio_dev)
		return -ENODEV;

	if (!ofnode_device_is_compatible(dev_ofnode(mdio_dev),
					 "airoha,arht-mdio"))
		dm_mdio_write(mdio_dev, 0, MDIO_DEVAD_NONE, MII_BMSR, 0x1);
	return dm_mdio_read(mdio_dev, addr, devad, reg);
}

static int airoha_mdio_c45_write(struct airoha_eth *eth, int addr, int devad,
				 int reg, u16 val)
{
	struct udevice *mdio_dev;

	mdio_dev = airoha_rtl8261_is_phy_addr(addr) ? eth->mdio_dev :
		   (eth->switch_mdio_dev ? eth->switch_mdio_dev : eth->mdio_dev);
	if (!mdio_dev)
		return -ENODEV;

	return dm_mdio_write(mdio_dev, addr, devad, reg, val);
}

static int airoha_mdio_c22_write(struct airoha_eth *eth, int addr, int reg,
				 u16 val)
{
	struct udevice *mdio_dev;

	mdio_dev = airoha_rtl8261_is_phy_addr(addr) ? eth->mdio_dev :
		   (eth->switch_mdio_dev ? eth->switch_mdio_dev : eth->mdio_dev);
	if (!mdio_dev)
		return -ENODEV;

	return dm_mdio_write(mdio_dev, addr, MDIO_DEVAD_NONE, reg, val);
}

static int airoha_mdio_c22_read(struct airoha_eth *eth, int addr, int reg)
{
	struct udevice *mdio_dev;

	mdio_dev = airoha_rtl8261_is_phy_addr(addr) ? eth->mdio_dev :
		   (eth->switch_mdio_dev ? eth->switch_mdio_dev : eth->mdio_dev);
	if (!mdio_dev)
		return -ENODEV;

	if (!ofnode_device_is_compatible(dev_ofnode(mdio_dev),
					 "airoha,arht-mdio"))
		dm_mdio_write(mdio_dev, 0, MDIO_DEVAD_NONE, MII_BMSR, 0x1);
	return dm_mdio_read(mdio_dev, addr, MDIO_DEVAD_NONE, reg);
}

static int airoha_mdio_c22_mmd_read(struct airoha_eth *eth, int addr, int devad,
				    int reg)
{
	int ret;

	ret = airoha_mdio_c22_write(eth, addr, MII_MMD_CTRL,
				    devad & MII_MMD_CTRL_DEVAD_MASK);
	if (ret)
		return ret;

	ret = airoha_mdio_c22_write(eth, addr, MII_MMD_DATA, reg);
	if (ret)
		return ret;

	ret = airoha_mdio_c22_write(eth, addr, MII_MMD_CTRL,
				    (devad & MII_MMD_CTRL_DEVAD_MASK) |
					    MII_MMD_CTRL_NOINCR);
	if (ret)
		return ret;

	return airoha_mdio_c22_read(eth, addr, MII_MMD_DATA);
}

static int airoha_mdio_c22_mmd_write(struct airoha_eth *eth, int addr, int devad,
				     int reg, u16 val)
{
	int ret;

	ret = airoha_mdio_c22_write(eth, addr, MII_MMD_CTRL,
				    devad & MII_MMD_CTRL_DEVAD_MASK);
	if (ret)
		return ret;

	ret = airoha_mdio_c22_write(eth, addr, MII_MMD_DATA, reg);
	if (ret)
		return ret;

	ret = airoha_mdio_c22_write(eth, addr, MII_MMD_CTRL,
				    (devad & MII_MMD_CTRL_DEVAD_MASK) |
					    MII_MMD_CTRL_NOINCR);
	if (ret)
		return ret;

	return airoha_mdio_c22_write(eth, addr, MII_MMD_DATA, val);
}

static void airoha_eth_gdm4_set_speed_mode(struct airoha_eth *eth, u32 mode,
					   u32 rate_adapt)
{
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_AN_CONTROL_7,
		   PCS_USXGMII_RATE_UPDATE_MODE | PCS_USXGMII_MODE,
		   PCS_USXGMII_RATE_UPDATE_MODE | mode);
	airoha_rmw(eth->eth_pcs_hsgmii_rate_adp, PCS_HSGMII_RATE_ADAPT_CTRL_11,
		   PCS_HSGMII_FORCE_RATE_MODE_EN | PCS_HSGMII_FORCE_RATE_MODE,
		   PCS_HSGMII_FORCE_RATE_MODE_EN | rate_adapt);
}

static void airoha_eth_gdm4_set_frag_size(struct airoha_eth *eth, u16 speed)
{
	u32 frag_size_tx, frag_size_rx;

	if (!eth->fe_regs)
		return;

	switch (speed & RTL8261_PHY_SPEED_MASK) {
	case RTL8261_PHY_SPEED_10000M:
	case RTL8261_PHY_SPEED_5000M:
		frag_size_tx = 8;
		frag_size_rx = 8;
		break;
	case RTL8261_PHY_SPEED_2500M:
		frag_size_tx = 2;
		frag_size_rx = 1;
		break;
	default:
		frag_size_tx = 1;
		frag_size_rx = 0;
		break;
	}

	/*
	 * Upstream phylink programs the FE-side GDM4 fragment settings from the
	 * negotiated copper speed. Keep U-Boot aligned so the 10G recovery
	 * datapath does not rely on reset defaults for 2.5G/5G/10G operation.
	 */
	airoha_fe_rmw(eth, REG_GDMA4_TMBI_FRAG, GDMA4_SGMII0_TX_FRAG_SIZE,
		      FIELD_PREP(GDMA4_SGMII0_TX_FRAG_SIZE, frag_size_tx));
	airoha_fe_rmw(eth, REG_GDMA4_RMBI_FRAG, GDMA4_SGMII0_RX_FRAG_SIZE,
		      FIELD_PREP(GDMA4_SGMII0_RX_FRAG_SIZE, frag_size_rx));
}

static void airoha_eth_gdm4_select_speed_mode(struct airoha_eth *eth)
{
	/*
	 * XR1710G uses RTL8261 host-side USXGMII with in-band autonegotiation.
	 * The Linux PCS driver does not program forced USXGMII mode/rate bits
	 * when PHYLINK_PCS_NEG_INBAND_ENABLED is used. Keep the serial lane in
	 * plain USXGMII with AN enabled and clear any forced-rate override
	 * instead of rewriting the mode field from the negotiated copper speed.
	 */
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_AN_CONTROL_7,
		   PCS_USXGMII_RATE_UPDATE_MODE | PCS_USXGMII_MODE,
		   0);
	airoha_rmw(eth->eth_pcs_hsgmii_rate_adp, PCS_HSGMII_RATE_ADAPT_CTRL_11,
		   PCS_HSGMII_FORCE_RATE_MODE_EN | PCS_HSGMII_FORCE_RATE_MODE,
		   0);
}

static int airoha_rtl8261_sds_wait_ready(struct airoha_eth *eth, int phy_addr)
{
	int i, val;

	for (i = 0; i < 10; i++) {
		val = airoha_mdio_c45_read(eth, phy_addr, RTL8261_MMD_VEND1,
					   RTL8261_PHY_SDS_CMD);
		if (val < 0)
			return val;
		if (!(val & RTL8261_PHY_SDS_BUSY))
			return 0;
		udelay(10);
	}

	return -ETIMEDOUT;
}

static u16 airoha_rtl8261_mask(u8 msb, u8 lsb)
{
	return GENMASK(msb, lsb);
}

static u16 airoha_rtl8261_field_set(u16 orig, u8 msb, u8 lsb, u16 data)
{
	u16 mask = airoha_rtl8261_mask(msb, lsb);
	u16 shift = __builtin_ctz(mask);

	return (orig & ~mask) | ((data << shift) & mask);
}

static u16 airoha_rtl8261_phy_reg_convert(u16 page, u16 addr)
{
	if (addr < 16)
		return 0xa400 + (page * 2);
	if (addr < 24)
		return (16 * page) + ((addr - 16) * 2);

	return 0xa430 + ((addr - 24) * 2);
}

static int airoha_rtl8261_top_get(struct airoha_eth *eth, int phy_addr,
				  u16 top_page, u16 top_reg, u16 *val)
{
	u16 top_addr = (top_page * 8) + (top_reg - 16);
	int ret;

	ret = airoha_mdio_c45_read(eth, phy_addr, RTL8261_MMD_VEND1, top_addr);
	if (ret < 0)
		return ret;

	*val = ret;

	return 0;
}

static int airoha_rtl8261_top_set(struct airoha_eth *eth, int phy_addr,
				  u16 top_page, u16 top_reg, u16 val)
{
	u16 top_addr = (top_page * 8) + (top_reg - 16);

	return airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1, top_addr,
				     val);
}

static int airoha_rtl8261_sds_get(struct airoha_eth *eth, int phy_addr,
				  u16 sds_page, u16 sds_reg, u16 *val)
{
	int ret;

	ret = airoha_rtl8261_top_set(eth, phy_addr, 40, 19,
				     RTL8261_PHY_SDS_CMD_READ(sds_page,
							      sds_reg));
	if (ret)
		return ret;

	ret = airoha_rtl8261_top_get(eth, phy_addr, 40, 18, val);
	if (ret)
		return ret;

	return airoha_rtl8261_sds_wait_ready(eth, phy_addr);
}

static int airoha_rtl8261_sds_write(struct airoha_eth *eth, int phy_addr,
				    u16 page, u16 reg, u16 val)
{
	int ret;

	ret = airoha_rtl8261_top_set(eth, phy_addr, 40, 17, val);
	if (ret)
		return ret;

	ret = airoha_rtl8261_top_set(eth, phy_addr, 40, 19,
				     RTL8261_PHY_SDS_CMD_WRITE(page, reg));
	if (ret)
		return ret;

	return airoha_rtl8261_sds_wait_ready(eth, phy_addr);
}

static int airoha_rtl8261_wait(struct airoha_eth *eth, int phy_addr, int mmd,
			       u16 reg, u16 data, u16 mask, bool equal)
{
	int i, val;

	for (i = 0; i < 1000; i++) {
		val = airoha_mdio_c45_read(eth, phy_addr, mmd, reg);
		if (val < 0)
			return val;
		if (equal ? ((val & mask) == data) : ((val & mask) != data))
			return 0;
		mdelay(1);
	}

	return -ETIMEDOUT;
}

static int airoha_rtl8261_apply_patch(struct airoha_eth *eth, int phy_addr,
				      const rtk_hwpatch_t *patch)
{
	u16 reg, cur, val;
	int ret;

	if (!(patch->portmask & BIT(0)))
		return 0;

	switch (patch->patch_op) {
	case RTK_PATCH_OP_TOP:
		if (patch->msb == 15 && patch->lsb == 0)
			return airoha_rtl8261_top_set(eth, phy_addr,
						      patch->pagemmd,
						      patch->addr, patch->data);

		ret = airoha_rtl8261_top_get(eth, phy_addr, patch->pagemmd,
					     patch->addr, &cur);
		if (ret)
			return ret;
		return airoha_rtl8261_top_set(
			eth, phy_addr, patch->pagemmd, patch->addr,
			airoha_rtl8261_field_set(cur, patch->msb, patch->lsb,
						 patch->data));

	case RTK_PATCH_OP_PSDS0:
		if (patch->msb == 15 && patch->lsb == 0)
			return airoha_rtl8261_sds_write(eth, phy_addr,
							patch->pagemmd,
							patch->addr,
							patch->data);

		ret = airoha_rtl8261_sds_get(eth, phy_addr, patch->pagemmd,
					     patch->addr, &cur);
		if (ret)
			return ret;
		return airoha_rtl8261_sds_write(
			eth, phy_addr, patch->pagemmd, patch->addr,
			airoha_rtl8261_field_set(cur, patch->msb, patch->lsb,
						 patch->data));

	case RTK_PATCH_OP_PHYOCP:
		if (patch->msb == 15 && patch->lsb == 0)
			return airoha_mdio_c45_write(eth, phy_addr,
						     RTL8261_MMD_VEND2,
						     patch->addr, patch->data);

		ret = airoha_mdio_c45_read(eth, phy_addr, RTL8261_MMD_VEND2,
					   patch->addr);
		if (ret < 0)
			return ret;
		val = airoha_rtl8261_field_set(ret, patch->msb, patch->lsb,
					       patch->data);
		return airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND2,
					     patch->addr, val);

	case RTK_PATCH_OP_DELAY_MS:
		mdelay(patch->data);
		return 0;

	default:
		reg = airoha_rtl8261_phy_reg_convert(patch->pagemmd,
						     patch->addr);
		if (patch->patch_op != RTK_PATCH_OP_PHY)
			return -EOPNOTSUPP;
		if (patch->msb == 15 && patch->lsb == 0)
			return airoha_mdio_c45_write(eth, phy_addr,
						     RTL8261_MMD_VEND2, reg,
						     patch->data);

		ret = airoha_mdio_c45_read(eth, phy_addr, RTL8261_MMD_VEND2,
					   reg);
		if (ret < 0)
			return ret;
		val = airoha_rtl8261_field_set(ret, patch->msb, patch->lsb,
					       patch->data);
		return airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND2,
					     reg, val);
	}
}

static int airoha_rtl8261_phy_modify(struct airoha_eth *eth, int phy_addr,
				     u16 page, u16 addr, u8 msb, u8 lsb,
				     u16 data)
{
	u16 reg = airoha_rtl8261_phy_reg_convert(page, addr);
	int ret;

	ret = airoha_mdio_c45_read(eth, phy_addr, RTL8261_MMD_VEND2, reg);
	if (ret < 0)
		return ret;

	return airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND2, reg,
				     airoha_rtl8261_field_set(ret, msb, lsb,
							      data));
}

static int airoha_rtl8261_vend1_modify(struct airoha_eth *eth, int phy_addr,
				       u16 reg, u16 mask, u16 val)
{
	int ret;

	ret = airoha_mdio_c45_read(eth, phy_addr, RTL8261_MMD_VEND1, reg);
	if (ret < 0)
		return ret;

	return airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1, reg,
				     (ret & ~mask) | (val & mask));
}

static int airoha_rtl8261_speed_status_get(struct airoha_eth *eth, int phy_addr,
					   u16 *status)
{
	int ret;

	ret = airoha_mdio_c45_read(eth, phy_addr, RTL8261_MMD_VEND2,
				   RTL8261_PHY_SPEED_STATUS);
	if (ret < 0)
		return ret;

	*status = ret;

	return 0;
}

static const char *airoha_rtl8261_speed_name(u16 speed)
{
	switch (speed & RTL8261_PHY_SPEED_MASK) {
	case RTL8261_PHY_SPEED_10M:
		return "10M";
	case RTL8261_PHY_SPEED_100M:
		return "100M";
	case RTL8261_PHY_SPEED_1000M:
		return "1000M";
	case RTL8261_PHY_SPEED_2500M:
		return "2500M";
	case RTL8261_PHY_SPEED_5000M:
		return "5000M";
	case RTL8261_PHY_SPEED_10000M:
		return "10000M";
	default:
		return "unknown";
	}
}

static const char *airoha_usxgmii_mode_name(u32 mode)
{
	if (mode == PCS_USXGMII_MODE_10000)
		return "10000";
	if (mode == PCS_USXGMII_MODE_5000)
		return "5000";
	if (mode == PCS_USXGMII_MODE_2500)
		return "2500";
	if (mode == PCS_USXGMII_MODE_1000)
		return "1000";
	if (mode == PCS_USXGMII_MODE_100)
		return "100";

	return "unknown";
}

static const char *airoha_rate_mode_name(u32 rate_adapt)
{
	if (rate_adapt == PCS_HSGMII_FORCE_RATE_MODE_10000)
		return "10000";
	if (rate_adapt == PCS_HSGMII_FORCE_RATE_MODE_5000)
		return "5000";
	if (rate_adapt == PCS_HSGMII_FORCE_RATE_MODE_2500)
		return "2500";
	if (rate_adapt == PCS_HSGMII_FORCE_RATE_MODE_1000)
		return "1000";
	if (rate_adapt == PCS_HSGMII_FORCE_RATE_MODE_100)
		return "100";

	return "unknown";
}

static bool airoha_rtl8261_is_phy_addr(int phy_addr)
{
	return phy_addr == RTL8261_PHY5_ADDR || phy_addr == RTL8261_PHY8_ADDR;
}

static int airoha_rtl8261_link_state_get(struct airoha_eth *eth, int phy_addr,
					 bool *link_up, bool *full_duplex,
					 u16 *speed_status, u32 *mode,
					 u32 *rate_adapt)
{
	u16 speed = RTL8261_PHY_SPEED_10000M;
	int pma_stat1, pcs_stat1, an_stat1, phyxs_stat1;
	bool duplex = true;
	bool link = false;
	int stat;

	if (!eth->mdio_dev || !airoha_rtl8261_is_phy_addr(phy_addr))
		return -ENODEV;

	stat = airoha_rtl8261_speed_status_get(eth, phy_addr, &speed);
	if (stat)
		return stat;

	pma_stat1 = airoha_mdio_read_lstatus(eth, phy_addr, MDIO_MMD_PMAPMD,
					     RTL8261_PMA_STATUS1);
	pcs_stat1 = airoha_mdio_read_lstatus(eth, phy_addr, MDIO_MMD_PCS,
					     RTL8261_PCS_STATUS1);
	an_stat1 = airoha_mdio_read_lstatus(eth, phy_addr, MDIO_MMD_AN,
					    RTL8261_AN_STATUS1);
	phyxs_stat1 = airoha_mdio_read_lstatus(eth, phy_addr, MDIO_MMD_PHYXS,
					       MDIO_STAT1);

	link = (pma_stat1 >= 0 && (pma_stat1 & MDIO_STAT1_LSTATUS)) ||
	       (pcs_stat1 >= 0 && (pcs_stat1 & MDIO_STAT1_LSTATUS)) ||
	       (an_stat1 >= 0 && (an_stat1 & MDIO_STAT1_LSTATUS)) ||
	       (phyxs_stat1 >= 0 && (phyxs_stat1 & MDIO_STAT1_LSTATUS));

	duplex = !!(speed & BIT(3));

	if (link_up)
		*link_up = link;
	if (full_duplex)
		*full_duplex = duplex;
	if (speed_status)
		*speed_status = speed;
	if (mode)
		*mode = PCS_USXGMII_MODE_10000;
	if (rate_adapt)
		*rate_adapt = PCS_HSGMII_FORCE_RATE_MODE_10000;

	switch (speed & RTL8261_PHY_SPEED_MASK) {
	case RTL8261_PHY_SPEED_5000M:
		if (mode)
			*mode = PCS_USXGMII_MODE_5000;
		if (rate_adapt)
			*rate_adapt = PCS_HSGMII_FORCE_RATE_MODE_5000;
		break;
	case RTL8261_PHY_SPEED_2500M:
		if (mode)
			*mode = PCS_USXGMII_MODE_2500;
		if (rate_adapt)
			*rate_adapt = PCS_HSGMII_FORCE_RATE_MODE_2500;
		break;
	case RTL8261_PHY_SPEED_1000M:
		if (mode)
			*mode = PCS_USXGMII_MODE_1000;
		if (rate_adapt)
			*rate_adapt = PCS_HSGMII_FORCE_RATE_MODE_1000;
		break;
	case RTL8261_PHY_SPEED_100M:
		if (mode)
			*mode = PCS_USXGMII_MODE_100;
		if (rate_adapt)
			*rate_adapt = PCS_HSGMII_FORCE_RATE_MODE_100;
		break;
	default:
		break;
	}

	return 0;
}

static int airoha_rtl8261_serdes_mode_usxgmii_set(struct airoha_eth *eth,
						  int phy_addr)
{
	int ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x0105, BIT(0), 0x0);
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x00c2, GENMASK(9, 0),
					  0x000d);
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x03f1, GENMASK(15, 0),
					  0x0072);
	if (ret)
		return ret;

	return airoha_rtl8261_vend1_modify(eth, phy_addr, 0x02a2, BIT(7), 0x0);
}

static int airoha_rtl8261_serdes_mode_10gr_set(struct airoha_eth *eth,
					       int phy_addr)
{
	int ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x0105, BIT(0), BIT(0));
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x00c6, GENMASK(4, 0),
					  0x001a);
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x00c2, GENMASK(9, 5),
					  0x0000);
	if (ret)
		return ret;

	return airoha_rtl8261_vend1_modify(eth, phy_addr, 0x02a2, BIT(7), 0x0);
}

static int airoha_rtl8261_serdes_mode_5gr_set(struct airoha_eth *eth,
					      int phy_addr)
{
	int ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x0105, BIT(0), BIT(0));
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x00c5, GENMASK(5, 0),
					  0x001a);
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x00c2, GENMASK(9, 5),
					  0x0040);
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x03f1, GENMASK(15, 0),
					  0x0072);
	if (ret)
		return ret;

	return airoha_rtl8261_vend1_modify(eth, phy_addr, 0x02a2, BIT(7), 0x0);
}

static int airoha_rtl8261_serdes_mode_2p5gx_set(struct airoha_eth *eth,
						int phy_addr)
{
	int ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x0105, BIT(0), BIT(0));
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x00c4, GENMASK(5, 0),
					  0x0036);
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x00c2, GENMASK(9, 5),
					  0x0000);
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x03f1, GENMASK(15, 0),
					  0x0072);
	if (ret)
		return ret;

	return airoha_rtl8261_vend1_modify(eth, phy_addr, 0x02a2, BIT(7), 0x0);
}

static int airoha_rtl8261_serdes_mode_sgmii_set(struct airoha_eth *eth,
						int phy_addr)
{
	int ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x0105, BIT(0), BIT(0));
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x00c3, GENMASK(3, 0),
					  0x0002);
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x00c2, GENMASK(9, 5),
					  0x0000);
	if (ret)
		return ret;

	return airoha_rtl8261_vend1_modify(eth, phy_addr, 0x02a2, BIT(7), 0x0);
}

static int airoha_rtl8261_serdes_mode_update(struct airoha_eth *eth,
					     int phy_addr)
{
	int ret;

	if (!airoha_rtl8261_host_update_enabled())
		return 0;

	/*
	 * On XR1710G the RTL8261 host-side link is wired to the AN7581 USXGMII
	 * MAC, not used as a media converter. Keep the PHY SDS in USXGMII mode
	 * regardless of the negotiated copper-side speed, and keep the
	 * host-side option/autoneg state in sync with that choice.
	 *
	 * Default this sequence off in U-Boot recovery. The vendor RTL8261
	 * loader only sets the board SDS mode and clears AN disable, and the
	 * extra host-side forcing done here is currently the most suspicious
	 * remaining divergence in the transmit path.
	 */
	ret = airoha_rtl8261_serdes_mode_usxgmii_set(eth, phy_addr);
	if (ret)
		return ret;

	ret = airoha_rtl8261_serdes_option_set(eth, phy_addr,
					       RTL8261_SERDES_SPEED_USXGMII);
	if (ret)
		return ret;

	return airoha_rtl8261_serdes_autoneg_set(eth, phy_addr, true);
}

static int airoha_rtl8261_restart_aneg(struct airoha_eth *eth, int phy_addr)
{
	int ctrl;

	ctrl = airoha_mdio_c45_read(eth, phy_addr, MDIO_MMD_AN, MDIO_CTRL1);
	if (ctrl < 0)
		return ctrl;

	ctrl |= MDIO_AN_CTRL1_ENABLE | MDIO_AN_CTRL1_RESTART;

	return airoha_mdio_c45_write(eth, phy_addr, MDIO_MMD_AN, MDIO_CTRL1,
				     ctrl);
}

static void airoha_eth_gdm4_sync_rtl8261(struct airoha_eth *eth)
{
	u16 speed = RTL8261_PHY_SPEED_10000M;
	int ret;

	if (!eth->rtl8261_init_done || !eth->mdio_dev)
		return;

	ret = airoha_rtl8261_serdes_mode_update(eth, RTL8261_PHY5_ADDR);
	if (ret)
		printf("rtl8261: phy%d serdes update failed: %d\n",
		       RTL8261_PHY5_ADDR, ret);

	ret = airoha_rtl8261_apply_board_cfg(eth, RTL8261_PHY5_ADDR);
	if (ret)
		printf("rtl8261: phy%d board cfg failed: %d\n",
		       RTL8261_PHY5_ADDR, ret);

	ret = airoha_rtl8261_link_state_get(eth, RTL8261_PHY5_ADDR, NULL, NULL,
					    &speed, NULL, NULL);
	if (!ret)
		airoha_eth_gdm4_set_frag_size(eth, speed);

	airoha_eth_gdm4_select_speed_mode(eth);
}

static void airoha_eth_gdm4_apply_runtime_phy_cfg(struct airoha_eth *eth)
{
	int ret;

	if (!eth->rtl8261_init_done || !eth->mdio_dev)
		return;

	ret = airoha_rtl8261_apply_board_cfg(eth, RTL8261_PHY5_ADDR);
	if (ret)
		printf("rtl8261: phy%d runtime board cfg failed: %d\n",
		       RTL8261_PHY5_ADDR, ret);

	if (!airoha_rtl8261_host_update_enabled())
		return;

	ret = airoha_rtl8261_serdes_mode_update(eth, RTL8261_PHY5_ADDR);
	if (ret)
		printf("rtl8261: phy%d runtime serdes update failed: %d\n",
		       RTL8261_PHY5_ADDR, ret);
}

static void airoha_qdma_dump_rx_window(const char *tag, int qdma_id,
				       struct airoha_queue *q)
{
	int i, count = q->ndesc;

	printf("rtl8261: %s qdma%d win", tag ? tag : "diag", qdma_id);
	for (i = 0; i < count; i++) {
		int idx = i;
		struct airoha_qdma_desc *desc = &q->desc[idx];
		u32 ctrl = le32_to_cpu(READ_ONCE(desc->ctrl));
		u32 msg1 = le32_to_cpu(READ_ONCE(desc->msg1));

		printf(" [%d:%08x/%08x]", idx, ctrl, msg1);
	}
	printf("\n");
}

static void airoha_eth_gdm4_diag(struct airoha_eth *eth, const char *tag)
{
	bool link_up = false, full_duplex = false;
	u32 pcs_sts1 = 0, base_r_sts1 = 0, an_sts0 = 0, an_sts2 = 0;
	u32 rx_freqdet = 0;
	u32 ppe1_route = 0, ppe2_route = 0, sp_cport = 0, fc_map6 = 0;
	u32 gdm4_fwd = 0, gdm4_src = 0, gdm4_src_legacy = 0;
	u32 gdm2_fwd = 0, gdm2_lpbk = 0, gdm2_txch = 0, gdm2_rxch = 0;
	u32 gdm4_tx_ok = 0, gdm4_rx0 = 0, gdm4_rx1 = 0, gdm4_rx2 = 0, gdm4_rx3 = 0;
	u32 qdma0_tx0 = 0, qdma0_tx1 = 0, qdma0_rx0 = 0, qdma0_rx1 = 0;
	u32 qdma1_tx0 = 0, qdma1_tx1 = 0, qdma1_rx0 = 0, qdma1_rx1 = 0;
	u16 qdma0_head = 0, qdma0_cpu_idx = 0, qdma0_dma_idx = 0;
	u16 qdma1_head = 0, qdma1_cpu_idx = 0, qdma1_dma_idx = 0;
	u32 qdma0_desc_ctrl = 0, qdma0_desc_addr = 0, qdma0_desc_msg1 = 0;
	u32 qdma1_desc_ctrl = 0, qdma1_desc_addr = 0, qdma1_desc_msg1 = 0;
	u32 fe_lan_mac_h = 0, fe_lan_mac_lmin = 0, fe_wan_mac_h = 0, fe_wan_mac_lmin = 0;
	u32 xfi_mac_h = 0, xfi_mac_l = 0, xfi_gib_cfg = 0, xfi_logic_rst = 0;
	u32 gdma4_tmbi_frag = 0, gdma4_rmbi_frag = 0;
	u64 gdm4_tx_ok_pkts = 0, gdm4_tx_eth_pkts = 0, gdm4_rx_ok_pkts = 0;
	u32 gdm4_tx_drop = 0, gdm4_rx_drop = 0;
	u32 phy5_serdes_cfg = 0;
	u32 mode = PCS_USXGMII_MODE_10000;
	u32 rate_adapt = PCS_HSGMII_FORCE_RATE_MODE_10000;
	u16 speed = 0;
	int an = -1, pma = -1, pcs = -1, phyxs = -1;
	int pma_mgbt = -1, pma_pair = -1, pcs_br1 = -1, pcs_br2 = -1;
	int an_mgbt_ctrl1 = -1, an_mgbt_stat1 = -1;

	if (!eth)
		return;

	if (eth->mdio_dev) {
		airoha_rtl8261_link_state_get(eth, RTL8261_PHY5_ADDR, &link_up,
					      &full_duplex, &speed, &mode,
					      &rate_adapt);
		an = airoha_mdio_read_lstatus(eth, RTL8261_PHY5_ADDR, MDIO_MMD_AN,
					      MDIO_STAT1);
		pma = airoha_mdio_read_lstatus(eth, RTL8261_PHY5_ADDR,
					       MDIO_MMD_PMAPMD, MDIO_STAT1);
		pcs = airoha_mdio_read_lstatus(eth, RTL8261_PHY5_ADDR, MDIO_MMD_PCS,
					       MDIO_STAT1);
		phyxs = airoha_mdio_read_lstatus(eth, RTL8261_PHY5_ADDR,
						 MDIO_MMD_PHYXS, MDIO_STAT1);
		pma_mgbt = airoha_mdio_c45_read(eth, RTL8261_PHY5_ADDR,
						MDIO_MMD_PMAPMD,
						RTL8261_PMA_MGBT_STATUS);
		pma_pair = airoha_mdio_c45_read(eth, RTL8261_PHY5_ADDR,
						MDIO_MMD_PMAPMD,
						RTL8261_PMA_MGBT_PAIR_SWAP);
		pcs_br1 = airoha_mdio_c45_read(eth, RTL8261_PHY5_ADDR,
					       MDIO_MMD_PCS,
					       RTL8261_PCS_BASE_R_STATUS1);
		pcs_br2 = airoha_mdio_c45_read(eth, RTL8261_PHY5_ADDR,
					       MDIO_MMD_PCS,
					       RTL8261_PCS_BASE_R_STATUS2);
		an_mgbt_ctrl1 = airoha_mdio_c45_read(eth, RTL8261_PHY5_ADDR,
						     MDIO_MMD_AN,
						     RTL8261_AN_MGBT_CTRL1);
		an_mgbt_stat1 = airoha_mdio_c45_read(eth, RTL8261_PHY5_ADDR,
						     MDIO_MMD_AN,
						     RTL8261_AN_MGBT_STATUS1);
		phy5_serdes_cfg = airoha_mdio_c45_read(eth, RTL8261_PHY5_ADDR,
						       RTL8261_MMD_VEND1,
						       RTL8261_SERDES_GLOBAL_CFG);
	}

	if (eth->eth_pcs_usxgmii) {
		pcs_sts1 = airoha_rr(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_STUS_1);
		base_r_sts1 = airoha_rr(eth->eth_pcs_usxgmii,
					PCS_USXGMII_BASE_R_10GB_T_PCS_STUS_1);
		an_sts0 = airoha_rr(eth->eth_pcs_usxgmii,
				    PCS_USXGMII_PCS_AN_STATS_0);
		an_sts2 = airoha_rr(eth->eth_pcs_usxgmii,
				    PCS_USXGMII_PCS_AN_STATS_2);
	}

	if (eth->fe_regs) {
		ppe1_route = airoha_fe_rr(eth,
					  REG_PPE_DFT_CPORT(0, FE_PSE_PORT_GDM4));
		ppe2_route = airoha_fe_rr(eth,
					  REG_PPE_DFT_CPORT(1, FE_PSE_PORT_GDM4));
		sp_cport = airoha_fe_rr(eth,
					REG_SP_DFT_CPORT(HSGMII_LAN_7581_ETH_SRCPORT >>
							 3));
		fc_map6 = airoha_fe_rr(eth, REG_SRC_PORT_FC_MAP6);
		gdm4_fwd = airoha_fe_rr(eth, REG_GDM_FWD_CFG(4));
		gdm4_src = airoha_fe_rr(eth, REG_GDM4_SRC_PORT_SET);
		gdm4_src_legacy = airoha_fe_rr(eth, REG_GDM4_SRC_PORT_SET_LEGACY);
		gdm2_fwd = airoha_fe_rr(eth, REG_GDM_FWD_CFG(2));
		gdm2_lpbk = airoha_fe_rr(eth, REG_GDM_LPBK_CFG(2));
		gdm2_txch = airoha_fe_rr(eth, REG_GDM_TXCHN_EN(2));
		gdm2_rxch = airoha_fe_rr(eth, REG_GDM_RXCHN_EN(2));
		gdm4_tx_ok = airoha_fe_rr(eth, REG_FE_DBG_GDM4_TX_OK);
		gdm4_rx0 = airoha_fe_rr(eth, REG_FE_DBG_GDM4_RX_BASE + 0x0);
		gdm4_rx1 = airoha_fe_rr(eth, REG_FE_DBG_GDM4_RX_BASE + 0x4);
		gdm4_rx2 = airoha_fe_rr(eth, REG_FE_DBG_GDM4_RX_BASE + 0x8);
		gdm4_rx3 = airoha_fe_rr(eth, REG_FE_DBG_GDM4_RX_BASE + 0xc);
		gdm4_tx_ok_pkts =
			((u64)airoha_fe_rr(eth, REG_FE_GDM_TX_OK_PKT_CNT_H(4)) << 32) |
			airoha_fe_rr(eth, REG_FE_GDM_TX_OK_PKT_CNT_L(4));
		gdm4_tx_eth_pkts =
			((u64)airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_PKT_CNT_H(4)) << 32) |
			airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_PKT_CNT_L(4));
		gdm4_rx_ok_pkts =
			((u64)airoha_fe_rr(eth, REG_FE_GDM_RX_OK_PKT_CNT_H(4)) << 32) |
			airoha_fe_rr(eth, REG_FE_GDM_RX_OK_PKT_CNT_L(4));
		gdm4_tx_drop = airoha_fe_rr(eth, REG_FE_GDM_TX_ETH_DROP_CNT(4));
		gdm4_rx_drop = airoha_fe_rr(eth, REG_FE_GDM_RX_ETH_DROP_CNT(4));
		gdma4_tmbi_frag = airoha_fe_rr(eth, REG_GDMA4_TMBI_FRAG);
		gdma4_rmbi_frag = airoha_fe_rr(eth, REG_GDMA4_RMBI_FRAG);
		fe_lan_mac_h = airoha_fe_rr(eth, REG_FE_LAN_MAC_H);
		fe_lan_mac_lmin = airoha_fe_rr(eth, REG_FE_MAC_LMIN(REG_FE_LAN_MAC_H));
		fe_wan_mac_h = airoha_fe_rr(eth, REG_FE_WAN_MAC_H);
		fe_wan_mac_lmin = airoha_fe_rr(eth, REG_FE_MAC_LMIN(REG_FE_WAN_MAC_H));
	}

	if (eth->eth_pcs_xfi_mac) {
		xfi_gib_cfg = airoha_rr(eth->eth_pcs_xfi_mac, PCS_XFI_GIB_CFG);
		xfi_logic_rst = airoha_rr(eth->eth_pcs_xfi_mac, PCS_XFI_LOGIC_RST);
		xfi_mac_h = airoha_rr(eth->eth_pcs_xfi_mac, PCS_XFI_MACADDRH);
		xfi_mac_l = airoha_rr(eth->eth_pcs_xfi_mac, PCS_XFI_MACADDRL);
	}

	if (eth->eth_pcs_xfi_pma)
		rx_freqdet = airoha_rr(eth->eth_pcs_xfi_pma, PCS_PMA_RX_FREQDET);

	if (airoha_qdma_ready(&eth->qdma[0])) {
		struct airoha_queue *q = &eth->qdma[0].q_rx[0];
		struct airoha_qdma_desc *desc = &q->desc[q->head];

		qdma0_tx0 = airoha_qdma_rr(&eth->qdma[0], REG_QDMA_DBG_TX_BASE + 0x0);
		qdma0_tx1 = airoha_qdma_rr(&eth->qdma[0], REG_QDMA_DBG_TX_BASE + 0x4);
		qdma0_rx0 = airoha_qdma_rr(&eth->qdma[0], REG_QDMA_DBG_RX_BASE + 0x0);
		qdma0_rx1 = airoha_qdma_rr(&eth->qdma[0], REG_QDMA_DBG_RX_BASE + 0x4);
		qdma0_head = q->head;
		qdma0_cpu_idx = FIELD_GET(RX_RING_CPU_IDX_MASK,
					 airoha_qdma_rr(&eth->qdma[0], REG_RX_CPU_IDX(0)));
		qdma0_dma_idx = FIELD_GET(RX_RING_DMA_IDX_MASK,
					 airoha_qdma_rr(&eth->qdma[0], REG_RX_DMA_IDX(0)));
		qdma0_desc_ctrl = le32_to_cpu(READ_ONCE(desc->ctrl));
		qdma0_desc_addr = le32_to_cpu(READ_ONCE(desc->addr));
		qdma0_desc_msg1 = le32_to_cpu(READ_ONCE(desc->msg1));
	}

	if (airoha_qdma_ready(&eth->qdma[1])) {
		struct airoha_queue *q = &eth->qdma[1].q_rx[0];
		struct airoha_qdma_desc *desc = &q->desc[q->head];

		qdma1_tx0 = airoha_qdma_rr(&eth->qdma[1], REG_QDMA_DBG_TX_BASE + 0x0);
		qdma1_tx1 = airoha_qdma_rr(&eth->qdma[1], REG_QDMA_DBG_TX_BASE + 0x4);
		qdma1_rx0 = airoha_qdma_rr(&eth->qdma[1], REG_QDMA_DBG_RX_BASE + 0x0);
		qdma1_rx1 = airoha_qdma_rr(&eth->qdma[1], REG_QDMA_DBG_RX_BASE + 0x4);
		qdma1_head = q->head;
		qdma1_cpu_idx = FIELD_GET(RX_RING_CPU_IDX_MASK,
					 airoha_qdma_rr(&eth->qdma[1], REG_RX_CPU_IDX(0)));
		qdma1_dma_idx = FIELD_GET(RX_RING_DMA_IDX_MASK,
					 airoha_qdma_rr(&eth->qdma[1], REG_RX_DMA_IDX(0)));
		qdma1_desc_ctrl = le32_to_cpu(READ_ONCE(desc->ctrl));
		qdma1_desc_addr = le32_to_cpu(READ_ONCE(desc->addr));
		qdma1_desc_msg1 = le32_to_cpu(READ_ONCE(desc->msg1));
	}

	printf("rtl8261: %s phy5 link=%d duplex=%s c1=%04x a434=%04x speed=%s mask=%03x mode=%s rate=%s mdio[an=%04x pma=%04x pcs=%04x phyxs=%04x] pcs[4=%08x 30=%08x 310=%08x 318=%08x] sigdet=%d pcslink=%d usx=%d freqdet=%08x fbck=%d\n",
	       tag ? tag : "diag", link_up, full_duplex ? "full" : "half",
	       phy5_serdes_cfg, speed, airoha_rtl8261_speed_name(speed),
	       speed & RTL8261_PHY_SPEED_MASK,
	       airoha_usxgmii_mode_name(mode),
	       airoha_rate_mode_name(rate_adapt), an, pma, pcs, phyxs,
	       pcs_sts1, base_r_sts1, an_sts0, an_sts2,
	       airoha_eth_gdm4_have_rx_signal(eth),
	       airoha_gdm4_pcs_link_up(eth), airoha_usxgmii_link_up(eth),
	       rx_freqdet, !!(rx_freqdet & PCS_PMA_FBCK_LOCK));
	printf("rtl8261: %s std pma[1=%04x rx=%d 81=%04x lpv=%d 82=%04x mdix=%x] pcs[1=%04x rx=%d 20=%04x lock=%d hiber=%d 21=%04x lat_lock=%d lat_hiber=%d ber=%02x err=%02x]\n",
	       tag ? tag : "diag", pma, !!(pma & MDIO_STAT1_LSTATUS),
	       pma_mgbt, !!(pma_mgbt & BIT(0)), pma_pair, pma_pair & 0x3, pcs,
	       !!(pcs & MDIO_STAT1_LSTATUS), pcs_br1, !!(pcs_br1 & BIT(0)),
	       !!(pcs_br1 & BIT(1)), pcs_br2, !!(pcs_br2 & BIT(15)),
	       !!(pcs_br2 & BIT(14)), (pcs_br2 >> 8) & 0x3f, pcs_br2 & 0xff);
	printf("rtl8261: %s std an[1=%04x done=%d link=%d page=%d 20=%04x adv10g=%d adv5g=%d adv2p5g=%d 21=%04x loc_rx=%d rem_rx=%d lp10g=%d lp5g=%d lp2p5g=%d]\n",
	       tag ? tag : "diag", an, !!(an & BIT(5)), !!(an & BIT(2)),
	       !!(an & BIT(6)), an_mgbt_ctrl1, !!(an_mgbt_ctrl1 & BIT(12)),
	       !!(an_mgbt_ctrl1 & BIT(8)), !!(an_mgbt_ctrl1 & BIT(7)),
	       an_mgbt_stat1, !!(an_mgbt_stat1 & BIT(13)),
	       !!(an_mgbt_stat1 & BIT(12)), !!(an_mgbt_stat1 & BIT(11)),
	       !!(an_mgbt_stat1 & BIT(6)), !!(an_mgbt_stat1 & BIT(5)));
	printf("rtl8261: %s fe route ppe1_dft=%08x ppe2_dft=%08x sport24_map=%08x fc_map6=%08x use_cdm2=%d loopback=%d gdm4_fwd=%08x gdm4_src[new=%08x old=%08x] gdm2[fwd=%08x lpbk=%08x txch=%08x rxch=%08x] last_rx[v=%d qdma=%u sport=0x%x crsn=0x%x ppe=%u len=%u ctrl=%08x msg1=%08x]\n",
	       tag ? tag : "diag", ppe1_route, ppe2_route, sp_cport, fc_map6,
	       airoha_gdm4_use_cdm2(eth), airoha_gdm4_loopback_enabled(eth),
	       gdm4_fwd, gdm4_src, gdm4_src_legacy, gdm2_fwd, gdm2_lpbk,
	       gdm2_txch, gdm2_rxch, eth->last_rx_valid, eth->last_rx_qdma,
	       eth->last_rx_sport, eth->last_rx_crsn, eth->last_rx_ppe_entry,
	       eth->last_rx_len, eth->last_rx_ctrl, eth->last_rx_msg1);
	printf("rtl8261: %s cnt gdm4_tx_ok=%08x gdm4_rx=%08x/%08x/%08x/%08x qdma0_tx=%08x/%08x qdma0_rx=%08x/%08x qdma1_tx=%08x/%08x qdma1_rx=%08x/%08x\n",
	       tag ? tag : "diag", gdm4_tx_ok, gdm4_rx0, gdm4_rx1, gdm4_rx2,
	       gdm4_rx3, qdma0_tx0, qdma0_tx1, qdma0_rx0, qdma0_rx1,
	       qdma1_tx0, qdma1_tx1, qdma1_rx0, qdma1_rx1);
	printf("rtl8261: %s mib gdm4[tx_ok=%llu tx_eth=%llu tx_drop=%08x rx_ok=%llu rx_drop=%08x] frag[tmbi=%08x rmbi=%08x]\n",
	       tag ? tag : "diag", gdm4_tx_ok_pkts, gdm4_tx_eth_pkts,
	       gdm4_tx_drop, gdm4_rx_ok_pkts, gdm4_rx_drop, gdma4_tmbi_frag,
	       gdma4_rmbi_frag);
	printf("rtl8261: %s mac fe[lan=%08x/%08x wan=%08x/%08x] xfi=%08x/%08x\n",
	       tag ? tag : "diag", fe_lan_mac_h, fe_lan_mac_lmin,
	       fe_wan_mac_h, fe_wan_mac_lmin, xfi_mac_h, xfi_mac_l);
	printf("rtl8261: %s xfi gib=%08x rst=%08x\n", tag ? tag : "diag",
	       xfi_gib_cfg, xfi_logic_rst);
	printf("rtl8261: %s qdma0 rxring head=%u cpu=%u dma=%u desc[ctrl=%08x addr=%08x msg1=%08x]\n",
	       tag ? tag : "diag", qdma0_head, qdma0_cpu_idx, qdma0_dma_idx,
	       qdma0_desc_ctrl, qdma0_desc_addr, qdma0_desc_msg1);
	printf("rtl8261: %s qdma1 rxring head=%u cpu=%u dma=%u desc[ctrl=%08x addr=%08x msg1=%08x]\n",
	       tag ? tag : "diag", qdma1_head, qdma1_cpu_idx, qdma1_dma_idx,
	       qdma1_desc_ctrl, qdma1_desc_addr, qdma1_desc_msg1);
	if (airoha_qdma_ready(&eth->qdma[0]))
		airoha_qdma_dump_rx_window(tag, 0, &eth->qdma[0].q_rx[0]);
	if (airoha_qdma_ready(&eth->qdma[1]))
		airoha_qdma_dump_rx_window(tag, 1, &eth->qdma[1].q_rx[0]);
}

static int airoha_rtl8261_intr_init(struct airoha_eth *eth, int phy_addr)
{
	int ret;

	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1, 0x00e1, 0);
	if (ret < 0)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1, 0x00e3, 0);
	if (ret < 0)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1, 0x00e4, 0x1);
	if (ret < 0)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1, 0x00e0, 0x2f);
	if (ret < 0)
		return ret;

	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND2, 0xa424, 0x10);
	if (ret < 0)
		return ret;
	ret = airoha_rtl8261_phy_modify(eth, phy_addr, 0xa44, 2, 15, 15, 0x1);
	if (ret)
		return ret;
	ret = airoha_rtl8261_phy_modify(eth, phy_addr, 0xa44, 8, 7, 7, 0x1);
	if (ret)
		return ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x01a0, BIT(11), BIT(11));
	if (ret)
		return ret;
	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x019d, BIT(11), BIT(11));
	if (ret)
		return ret;
	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x01a1, BIT(11), BIT(11));
	if (ret)
		return ret;
	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x019f, BIT(11), BIT(11));
	if (ret)
		return ret;

	return airoha_rtl8261_phy_modify(eth, phy_addr, 0xa42, 4, 7, 7, 0x1);
}

static int airoha_rtl8261_macsec_init(struct airoha_eth *eth, int phy_addr)
{
	int ret;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x02e0, GENMASK(1, 0), 0x3);
	if (ret)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1, 0x02d8, 0x5313);
	if (ret < 0)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1, 0x02da, 0x0101);
	if (ret < 0)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1, 0x02dc, 0x0101);
	if (ret < 0)
		return ret;
	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x03c6, GENMASK(7, 0), 0x0a);
	if (ret)
		return ret;
	ret = airoha_rtl8261_vend1_modify(eth, phy_addr, 0x037b, GENMASK(7, 0), 0x06);
	if (ret)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1, 0x02f7, 0x486c);
	if (ret < 0)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1, 0x03f1, 0x0072);
	if (ret < 0)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1, 0x03f0, 0x0b0b);
	if (ret < 0)
		return ret;

	return airoha_rtl8261_vend1_modify(eth, phy_addr, 0x03ee, GENMASK(15, 13), 0x2);
}

static int airoha_rtl8261_serdes_option_set(struct airoha_eth *eth, int phy_addr,
					    u16 option)
{
	static const u16 regs[] = {
		RTL8261_PHY_SERDES_OPTION_5G,
		RTL8261_PHY_SERDES_OPTION_2P5G,
		RTL8261_PHY_SERDES_OPTION_1G,
		RTL8261_PHY_SERDES_OPTION_100M,
		RTL8261_PHY_SERDES_OPTION_10M,
	};
	int i, ret, val;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		val = airoha_mdio_c45_read(eth, phy_addr, RTL8261_MMD_VEND1,
					   regs[i]);
		if (val < 0)
			return val;

		val &= ~0x0f0f;
		val |= option | (option << 8);

		ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1,
					    regs[i], val);
		if (ret)
			return ret;
	}

	return 0;
}

static int airoha_rtl8261_serdes_autoneg_set(struct airoha_eth *eth, int phy_addr,
					     bool enable)
{
	int ret;

	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1,
				    RTL8261_PHY_SERDES_OP_ADDR, 0x00f1);
	if (ret)
		return ret;

	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1,
				    RTL8261_PHY_SERDES_OP_DATA,
				    enable ? 0x854f : 0x854e);
	if (ret)
		return ret;

	return airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1,
				     RTL8261_PHY_SERDES_OP_CTRL, 0x0003);
}

static int airoha_rtl8261_phy_reset(struct airoha_eth *eth, int phy_addr)
{
	int ctrl, ret;
	u32 timeout = 500;

	ctrl = airoha_mdio_c45_read(eth, phy_addr, MDIO_MMD_PMAPMD,
				    RTL8261_PHY_10G_CTRL);
	if (ctrl < 0)
		return ctrl;

	ret = airoha_mdio_c45_write(eth, phy_addr, MDIO_MMD_PMAPMD,
				    RTL8261_PHY_10G_CTRL, ctrl | BIT(15));
	if (ret)
		return ret;

	do {
		ctrl = airoha_mdio_c45_read(eth, phy_addr, MDIO_MMD_PMAPMD,
					    RTL8261_PHY_10G_CTRL);
		if (ctrl < 0)
			return ctrl;
		if (!(ctrl & BIT(15)))
			break;
		udelay(10);
	} while (--timeout);

	if (!timeout)
		return -ETIMEDOUT;

	return airoha_mdio_c45_write(eth, phy_addr, MDIO_MMD_PMAPMD,
				     RTL8261_PHY_10G_CTRL, ctrl & ~BIT(15));
}

static void airoha_rtl8261_dump_state(struct airoha_eth *eth, int phy_addr)
{
	(void)eth;
	(void)phy_addr;

	/* Intentionally silent in normal runtime. */
}

static int airoha_rtl8261_apply_board_cfg(struct airoha_eth *eth, int phy_addr)
{
	u16 serdes_cfg = 0;
	bool pnswap_tx, pnswap_rx;
	int old_cfg, new_cfg, ret;
	int old_cfg_c22;

	pnswap_tx = phy_addr == RTL8261_PHY5_ADDR ? eth->rtl8261_phy5_pnswap_tx :
		    phy_addr == RTL8261_PHY8_ADDR ? eth->rtl8261_phy8_pnswap_tx :
						    false;
	pnswap_rx = phy_addr == RTL8261_PHY5_ADDR ? eth->rtl8261_phy5_pnswap_rx :
		    phy_addr == RTL8261_PHY8_ADDR ? eth->rtl8261_phy8_pnswap_rx :
						    false;

	if (phy_addr == RTL8261_PHY5_ADDR) {
		airoha_env_override_bool("rtl8261_phy5_pnswap_tx", &pnswap_tx);
		airoha_env_override_bool("rtl8261_phy5_pnswap_rx", &pnswap_rx);
	} else if (phy_addr == RTL8261_PHY8_ADDR) {
		airoha_env_override_bool("rtl8261_phy8_pnswap_tx", &pnswap_tx);
		airoha_env_override_bool("rtl8261_phy8_pnswap_rx", &pnswap_rx);
	}

	if (pnswap_tx)
		serdes_cfg |= RTL8261_SERDES_HSO_INV;
	if (pnswap_rx)
		serdes_cfg |= RTL8261_SERDES_HSI_INV;

	old_cfg = airoha_mdio_c45_read(eth, phy_addr, RTL8261_MMD_VEND1,
				       RTL8261_SERDES_GLOBAL_CFG);
	if (old_cfg < 0)
		return old_cfg;

	ret = airoha_rtl8261_vend1_modify(eth, phy_addr,
					  RTL8261_SERDES_GLOBAL_CFG,
					  RTL8261_SERDES_HSO_INV |
						  RTL8261_SERDES_HSI_INV,
					  serdes_cfg);
	if (ret)
		return ret;

	new_cfg = airoha_mdio_c45_read(eth, phy_addr, RTL8261_MMD_VEND1,
				       RTL8261_SERDES_GLOBAL_CFG);
	if (new_cfg < 0)
		return new_cfg;

	old_cfg_c22 = airoha_mdio_c22_mmd_read(eth, phy_addr, RTL8261_MMD_VEND1,
					       RTL8261_SERDES_GLOBAL_CFG);
	if (old_cfg_c22 >= 0 &&
	    (new_cfg & (RTL8261_SERDES_HSO_INV | RTL8261_SERDES_HSI_INV)) !=
		    serdes_cfg) {
		ret = airoha_mdio_c22_mmd_write(
			eth, phy_addr, RTL8261_MMD_VEND1, RTL8261_SERDES_GLOBAL_CFG,
			(old_cfg_c22 &
			 ~(RTL8261_SERDES_HSO_INV | RTL8261_SERDES_HSI_INV)) |
				serdes_cfg);
		if (!ret)
			new_cfg = airoha_mdio_c45_read(eth, phy_addr,
						       RTL8261_MMD_VEND1,
						       RTL8261_SERDES_GLOBAL_CFG);
	}

	return 0;
}

static int airoha_rtl8261_run_patch_table(struct airoha_eth *eth, int phy_addr,
					  const rtk_hwpatch_t *table, size_t n)
{
	size_t i;
	int ret;

	for (i = 0; i < n; i++) {
		ret = airoha_rtl8261_apply_patch(eth, phy_addr, &table[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int airoha_rtl8261_flow_r1(struct airoha_eth *eth, int phy_addr)
{
	int ret;

	ret = airoha_rtl8261_phy_modify(eth, phy_addr, 0xb82, 16, 4, 4, 0x1);
	if (ret)
		return ret;
	ret = airoha_rtl8261_phy_modify(eth, phy_addr, 0xb82, 16, 4, 4, 0x1);
	if (ret)
		return ret;
	ret = airoha_rtl8261_wait(eth, phy_addr, RTL8261_MMD_VEND2,
				  airoha_rtl8261_phy_reg_convert(0xb80, 16),
				  BIT(6), BIT(6), true);
	if (ret)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND2,
				    airoha_rtl8261_phy_reg_convert(0xa43, 27),
				    0x8023);
	if (ret)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND2,
				    airoha_rtl8261_phy_reg_convert(0xa43, 28),
				    0x3802);
	if (ret)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND2,
				    airoha_rtl8261_phy_reg_convert(0xa43, 27),
				    0xb82e);
	if (ret)
		return ret;
	return airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND2,
				     airoha_rtl8261_phy_reg_convert(0xa43, 28),
				     0x0001);
}

static int airoha_rtl8261_flow_r2(struct airoha_eth *eth, int phy_addr)
{
	int ret;

	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND2,
				    airoha_rtl8261_phy_reg_convert(0xa43, 27),
				    0x0000);
	if (ret)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND2,
				    airoha_rtl8261_phy_reg_convert(0xa43, 28),
				    0x0000);
	if (ret)
		return ret;
	ret = airoha_rtl8261_phy_modify(eth, phy_addr, 0xb82, 23, 0, 0, 0x0);
	if (ret)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND2,
				    airoha_rtl8261_phy_reg_convert(0xa43, 27),
				    0x8023);
	if (ret)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND2,
				    airoha_rtl8261_phy_reg_convert(0xa43, 28),
				    0x0000);
	if (ret)
		return ret;
	ret = airoha_rtl8261_phy_modify(eth, phy_addr, 0xb82, 16, 4, 4, 0x0);
	if (ret)
		return ret;
	return airoha_rtl8261_wait(eth, phy_addr, RTL8261_MMD_VEND2,
				   airoha_rtl8261_phy_reg_convert(0xb80, 16),
				   BIT(6), BIT(6), false);
}

static int airoha_rtl8261_flow_l1(struct airoha_eth *eth, int phy_addr)
{
	int ret;

	ret = airoha_rtl8261_phy_modify(eth, phy_addr, 0xa4a, 16, 10, 10, 0x1);
	if (ret)
		return ret;
	ret = airoha_rtl8261_phy_modify(eth, phy_addr, 0xa4a, 16, 10, 10, 0x1);
	if (ret)
		return ret;

	return airoha_rtl8261_wait(eth, phy_addr, RTL8261_MMD_VEND2,
				   airoha_rtl8261_phy_reg_convert(0xa60, 16),
				   0x1, 0xff, true);
}

static int airoha_rtl8261_flow_l2(struct airoha_eth *eth, int phy_addr)
{
	int ret;

	ret = airoha_rtl8261_phy_modify(eth, phy_addr, 0xa4a, 16, 10, 10, 0x0);
	if (ret)
		return ret;

	return airoha_rtl8261_wait(eth, phy_addr, RTL8261_MMD_VEND2,
				   airoha_rtl8261_phy_reg_convert(0xa60, 16),
				   0x1, 0xff, false);
}

static int airoha_rtl8261_full_patch_one(struct airoha_eth *eth, int phy_addr)
{
	int ret, chip_id, phy_id2, ctrl;

	chip_id = airoha_mdio_c45_read(eth, phy_addr, RTL8261_MMD_VEND1,
				       RTL8261_PHY_CHIP_ID);
	phy_id2 = airoha_mdio_c22_read(eth, phy_addr, RTL8261_PHY_ID2);

	if ((chip_id < 0 ||
	     (chip_id & RTL8261BE_CHIP_ID_MASK) != RTL8261BE_CHIP_ID) &&
	    phy_id2 != RTL8261_PHY_ID2_EXPECT) {
		return 0;
	}

	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1,
				    RTL8261_PHY_EXT_RESET, 0x0001);
	if (ret)
		return ret;
	ret = airoha_mdio_c45_write(eth, phy_addr, RTL8261_MMD_VEND1,
				    RTL8261_PHY_EXT_RESET, 0x0000);
	if (ret)
		return ret;

	mdelay(30);

	ret = airoha_rtl8261_flow_r1(eth, phy_addr);
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(eth, phy_addr,
					     rtl8261n_c_nctl0_conf,
					     ARRAY_SIZE(rtl8261n_c_nctl0_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(eth, phy_addr,
					     rtl8261n_c_nctl1_conf,
					     ARRAY_SIZE(rtl8261n_c_nctl1_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(eth, phy_addr,
					     rtl8261n_c_nctl2_conf,
					     ARRAY_SIZE(rtl8261n_c_nctl2_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(eth, phy_addr, rtl8261n_c_uc2_conf,
					     ARRAY_SIZE(rtl8261n_c_uc2_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(eth, phy_addr, rtl8261n_c_uc_conf,
					     ARRAY_SIZE(rtl8261n_c_uc_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(
		eth, phy_addr, rtl8261n_c_dataram_conf,
		ARRAY_SIZE(rtl8261n_c_dataram_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_flow_r2(eth, phy_addr);
	if (ret)
		return ret;
	ret = airoha_rtl8261_flow_l1(eth, phy_addr);
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(eth, phy_addr,
					     rtl8261n_c_algxg_conf,
					     ARRAY_SIZE(rtl8261n_c_algxg_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(
		eth, phy_addr, rtl8261n_c_alg_giga_conf,
		ARRAY_SIZE(rtl8261n_c_alg_giga_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(
		eth, phy_addr, rtl8261n_c_normal_conf,
		ARRAY_SIZE(rtl8261n_c_normal_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(eth, phy_addr, rtl8261n_c_top_conf,
					     ARRAY_SIZE(rtl8261n_c_top_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(eth, phy_addr, rtl8261n_c_sds_conf,
					     ARRAY_SIZE(rtl8261n_c_sds_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(eth, phy_addr, rtl8261n_c_afe_conf,
					     ARRAY_SIZE(rtl8261n_c_afe_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_run_patch_table(eth, phy_addr,
					     rtl8261n_c_rtct_conf,
					     ARRAY_SIZE(rtl8261n_c_rtct_conf));
	if (ret)
		return ret;
	ret = airoha_rtl8261_flow_l2(eth, phy_addr);
	if (ret)
		return ret;

	ret = airoha_rtl8261_apply_board_cfg(eth, phy_addr);
	if (ret)
		return ret;

	/*
	 * Keep the recovery path aligned with the vendor RTL8261 U-Boot flow:
	 * it patches the PHY and sets the host-side serdes mode, but does not
	 * enable the extra interrupt/MACsec blocks here. These are unnecessary
	 * for bare ARP/ICMP recovery and are a plausible source of the current
	 * TX-only failure on the host-facing 10G path.
	 */

	ret = airoha_rtl8261_sds_write(eth, phy_addr, 6, 3,
				       RTL8261_PHY_SDS_XR1710G_MODE);
	if (ret)
		return ret;

	ret = airoha_rtl8261_serdes_mode_update(eth, phy_addr);
	if (ret)
		return ret;

	ctrl = airoha_mdio_c45_read(eth, phy_addr, MDIO_MMD_PMAPMD,
				    RTL8261_PHY_10G_CTRL);
	if (ctrl < 0)
		return ctrl;

	ret = airoha_mdio_c45_write(eth, phy_addr, MDIO_MMD_PMAPMD,
				    RTL8261_PHY_10G_CTRL,
				    ctrl & ~RTL8261_PHY_10G_AN_DIS);
	if (ret)
		return ret;

	ret = airoha_rtl8261_restart_aneg(eth, phy_addr);
	if (ret)
		return ret;

	airoha_rtl8261_dump_state(eth, phy_addr);

	return 0;
}

static void airoha_rtl8261_minimal_init(struct airoha_eth *eth)
{
	int ret;

	if (eth->rtl8261_init_done)
		return;

	if (!eth->mdio_dev)
		return;

	ret = airoha_rtl8261_full_patch_one(eth, RTL8261_PHY5_ADDR);
	if (ret) {
		printf("rtl8261: phy%d patch failed: %d\n", RTL8261_PHY5_ADDR,
		       ret);
		return;
	}

	ret = airoha_rtl8261_full_patch_one(eth, RTL8261_PHY8_ADDR);
	if (ret) {
		printf("rtl8261: phy%d patch failed: %d\n", RTL8261_PHY8_ADDR,
		       ret);
		return;
	}

	eth->rtl8261_init_done = true;
}

static void airoha_rtl8261_read_phy_props(struct airoha_eth *eth, ofnode mdio_node)
{
	ofnode child;

	ofnode_for_each_subnode(child, mdio_node) {
		u32 reg;

		reg = ofnode_read_u32_default(child, "reg", U32_MAX);
		switch (reg) {
		case RTL8261_PHY5_ADDR:
			eth->rtl8261_phy5_pnswap_tx =
				ofnode_read_bool(child, "realtek,pnswap-tx");
			eth->rtl8261_phy5_pnswap_rx =
				ofnode_read_bool(child, "realtek,pnswap-rx");
			break;
		case RTL8261_PHY8_ADDR:
			eth->rtl8261_phy8_pnswap_tx =
				ofnode_read_bool(child, "realtek,pnswap-tx");
			eth->rtl8261_phy8_pnswap_rx =
				ofnode_read_bool(child, "realtek,pnswap-rx");
			break;
		}
	}
}

static ofnode airoha_ext_mdio_node(struct udevice *dev)
{
	ofnode mdio_node;

	mdio_node = ofnode_by_compatible(ofnode_null(), "airoha,arht-mdio");
	if (ofnode_valid(mdio_node))
		return mdio_node;

	return ofnode_null();
}

static ofnode airoha_switch_mdio_node(struct udevice *dev)
{
	struct airoha_eth_soc_data *data = (void *)dev_get_driver_data(dev);
	ofnode switch_node;

	switch_node =
		ofnode_by_compatible(ofnode_null(), data->switch_compatible);
	if (!ofnode_valid(switch_node))
		return ofnode_null();

	return ofnode_find_subnode(switch_node, "mdio");
}

static int airoha_mdio_link_up(struct airoha_eth *eth, int addr)
{
	static const int c45_devads[] = {
		MDIO_MMD_AN,
		MDIO_MMD_PMAPMD,
		MDIO_MMD_PCS,
		MDIO_MMD_PHYXS,
	};
	int bmsr, i;

	if (!airoha_rtl8261_is_phy_addr(addr) && !eth->switch_mdio_dev &&
	    !eth->mdio_dev)
		return 0;

	if (airoha_rtl8261_is_phy_addr(addr) && !eth->mdio_dev)
		return 0;

	if (airoha_rtl8261_is_phy_addr(addr)) {
		bool link_up;

		if (!airoha_rtl8261_link_state_get(eth, addr, &link_up, NULL,
						   NULL, NULL, NULL))
			return link_up;
	}

	bmsr = airoha_mdio_read_lstatus(eth, addr, MDIO_DEVAD_NONE, MII_BMSR);
	if (bmsr >= 0 && (bmsr & BMSR_LSTATUS))
		return 1;

	for (i = 0; i < ARRAY_SIZE(c45_devads); i++) {
		bmsr = airoha_mdio_read_lstatus(eth, addr, c45_devads[i],
						MDIO_STAT1);
		if (bmsr >= 0 && (bmsr & MDIO_STAT1_LSTATUS))
			return 1;
	}

	return 0;
}

static int airoha_usxgmii_link_up(struct airoha_eth *eth)
{
	u32 lpa, an_done;

	if (!eth->eth_pcs_usxgmii)
		return 0;

	/*
	 * Match Linux PCS state readout: toggle AN status latch, then read the
	 * current link partner ability and completion state from USXGMII PCS.
	 */
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_AN_CONTROL_6, 0,
		   PCS_USXGMII_TOG_PCS_AUTONEG_STS);
	airoha_rmw(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_AN_CONTROL_6,
		   PCS_USXGMII_TOG_PCS_AUTONEG_STS, 0);

	lpa = airoha_rr(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_AN_STATS_0);
	an_done = airoha_rr(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_AN_STATS_2);

	return !!(lpa & MDIO_USXGMII_LINK) || !!(an_done & BIT(24));
}

static int airoha_gdm4_pcs_link_up(struct airoha_eth *eth)
{
	u32 status;

	if (!eth->eth_pcs_usxgmii)
		return 0;

	status = airoha_rr(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_STUS_1);
	if (status & PCS_USXGMII_PCS_RX_LINK_STATUS)
		return 1;

	status = airoha_rr(eth->eth_pcs_usxgmii,
			   PCS_USXGMII_BASE_R_10GB_T_PCS_STUS_1);

	return !!(status & PCS_USXGMII_RX_LINK_STUS);
}

static int airoha_eth_gdm4_have_rx_signal(struct airoha_eth *eth)
{
	u32 val;
	int count = 0;
	int i;

	if (!eth->eth_pcs_xfi_pma)
		return 0;

	airoha_wr(eth->eth_pcs_xfi_pma, PCS_PMA_DIG_RESERVE_0,
		  FIELD_PREP(PCS_TRIGGER_RX_SIGDET_SCAN, 0x3));

	for (i = 0; i <= 5; i++) {
		val = airoha_rr(eth->eth_pcs_xfi_pma, PCS_PMA_DIG_RO_RESERVE_2);
		if (val & PCS_RX_SIGDET)
			count++;
	}

	return count >= 4;
}

static int airoha_eth_gdm4_rxlock_workaround(struct airoha_eth *eth)
{
	u32 val;

	if (!eth->eth_pcs_usxgmii || !eth->eth_pcs_xfi_pma)
		return 0;

	val = airoha_rr(eth->eth_pcs_usxgmii, PCS_USXGMII_PCS_STUS_1);
	if (val & PCS_USXGMII_PCS_RX_LINK_STATUS)
		return 0;

	if (!airoha_eth_gdm4_have_rx_signal(eth))
		return 0;

	airoha_eth_gdm4_cdr_reset(eth);
	airoha_eth_gdm4_restart_an(eth);

	return -EINVAL;
}

static int airoha_forced_tx_fport(struct airoha_eth *eth)
{
	const char *port;
	bool need_gdm4_bringup;

	port = env_get("recovery_port");
	if (!port || !*port || !strcmp(port, "auto"))
		return 0;

	if (!strcmp(port, "1") || !strcmp(port, "1g") ||
	    !strcmp(port, "lan2") || !strcmp(port, "lan3")) {
		eth->gdm4_link_up = false;
		return 1;
	}

	if (!strcmp(port, "2") || !strcmp(port, "wan")) {
		eth->gdm4_link_up = false;
		return 2;
	}

	if (!strcmp(port, "4") || !strcmp(port, "10g") ||
	    !strcmp(port, "lan1")) {
		airoha_eth_gdm4_apply_runtime_phy_cfg(eth);
		need_gdm4_bringup = eth->tx_fport != 4;

		/*
		 * Do not restart in-band autonegotiation before every packet.
		 * On XR1710G that keeps retraining the external serdes link and
		 * can drop the very ARP/ICMP frames we are trying to emit.
		 * Only run the heavy bring-up path when link is not already up.
		 */
		if (need_gdm4_bringup || !eth->gdm4_link_up ||
		    !airoha_usxgmii_link_up(eth) ||
		    !airoha_eth_gdm4_have_rx_signal(eth)) {
			airoha_eth_gdm4_sync_rtl8261(eth);
			airoha_eth_gdm4_restart_an(eth);
			airoha_eth_gdm4_rxlock_workaround(eth);
			airoha_eth_gdm4_link_up_config(eth);
			eth->gdm4_link_up = true;
		}
		return 4;
	}

	return 0;
}

static bool airoha_recovery_lan_up(struct airoha_eth *eth)
{
	return airoha_mdio_link_up(eth, 9) || airoha_mdio_link_up(eth, 10);
}

static bool airoha_recovery_wan_up(struct airoha_eth *eth)
{
	return airoha_mdio_link_up(eth, 8);
}

static bool airoha_recovery_gdm4_candidate(struct airoha_eth *eth)
{
	return (airoha_gdm4_pcs_link_up(eth) &&
		airoha_eth_gdm4_have_rx_signal(eth)) ||
	       airoha_usxgmii_link_up(eth) ||
	       airoha_mdio_link_up(eth, 5);
}

static void airoha_gdm4_ensure_ready(struct airoha_eth *eth)
{
	airoha_eth_gdm4_apply_runtime_phy_cfg(eth);

	if (!eth->gdm4_link_up || !airoha_usxgmii_link_up(eth) ||
	    !airoha_eth_gdm4_have_rx_signal(eth)) {
		airoha_eth_gdm4_sync_rtl8261(eth);
		airoha_eth_gdm4_restart_an(eth);
		airoha_eth_gdm4_rxlock_workaround(eth);
		airoha_eth_gdm4_link_up_config(eth);
		eth->gdm4_link_up = true;
	}
}

static u8 airoha_pick_tx_fport(struct airoha_eth *eth)
{
	bool lan_up;
	int wan_up;
	int forced_fport;
	bool gdm4_candidate;
	bool dual_service;

	forced_fport = airoha_forced_tx_fport(eth);
	if (forced_fport > 0)
		return forced_fport;

	airoha_eth_gdm4_rxlock_workaround(eth);

	lan_up = airoha_recovery_lan_up(eth);
	wan_up = airoha_recovery_wan_up(eth);
	gdm4_candidate = airoha_recovery_gdm4_candidate(eth);
	dual_service = airoha_recovery_dual_service_enabled();

	if (dual_service && gdm4_candidate)
		airoha_gdm4_ensure_ready(eth);

	/*
	 * In auto mode, keep the dedicated 1G recovery ports as the default
	 * egress whenever they have live link. Dual-port service may still send
	 * individual replies back to 10G based on the peer MAC cache.
	 */
	if (lan_up) {
		if (!dual_service)
			eth->gdm4_link_up = false;
		return 1;
	}

	if (eth->tx_fport == 4 && gdm4_candidate) {
		if (!dual_service)
			airoha_gdm4_ensure_ready(eth);
		return 4;
	}

	if (eth->tx_fport == 2 && wan_up) {
		if (!dual_service)
			eth->gdm4_link_up = false;
		return 2;
	}

	if (gdm4_candidate) {
		if (!dual_service)
			airoha_gdm4_ensure_ready(eth);
		return 4;
	}

	eth->gdm4_link_up = false;

	if (wan_up)
		return 2;

	if (eth->tx_fport == 2 || eth->tx_fport == 4)
		return eth->tx_fport;

	return 1;
}

static u8 airoha_encode_tx_fport(u8 fport)
{
	/*
	 * Linux programs QDMA TXMSG.FPORT with the FE/PSE port id for GDM4,
	 * not the logical netdev id 4. Keep the recovery policy in logical
	 * ports and translate only at the QDMA boundary.
	 */
	return fport == 4 ? FE_PSE_PORT_GDM4 : fport;
}

static u32 airoha_fe_get_pse_queue_rsv_pages(struct airoha_eth *eth, u32 port,
					     u32 queue)
{
	u32 val;

	airoha_fe_rmw(eth, REG_FE_PSE_QUEUE_CFG_WR,
		      PSE_CFG_PORT_ID_MASK | PSE_CFG_QUEUE_ID_MASK,
		      FIELD_PREP(PSE_CFG_PORT_ID_MASK, port) |
			      FIELD_PREP(PSE_CFG_QUEUE_ID_MASK, queue));
	val = airoha_fe_rr(eth, REG_FE_PSE_QUEUE_CFG_VAL);

	return FIELD_GET(PSE_CFG_OQ_RSV_MASK, val);
}

static void airoha_fe_set_pse_queue_rsv_pages(struct airoha_eth *eth, u32 port,
					      u32 queue, u32 val)
{
	airoha_fe_rmw(eth, REG_FE_PSE_QUEUE_CFG_VAL, PSE_CFG_OQ_RSV_MASK,
		      FIELD_PREP(PSE_CFG_OQ_RSV_MASK, val));
	airoha_fe_rmw(eth, REG_FE_PSE_QUEUE_CFG_WR,
		      PSE_CFG_PORT_ID_MASK | PSE_CFG_QUEUE_ID_MASK |
			      PSE_CFG_WR_EN_MASK | PSE_CFG_OQRSV_SEL_MASK,
		      FIELD_PREP(PSE_CFG_PORT_ID_MASK, port) |
			      FIELD_PREP(PSE_CFG_QUEUE_ID_MASK, queue) |
			      PSE_CFG_WR_EN_MASK | PSE_CFG_OQRSV_SEL_MASK);
}

static u32 airoha_fe_get_pse_all_rsv(struct airoha_eth *eth)
{
	u32 val = airoha_fe_rr(eth, REG_FE_PSE_BUF_SET);

	return FIELD_GET(PSE_ALLRSV_MASK, val);
}

static void airoha_fe_set_pse_oq_rsv(struct airoha_eth *eth, u32 port,
				     u32 queue, u32 val)
{
	u32 orig_val = airoha_fe_get_pse_queue_rsv_pages(eth, port, queue);
	u32 tmp, all_rsv, fq_limit;

	airoha_fe_set_pse_queue_rsv_pages(eth, port, queue, val);

	all_rsv = airoha_fe_get_pse_all_rsv(eth);
	all_rsv += (val - orig_val);
	airoha_fe_rmw(eth, REG_FE_PSE_BUF_SET, PSE_ALLRSV_MASK,
		      FIELD_PREP(PSE_ALLRSV_MASK, all_rsv));

	tmp = airoha_fe_rr(eth, PSE_FQ_CFG);
	fq_limit = FIELD_GET(PSE_FQ_LIMIT_MASK, tmp);
	tmp = fq_limit - all_rsv - 0x20;
	airoha_fe_rmw(eth, REG_PSE_SHARE_USED_THD, PSE_SHARE_USED_HTHD_MASK,
		      FIELD_PREP(PSE_SHARE_USED_HTHD_MASK, tmp));

	tmp = fq_limit - all_rsv - 0x100;
	airoha_fe_rmw(eth, REG_PSE_SHARE_USED_THD, PSE_SHARE_USED_MTHD_MASK,
		      FIELD_PREP(PSE_SHARE_USED_MTHD_MASK, tmp));
	tmp = (3 * tmp) >> 2;
	airoha_fe_rmw(eth, REG_FE_PSE_BUF_SET, PSE_SHARE_USED_LTHD_MASK,
		      FIELD_PREP(PSE_SHARE_USED_LTHD_MASK, tmp));
}

static void airoha_fe_pse_ports_init(struct airoha_eth *eth)
{
	static const u8 pse_port_num_queues[] = {
		[FE_PSE_PORT_CDM1] = 6,	 [FE_PSE_PORT_GDM1] = 6,
		[FE_PSE_PORT_GDM2] = 32, [FE_PSE_PORT_GDM3] = 6,
		[FE_PSE_PORT_PPE1] = 4,	 [FE_PSE_PORT_CDM2] = 6,
		[FE_PSE_PORT_CDM3] = 8,	 [FE_PSE_PORT_CDM4] = 10,
		[FE_PSE_PORT_PPE2] = 4,	 [FE_PSE_PORT_GDM4] = 2,
		[FE_PSE_PORT_CDM5] = 2,
	};
	u32 all_rsv;
	int q;

	all_rsv = airoha_fe_get_pse_all_rsv(eth);
	all_rsv += PSE_QUEUE_RSV_PAGES * pse_port_num_queues[FE_PSE_PORT_PPE2];
	airoha_fe_set(eth, REG_FE_PSE_BUF_SET, all_rsv);

	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_CDM1]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM1, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_GDM1]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_GDM1, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 6; q < pse_port_num_queues[FE_PSE_PORT_GDM2]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_GDM2, q, 0);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_GDM3]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_GDM3, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_PPE1]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_PPE1, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_CDM2]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM2, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_CDM3] - 1; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM3, q, 0);
	for (q = 4; q < pse_port_num_queues[FE_PSE_PORT_CDM4]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM4, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_PPE2] / 2; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_PPE2, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = pse_port_num_queues[FE_PSE_PORT_PPE2] / 2;
	     q < pse_port_num_queues[FE_PSE_PORT_PPE2]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_PPE2, q, 0);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_GDM4]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_GDM4, q,
					 PSE_QUEUE_RSV_PAGES);
	for (q = 0; q < pse_port_num_queues[FE_PSE_PORT_CDM5]; q++)
		airoha_fe_set_pse_oq_rsv(eth, FE_PSE_PORT_CDM5, q,
					 PSE_QUEUE_RSV_PAGES);
}

static int airoha_fe_init(struct airoha_eth *eth)
{
	u32 src_port, src_port_slot, src_port_mask;

	airoha_fe_maccr_init(eth);

	airoha_fe_rmw(eth, REG_PSE_IQ_REV1, PSE_IQ_RES1_P2_MASK,
		      FIELD_PREP(PSE_IQ_RES1_P2_MASK, 0x10));
	airoha_fe_rmw(eth, REG_PSE_IQ_REV2,
		      PSE_IQ_RES2_P5_MASK | PSE_IQ_RES2_P4_MASK,
		      FIELD_PREP(PSE_IQ_RES2_P5_MASK, 0x40) |
			      FIELD_PREP(PSE_IQ_RES2_P4_MASK, 0x34));

	airoha_fe_wr(eth, REG_FE_PCE_CFG,
		     PCE_DPI_EN_MASK | PCE_KA_EN_MASK | PCE_MC_EN_MASK);
	/*
	 * The full Linux driver uses ring1 for VIP traffic, but recovery only
	 * drains ring0. Keep VIP packets on ring0 in U-Boot.
	 */
	airoha_fe_rmw(eth, REG_CDM1_FWD_CFG, CDM1_VIP_QSEL_MASK,
		      FIELD_PREP(CDM1_VIP_QSEL_MASK, 0x0));
	airoha_fe_rmw(eth, REG_CDM2_FWD_CFG, CDM2_VIP_QSEL_MASK,
		      FIELD_PREP(CDM2_VIP_QSEL_MASK, 0x0));
	airoha_fe_crsn_qsel_init(eth);
	airoha_set_gdm_port_fwd_cfg(eth, REG_GDM_FWD_CFG(1), FE_PSE_PORT_PPE1);
	airoha_set_gdm_port_fwd_cfg(eth, REG_GDM_FWD_CFG(2), FE_PSE_PORT_PPE1);
	airoha_set_gdm_port_fwd_cfg(eth, REG_GDM_FWD_CFG(4), FE_PSE_PORT_PPE1);
	/*
	 * Linux assigns the PPE default CPU port per GDM netdev in ndo_init.
	 * Recovery only needs PPE1 and QDMA0, so route GDM1/GDM2/GDM4 traffic
	 * received by PPE1 back to CDM1 explicitly. GDM4 is FE/PSE port 9, so
	 * its default CPU-port entry lives in the next PPE default-cport
	 * register word rather than alongside GDM1/GDM2.
	 */
	airoha_fe_rmw(eth, REG_PPE_DFT_CPORT(0, FE_PSE_PORT_GDM1),
		      DFT_CPORT_MASK(FE_PSE_PORT_GDM1) |
			      DFT_CPORT_MASK(FE_PSE_PORT_GDM2),
		      FIELD_PREP(DFT_CPORT_MASK(FE_PSE_PORT_GDM1),
				 FE_PSE_PORT_CDM1) |
			      FIELD_PREP(DFT_CPORT_MASK(FE_PSE_PORT_GDM2),
					 FE_PSE_PORT_CDM1));
	airoha_fe_rmw(eth, REG_PPE_DFT_CPORT(0, FE_PSE_PORT_GDM4),
		      DFT_CPORT_MASK(FE_PSE_PORT_GDM4),
		      FIELD_PREP(DFT_CPORT_MASK(FE_PSE_PORT_GDM4),
				 FE_PSE_PORT_CDM1));
	airoha_fe_clear(eth, REG_GDM_INGRESS_CFG(1), GDM_STAG_EN_MASK);
	airoha_fe_clear(eth, REG_GDM_INGRESS_CFG(2), GDM_STAG_EN_MASK);
	airoha_fe_clear(eth, REG_GDM_INGRESS_CFG(4), GDM_STAG_EN_MASK);
	airoha_fe_rmw(
		eth, REG_GDM_LEN_CFG(1), GDM_SHORT_LEN_MASK | GDM_LONG_LEN_MASK,
		FIELD_PREP(GDM_SHORT_LEN_MASK, 60) |
			FIELD_PREP(GDM_LONG_LEN_MASK, AIROHA_MAX_PACKET_SIZE));
	airoha_fe_rmw(
		eth, REG_GDM_LEN_CFG(2), GDM_SHORT_LEN_MASK | GDM_LONG_LEN_MASK,
		FIELD_PREP(GDM_SHORT_LEN_MASK, 60) |
			FIELD_PREP(GDM_LONG_LEN_MASK, AIROHA_MAX_PACKET_SIZE));
	airoha_fe_rmw(
		eth, REG_GDM_LEN_CFG(4), GDM_SHORT_LEN_MASK | GDM_LONG_LEN_MASK,
		FIELD_PREP(GDM_SHORT_LEN_MASK, 60) |
			FIELD_PREP(GDM_LONG_LEN_MASK, AIROHA_MAX_PACKET_SIZE));

	/*
	 * U-Boot only needs a minimal data path for recovery traffic, but the
	 * stock Linux driver still programs a handful of FE registers that are
	 * required to make EN7581 ports come out of reset reliably.
	 */
	airoha_fe_rmw(eth, REG_GDM4_SRC_PORT_SET,
		      GDM4_SPORT_OFF2_MASK | GDM4_SPORT_OFF1_MASK |
			      GDM4_SPORT_OFF0_MASK,
		      FIELD_PREP(GDM4_SPORT_OFF2_MASK, 8) |
			      FIELD_PREP(GDM4_SPORT_OFF1_MASK, 8) |
			      FIELD_PREP(GDM4_SPORT_OFF0_MASK, 8));
	airoha_fe_rmw(eth, REG_GDM4_SRC_PORT_SET_LEGACY,
		      GDM4_SPORT_OFF2_MASK | GDM4_SPORT_OFF1_MASK |
			      GDM4_SPORT_OFF0_MASK,
		      FIELD_PREP(GDM4_SPORT_OFF2_MASK, 8) |
			      FIELD_PREP(GDM4_SPORT_OFF1_MASK, 8) |
			      FIELD_PREP(GDM4_SPORT_OFF0_MASK, 8));

	/*
	 * EN7581 external Ethernet on GDM4 uses source-port 0x18. Without
	 * programming the FE WAN/src-port maps, traffic from lan1 never enters
	 * the simple recovery path even if the PCS link is up.
	 *
	 * Linux loopback for full netdev datapaths routes this source-port to
	 * CDM2, but recovery receives packets through the reduced CDM1/QDMA
	 * path configured below. Point source-port 0x18 at CDM1 here so GDM4
	 * ingress can actually reach the RX ring used by recovery.
	 */
	src_port = HSGMII_LAN_7581_ETH_SRCPORT;
	src_port_slot = src_port & SP_CPORT_DFT_MASK;
	src_port_mask = SP_CPORT_MASK(src_port_slot);
	airoha_fe_rmw(eth, REG_FE_WAN_PORT,
		      WAN1_EN_MASK | WAN1_MASK | WAN0_MASK,
		      FIELD_PREP(WAN0_MASK, src_port));
	airoha_fe_rmw(eth, REG_SRC_PORT_FC_MAP6, FC_ID_OF_SRC_PORT24_MASK,
		      FIELD_PREP(FC_ID_OF_SRC_PORT24_MASK, 2));
	airoha_set_xsi_eth_port(eth, true);
	airoha_gdm4_update_cpu_path(eth);

	airoha_fe_rmw(eth, REG_FE_DMA_GLO_CFG,
		      FE_DMA_GLO_L2_SPACE_MASK | FE_DMA_GLO_PG_SZ_MASK,
		      FIELD_PREP(FE_DMA_GLO_L2_SPACE_MASK, 2) |
			      FE_DMA_GLO_PG_SZ_MASK);
	airoha_fe_wr(eth, REG_FE_RST_GLO_CFG,
		     FE_RST_CORE_MASK | FE_RST_GDM3_MBI_ARB_MASK |
			     FE_RST_GDM4_MBI_ARB_MASK);
	udelay(2000);

	/*
	 * Linux maps ring1/ring15 to OQ-1, but this recovery driver only owns
	 * ring0. Route every RX ring to the default OQ-0 so FE traffic is not
	 * stranded on an unmapped alternate queue.
	 */
	airoha_fe_wr(eth, REG_FE_CDM1_OQ_MAP0, 0);
	airoha_fe_wr(eth, REG_FE_CDM1_OQ_MAP1, 0);
	airoha_fe_wr(eth, REG_FE_CDM1_OQ_MAP2, 0);
	airoha_fe_wr(eth, REG_FE_CDM1_OQ_MAP3, 0);

	airoha_fe_pse_ports_init(eth);

	airoha_fe_set(eth, REG_GDM_MISC_CFG,
		      GDM2_RDM_ACK_WAIT_PREF_MASK | GDM2_CHN_VLD_MODE_MASK);
	airoha_fe_rmw(eth, REG_CDM2_FWD_CFG, CDM2_OAM_QSEL_MASK,
		      FIELD_PREP(CDM2_OAM_QSEL_MASK, 0));
	airoha_fe_set(eth, REG_GDM3_FWD_CFG, GDM3_PAD_EN_MASK);
	airoha_fe_set(eth, REG_GDM4_FWD_CFG, GDM4_PAD_EN_MASK);

	airoha_fe_clear(eth, REG_FE_CPORT_CFG, FE_CPORT_QUEUE_XFC_MASK);
	airoha_fe_set(eth, REG_FE_CPORT_CFG, FE_CPORT_PORT_XFC_MASK);

	airoha_fe_rmw(eth, REG_GDM2_CHN_RLS,
		      MBI_RX_AGE_SEL_MASK | MBI_TX_AGE_SEL_MASK,
		      FIELD_PREP(MBI_RX_AGE_SEL_MASK, 3) |
			      FIELD_PREP(MBI_TX_AGE_SEL_MASK, 3));

	/* IFC interferes with the simple recovery datapath, keep it disabled. */
	airoha_fe_clear(eth, REG_FE_CSR_IFC_CFG, FE_IFC_EN_MASK);

	return 0;
}

static void airoha_reset_ext_phys(ofnode mdio_node)
{
	ofnode child;

	ofnode_for_each_subnode(child, mdio_node)
	{
		struct gpio_desc reset = { 0 };
		u32 gpios[3];
		u32 assert_us, deassert_us;
		u32 phy_addr, min_reset_us = 0;
		int gpio_num, gpio_flags;
		int ret;

		phy_addr = ofnode_read_u32_default(child, "reg", U32_MAX);
		if (airoha_rtl8261_is_phy_addr(phy_addr))
			min_reset_us = 200000;

		ret = ofnode_read_u32_array(child, "reset-gpios", gpios,
					    ARRAY_SIZE(gpios));
		if (!ret) {
			gpio_num = gpios[1];
			gpio_flags = gpios[2];
			assert_us = ofnode_read_u32_default(
				child, "reset-assert-us",
				min_reset_us ? min_reset_us : 10000);
			deassert_us = ofnode_read_u32_default(
				child, "reset-deassert-us",
				min_reset_us ? min_reset_us : 10000);
			if (min_reset_us) {
				assert_us = max(assert_us, min_reset_us);
				deassert_us = max(deassert_us, min_reset_us);
			}

			airoha_gpio_direction_output(gpio_num);
			airoha_gpio_set_active_low(gpio_num,
						   !!(gpio_flags & BIT(0)));
			airoha_delay_us(assert_us);
			airoha_gpio_set_active_low(gpio_num,
						   !(gpio_flags & BIT(0)));
			airoha_delay_us(deassert_us);
			continue;
		}

		ret = gpio_request_by_name_nodev(
			child, "reset-gpios", 0, &reset,
			GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
		if (ret)
			continue;

		assert_us = ofnode_read_u32_default(child, "reset-assert-us",
						    min_reset_us ? min_reset_us :
								   10000);
		deassert_us = ofnode_read_u32_default(
			child, "reset-deassert-us",
			min_reset_us ? min_reset_us : 10000);
		if (min_reset_us) {
			assert_us = max(assert_us, min_reset_us);
			deassert_us = max(deassert_us, min_reset_us);
		}

		airoha_delay_us(assert_us);
		dm_gpio_set_value(&reset, 0);
		airoha_delay_us(deassert_us);
		gpio_free_list_nodev(&reset, 1);
	}
}

static void airoha_enable_mdio_pins(struct airoha_eth *eth)
{
	if (!eth->chip_scu_regmap)
		return;

	/*
	 * Match the corrected Linux pinctrl-airoha EN7581 mdio function:
	 * - force MDC/MDIO onto GPIO1/GPIO2 through FORCE_GPIO_EN
	 * - enable MDC master mode on REG_GPIO_2ND_I2C_MODE bit14
	 * - clear the adjacent I2C master bit13, which shares the same mux word
	 *
	 * Earlier U-Boot experiments incorrectly toggled GPIO_SGMII_MDIO_MODE.
	 * The upstream Linux fix notes the bootloader never uses that bit for
	 * external PHY MDIO and instead forces the GPIO pair directly.
	 */
	regmap_update_bits(eth->chip_scu_regmap, SCU_GPIO_2ND_I2C_MODE,
			   SCU_GPIO_MDC_IO_MASTER_MODE |
				   SCU_GPIO_I2C_MASTER_MODE,
			   SCU_GPIO_MDC_IO_MASTER_MODE);
	regmap_update_bits(eth->chip_scu_regmap, SCU_FORCE_GPIO_EN,
			   SCU_FORCE_GPIO1_EN | SCU_FORCE_GPIO2_EN,
			   SCU_FORCE_GPIO1_EN | SCU_FORCE_GPIO2_EN);

	/*
	 * Keep GPIO2 aligned with the vendor DT pinconf state ("output-high")
	 * so the helper/strap pin is driven once the force-gpio mux is active.
	 */
	airoha_gpio_direction_output(2);
	airoha_gpio_set_active_low(2, 0);
}

static void airoha_qdma_reset_rx_desc(struct airoha_queue *q, int index)
{
	struct airoha_qdma_desc *desc;
	uchar *rx_packet;
	u32 val;

	desc = &q->desc[index];
	rx_packet = q->rx_buf + (index * AIROHA_RX_BUF_SIZE);
	index = (index + 1) % q->ndesc;

	/*
	 * Keep RX descriptor programming aligned with upstream U-Boot:
	 * publish the packet buffer itself to hardware and perform the
	 * cache sync in the TX-direction before ownership is handed over
	 * to QDMA. The EN7581 driver is sensitive to this.
	 */
	dma_map_single(rx_packet, AIROHA_RX_BUF_SIZE, DMA_TO_DEVICE);

	WRITE_ONCE(desc->msg0, cpu_to_le32(0));
	WRITE_ONCE(desc->msg1, cpu_to_le32(0));
	WRITE_ONCE(desc->msg2, cpu_to_le32(0));
	WRITE_ONCE(desc->msg3, cpu_to_le32(0));
	WRITE_ONCE(desc->addr, cpu_to_le32(virt_to_phys(rx_packet)));
	WRITE_ONCE(desc->data, cpu_to_le32(index));
	val = FIELD_PREP(QDMA_DESC_LEN_MASK, AIROHA_RX_BUF_SIZE);
	WRITE_ONCE(desc->ctrl, cpu_to_le32(val));

	dma_map_unaligned(desc, sizeof(*desc), DMA_TO_DEVICE);
}

static void airoha_qdma_init_rx_desc(struct airoha_queue *q)
{
	int i;

	for (i = 0; i < q->ndesc; i++)
		airoha_qdma_reset_rx_desc(q, i);
}

static int airoha_qdma_init_rx_queue(struct airoha_queue *q,
				     struct airoha_qdma *qdma, int ndesc)
{
	int qid = q - &qdma->q_rx[0];
	unsigned long dma_addr;
	size_t rx_buf_size;

	q->ndesc = ndesc;
	q->head = 0;

	rx_buf_size = ALIGN(q->ndesc * AIROHA_RX_BUF_SIZE, ARCH_DMA_MINALIGN);
	q->rx_buf = memalign(ARCH_DMA_MINALIGN, rx_buf_size);
	if (!q->rx_buf)
		return -ENOMEM;

	memset(q->rx_buf, 0, rx_buf_size);

	q->desc = dma_alloc_coherent(q->ndesc * sizeof(*q->desc), &dma_addr);
	if (!q->desc)
		return -ENOMEM;

	memset(q->desc, 0, q->ndesc * sizeof(*q->desc));
	dma_map_single(q->desc, q->ndesc * sizeof(*q->desc), DMA_TO_DEVICE);

	airoha_qdma_wr(qdma, REG_RX_RING_BASE(qid), dma_addr);
	airoha_qdma_rmw(qdma, REG_RX_RING_SIZE(qid), RX_RING_SIZE_MASK,
			FIELD_PREP(RX_RING_SIZE_MASK, ndesc));

	airoha_qdma_rmw(qdma, REG_RX_RING_SIZE(qid), RX_RING_THR_MASK,
			FIELD_PREP(RX_RING_THR_MASK, 0));
	airoha_qdma_rmw(qdma, REG_RX_CPU_IDX(qid), RX_RING_CPU_IDX_MASK,
			FIELD_PREP(RX_RING_CPU_IDX_MASK, q->ndesc - 1));
	airoha_qdma_rmw(qdma, REG_RX_DMA_IDX(qid), RX_RING_DMA_IDX_MASK,
			FIELD_PREP(RX_RING_DMA_IDX_MASK, q->head));

	return 0;
}

static int airoha_qdma_init_rx(struct airoha_qdma *qdma)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(qdma->q_rx); i++) {
		int err;

		err = airoha_qdma_init_rx_queue(&qdma->q_rx[i], qdma,
						RX_DSCP_NUM);
		if (err)
			return err;
	}

	return 0;
}

static int airoha_qdma_init_tx_queue(struct airoha_queue *q,
				     struct airoha_qdma *qdma, int size)
{
	int qid = q - &qdma->q_tx[0];
	unsigned long dma_addr;

	q->ndesc = size;
	q->head = 0;

	q->tx_buf = memalign(ARCH_DMA_MINALIGN, AIROHA_MAX_PACKET_SIZE);
	if (!q->tx_buf)
		return -ENOMEM;

	memset(q->tx_buf, 0, AIROHA_MAX_PACKET_SIZE);

	q->desc = dma_alloc_coherent(q->ndesc * sizeof(*q->desc), &dma_addr);
	if (!q->desc)
		return -ENOMEM;

	memset(q->desc, 0, q->ndesc * sizeof(*q->desc));
	dma_map_single(q->desc, q->ndesc * sizeof(*q->desc), DMA_TO_DEVICE);

	airoha_qdma_wr(qdma, REG_TX_RING_BASE(qid), dma_addr);
	airoha_qdma_rmw(qdma, REG_TX_CPU_IDX(qid), TX_RING_CPU_IDX_MASK,
			FIELD_PREP(TX_RING_CPU_IDX_MASK, q->head));
	airoha_qdma_rmw(qdma, REG_TX_DMA_IDX(qid), TX_RING_DMA_IDX_MASK,
			FIELD_PREP(TX_RING_DMA_IDX_MASK, q->head));

	return 0;
}

static int airoha_qdma_tx_irq_init(struct airoha_tx_irq_queue *irq_q,
				   struct airoha_qdma *qdma, int size)
{
	int id = irq_q - &qdma->q_tx_irq[0];
	unsigned long dma_addr;

	irq_q->q = dma_alloc_coherent(size * sizeof(u32), &dma_addr);
	if (!irq_q->q)
		return -ENOMEM;

	memset(irq_q->q, 0xffffffff, size * sizeof(u32));
	irq_q->size = size;
	irq_q->qdma = qdma;

	dma_map_single(irq_q->q, size * sizeof(u32), DMA_TO_DEVICE);

	airoha_qdma_wr(qdma, REG_TX_IRQ_BASE(id), dma_addr);
	airoha_qdma_rmw(qdma, REG_TX_IRQ_CFG(id), TX_IRQ_DEPTH_MASK,
			FIELD_PREP(TX_IRQ_DEPTH_MASK, size));

	return 0;
}

static int airoha_qdma_init_tx(struct airoha_qdma *qdma)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(qdma->q_tx_irq); i++) {
		err = airoha_qdma_tx_irq_init(&qdma->q_tx_irq[i], qdma,
					      IRQ_QUEUE_LEN);
		if (err)
			return err;
	}

	for (i = 0; i < ARRAY_SIZE(qdma->q_tx); i++) {
		err = airoha_qdma_init_tx_queue(&qdma->q_tx[i], qdma,
						TX_DSCP_NUM);
		if (err)
			return err;
	}

	return 0;
}

static int airoha_qdma_init_hfwd_queues(struct airoha_qdma *qdma)
{
	unsigned long dma_addr;
	u32 status;
	int size;

	size = HW_DSCP_NUM * sizeof(struct airoha_qdma_fwd_desc);
	qdma->hfwd.desc = dma_alloc_coherent(size, &dma_addr);
	if (!qdma->hfwd.desc)
		return -ENOMEM;

	memset(qdma->hfwd.desc, 0, size);
	dma_map_single(qdma->hfwd.desc, size, DMA_TO_DEVICE);

	airoha_qdma_wr(qdma, REG_FWD_DSCP_BASE, dma_addr);

	size = AIROHA_MAX_PACKET_SIZE * HW_DSCP_NUM;
	qdma->hfwd.q = dma_alloc_coherent(size, &dma_addr);
	if (!qdma->hfwd.q)
		return -ENOMEM;

	memset(qdma->hfwd.q, 0, size);
	dma_map_single(qdma->hfwd.q, size, DMA_TO_DEVICE);

	airoha_qdma_wr(qdma, REG_FWD_BUF_BASE, dma_addr);

	airoha_qdma_rmw(qdma, REG_HW_FWD_DSCP_CFG,
			HW_FWD_DSCP_PAYLOAD_SIZE_MASK |
				HW_FWD_DSCP_MIN_SCATTER_LEN_MASK,
			FIELD_PREP(HW_FWD_DSCP_PAYLOAD_SIZE_MASK, 0) |
				FIELD_PREP(HW_FWD_DSCP_MIN_SCATTER_LEN_MASK,
					   1));
	airoha_qdma_rmw(qdma, REG_LMGR_INIT_CFG,
			LMGR_INIT_START | LMGR_SRAM_MODE_MASK |
				HW_FWD_DESC_NUM_MASK,
			FIELD_PREP(HW_FWD_DESC_NUM_MASK, HW_DSCP_NUM) |
				LMGR_INIT_START);

	udelay(1000);
	return read_poll_timeout(airoha_qdma_rr, status,
				 !(status & LMGR_INIT_START), USEC_PER_MSEC,
				 30 * USEC_PER_MSEC, qdma, REG_LMGR_INIT_CFG);
}

static int airoha_qdma_hw_init(struct airoha_qdma *qdma)
{
	int i;

	/* clear pending irqs */
	for (i = 0; i < 2; i++)
		airoha_qdma_wr(qdma, REG_INT_STATUS(i), 0xffffffff);

	airoha_qdma_wr(
		qdma, REG_QDMA_GLOBAL_CFG,
		GLOBAL_CFG_CPU_TXR_RR_MASK |
			GLOBAL_CFG_PAYLOAD_BYTE_SWAP_MASK |
			GLOBAL_CFG_IRQ0_EN_MASK | GLOBAL_CFG_TX_WB_DONE_MASK |
			FIELD_PREP(GLOBAL_CFG_MAX_ISSUE_NUM_MASK, 3));

	/* disable qdma rx delay interrupt */
	for (i = 0; i < ARRAY_SIZE(qdma->q_rx); i++) {
		if (!qdma->q_rx[i].ndesc)
			continue;

		airoha_qdma_clear(qdma, REG_RX_DELAY_INT_IDX(i),
				  RX_DELAY_INT_MASK);
	}

	return 0;
}

static void airoha_qdma_sync_rx_head(struct airoha_qdma *qdma,
				     struct airoha_queue *q, int qid)
{
	u16 dma_idx;

	dma_idx = FIELD_GET(RX_RING_DMA_IDX_MASK,
			    airoha_qdma_rr(qdma, REG_RX_DMA_IDX(qid)));
	if (dma_idx < q->ndesc)
		q->head = dma_idx;
}

static int airoha_qdma_init(struct udevice *dev, struct airoha_eth *eth,
			    struct airoha_qdma *qdma)
{
	const char *reg_name;
	int qdma_id = qdma - &eth->qdma[0];
	int err;

	qdma->eth = eth;
	reg_name = qdma_id ? "qdma1" : "qdma0";
	qdma->regs = dev_remap_addr_name(dev, reg_name);
	if (IS_ERR(qdma->regs))
		return PTR_ERR(qdma->regs);

	err = airoha_qdma_init_rx(qdma);
	if (err)
		return err;

	err = airoha_qdma_init_tx(qdma);
	if (err)
		return err;

	err = airoha_qdma_init_hfwd_queues(qdma);
	if (err)
		return err;

	debug("airoha: qdma%d regs=%p tx0=%p rx0=%p\n", qdma_id, qdma->regs,
	      qdma->q_tx[0].desc, qdma->q_rx[0].desc);

	return airoha_qdma_hw_init(qdma);
}

static bool airoha_qdma_ready(struct airoha_qdma *qdma)
{
	uintptr_t tx_desc = (uintptr_t)qdma->q_tx[0].desc;
	uintptr_t rx_desc = (uintptr_t)qdma->q_rx[0].desc;

	if (!qdma->regs || !tx_desc || !rx_desc)
		return false;

	/*
	 * U-Boot runs from DRAM mapped at 0x8000_0000+. If a descriptor pointer
	 * falls below that range it is not a usable virtual address and touching
	 * it will abort before the packet ever reaches hardware.
	 */
	if (tx_desc < 0x80000000UL || rx_desc < 0x80000000UL)
		return false;

	return true;
}

static struct airoha_qdma *airoha_active_qdma(struct airoha_eth *eth)
{
	struct airoha_qdma *qdma1 = &eth->qdma[1];

	if (airoha_gdm4_use_cdm2(eth)) {
		if (airoha_qdma_ready(qdma1))
			return qdma1;

		printf("gdm4_cdm2 enabled: qdma1 unavailable regs=%p tx0=%p rx0=%p\n",
		       qdma1->regs, qdma1->q_tx[0].desc, qdma1->q_rx[0].desc);
	}

	return &eth->qdma[0];
}

static struct airoha_qdma *airoha_active_tx_qdma(struct airoha_eth *eth, u8 fport)
{
	/*
	 * Keep TX on the same CDM/QDMA path selected for the reduced recovery
	 * datapath. For XR1710G 10G recovery the default is CDM1/QDMA0 unless
	 * gdm4_cdm2=1 is explicitly requested.
	 */
	return airoha_active_qdma(eth);
}

static u32 airoha_recovery_tx_msg0(int qid)
{
	/*
	 * Keep descriptor metadata aligned with the actual TX ring, matching the
	 * Linux driver layout used by the GDM4 netdev.
	 */
	return FIELD_PREP(QDMA_ETH_TXMSG_CHAN_MASK, qid / AIROHA_NUM_TX_RING) |
	       FIELD_PREP(QDMA_ETH_TXMSG_QUEUE_MASK, qid % AIROHA_NUM_TX_RING);
}

static int airoha_recovery_tx_qid(struct airoha_eth *eth, u8 fport)
{
	/*
	 * U-Boot recovery does not replicate Linux netdev queue selection, so
	 * stay on the conservative queue0 path unless the user explicitly opts
	 * into the upstream-like CDM2/QDMA1 topology.
	 */
	if (fport == 4 && airoha_gdm4_use_cdm2(eth))
		return 7;

	return 0;
}

static int airoha_hw_init(struct udevice *dev, struct airoha_eth *eth)
{
	int ret, i;

	if (eth->has_switch_rst) {
		ret = reset_assert(&eth->switch_rst);
		if (ret)
			return ret;
	}

	ret = reset_assert_bulk(&eth->rsts);
	if (ret)
		return ret;

	if (eth->scu_regmap) {
		regmap_update_bits(eth->scu_regmap, SCU_SSR3, SCU_ETH_XSI_SEL,
				   SCU_ETH_XSI_USXGMII);
		regmap_update_bits(eth->scu_regmap, SCU_WAN_CONF, SCU_WAN_SEL,
				   SCU_WAN_SEL_USXGMII);
	}

	mdelay(20);

	ret = reset_deassert_bulk(&eth->xsi_rsts);
	if (ret)
		return ret;

	mdelay(20);

	ret = reset_deassert_bulk(&eth->rsts);
	if (ret)
		return ret;

	if (eth->has_switch_rst) {
		ret = reset_deassert(&eth->switch_rst);
		if (ret)
			return ret;
	}

	airoha_eth_gdm4_pcs_init(eth);

	ret = airoha_fe_init(eth);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(eth->qdma); i++) {
		ret = airoha_qdma_init(dev, eth, &eth->qdma[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static void airoha_switch_recovery_runtime_init(struct airoha_eth *eth)
{
	u32 cpu_port_mask = BIT(AIROHA_RECOVERY_SWITCH_CPU_PORT);

	if (!eth->switch_regs)
		return;

	/*
	 * This mirrors the recovery-facing switch setup done at probe time, but
	 * avoids PHY resets and RTL8261 patching. It makes http_recovery
	 * re-enterable after eth_halt()/eth_init().
	 */
	airoha_switch_wr(eth, SWITCH_MFC,
			 FIELD_PREP(SWITCH_BC_FFP, cpu_port_mask) |
				 FIELD_PREP(SWITCH_UNM_FFP, cpu_port_mask) |
				 FIELD_PREP(SWITCH_UNU_FFP, cpu_port_mask));
	airoha_switch_rmw(eth, SWITCH_CFC, SWITCH_CPU_PMAP,
			  FIELD_PREP(SWITCH_CPU_PMAP, cpu_port_mask));
	airoha_switch_rmw(eth, SWITCH_AGC, 0, SWITCH_LOCAL_EN);
	airoha_switch_wr(eth, SWITCH_CPORT_SPTAG_CFG,
			 SWITCH_SW2FE_STAG_EN | SWITCH_FE2SW_STAG_EN);

	airoha_switch_recovery_program_ports(eth, true);
	airoha_switch_fdb_flush(eth);
}

static int airoha_switch_init(struct udevice *dev, struct airoha_eth *eth)
{
	struct airoha_eth_soc_data *data = (void *)dev_get_driver_data(dev);
	ofnode mdio_node, switch_node;
	u32 phy_poll_start, phy_poll_end;
	u32 cpu_port_mask = BIT(6);
	u32 user_port_mask = GENMASK(5, 0);
	u32 switch_pmcr;
	fdt_addr_t addr;
	int port;

	switch_node =
		ofnode_by_compatible(ofnode_null(), data->switch_compatible);
	if (!ofnode_valid(switch_node))
		return -EINVAL;

	addr = ofnode_get_addr(switch_node);
	if (addr == FDT_ADDR_T_NONE)
		return -ENOMEM;

	/* Switch doesn't have a DEV, gets address and setup Flood and CPU port */
	eth->switch_regs = map_sysmem(addr, 0);

	/*
	 * Forward unknown/broadcast traffic to the CPU port only. Without
	 * this, recovery packets may get trapped inside the switch fabric.
	 */
	airoha_switch_wr(eth, SWITCH_MFC,
			 FIELD_PREP(SWITCH_BC_FFP, cpu_port_mask) |
				 FIELD_PREP(SWITCH_UNM_FFP, cpu_port_mask) |
				 FIELD_PREP(SWITCH_UNU_FFP, cpu_port_mask));

	/* Treat port 6 as the dedicated CPU port. */
	airoha_switch_rmw(eth, SWITCH_CFC, SWITCH_CPU_PMAP,
			  FIELD_PREP(SWITCH_CPU_PMAP, cpu_port_mask));
	airoha_switch_rmw(eth, SWITCH_AGC, 0, SWITCH_LOCAL_EN);
	airoha_switch_wr(eth, SWITCH_CPORT_SPTAG_CFG,
			 SWITCH_SW2FE_STAG_EN | SWITCH_FE2SW_STAG_EN);

	switch_pmcr = SWITCH_IPG_CFG_SHORT | SWITCH_MAC_MODE |
		      SWITCH_FORCE_MODE | SWITCH_MAC_TX_EN | SWITCH_MAC_RX_EN |
		      SWITCH_BKOFF_EN | SWITCH_BKPR_EN | SWITCH_FORCE_RX_FC |
		      SWITCH_FORCE_TX_FC | SWITCH_FORCE_SPD_1000 |
		      SWITCH_FORCE_DPX | SWITCH_FORCE_LNK;

	/* Set CPU 6 PMCR */
	airoha_switch_wr(eth, SWITCH_PMCR(6), switch_pmcr);

	/*
	 * XR1710G recovery traffic uses the internal switch user ports that map
	 * to lan2/lan3. The stock reset state leaves their MACs partially
	 * disabled; force them up so broadcast ARP can actually egress.
	 */
	airoha_switch_wr(eth, SWITCH_PMCR(1), switch_pmcr);
	airoha_switch_wr(eth, SWITCH_PMCR(2), switch_pmcr);

	/*
	 * Match Linux switch setup: user ports only talk to the CPU port,
	 * while the CPU port can reach all user ports. The switch-specific
	 * tag stays in the DMA metadata rather than the frame payload.
	 */
	for (port = 0; port <= 6; port++) {
		u32 matrix = port == 6 ? user_port_mask : cpu_port_mask;
		u32 pvc = FIELD_PREP(SWITCH_STAG_VPID, 0x8100) |
			  FIELD_PREP(SWITCH_VLAN_ATTR, SWITCH_VLAN_ATTR_USER);

		if (port == 6)
			pvc |= SWITCH_PORT_SPEC_TAG;

		airoha_switch_rmw(
			eth, SWITCH_PCR(port),
			SWITCH_PORT_MATRIX | SWITCH_PORT_VLAN_MASK,
			FIELD_PREP(SWITCH_PORT_MATRIX, matrix) |
				FIELD_PREP(SWITCH_PORT_VLAN_MASK,
					   SWITCH_PORT_FALLBACK_MODE));
		airoha_switch_rmw(eth, SWITCH_PVC(port),
				  SWITCH_STAG_VPID | SWITCH_VLAN_ATTR |
					  SWITCH_PORT_SPEC_TAG,
				  pvc);
	}

	phy_poll_start = ofnode_read_u32_default(switch_node,
						 "airoha,phy-poll-start", 0x8);
	phy_poll_end = ofnode_read_u32_default(switch_node,
					       "airoha,phy-poll-end", 0xc);
	if (phy_poll_end < phy_poll_start)
		phy_poll_end = phy_poll_start;

	/*
	 * Some boards hang external ports off non-default MDIO addresses.
	 * Allow the DTS to override the switch PHY auto-poll window so link
	 * state is updated for the actual recovery-facing ports.
	 */
	airoha_switch_wr(eth, SWITCH_PHY_POLL,
			 FIELD_PREP(SWITCH_PHY_AP_EN, 0x7f) |
				 FIELD_PREP(SWITCH_EEE_POLL_EN, 0x7f) |
				 SWITCH_PHY_PRE_EN |
				 FIELD_PREP(SWITCH_PHY_END_ADDR, phy_poll_end) |
				 FIELD_PREP(SWITCH_PHY_ST_ADDR,
					    phy_poll_start));

	mdio_node = airoha_switch_mdio_node(dev);
	if (ofnode_valid(mdio_node) &&
	    uclass_get_device_by_ofnode(UCLASS_MDIO, mdio_node,
					&eth->switch_mdio_dev))
		eth->switch_mdio_dev = NULL;

	mdio_node = airoha_ext_mdio_node(dev);
	if (ofnode_valid(mdio_node)) {
		airoha_rtl8261_read_phy_props(eth, mdio_node);
		if (uclass_get_device_by_ofnode(UCLASS_MDIO, mdio_node,
						&eth->mdio_dev))
			eth->mdio_dev = NULL;
	}

	airoha_rtl8261_minimal_init(eth);

	return 0;
}

static int airoha_eth_probe(struct udevice *dev)
{
	struct airoha_eth_soc_data *data = (void *)dev_get_driver_data(dev);
	struct airoha_eth *eth = dev_get_priv(dev);
	ofnode switch_node;
	struct regmap *chip_scu_regmap;
	struct regmap *scu_regmap;
	int i, ret;

	scu_regmap = airoha_get_scu_regmap();
	if (IS_ERR(scu_regmap))
		return PTR_ERR(scu_regmap);
	eth->scu_regmap = scu_regmap;

	chip_scu_regmap = airoha_get_chip_scu_regmap();
	if (IS_ERR(chip_scu_regmap))
		return PTR_ERR(chip_scu_regmap);
	eth->chip_scu_regmap = chip_scu_regmap;

	eth->tx_fport = 1;

	/* It seems by default the FEMEM_SEL is set to Memory (0x1)
	 * preventing any access to any QDMA and FrameEngine register
	 * reporting all 0xdeadbeef (poor cow :( )
	 */
	regmap_write(scu_regmap, SCU_SHARE_FEMEM_SEL, 0x0);
	airoha_enable_mdio_pins(eth);

	eth->fe_regs = dev_remap_addr_name(dev, "fe");
	if (!eth->fe_regs)
		return -ENOMEM;
	eth->eth_pcs_xfi_mac = map_sysmem(ETH_PCS_XFI_MAC_BASE, 0x1000);
	eth->eth_pcs_multi_sgmii = map_sysmem(ETH_PCS_MULTI_SGMII_BASE, 0x450);
	eth->eth_pcs_usxgmii = map_sysmem(ETH_PCS_USXGMII_BASE, 0x338);
	eth->eth_pcs_hsgmii_rate_adp =
		map_sysmem(ETH_PCS_HSGMII_RATE_ADP_BASE, 0x300);
	eth->eth_pcs_xfi_ana = map_sysmem(ETH_PCS_XFI_ANA_BASE, 0x1000);
	eth->eth_pcs_xfi_pma = map_sysmem(ETH_PCS_XFI_PMA_BASE, 0x1000);

	eth->rsts.resets = devm_kcalloc(dev, AIROHA_MAX_NUM_RSTS,
					sizeof(struct reset_ctl), GFP_KERNEL);
	if (!eth->rsts.resets)
		return -ENOMEM;
	eth->rsts.count = AIROHA_MAX_NUM_RSTS;

	eth->xsi_rsts.resets = devm_kcalloc(
		dev, data->num_xsi_rsts, sizeof(struct reset_ctl), GFP_KERNEL);
	if (!eth->xsi_rsts.resets)
		return -ENOMEM;
	eth->xsi_rsts.count = data->num_xsi_rsts;

	ret = reset_get_by_name(dev, "fe", &eth->rsts.resets[0]);
	if (ret)
		return ret;

	ret = reset_get_by_name(dev, "pdma", &eth->rsts.resets[1]);
	if (ret)
		return ret;

	ret = reset_get_by_name(dev, "qdma", &eth->rsts.resets[2]);
	if (ret)
		return ret;

	for (i = 0; i < data->num_xsi_rsts; i++) {
		ret = reset_get_by_name(dev, data->xsi_rsts_names[i],
					&eth->xsi_rsts.resets[i]);
		if (ret)
			return ret;
	}

	switch_node =
		ofnode_by_compatible(ofnode_null(), data->switch_compatible);
	if (ofnode_valid(switch_node)) {
		ret = reset_get_by_index_nodev(switch_node, 0,
					       &eth->switch_rst);
		if (!ret)
			eth->has_switch_rst = true;
		else if (ret != -ENOENT && ret != -ENODEV && ret != -ENOSYS &&
			 ret != -ENOTSUPP)
			return ret;
	}

	ret = airoha_hw_init(dev, eth);
	if (ret)
		return ret;

	switch_node =
		ofnode_by_compatible(ofnode_null(), data->switch_compatible);
	if (ofnode_valid(switch_node)) {
		ofnode mdio_node = airoha_ext_mdio_node(dev);

		if (ofnode_valid(mdio_node))
			airoha_reset_ext_phys(mdio_node);
	}

	return airoha_switch_init(dev, eth);
}

static int airoha_eth_init(struct udevice *dev)
{
	struct airoha_eth *eth = dev_get_priv(dev);
	int i, qid;

	arht_eth_write_hwaddr(dev);
	airoha_gdm4_update_cpu_path(eth);
	eth->last_rx_valid = false;
	eth->last_rx_qdma = 0;
	eth->last_rx_sport = 0;
	eth->last_rx_crsn = 0;
	eth->last_rx_index = 0;
	eth->last_rx_ppe_entry = 0;
	eth->last_rx_len = 0;
	eth->last_rx_ctrl = 0;
	eth->last_rx_msg1 = 0;
	eth->peer_fport_next = 0;
	memset(eth->peer_fport, 0, sizeof(eth->peer_fport));
	qid = 0;

	for (i = 0; i < AIROHA_MAX_NUM_QDMA; i++) {
		struct airoha_qdma *qdma = &eth->qdma[i];
		struct airoha_queue *q;

		if (!airoha_qdma_ready(qdma))
			continue;

		q = &qdma->q_rx[qid];
		q->head = 0;
		airoha_qdma_init_rx_desc(q);
		airoha_qdma_rmw(qdma, REG_RX_CPU_IDX(qid), RX_RING_CPU_IDX_MASK,
				FIELD_PREP(RX_RING_CPU_IDX_MASK, q->ndesc - 1));
		airoha_qdma_set(qdma, REG_QDMA_GLOBAL_CFG,
				GLOBAL_CFG_TX_DMA_EN_MASK |
				GLOBAL_CFG_RX_DMA_EN_MASK);
		/*
		 * On XR1710G the QDMA RX engine reports an initial DMA index of
		 * 2 as soon as RX DMA is enabled, and the first completed
		 * descriptors then appear starting at that slot rather than at
		 * descriptor 0. Track the hardware start position so the U-Boot
		 * RX path looks at the same descriptor the engine will fill.
		 */
		airoha_qdma_sync_rx_head(qdma, q, qid);
	}

	return 0;
}

static void airoha_eth_stop(struct udevice *dev)
{
	struct airoha_eth *eth = dev_get_priv(dev);
	int i;

	for (i = 0; i < AIROHA_MAX_NUM_QDMA; i++) {
		struct airoha_qdma *qdma = &eth->qdma[i];

		if (!airoha_qdma_ready(qdma))
			continue;

		airoha_qdma_clear(qdma, REG_QDMA_GLOBAL_CFG,
				  GLOBAL_CFG_TX_DMA_EN_MASK |
					  GLOBAL_CFG_RX_DMA_EN_MASK);
	}
}

static int airoha_eth_send_on_fport(struct udevice *dev, void *packet,
				     int length, u8 fport)
{
	struct airoha_eth *eth = dev_get_priv(dev);
	struct airoha_qdma *qdma;
	struct airoha_qdma_desc *desc;
	struct airoha_queue *q;
	dma_addr_t dma_addr;
	u32 msg0, msg1;
	int qid, index;
	u32 val;
	int i;
	void *tx_packet = packet;
	int tx_length = length;
	uchar padded_packet[60] __aligned(ARCH_DMA_MINALIGN);

	/*
	 * Linux relies on GDM_PAD_EN for short frames, but for the XR1710G 10G
	 * recovery path we still see no reply to 42-byte ARP requests even after
	 * RX starts working. Manually pad runt frames to 60 bytes so we can rule
	 * out any GDM4/XFI path issue around automatic short-frame padding.
	 */
	if (length < 60) {
		memset(padded_packet, 0, sizeof(padded_packet));
		memcpy(padded_packet, packet, length);
		tx_packet = padded_packet;
		tx_length = sizeof(padded_packet);
	}

	dma_addr = dma_map_single(tx_packet, tx_length, DMA_TO_DEVICE);

	if (fport == 4 && airoha_recovery_dual_service_enabled())
		airoha_gdm4_ensure_ready(eth);

	eth->tx_fport = fport;
	arht_eth_write_hwaddr(dev);
	airoha_gdm4_update_cpu_path(eth);
	qdma = airoha_active_tx_qdma(eth, fport);

	qid = airoha_recovery_tx_qid(eth, fport);
	q = &qdma->q_tx[qid];
	desc = &q->desc[q->head];
	if (q->pending) {
		ret = airoha_qdma_wait_tx_done(desc);
		if (ret) {
			eth->last_tx_ret = ret;
			eth->tx_err++;
			return ret;
		}

		q->pending = false;
		q->head = (q->head + 1) % q->ndesc;
		airoha_qdma_rmw(qdma, REG_IRQ_CLEAR_LEN(0), IRQ_CLEAR_LEN_MASK, 1);
		desc = &q->desc[q->head];
	}
	index = (q->head + 1) % q->ndesc;
	memcpy(q->tx_buf, packet, length);
	if (tx_length > length)
		memset(q->tx_buf + length, 0, tx_length - length);
	dma_addr = dma_map_single(q->tx_buf, tx_length, DMA_TO_DEVICE);

	msg0 = airoha_recovery_tx_msg0(qid);
	msg1 = FIELD_PREP(QDMA_ETH_TXMSG_FPORT_MASK,
			  airoha_encode_tx_fport(fport)) |
	       FIELD_PREP(QDMA_ETH_TXMSG_METER_MASK, 0x7f);

	val = FIELD_PREP(QDMA_DESC_LEN_MASK, tx_length);
	WRITE_ONCE(desc->ctrl, cpu_to_le32(val));
	WRITE_ONCE(desc->addr, cpu_to_le32(dma_addr));
	val = FIELD_PREP(QDMA_DESC_NEXT_ID_MASK, index);
	WRITE_ONCE(desc->data, cpu_to_le32(val));
	WRITE_ONCE(desc->msg0, cpu_to_le32(msg0));
	WRITE_ONCE(desc->msg1, cpu_to_le32(msg1));
	WRITE_ONCE(desc->msg2, cpu_to_le32(0xffff));

	dma_map_unaligned(desc, sizeof(*desc), DMA_TO_DEVICE);
	q->pending = true;

	airoha_qdma_rmw(qdma, REG_TX_CPU_IDX(qid), TX_RING_CPU_IDX_MASK,
			FIELD_PREP(TX_RING_CPU_IDX_MASK, index));

	ret = airoha_qdma_wait_tx_done(desc);
	if (ret) {
		eth->last_tx_ret = ret;
		eth->tx_err++;
		return ret;
	}

	q->pending = false;
	q->head = index;
	airoha_qdma_rmw(qdma, REG_IRQ_CLEAR_LEN(0), IRQ_CLEAR_LEN_MASK, 1);

	return 0;
}

static u8 airoha_pick_packet_tx_fport(struct airoha_eth *eth,
				       const uchar *packet, int length)
{
	u8 fport;

	if (!airoha_recovery_dual_service_enabled() || length < ARP_HLEN)
		return airoha_pick_tx_fport(eth);

	if (airoha_eth_is_multicast_addr(packet))
		return 0;

	fport = airoha_peer_fport_lookup(eth, packet);
	if (fport == 1 && !airoha_recovery_lan_up(eth))
		fport = 0;
	else if (fport == 4) {
		airoha_eth_gdm4_rxlock_workaround(eth);
		if (!airoha_recovery_gdm4_candidate(eth))
			fport = 0;
	}

	if (fport)
		return fport;

	return airoha_pick_tx_fport(eth);
}

static int airoha_eth_send(struct udevice *dev, void *packet, int length)
{
	struct airoha_eth *eth = dev_get_priv(dev);
	bool dual_service = airoha_recovery_dual_service_enabled();
	bool send_lan = false, send_10g = false;
	int attempts = 0, successes = 0;
	int first_err = -EAGAIN;
	u8 fport;
	int ret;

	if (!dual_service || length < ARP_HLEN)
		return airoha_eth_send_on_fport(dev, packet, length,
						airoha_pick_tx_fport(eth));

	if (airoha_eth_is_broadcast_addr(packet) ||
	    airoha_eth_is_multicast_addr(packet)) {
		airoha_eth_gdm4_rxlock_workaround(eth);
		send_lan = airoha_recovery_lan_up(eth);
		send_10g = airoha_recovery_gdm4_candidate(eth);

		if (send_lan) {
			attempts++;
			ret = airoha_eth_send_on_fport(dev, packet, length, 1);
			if (!ret)
				successes++;
			else if (first_err == -EAGAIN)
				first_err = ret;
		}

		if (send_10g) {
			attempts++;
			ret = airoha_eth_send_on_fport(dev, packet, length, 4);
			if (!ret)
				successes++;
			else if (first_err == -EAGAIN)
				first_err = ret;
		}

		if (successes)
			return 0;
		if (attempts)
			return first_err;
	}

	fport = airoha_pick_packet_tx_fport(eth, packet, length);

	return airoha_eth_send_on_fport(dev, packet, length, fport);
}

static void airoha_qdma_recycle_rx_desc(struct airoha_qdma *qdma,
					struct airoha_queue *q, int qid)
{
	/*
	 * Due to cpu cache issue the airoha_qdma_reset_rx_desc() function
	 * will always touch 2 descriptors placed on the same cacheline:
	 *   - if current descriptor is even, then current and next
	 *     descriptors will be touched
	 *   - if current descriptor is odd, then current and previous
	 *     descriptors will be touched
	 *
	 * Thus, to prevent possible destroying of rx queue, we should:
	 *   - do nothing in the even descriptor case,
	 *   - utilize 2 descriptors (current and previous one) in the
	 *     odd descriptor case.
	 *
	 * WARNING: Observations shows that PKTBUFSRX must be even and
	 *          larger than 7 for reliable driver operations.
	 */
	if (q->head & 0x01) {
		airoha_qdma_reset_rx_desc(q, q->head - 1);
		airoha_qdma_reset_rx_desc(q, q->head);

		airoha_qdma_rmw(qdma, REG_RX_CPU_IDX(qid), RX_RING_CPU_IDX_MASK,
				FIELD_PREP(RX_RING_CPU_IDX_MASK, q->head));
	}

	q->head = (q->head + 1) % q->ndesc;
}

static int airoha_eth_recv_qdma(struct airoha_eth *eth, struct airoha_qdma *qdma,
				uchar **packetp)
{
	struct airoha_queue *q;
	struct airoha_qdma_desc *desc;
	dma_addr_t dma_addr;
	u32 desc_ctrl, msg1;
	u8 qdma_id = qdma - &eth->qdma[0];
	u8 sport, crsn;
	u16 ppe_entry, length;
	uchar *packet;
	int qid;
	int n;

	qid = 0;
	q = &qdma->q_rx[qid];
	desc = &q->desc[q->head];

	dma_unmap_unaligned(virt_to_phys(desc), sizeof(*desc), DMA_FROM_DEVICE);

	desc_ctrl = le32_to_cpu(desc->ctrl);
	if (!(desc_ctrl & QDMA_DESC_DONE_MASK)) {
		for (n = 1; n < q->ndesc; n++) {
			u16 idx = (q->head + n) % q->ndesc;

			desc = &q->desc[idx];
			dma_unmap_unaligned(virt_to_phys(desc), sizeof(*desc),
					    DMA_FROM_DEVICE);
			desc_ctrl = le32_to_cpu(desc->ctrl);
			if (!(desc_ctrl & QDMA_DESC_DONE_MASK))
				continue;

			q->head = idx;
			break;
		}

		if (n == q->ndesc)
			return -EAGAIN;
	}

	dma_addr = le32_to_cpu(desc->addr);
	if (dma_addr)
		dma_unmap_single(dma_addr, AIROHA_RX_BUF_SIZE, DMA_FROM_DEVICE);

	length = FIELD_GET(QDMA_DESC_LEN_MASK, desc_ctrl);
	msg1 = le32_to_cpu(desc->msg1);
	sport = FIELD_GET(QDMA_ETH_RXMSG_SPORT_MASK, msg1);
	crsn = FIELD_GET(QDMA_ETH_RXMSG_CRSN_MASK, msg1);
	ppe_entry = FIELD_GET(QDMA_ETH_RXMSG_PPE_ENTRY_MASK, msg1);

	/*
	 * EN7581 reports GDM4 traffic as SPORT 0x18 in RX metadata,
	 * while GDM2 still comes back as SPORT 0x2.
	 */
	eth->tx_fport = airoha_rx_sport_to_fport(sport);
	if (!eth->tx_fport)
		eth->tx_fport = airoha_pick_tx_fport(eth);
	airoha_gdm4_update_cpu_path(eth);

	eth->last_rx_valid = true;
	eth->last_rx_qdma = qdma_id;
	eth->last_rx_sport = sport;
	eth->last_rx_crsn = crsn;
	eth->last_rx_index = q->head;
	eth->last_rx_ppe_entry = ppe_entry;
	eth->last_rx_len = length;
	eth->last_rx_ctrl = desc_ctrl;
	eth->last_rx_msg1 = msg1;

	packet = q->rx_buf + (q->head * AIROHA_RX_BUF_SIZE);
	if (length >= 12)
		airoha_peer_fport_learn(eth, packet + ARP_HLEN,
					 airoha_rx_sport_to_fport(sport));

	/*
	 * The Linux driver does not discard RX frames solely due to
	 * QDMA_DESC_DROP_MASK. On XR1710G 10G recovery we now see
	 * GDM4 traffic arriving with SPORT 0x18 and this bit set,
	 * so keep those frames visible to the network stack instead
	 * of dropping them unconditionally in U-Boot.
	 */
	if ((desc_ctrl & QDMA_DESC_DROP_MASK) &&
	    !(airoha_recovery_accept_gdm4_rx() && sport == 0x18)) {
		airoha_qdma_recycle_rx_desc(qdma, q, qid);
		return -EAGAIN;
	}

	*packetp = packet;

	return length;
}

static int airoha_eth_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct airoha_eth *eth = dev_get_priv(dev);
	struct airoha_qdma *active = airoha_active_qdma(eth);
	int i, ret;

	for (i = 0; i < AIROHA_MAX_NUM_QDMA; i++) {
		struct airoha_qdma *qdma = i ? &eth->qdma[active == &eth->qdma[0]] :
					      active;

		if (!airoha_qdma_ready(qdma))
			continue;

		ret = airoha_eth_recv_qdma(eth, qdma, packetp);
		if (ret != -EAGAIN)
			return ret;
	}

	return -EAGAIN;
}

static int arht_eth_free_pkt(struct udevice *dev, uchar *packet, int length)
{
	struct airoha_eth *eth = dev_get_priv(dev);
	struct airoha_qdma *qdma = airoha_active_qdma(eth);
	struct airoha_queue *q;
	int qid;

	if (!packet)
		return 0;

	if (eth->last_rx_valid && eth->last_rx_qdma < AIROHA_MAX_NUM_QDMA &&
	    airoha_qdma_ready(&eth->qdma[eth->last_rx_qdma]))
		qdma = &eth->qdma[eth->last_rx_qdma];

	qid = 0;
	q = &qdma->q_rx[qid];
	airoha_qdma_recycle_rx_desc(qdma, q, qid);

	return 0;
}

static int arht_eth_write_hwaddr(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct airoha_eth *eth = dev_get_priv(dev);
	unsigned char *mac = pdata->enetaddr;
	u32 macaddr_lsb, macaddr_msb;
	u32 fe_mac_h, fe_mac_l;

	macaddr_lsb = FIELD_PREP(SMACCR0_MAC2, mac[2]) |
		      FIELD_PREP(SMACCR0_MAC3, mac[3]) |
		      FIELD_PREP(SMACCR0_MAC4, mac[4]) |
		      FIELD_PREP(SMACCR0_MAC5, mac[5]);
	macaddr_msb = FIELD_PREP(SMACCR1_MAC1, mac[1]) |
		      FIELD_PREP(SMACCR1_MAC0, mac[0]);

	/* Set MAC for Switch */
	airoha_switch_wr(eth, SWITCH_SMACCR0, macaddr_lsb);
	airoha_switch_wr(eth, SWITCH_SMACCR1, macaddr_msb);

	/*
	 * External GDM2/GDM4 ports use FE WAN/LAN MAC registers rather than
	 * switch SMACCR*, so program both views from the same netdev address.
	 */
	fe_mac_h = (mac[0] << 16) | (mac[1] << 8) | mac[2];
	fe_mac_l = (mac[3] << 16) | (mac[4] << 8) | mac[5];

	airoha_fe_wr(eth, REG_FE_LAN_MAC_H, fe_mac_h);
	airoha_fe_wr(eth, REG_FE_MAC_LMIN(REG_FE_LAN_MAC_H), fe_mac_l);
	airoha_fe_wr(eth, REG_FE_MAC_LMAX(REG_FE_LAN_MAC_H), fe_mac_l);
	airoha_fe_wr(eth, REG_FE_WAN_MAC_H, fe_mac_h);
	airoha_fe_wr(eth, REG_FE_MAC_LMIN(REG_FE_WAN_MAC_H), fe_mac_l);
	airoha_fe_wr(eth, REG_FE_MAC_LMAX(REG_FE_WAN_MAC_H), fe_mac_l);

	if (eth->eth_pcs_xfi_mac) {
		airoha_rmw(eth->eth_pcs_xfi_mac, PCS_XFI_MACADDRH,
			   PCS_XFI_MACADDRH_MASK,
			   FIELD_PREP(PCS_XFI_MACADDRH_MASK,
				      (mac[0] << 8) | mac[1]));
		airoha_rmw(eth->eth_pcs_xfi_mac, PCS_XFI_MACADDRL,
			   PCS_XFI_MACADDRL_MASK,
			   FIELD_PREP(PCS_XFI_MACADDRL_MASK,
				      (mac[2] << 24) | (mac[3] << 16) |
					      (mac[4] << 8) | mac[5]));
	}

	return 0;
}

static int airoha_eth_bind(struct udevice *dev)
{
	ofnode mdio_node;
	struct udevice *mdio_dev;
	int ret;

	mdio_node = airoha_switch_mdio_node(dev);
	if (ofnode_valid(mdio_node) && CONFIG_IS_ENABLED(MDIO_MT7531_MMIO)) {
		ret = device_bind_driver_to_node(dev, "mt7531-mdio-mmio",
						 "switch-mdio", mdio_node,
						 &mdio_dev);
		if (ret)
			debug("Warning: failed to bind switch mdio controller\n");
	}

	mdio_node = airoha_ext_mdio_node(dev);
	if (ofnode_valid(mdio_node) && CONFIG_IS_ENABLED(MDIO_AIROHA_ARHT)) {
		ret = device_bind_driver_to_node(dev, "airoha-arht-mdio",
						 "ext-mdio", mdio_node,
						 &mdio_dev);
		if (ret)
			debug("Warning: failed to bind external mdio controller\n");
	}

	return 0;
}

static const struct airoha_eth_soc_data en7523_data = {
	.xsi_rsts_names = en7523_xsi_rsts_names,
	.num_xsi_rsts = ARRAY_SIZE(en7523_xsi_rsts_names),
	.switch_compatible = "airoha,en7523-switch",
};

static const struct airoha_eth_soc_data en7581_data = {
	.xsi_rsts_names = en7581_xsi_rsts_names,
	.num_xsi_rsts = ARRAY_SIZE(en7581_xsi_rsts_names),
	.switch_compatible = "airoha,en7581-switch",
};

static const struct udevice_id airoha_eth_ids[] = {
	{
		.compatible = "airoha,en7523-eth",
		.data = (ulong)&en7523_data,
	},
	{
		.compatible = "airoha,en7581-eth",
		.data = (ulong)&en7581_data,
	},
	{}
};

static const struct eth_ops airoha_eth_ops = {
	.start = airoha_eth_init,
	.stop = airoha_eth_stop,
	.send = airoha_eth_send,
	.recv = airoha_eth_recv,
	.free_pkt = arht_eth_free_pkt,
	.write_hwaddr = arht_eth_write_hwaddr,
};

U_BOOT_DRIVER(airoha_eth) = {
	.name = "airoha-eth",
	.id = UCLASS_ETH,
	.of_match = airoha_eth_ids,
	.probe = airoha_eth_probe,
	.bind = airoha_eth_bind,
	.ops = &airoha_eth_ops,
	.priv_auto = sizeof(struct airoha_eth),
	.plat_auto = sizeof(struct eth_pdata),
};

static int do_rtl8261_diag(struct cmd_tbl *cmdtp, int flag, int argc,
			   char *const argv[])
{
	struct udevice *dev = eth_get_dev();
	struct airoha_eth *eth;

	if (!dev) {
		printf("rtl8261: no active ethernet device\n");
		return CMD_RET_FAILURE;
	}

	eth = dev_get_priv(dev);
	if (!eth) {
		printf("rtl8261: missing driver context\n");
		return CMD_RET_FAILURE;
	}

	airoha_eth_gdm4_diag(eth, "cmd");

	return 0;
}

U_BOOT_CMD(
	rtl8261_diag, 1, 1, do_rtl8261_diag,
	"dump rtl8261/usxgmii/gdm4 diagnostic state",
	""
);
