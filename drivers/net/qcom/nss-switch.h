/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2016-2019, 2021, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __NSS_SWITCH_H__
#define __NSS_SWITCH_H__

/* System includes - alphabetically ordered */
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm-generic/gpio.h>
#include <clk.h>
#include <cpu_func.h>
#include <dm/device_compat.h>
#include <dm/pinctrl.h>
#include <dt-bindings/net/qcom_ipqsoc.h>
#include <errno.h>
#include <fdtdec.h>
#include <i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <malloc.h>
#include <memalign.h>
#include <miiphy.h>
#include <net.h>
#include <phy.h>
#include <regmap.h>
#include <reset.h>
#include <serial.h>
#include <syscon.h>

#ifdef CONFIG_NSS_PPE_V4
#include "nss_ppe_v4.h"
#endif

/* PPE Bridge Control Register Layout Types */
enum ppe_bridge_ctrl_layout {
	PPE_BRIDGE_CTRL_LAYOUT_UNKNOWN = 0,
	PPE_V2,
	PPE_V4,
};

/*
 * ========================================================================
 * CSR Version/Method Definitions
 * ========================================================================
 */

enum csr_version {
	CSR_VERSION_V1 = 1,
	CSR_VERSION_V2 = 2,
};

/*
 * reset_version - USXGMII port reset mechanism selector
 *
 * RESET_VERSION_V1:
 *   Uses VR_XS_PCS_DIG_CTRL1 USRA_RST (bit 10) - active-HIGH, self-clearing.
 *   For UQXGMII/UDXGMII also resets per-channel VR_MII_DIG_CTRL1 USRA_RST_MII.
 *
 * RESET_VERSION_V2:
 *   Uses QP_USXG_RESET (0x630) active-LOW RST_N bits.
 *   Requires explicit assert (clear bit) then de-assert (set bit).
 */
enum reset_version {
	RESET_VERSION_V1 = 1,
	RESET_VERSION_V2 = 2,
};

/* UNIPHY_MODE_CTRL (CSR0: 0x46c) */
union uniphy_mode_ctrl_u {
	u32 val;
	struct {
		u32 ch0_autoneg_mode:1;     /* [0] Autoneg mode enable */
		u32 ch1_ch0_sgmii:1;        /* [1] CH1/CH0 SGMII select */
		u32 ch4_ch1_0_sgmii:1;      /* [2] CH4/CH1_0 SGMII select */
		u32 sgmii_even_low:1;       /* [3] SGMII even low */
		u32 ch0_mode_ctrl_25m:3;    /* [6:4] CH0 mode control */
		u32 xpcs_mode_12p5g:1;      /* [7] XPCS 12.5G mode */
		u32 ch0_qsgmii_sgmii:1;     /* [8] CH0 QSGMII/SGMII select */
		u32 ch0_psgmii_qsgmii:1;    /* [9] CH0 PSGMII/QSGMII select */
		u32 sg_mode:1;              /* [10] SG mode enable */
		u32 sgplus_mode:1;          /* [11] SGMII+ mode enable */
		u32 xpcs_mode:1;            /* [12] XPCS mode enable */
		u32 usxg_en:1;              /* [13] USXGMII enable */
		u32 xlgpcs_en:1;            /* [14] XLGPCS enable */
		u32 sw_v17_v18:1;           /* [15] SW version select */
		u32 _reserved0:16;          /* [31:16] Reserved */
	} bf;
};

#define UPDATE_EDMA_CONFIG(_src, _dest)					\
	do {								\
		typeof(_src) __src = (_src);				\
		typeof(_dest) __dest = (_dest);				\
		__dest->txdesc_ring_start = __src->txdesc_ring_start;	\
		__dest->txdesc_rings = __src->txdesc_rings;		\
		__dest->txdesc_ring_end = __src->txdesc_ring_end;	\
		__dest->txcmpl_ring_start = __src->txcmpl_ring_start;	\
		__dest->txcmpl_rings = __src->txcmpl_rings;		\
		__dest->txcmpl_ring_end = __src->txcmpl_ring_end;	\
		__dest->rxfill_ring_start = __src->rxfill_ring_start;	\
		__dest->rxfill_rings = __src->rxfill_rings;		\
		__dest->rxfill_ring_end = __src->rxfill_ring_end;	\
		__dest->rxdesc_ring_start = __src->rxdesc_ring_start;	\
		__dest->rxdesc_rings = __src->rxdesc_rings;		\
		__dest->rxdesc_ring_end = __src->rxdesc_ring_end;	\
		__dest->max_txcmpl_rings = __src->max_txcmpl_rings;	\
		__dest->max_txdesc_rings = __src->max_txdesc_rings;	\
		__dest->max_rxdesc_rings = __src->max_rxdesc_rings;	\
		__dest->max_rxfill_rings = __src->max_rxfill_rings;	\
		__dest->max_ports = __src->ports;			\
		__dest->start_ports = __src->start_ports;		\
		__dest->tx_map = __src->tx_map;				\
		__dest->rx_map = __src->rx_map;				\
	} while (0)

#define	WRITE_REG_ARRAY(_base, _offset, _size, _val, _count)		\
	do {								\
		int _i;							\
		for (_i = 0; _i < (_count); ++_i)			\
			writel((_val), ((_base) + (_offset) + (_i * (_size))));\
	} while (0)

#define UPDATE_ACL_SET(_base, _var1, _var2, _var3, _var4, _var5, _var6,	\
			_var7, _var8, _var9)				\
			do {						\
				typeof(_base) *__base = &(_base);	\
				__base->reg_base = (_var1);		\
				__base->rule_id = (_var2);		\
				__base->rule_type = (_var3);		\
				__base->field0 = (_var4);		\
				__base->field1 = (_var5);		\
				__base->mask = (_var6);			\
				__base->permit = (_var7);		\
				__base->deny = (_var8);			\
				__base->ipo_cnt = (_var9);		\
			} while (0)

#define LINK_STATUS				BIT(7)
#define DUPLEX					BIT(5)
#define SPEED					(BIT(0) | BIT(1) | BIT(2))

#define CLKOUT_50M_CTRL_OPTION			0x610

#define EDMA_SW_VER_1_ID			0x01
#define EDMA_SW_VER_2_ID			0x02
#define EDMA_SW_VER_3_ID			0x03

#define SKU_ENABLED				0x1
#define SKU_DISABLED				0x0

/* Number of descriptors in each ring is defined with below macro */
#ifdef CONFIG_ETH_LOW_MEM
#define EDMA_TX_RING_SIZE			32
#define EDMA_RX_RING_SIZE			32
#else
#define EDMA_TX_RING_SIZE			128
#define EDMA_RX_RING_SIZE			128
#endif
#define EDMA_TX_BUFF_SIZE			2048
#define EDMA_RX_BUFF_SIZE			2048

/*
 * QSDK EDMA v2 pads Ethernet frames in hardware, but still rejects very small
 * TX descriptors before the port padding stage.
 */
#define EDMA_TX_MIN_PKT_SIZE			49

/* Number of byte in a descriptor is defined with below macros for each of
 * the rings respectively
 */
#define EDMA_TXDESC_DESC_SIZE	(sizeof(struct ipq_edma_txdesc_desc))
#define EDMA_TXCMPL_DESC_SIZE	(sizeof(struct ipq_edma_txcmpl_desc))
#define EDMA_RXDESC_DESC_SIZE	(sizeof(struct ipq_edma_rxdesc_desc))
#define EDMA_RXFILL_DESC_SIZE	(sizeof(struct ipq_edma_rxfill_desc))
#define EDMA_RX_SEC_DESC_SIZE	(sizeof(struct ipq_edma_rx_sec_desc))
#define EDMA_TX_SEC_DESC_SIZE	(sizeof(struct ipq_edma_tx_sec_desc))

#define EDMA_GET_DESC(R, i, type) (&(((type *)((R)->desc))[i]))
#define EDMA_RXFILL_DESC(R, i)	EDMA_GET_DESC(R, i, struct ipq_edma_rxfill_desc)
#define EDMA_RXDESC_DESC(R, i)	EDMA_GET_DESC(R, i, struct ipq_edma_rxdesc_desc)
#define EDMA_TXDESC_DESC(R, i)	EDMA_GET_DESC(R, i, struct ipq_edma_txdesc_desc)
#define EDMA_TXCMPL_DESC(R, i)	EDMA_GET_DESC(R, i, struct ipq_edma_txcmpl_desc)

/*
 * ========================================================================
 * CSR Address Encoding Macros
 * ========================================================================
 */

#define UNIPHY_CSR_BLOCK_SHIFT		24
#define UNIPHY_CSR_BLOCK_MASK		0xFF000000
#define UNIPHY_REG_ADDR_MASK		0x00FFFFFF

/* CSR type encoding */
#define CSR0_ADDR(addr)		((0x00 << UNIPHY_CSR_BLOCK_SHIFT) | (addr))
#define CSR1_ADDR(addr)		((0x01 << UNIPHY_CSR_BLOCK_SHIFT) | (addr))
#define CSR2_ADDR(addr)		((0x02 << UNIPHY_CSR_BLOCK_SHIFT) | (addr))
/* uniphy CSR details*/
#define UNIPHY_PHY_SHIFT	16

/*
 * ========================================================================
 * V2 (New) CSR Constants
 * ========================================================================
 */

/* V2 CSR indirect registers */
#define V2_CSR1_INDIRECT_REG	0x43FC
#define V2_CSR2_INDIRECT_REG	0x83FC

/* V2 CSR data offsets */
#define V2_CSR1_DATA_OFFSET	0x10
#define V2_CSR2_DATA_OFFSET	0x20

/*
 * ========================================================================
 * V1 (Legacy) CSR Constants
 * ========================================================================
 */

/* V1 uses CSR2 method (0x83FC, 0x20) - this is what old u-boot used */
#define V1_CSR_INDIRECT_REG	0x83FC
#define V1_CSR_DATA_OFFSET	0x20

/* Low address mask for indirect access */
#define CSR_INDIRECT_LOW_ADDR	0xFF

/* EDMA register structures - configurable register layout */
struct edma_global_regs {
	u32 mas_ctrl;
	u32 port_ctrl;
	u32 rxdesc2fill_map_0;
	u32 rxdesc2fill_map_1;
	u32 rxdesc2fill_map_2;
	u32 dmar_ctrl;
	u32 misc_int_stat;
	u32 misc_int_mask;
	u32 txdesc2cmpl_map_0;
	u32 txdesc2cmpl_map_1;
	u32 txdesc2cmpl_map_2;
	u32 txdesc2cmpl_map_3;
	u32 txdesc2cmpl_map_4;
	u32 txdesc2cmpl_map_5;
	u32 qid2rid_table_base;
};

struct edma_ring_regs {
	u32 base_addr;
	u32 prod_idx;
	u32 cons_idx;
	u32 ring_size;
	u32 ctrl;
	u32 base_addr2;
	u32 base_addr_high;
	u32 base_addr2_high;
	u32 ring_en;
	u32 int_stat;
	u32 int_mask;
	u32 int_ctrl;
	u32 fc_thre;
};

struct edma_reg_offsets {
	struct edma_global_regs global;
	struct {
		u32 base_offset;
		u32 ring_increment;
		struct edma_ring_regs offsets;
	} txdesc;
	struct {
		u32 base_offset;
		u32 ring_increment;
		struct edma_ring_regs offsets;
	} rxfill;
	struct {
		u32 base_offset;
		u32 ring_increment;
		struct edma_ring_regs offsets;
	} rxdesc;
	struct {
		u32 base_offset;
		u32 ring_increment;
		struct edma_ring_regs offsets;
	} txcmpl;
	struct {
		u32 base_offset;
		u32 increment;
	} qid2rid;
};

/* EDMA register bit masks and shifts - configurable per SoC */

/* Unified ring register structure - combines offset and mask for each register
 */
struct edma_ring_reg {
	u32 offset;		/* Register offset */
	u32 mask;		/* Register mask */
	u32 shift;		/* Bit shift (if applicable) */
};

/* Unified TXDESC ring configuration
 */
struct edma_txdesc_ring_cfg {
	u32 base_offset;
	u32 ring_increment;

	/* Register configurations with masks */
	struct edma_ring_reg base_addr;
	struct edma_ring_reg base_addr_high;
	struct edma_ring_reg base_addr2;
	struct edma_ring_reg base_addr2_high;
	struct edma_ring_reg prod_idx;
	struct edma_ring_reg cons_idx;
	struct edma_ring_reg ring_size;
	struct edma_ring_reg ctrl;
	struct edma_ring_reg ring_en;
	struct edma_ring_reg int_stat;
	struct edma_ring_reg int_mask;
	struct edma_ring_reg int_ctrl;
	struct edma_ring_reg fc_thre;

	/* TXDESC-specific masks */
	u32 tx_en;
	u32 buf_hi_add_mask;
	u32 data_offset_mask;
	u32 data_offset_shift;
	u32 data_length_mask;
	u32 data_length_shift;
};

/* Unified TXCMPL ring configuration
 */
struct edma_txcmpl_ring_cfg {
	u32 base_offset;
	u32 ring_increment;

	/* Register configurations with masks */
	struct edma_ring_reg base_addr;
	struct edma_ring_reg base_addr_high;
	struct edma_ring_reg base_addr2;
	struct edma_ring_reg base_addr2_high;
	struct edma_ring_reg prod_idx;
	struct edma_ring_reg cons_idx;
	struct edma_ring_reg ring_size;
	struct edma_ring_reg ctrl;
	struct edma_ring_reg ring_en;
	struct edma_ring_reg int_stat;
	struct edma_ring_reg int_mask;
	struct edma_ring_reg int_ctrl;
	struct edma_ring_reg fc_thre;

	/* TXCMPL-specific masks */
	u32 ring_int_status_mask;
};

/* Unified RXFILL ring configuration
 */
struct edma_rxfill_ring_cfg {
	u32 base_offset;
	u32 ring_increment;

	/* Register configurations with masks */
	struct edma_ring_reg base_addr;
	struct edma_ring_reg base_addr_high;
	struct edma_ring_reg base_addr2;
	struct edma_ring_reg base_addr2_high;
	struct edma_ring_reg prod_idx;
	struct edma_ring_reg cons_idx;
	struct edma_ring_reg ring_size;
	struct edma_ring_reg buf_size;		/* Combined ring_size + buf_size */
	struct edma_ring_reg ctrl;
	struct edma_ring_reg ring_en;
	struct edma_ring_reg int_stat;
	struct edma_ring_reg int_mask;
	struct edma_ring_reg int_ctrl;
	struct edma_ring_reg fc_thre;

	/* RXFILL-specific masks */
	u32 buf_hi_add_mask;
	u32 ring_int_status_mask;
};

/* Unified RXDESC ring configuration
 */
struct edma_rxdesc_ring_cfg {
	u32 base_offset;
	u32 ring_increment;

	/* Register configurations with masks */
	struct edma_ring_reg base_addr;
	struct edma_ring_reg base_addr_high;
	struct edma_ring_reg base_addr2;
	struct edma_ring_reg base_addr2_high;
	struct edma_ring_reg prod_idx;
	struct edma_ring_reg cons_idx;
	struct edma_ring_reg ring_size;
	struct edma_ring_reg pl_offset;		/* Payload offset in ring_size reg */
	struct edma_ring_reg ctrl;
	struct edma_ring_reg ring_en;
	struct edma_ring_reg int_stat;
	struct edma_ring_reg int_mask;
	struct edma_ring_reg int_ctrl;
	struct edma_ring_reg fc_thre;

	/* RXDESC-specific masks */
	u32 rx_en;
	u32 srcinfo_type_mask;
	u32 pkt_size_mask;
	u32 pkt_size_shift;
	u32 ring_int_status_mask;
	u32 portnum_bits;
};

/* Unified global register configuration
 */
struct edma_global_cfg {
	struct edma_ring_reg mas_ctrl;
	struct edma_ring_reg port_ctrl;
	struct edma_ring_reg rxdesc2fill_map_0;
	struct edma_ring_reg rxdesc2fill_map_1;
	struct edma_ring_reg rxdesc2fill_map_2;
	struct edma_ring_reg dmar_ctrl;
	struct edma_ring_reg misc_int_stat;
	struct edma_ring_reg misc_int_mask;
	struct edma_ring_reg txdesc2cmpl_map_0;
	struct edma_ring_reg txdesc2cmpl_map_1;
	struct edma_ring_reg txdesc2cmpl_map_2;
	struct edma_ring_reg txdesc2cmpl_map_3;
	struct edma_ring_reg txdesc2cmpl_map_4;
	struct edma_ring_reg txdesc2cmpl_map_5;
	u32 qid2rid_table_base;
};

/* Unified EDMA hardware configuration
 * This structure combines all register offsets with their associated masks
 */
struct edma_hw_cfg {
	struct edma_global_cfg global;
	struct edma_txdesc_ring_cfg txdesc;
	struct edma_txcmpl_ring_cfg txcmpl;
	struct edma_rxfill_ring_cfg rxfill;
	struct edma_rxdesc_ring_cfg rxdesc;

	/* QID2RID configuration */
	u32 qid2rid_base_offset;
	u32 qid2rid_increment;

	/* Interrupt control configuration - independent from descriptor rings */
	u32 int_ctrl_base_offset;
	u32 int_ctrl_ring_increment;
	u32 int_ctrl_reg_offset;

	/* Hardware-specific parameters - configurable per SoC */
	u32 ring_dma_mask;
	u32 rx_ring_size;
	u32 tx_ring_size;
	u32 rx_buff_size;
	u32 tx_buff_size;
	u32 rxfill_desc_size;
	u32 rxdesc_desc_size;
	u32 txdesc_desc_size;
	u32 txdesc_sec_desc_size;
	u32 txcmpl_desc_size;

	/* Interrupt masks - configurable per SoC */
	u32 rxfill_int_mask;
	u32 rxdesc_int_mask;
	u32 txcmpl_int_mask;
	u32 misc_intr_mask;
	u32 rx_payload_offset;

	/* Common masks used across multiple rings */
	u32 tx_int_mask;
	u32 rx_ne_int_en;
	u32 tx_ne_int_en;

	/* Destination port configuration */
	u32 dst_port_type;
	u32 dst_port_type_shift;
	u32 dst_port_type_mask;
	u32 dst_port_id_shift;
	u32 dst_port_id_mask;

	/* Chip-specific register initialization values */
	u32 port_ctrl_init_val;	/* Value to write to PORT_CTRL register */

	/* DMAR_CTRL configurable values - chip-specific differences */
	u32 dmar_txdata_outstanding_num;	/* TXDATA outstanding number */
	u32 dmar_txdesc_outstanding_num;	/* TXDESC outstanding number */
	u32 dmar_rxfill_outstanding_num;	/* RXFILL outstanding number */

	/* DMAR_CTRL bit field configuration - differs between chip variants */
	u32 dmar_txdata_mask;			/* Mask for TXDATA field */
	u32 dmar_txdata_shift;			/* Shift for TXDATA field */
	u32 dmar_txdesc_mask;			/* Mask for TXDESC field */
	u32 dmar_txdesc_shift;			/* Shift for TXDESC field */
	u32 dmar_rxfill_mask;			/* Mask for RXFILL field */
	u32 dmar_rxfill_shift;			/* Shift for RXFILL field */
};

/* Helper functions for register access using configurable offsets */
static inline u32 edma_reg_addr(phys_addr_t base, u32 offset)
{
	return base + offset;
}

static inline phys_addr_t edma_ring_reg_addr(phys_addr_t base,
					      u32 ring_base_offset,
					      u32 ring_increment,
					      u32 ring_id,
					      u32 reg_offset)
{
	return base + ring_base_offset + (ring_id * ring_increment) +
	       reg_offset;
}

static inline phys_addr_t edma_qid2rid_addr(phys_addr_t base,
					     const struct edma_hw_cfg *hw_cfg,
					     u32 qid)
{
	return base + hw_cfg->qid2rid_base_offset + (qid * 4);
}

/* New unified helper functions for cleaner register access */
static inline phys_addr_t edma_unified_ring_addr(phys_addr_t base,
						  u32 ring_base_offset,
						  u32 ring_increment,
						  u32 ring_id,
						  struct edma_ring_reg *reg)
{
	return base + ring_base_offset + (ring_id * ring_increment) +
	       reg->offset;
}

static inline u32 edma_unified_read_masked(phys_addr_t addr, struct edma_ring_reg *reg)
{
	return readl(addr) & reg->mask;
}

static inline void edma_unified_write_masked(phys_addr_t addr, u32 val, struct edma_ring_reg *reg)
{
	u32 current = readl(addr);

	current = (current & ~reg->mask) | (val & reg->mask);
	writel(current, addr);
}

/* EDMA register */
#define EDMA_REG_MAS_CTRL		0x0
#define EDMA_REG_PORT_CTRL		0x4
#define EDMA_REG_RXDESC2FILL_MAP_0	0x14
#define EDMA_REG_RXDESC2FILL_MAP_1	0x18
#define EDMA_REG_RXDESC2FILL_MAP_2	0x1c
#define EDMA_REG_DMAR_CTRL		0x48
#define EDMA_REG_MISC_INT_STAT		0x5c
#define EDMA_REG_MISC_INT_MASK		0x60
#define EDMA_REG_TX_TIMEOUT_THRESH	0x6c
#define EDMA_REG_TXDESC2CMPL_MAP_0	0x8c
#define EDMA_REG_TXDESC2CMPL_MAP_1	0x90
#define EDMA_REG_TXDESC2CMPL_MAP_2	0x94
#define EDMA_REG_TXDESC2CMPL_MAP_3	0x98
#define EDMA_REG_TXDESC2CMPL_MAP_4	0x9c
#define EDMA_REG_TXDESC2CMPL_MAP_5	0xa0

#define EDMA_REG_TXDESC_BA(n)		(0x1000 + (0x1000 * (n)))
#define EDMA_REG_TXDESC_PROD_IDX(n)	(0x1004 + (0x1000 * (n)))
#define EDMA_REG_TXDESC_CONS_IDX(n)	(0x1008 + (0x1000 * (n)))
#define EDMA_REG_TXDESC_RING_SIZE(n)	(0x100c + (0x1000 * (n)))
#define EDMA_REG_TXDESC_CTRL(n)		(0x1010 + (0x1000 * (n)))
#define EDMA_REG_TXDESC_BA2(n)		(0x1014 + (0x1000 * (n)))
#define EDMA_REG_TXDESC_BA_HIGH(n)	(0x1018 + (0x1000 * (n)))
#define EDMA_REG_TXDESC_BA2_HIGH(n)	(0x101c + (0x1000 * (n)))

#define EDMA_REG_RXFILL_BA(n)		(0x29000 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_PROD_IDX(n)	(0x29004 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_CONS_IDX(n)	(0x29008 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_RING_SIZE(n)	(0x2900c + (0x1000 * (n)))
#define EDMA_REG_RXFILL_RING_SIZE_V2(n)	(0x29010 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_RING_EN(n)	(0x2901c + (0x1000 * (n)))
#define EDMA_REG_RXFILL_DISABLE(n)	(0x29020 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_DISABLE_DONE(n)	(0x29024 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_INT_STAT(n)	(0x31000 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_INT_MASK(n)	(0x31004 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_BA_HIGH(n)	(0x29028 + (0x1000 * (n)))
#define EDMA_REG_RXFILL_INDEX_RESET(n)	(0x2902c + (0x1000 * (n)))

#define EDMA_REG_RXDESC_BA(n)		(0x39000 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_PROD_IDX(n)	(0x39004 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_CONS_IDX(n)	(0x39008 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_RING_SIZE(n)	(0x3900c + (0x1000 * (n)))
#define EDMA_REG_RXDESC_FC_THRE(n)	(0x39010 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_CTRL(n)		(0x39018 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_DISABLE(n)	(0x39020 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_DISABLE_DONE(n)	(0x39024 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_BA2(n)		(0x39028 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_INT_STAT(n)	(0x59000 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_INT_MASK(n)	(0x59004 + (0x1000 * (n)))
#define EDMA_REG_RX_INT_CTRL(n)		(0x5900c + (0x1000 * (n)))
#define EDMA_REG_RXDESC_BA_HIGH(n)	(0x3902c + (0x1000 * (n)))
#define EDMA_REG_RXDESC_BA2_HIGH(n)	(0x39030 + (0x1000 * (n)))
#define EDMA_REG_RXDESC_RESET(n)	(0x39034 + (0x1000 * (n)))

#define EDMA_REG_TXCMPL_BA(n)		(0x79000 + (0x1000 * (n)))
#define EDMA_REG_TXCMPL_PROD_IDX(n)	(0x79004 + (0x1000 * (n)))
#define EDMA_REG_TXCMPL_CONS_IDX(n)	(0x79008 + (0x1000 * (n)))
#define EDMA_REG_TXCMPL_RING_SIZE(n)	(0x7900c + (0x1000 * (n)))
#define EDMA_REG_TXCMPL_CTRL(n)		(0x79014 + (0x1000 * (n)))
#define EDMA_REG_TXCMPL_BA_HIGH(n)	(0x7901C + (0x1000 * (n)))

#define EDMA_REG_TX_INT_STAT(n)		(0x99000 + (0x1000 * (n)))
#define EDMA_REG_TX_INT_MASK(n)		(0x99004 + (0x1000 * (n)))
#define EDMA_REG_TX_INT_CTRL(n)		(0x9900c + (0x1000 * (n)))

/* EDMA QID2RID configuration */
#define EDMA_QID2RID_TABLE_MEM(q)	(0xb9000 + (0x4 * (q)))

#define EDMA_CPU_PORT_QID_MIN			0
#define EDMA_CPU_PORT_QID_MAX			31
#define EDMA_CPU_PORT_MC_QID_MIN		256
#define EDMA_CPU_PORT_MC_QID_MAX		271
#define EDMA_QID2RID_NUM_PER_REG		4

/* EDMA_REG_DMAR_CTRL register */
#define EDMA_DMAR_REQ_PRI_MASK			0x7
#define EDMA_DMAR_REQ_PRI_SHIFT			0x0
#define EDMA_DMAR_BURST_LEN_MASK		0x1
#define EDMA_DMAR_BURST_LEN_SHIFT		3
#define EDMA_DMAR_TXDATA_OUTSTANDING_NUM_MASK	0x1f
#define EDMA_DMAR_TXDATA_OUTSTANDING_NUM_SHIFT	4
#define EDMA_DMAR_TXDESC_OUTSTANDING_NUM_MASK	0x7
#define EDMA_DMAR_TXDESC_OUTSTANDING_NUM_SHIFT	9
#define EDMA_DMAR_RXFILL_OUTSTANDING_NUM_MASK	0x7
#define EDMA_DMAR_RXFILL_OUTSTANDING_NUM_SHIFT	12

#define EDMA_DMAR_REQ_PRI_SET(x)				\
		(((x) & EDMA_DMAR_REQ_PRI_MASK)			\
		<< EDMA_DMAR_REQ_PRI_SHIFT)
#define EDMA_DMAR_TXDATA_OUTSTANDING_NUM_SET(x)			\
		(((x) & EDMA_DMAR_TXDATA_OUTSTANDING_NUM_MASK)	\
		<< EDMA_DMAR_TXDATA_OUTSTANDING_NUM_SHIFT)
#define EDMA_DMAR_TXDESC_OUTSTANDING_NUM_SET(x)			\
		(((x) & EDMA_DMAR_TXDESC_OUTSTANDING_NUM_MASK)	\
		<< EDMA_DMAR_TXDESC_OUTSTANDING_NUM_SHIFT)
#define EDMA_DMAR_RXFILL_OUTSTANDING_NUM_SET(x)			\
		(((x) & EDMA_DMAR_RXFILL_OUTSTANDING_NUM_MASK)	\
		<< EDMA_DMAR_RXFILL_OUTSTANDING_NUM_SHIFT)
#define EDMA_DMAR_BURST_LEN_SET(x)				\
		(((x) & EDMA_DMAR_BURST_LEN_MASK)		\
		<< EDMA_DMAR_BURST_LEN_SHIFT)

#define EDMA_BURST_LEN_ENABLE			0x0

/* EDMA_REG_PORT_CTRL register */
#define EDMA_PORT_CTRL_EN			0x3

/* EDMA_REG_TXDESC_PROD_IDX register */
#define EDMA_TXDESC_PROD_IDX_MASK		0xffff

/* EDMA_REG_TXDESC_CONS_IDX register */
#define EDMA_TXDESC_CONS_IDX_MASK		0xffff

/* EDMA_REG_TXDESC_RING_SIZE register */
#define EDMA_TXDESC_RING_SIZE_MASK		0xffff

/* EDMA_REG_TXDESC_CTRL register */
#define EDMA_TXDESC_TX_EN			0x1
#define EDMA_TXDESC_TX_RESET			BIT(9)
#define EDMA_TXDESC_CTRL_FC_GRP_ID_SHIFT	15
#define EDMA_TXDESC_CTRL_FC_GRP_ID_MASK		(0x1f << EDMA_TXDESC_CTRL_FC_GRP_ID_SHIFT)
#define EDMA_TXDESC_CTRL_FC_GRP_ID_SET(x)		\
	(((x) << EDMA_TXDESC_CTRL_FC_GRP_ID_SHIFT) &	\
	 EDMA_TXDESC_CTRL_FC_GRP_ID_MASK)
#define EDMA_TXDESC_TSO_IDENT_UPDATE_CTRL_SHIFT	13
#define EDMA_TXDESC_TSO_IDENT_UPDATE_CTRL_MASK	(0x3 << EDMA_TXDESC_TSO_IDENT_UPDATE_CTRL_SHIFT)
#define EDMA_TXDESC_TSO_IDENT_UPDATE_BY_PARSER	0x3
#define EDMA_TXDESC_TSO_IDENT_UPDATE_CTRL_SET(x)		\
	(((x) << EDMA_TXDESC_TSO_IDENT_UPDATE_CTRL_SHIFT) &	\
	 EDMA_TXDESC_TSO_IDENT_UPDATE_CTRL_MASK)
#define EDMA_TXDESC_PH_EN			1
#define EDMA_TXDESC_CTRL_PH_EN_SHIFT		20
#define EDMA_TXDESC_CTRL_PH_EN_MASK		BIT(EDMA_TXDESC_CTRL_PH_EN_SHIFT)
#define EDMA_TXDESC_CTRL_PH_EN_SET(x)			\
	(((x) << EDMA_TXDESC_CTRL_PH_EN_SHIFT) &		\
	 EDMA_TXDESC_CTRL_PH_EN_MASK)

#define EDMA_TXDESC_BUF_HI_ADD_MASK		0xFF

/* EDMA_REG_TXCMPL_PROD_IDX register */
#define EDMA_TXCMPL_PROD_IDX_MASK		0xffff

/* EDMA_REG_TXCMPL_CONS_IDX register */
#define EDMA_TXCMPL_CONS_IDX_MASK		0xffff

/* EDMA_REG_TX_INT_CTRL register */
#define EDMA_TX_INT_MASK			0x3

/* EDMA_REG_RXFILL_PROD_IDX register */
#define EDMA_RXFILL_PROD_IDX_MASK		0xffff

/* EDMA_REG_RXFILL_CONS_IDX register */
#define EDMA_RXFILL_CONS_IDX_MASK		0xffff

/* EDMA_REG_RXFILL_RING_SIZE register */
#define EDMA_RXFILL_RING_SIZE_MASK		0xffff
#define EDMA_RXFILL_BUF_SIZE_MASK		0xffff0000
#define EDMA_RXFILL_BUF_SIZE_SHIFT		16

/* EDMA_REG_RXFILL_RING_EN register */
#define EDMA_RXFILL_RING_EN			0x1

/* EDMA_REG_RXFILL_INT_MASK register */
#define EDMA_RXFILL_INT_MASK			0x1

#define EDMA_RXFILL_BUF_HI_ADD_MASK		0xFF

/* EDMA_REG_RXDESC_PROD_IDX register */
#define EDMA_RXDESC_PROD_IDX_MASK		0xffff

/* EDMA_REG_RXDESC_CONS_IDX register */
#define EDMA_RXDESC_CONS_IDX_MASK		0xffff

/* EDMA_REG_RXDESC_RING_SIZE register */
#define EDMA_RXDESC_RING_SIZE_MASK		0xffff
#define EDMA_RXDESC_PL_OFFSET_MASK		0x1ff
#define EDMA_RXDESC_PL_OFFSET_SHIFT		16
#define EDMA_RXDESC_PL_OFFSET_SHIFT_V2		23

/* EDMA_REG_RXDESC_CTRL register */
#define EDMA_RXDESC_RX_EN			0x1
#define EDMA_RXDESC_RX_RESET			0x1

/* EDMA_REG_TX_INT_MASK register */
#define EDMA_TX_INT_MASK_PKT_INT		0x1
#define EDMA_TX_INT_MASK_UGT_INT		0x2

/* EDMA_REG_RXDESC_INT_MASK register */
#define EDMA_RXDESC_INT_MASK_PKT_INT		0x1
#define EDMA_MASK_INT_DISABLE			0x0

/* TXDESC shift values */
#define EDMA_TXDESC_DATA_OFFSET_SHIFT		0
#define EDMA_TXDESC_DATA_OFFSET_MASK		0xfff

#define EDMA_TXDESC_DATA_LENGTH_SHIFT		0
#define EDMA_TXDESC_DATA_LENGTH_MASK		0x1ffff

/* TXDESC pass-through enable */
#define EDMA_TXDESC_PASSTHROUGH_EN_MASK		0x3
#define EDMA_TXDESC_PASSTHROUGH_EN_SHIFT	14
#define EDMA_TXDESC_PASSTHROUGH_EN \
	(EDMA_TXDESC_PASSTHROUGH_EN_MASK << EDMA_TXDESC_PASSTHROUGH_EN_SHIFT)

#define EDMA_SRC_PORT_TYPE			2
#define EDMA_SRC_PORT_TYPE_SHIFT		12
#define EDMA_SRC_PORT_TYPE_MASK					\
			(0xf << EDMA_SRC_PORT_TYPE_SHIFT)
#define EDMA_SRC_PORT_ID_SHIFT			0
#define EDMA_SRC_PORT_ID_MASK					\
			(0xfff << EDMA_SRC_PORT_ID_SHIFT)

#define EDMA_SRC_PORT_TYPE_SET(x)				\
			(((x) << EDMA_SRC_PORT_TYPE_SHIFT) &	\
			EDMA_SRC_PORT_TYPE_MASK)
#define EDMA_SRC_PORT_ID_SET(x)					\
			(((x) << EDMA_SRC_PORT_ID_SHIFT) &	\
			EDMA_SRC_PORT_ID_MASK)

#define EDMA_DST_PORT_TYPE			2
#define EDMA_DST_PORT_TYPE_SHIFT		28
#define EDMA_DST_PORT_TYPE_MASK					\
			(0xf << EDMA_DST_PORT_TYPE_SHIFT)
#define EDMA_DST_PORT_ID_SHIFT			16
#define EDMA_DST_PORT_ID_MASK					\
			(0xfff << EDMA_DST_PORT_ID_SHIFT)

#define EDMA_DST_PORT_TYPE_SET(x)				\
			(((x) << EDMA_DST_PORT_TYPE_SHIFT) &	\
			EDMA_DST_PORT_TYPE_MASK)
#define EDMA_DST_PORT_ID_SET(x)					\
			(((x) << EDMA_DST_PORT_ID_SHIFT) &	\
			EDMA_DST_PORT_ID_MASK)

#define EDMA_RXDESC_SRCINFO_TYPE_PORTID		0x2000
#define EDMA_RXDESC_SRCINFO_TYPE_SHIFT		8
#define EDMA_RXDESC_SRCINFO_TYPE_MASK		0xf000
#define EDMA_RXDESC_PORTNUM_BITS		0x0FFF

#define EDMA_RING_DMA_MASK			0xffffffff

/* RXDESC shift values */
#define EDMA_RXDESC_PKT_SIZE_MASK		0x3ffff
#define EDMA_RXDESC_PKT_SIZE_SHIFT		0
#define EDMA_RXDESC_SRC_INFO_GET(x)		((x) & 0xFFFF)
#define EDMA_RXDESC_RING_INT_STATUS_MASK	0x3
#define EDMA_RXFILL_RING_INT_STATUS_MASK	0x1

#define EDMA_TXCMPL_RING_INT_STATUS_MASK	0x3
#define EDMA_TXCMPL_RETMODE_OPAQUE		0x0
#define EDMA_TXCMPL_MORE_BIT_MASK		0x40000000
#define EDMA_TXCOMP_RING_ERROR_MASK		0x7fffff
#define EDMA_TX_NE_INT_EN			0x2
#define EDMA_RX_NE_INT_EN			0x2
#define EDMA_TX_INITIAL_PROD_IDX		0x0

#define EDMA_MISC_INTR_MASK			0xFF
#define EDMA_MISC_AXI_RD_ERR_MASK		0x1
#define EDMA_MISC_AXI_WR_ERR_MASK		0x2
#define EDMA_MISC_RX_DESC_FIFO_FULL_MASK	0x4
#define EDMA_MISC_RX_ERR_BUF_SIZE_MASK		0x8
#define EDMA_MISC_TX_SRAM_FULL_MASK		0x10
#define EDMA_MISC_TX_CMPL_BUF_FULL_MASK		0x20
#define EDMA_MISC_DATA_LEN_ERR_MASK		0x40
#define EDMA_MISC_TX_TIMEOUT_MASK		0x80
#define EDMA_RX_PAYLOAD_OFFSET			0x0

/* PPE register */
#define PORT5_MUX_PCS_UNIPHY0			0x0
#define PORT5_MUX_PCS_UNIPHY1			0x1

#define PORT_MUX_MAC_TYPE			0
#define PORT_MUX_XMAC_TYPE			1

#define ADPT_ACL_HPPE_IPV4_DIP_RULE		4
#define ADPT_ACL_HPPE_MAC_SA_RULE		1
#define ADPT_ACL_HPPE_MAC_DA_RULE		0
#define MAX_RULE				512

#define PORT_MUX_CTRL				0x10
#define PORT_MUX_CTRL_NUM			1
#define PORT_MUX_CTRL_INC			0x4
#define PORT_MUX_CTRL_DEFAULT			0x0

#define PORT_PHY_STATUS_ADDRESS			0x40
#define PORT_PHY_STATUS_ADDRESS1		0x44

#define PORT_PHY_STATUS_PORT2_OFFSET		8
#define PORT_PHY_STATUS_PORT3_OFFSET		16
#define PORT_PHY_STATUS_PORT4_OFFSET		24
#define PORT_PHY_STATUS_PORT5_1_OFFSET		8
#define PORT_PHY_STATUS_PORT6_OFFSET		16

#define PPE_IPE_L3_BASE_ADDR			0x200000
#define PPE_L3_VP_PORT_TBL_ADDR			(PPE_IPE_L3_BASE_ADDR + 0x4000)
#define PPE_L3_VP_PORT_TBL_INC			0x10

#define PPE_TL_PORT_VP_TBL_ADDR			0x302000
#define PPE_MRU_MTU_CTRL_TBL_ADDR		0x65000
#define PPE_MC_MTU_CTRL_TBL_ADDR		0x60a00
#define PPE_PORT_EG_VLAN_TBL_ADDR		0x20020

#define PPE_PORT_EG_VLAN_TBL_INC		0x10
#define PPE_PORT_EG_VLAN_TBL_NUM		7
#define PPE_PORT_EG_VLAN_TBL_PORT0		0

#define PPE_UCAST_QUEUE_AC_EN_BASE_ADDR		0x848000
#define PPE_MCAST_QUEUE_AC_EN_BASE_ADDR		0x84a000
#define PPE_QUEUE_MANAGER_BASE_ADDR		0x800000
#define PPE_UCAST_QUEUE_MAP_TBL_ADDR		0x10000
#define PPE_UCAST_QUEUE_MAP_TBL_INC		0x10
#define PPE_QM_UQM_TBL			(PPE_QUEUE_MANAGER_BASE_ADDR +\
					 PPE_UCAST_QUEUE_MAP_TBL_ADDR)
#define PPE_UCAST_PRIORITY_MAP_TBL_ADDR		0x42000
#define PPE_QM_UPM_TBL			(PPE_QUEUE_MANAGER_BASE_ADDR +\
					 PPE_UCAST_PRIORITY_MAP_TBL_ADDR)
#define PPE_QM_AC_UNI_QUEUE_CFG_TBL_ADDR	0x48000
#define PPE_QM_AC_UNI_QUEUE_CFG_TBL_INC		0x10
#define PPE_QM_AC_UNI_QUEUE_CNT_TBL_ADDR	0x53000
#define PPE_QM_AC_UNI_QUEUE_CNT_TBL_INC		0x10
#define PPE_QM_AC_UNI_QUEUE_DROP_STATE_TBL_ADDR	0x56000
#define PPE_QM_AC_UNI_QUEUE_DROP_STATE_TBL_INC	0x10
#define PPE_QM_QUEUE_TX_COUNTER_TBL_ADDR	0x1a000
#define PPE_QM_QUEUE_TX_COUNTER_TBL_INC		0x10
#define PPE_QM_UNI_DROP_CNT_TBL_ADDR		0x1e0000
#define PPE_QM_UNI_DROP_CNT_TBL_INC		0x10

#define PPE_PRX_CSR_BASE_ADDR			0x00b000
#define PPE_DROP_STAT_ADDR			0x3000
#define PPE_DROP_STAT_INC			0x10

#define PPE_PTX_CSR_BASE_ADDR			0x020000
#define PPE_PORT_TX_COUNTER_TBL_ADDR		0x14000
#define PPE_PORT_TX_COUNTER_TBL_INC		0x10
#define PPE_VP_TX_COUNTER_TBL_ADDR		0x16000
#define PPE_VP_TX_COUNTER_TBL_INC		0x10

#define PPE_BM_CSR_BASE_ADDR			0x600000
#define PPE_BM_PORT_CNT_ADDR			0x2e0
#define PPE_BM_PORT_CNT_INC			0x4
#define PPE_BM_PORT_REACTED_CNT_ADDR		0x380
#define PPE_BM_PORT_REACTED_CNT_INC		0x4
#define PPE_BM_PORT_FC_STATUS_ADDR		0x1a0
#define PPE_BM_PORT_FC_STATUS_INC		0x4

#define PPE_STP_BASE				0x060100
#define PPE_MAC_ENABLE				0x001000
#define PPE_MAC_SPEED_OFF			0x4
#define PPE_MAC_MIB_CTL_OFF			0x34

#define PPE_TRAFFIC_MANAGER_BASE_ADDR		0x400000
#define PPE_TM_SHP_CFG_L0_OFFSET		0x00000030
#define PPE_TM_SHP_CFG_L1_OFFSET		0x00000034
#define PPE_TM_SHP_CFG_L0		(PE_TRAFFIC_MANAGER_BASE_ADDR + PPE_TM_SHP_CFG_L0_OFFSET)
#define PPE_TM_SHP_CFG_L1		(PE_TRAFFIC_MANAGER_BASE_ADDR + PPE_TM_SHP_CFG_L1_OFFSET)

#define PPE_L0_FLOW_PORT_MAP_TBL_ADDR		0x10000
#define PPE_L0_FLOW_PORT_MAP_TBL_INC		0x10
#define PPE_L0_FLOW_PORT_MAP_TBL	(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L0_FLOW_PORT_MAP_TBL_ADDR)

#define PPE_L0_FLOW_MAP_TBL_ADDR		0x2000
#define PPE_L0_FLOW_MAP_TBL_INC			0x10
#define PPE_L0_FLOW_MAP_TBL		(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L0_FLOW_MAP_TBL_ADDR)

#define PPE_L1_FLOW_PORT_MAP_TBL_ADDR		0x46000
#define PPE_L1_FLOW_PORT_MAP_TBL_INC		0x10
#define PPE_L1_FLOW_PORT_MAP_TBL	(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L1_FLOW_PORT_MAP_TBL_ADDR)

#define PPE_L1_FLOW_MAP_TBL_ADDR		0x40000
#define PPE_L1_FLOW_MAP_TBL_INC			0x10
#define PPE_L1_FLOW_MAP_TBL		(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L1_FLOW_MAP_TBL_ADDR)

#define PPE_L0_C_SP_CFG_TBL_ADDR		0x4000
#define PPE_L0_C_SP_CFG_TBL		(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L0_C_SP_CFG_TBL_ADDR)

#define PPE_L1_C_SP_CFG_TBL_ADDR		0x42000
#define PPE_L1_C_SP_CFG_TBL		(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L1_C_SP_CFG_TBL_ADDR)

#define PPE_L0_E_SP_CFG_TBL_ADDR		0x6000
#define PPE_L0_E_SP_CFG_TBL		(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L0_E_SP_CFG_TBL_ADDR)

#define PPE_L1_E_SP_CFG_TBL_ADDR		0x44000
#define PPE_L1_E_SP_CFG_TBL		(PPE_TRAFFIC_MANAGER_BASE_ADDR +\
					 PPE_L1_E_SP_CFG_TBL_ADDR)

#define PPE_FPGA_GPIO_BASE_ADDR			0x01008000

#define PPE_MAC_PORT_MUX_OFFSET			0x10
#define PPE_FPGA_GPIO_OFFSET			0xc000
#define PPE_FPGA_SCHED_OFFSET			0x47a000
#define PPE_TDM_CFG_DEPTH_OFFSET		0xb000
#define PPE_TDM_SCHED_DEPTH_OFFSET		0x400000
#define PPE_PORT_BRIDGE_CTRL_OFFSET		0x060300

#define PPE_TDM_CFG_DEPTH_VAL			0x80000064
#define PPE_MAC_PORT_MUX_OFFSET_VAL		0x15
#define PPE_TDM_SCHED_DEPTH_VAL			0x32
#define PPE_TDM_CFG_VALID			0x20
#define PPE_TDM_CFG_DIR_INGRESS			0x0
#define PPE_TDM_CFG_DIR_EGRESS			0x10
#define PPE_PORT_EDMA				0x0
#define PPE_PORT_QTI1				0x1
#define PPE_PORT_QTI2				0x2
#define PPE_PORT_QTI3				0x3
#define PPE_PORT_QTI4				0x4
#define PPE_PORT_XGMAC1				0x5
#define PPE_PORT_XGMAC2				0x6
#define PPE_PORT_CRYPTO1			0x7
#define PPE_PORT_BRIDGE_CTRL_PROMISC_EN		0x20000
#define PPE_PORT_BRIDGE_CTRL_TXMAC_EN		0x10000
#define PPE_PORT_BRIDGE_CTRL_PORT_ISOLATION_BMP	0x7f00
#define PPE_PORT_BRIDGE_CTRL_STATION_LRN_EN	0x8
#define PPE_PORT_BRIDGE_CTRL_NEW_ADDR_LRN_EN	0x1
#define PPE_PORT_EDMA_BITPOS		0x1
#define PPE_PORT_QTI1_BITPOS		BIT(PPE_PORT_QTI1)
#define PPE_PORT_QTI2_BITPOS		BIT(PPE_PORT_QTI2)
#define PPE_PORT_QTI3_BITPOS		BIT(PPE_PORT_QTI3)
#define PPE_PORT_QTI4_BITPOS		BIT(PPE_PORT_QTI4)
#define PPE_PORT_XGMAC1_BITPOS		BIT(PPE_PORT_XGMAC1)
#define PPE_PORT_XGMAC2_BITPOS		BIT(PPE_PORT_XGMAC2)
#define PPE_PORT_CRYPTO1_BITPOS		BIT(PPE_PORT_CRYPTO1)

#define PPE_SWITCH_NSS_SWITCH_XGMAC0		0x500000
#define NSS_SWITCH_XGMAC_MAC_TX_CONFIGURATION	0x4000
#define NSS_SWITCH_XGMAC_MMC_CONTROL		0x800
#define NSS_SWITCH_XGMAC_MIB_TX_OCTETS		0x814
#define NSS_SWITCH_XGMAC_MIB_TX_FRAMES		0x81c
#define NSS_SWITCH_XGMAC_MIB_RX_FRAMES		0x900
#define NSS_SWITCH_XGMAC_MIB_RX_OCTETS		0x908
#define USS					BIT(31)
#define SS(i)					((i) << 29)
#define XGMAC_SPEED_SELECT_MASK			GENMASK(31, 29)
#define JD					BIT(16)
#define TE					BIT(0)
#define NSS_SWITCH_XGMAC_MAC_RX_CONFIGURATION	0x4000
#define MAC_RX_CONFIGURATION_ADDRESS		0x4
#define RE					BIT(0)
#define ACS					BIT(1)
#define CST					BIT(2)
#define MAC_PACKET_FILTER_INC			0x4000
#define MAC_PACKET_FILTER_ADDRESS		0x8

#define XGMAC_SPEED_SELECT_10000M		0
#define XGMAC_SPEED_SELECT_5000M		1
#define XGMAC_SPEED_SELECT_2500M		2
#define XGMAC_SPEED_SELECT_1000M		3

#define IPE_L2_BASE_ADDR			0x060000
#define PORT_BRIDGE_CTRL_ADDRESS		0x300
#define PORT_BRIDGE_CTRL_INC			0x4
#define TX_MAC_EN				BIT(16)

#define IPO_CSR_BASE_ADDR			0x0b0000

#define IPO_RULE_REG_ADDRESS			0x0
#define IPO_RULE_REG_INC			0x10

#define IPO_MASK_REG_ADDRESS			0x2000
#define IPO_MASK_REG_INC			0x10

#define IPO_ACTION_ADDRESS			0x8000
#define IPO_ACTION_INC				0x20
/* Uniphy register */
#define GCC_UNIPHY_REG_INC			0x10

#define PPE_UNIPHY_OFFSET_CALIB_4		0x1E0
#define UNIPHY_CALIBRATION_DONE			0x1

/* UNIPHY calibration registers (QSERDES-based SerDes) */
#define PCS0_UNIPHY_OPTION_3_ADDRESS		0x5AC
#define PCS_UNIPHY_OPTION_3_ADDRESS		0x588
#define PCS_UNIPHY_OPTION_3_UNIPHY_START_BIT	BIT(4)
#define QSERDES_RX_EXT_RO_POWER_STATE_ADDRESS	0xCDF4
#define UNIPHY_POWER_STATE_DONE			0xF
#define UNIPHY_POLLING_TIMEOUT			2000
#define UNIPHY_POLLING_DELAY			1

#define PPE_UNIPHY_REG_INC			0
#define PPE_UNIPHY_MODE_CONTROL			0x46C
#define UNIPHY_XPCS_MODE			BIT(12)
#define UNIPHY_SG_PLUS_MODE			BIT(11)
#define UNIPHY_SG_MODE				BIT(10)
#define UNIPHY_CH0_PSGMII_QSGMII		BIT(9)
#define UNIPHY_CH0_QSGMII_SGMII			BIT(8)
#define UNIPHY_CH4_CH1_0_SGMII			BIT(2)
#define UNIPHY_CH1_CH0_SGMII			BIT(1)
#define UNIPHY_CH0_ATHR_CSCO_MODE_25M		BIT(0)

#define UNIPHY_INSTANCE_LINK_DETECT		0x570
#define UNIPHY_DETECT_LOS_FROM_SFP		GENMASK(8, 6)

#define UNIPHY_MISC2_REG_OFFSET			0x218
#define UNIPHY_MISC2_REG_SGMII_MODE		0x30
#define UNIPHY_MISC2_REG_SGMII_PLUS_MODE	0x50

#define UNIPHY_MISC2_REG_VALUE			0x70

#define UNIPHY_MISC_SOURCE_SELECTION_REG_OFFSET	0x21c
#define UNIPHY_MISC_SRC_PHY_MODE		0xa882

#define UNIPHY_MII_ADPT_RESET			BIT(11)
#define UNIPHY_DEC_CHANNEL_0_INPUT_OUTPUT_4	0x480
#define UNIPHY_FORCE_SPEED_25M			BIT(3)

#define UNIPHY1_CLKOUT_50M_CTRL_OPTION		0x610
#define UNIPHY1_CLKOUT_50M_CTRL_CLK50M_DIV2_SEL	BIT(5)
#define UNIPHY1_CLKOUT_50M_CTRL_50M_25M_EN	0x1

#define UNIPHY_PLL_RESET_REG_OFFSET		0x780
#define UNIPHY_PLL_RESET_REG_VALUE		0x02bf
#define UNIPHY_PLL_RESET_REG_DEFAULT_VALUE	0x02ff

#define SR_XS_PCS_KR_STS1_ADDRESS		0x30020
#define UNIPHY_10GR_LINKUP			0x1

#define VR_XS_PCS_DIG_CTRL1_ADDRESS		0x38000
#define VR_XS_PCS_EEE_MCTRL0_ADDRESS		0x38006
#define VR_XS_PCS_KR_CTRL_ADDRESS		0x38007
#define VR_XS_PCS_EEE_TXTIMER_ADDRESS		0x38008
#define VR_XS_PCS_EEE_RXTIMER_ADDRESS		0x38009
#define VR_XS_PCS_EEE_MCTRL1_ADDRESS		0x3800b
#define VR_XS_PCS_DIG_STS_ADDRESS		0x3800a

#define USXG_EN					BIT(9)
#define USRA_RST				BIT(10)
#define USXG_MODE                               (5 << 10)
#define VR_RST					BIT(15)
#define AM_COUNT				(0x6018 << 0)

#define SR_MII_CTRL_CHANNEL1_ADDRESS		0x1a0000
#define SR_MII_CTRL_CHANNEL2_ADDRESS		0x1b0000
#define SR_MII_CTRL_CHANNEL3_ADDRESS		0x1c0000
#define SR_MII_CTRL_ADDRESS			0x1f0000

#define VR_MII_DIG_CTRL1_CHANNEL1_ADDRESS	0x1a8000
#define VR_MII_DIG_CTRL1_CHANNEL2_ADDRESS	0x1b8000
#define VR_MII_DIG_CTRL1_CHANNEL3_ADDRESS	0x1c8000
#define USRA_RST_MII				BIT(5)  /* usra_rst in VR_MII_DIG_CTRL1 */

#define VR_MII_AN_CTRL_CHANNEL1_ADDRESS		0x1a8001
#define VR_MII_AN_CTRL_CHANNEL2_ADDRESS		0x1b8001
#define VR_MII_AN_CTRL_CHANNEL3_ADDRESS		0x1c8001
#define VR_MII_AN_CTRL_ADDRESS			0x1f8001
#define MII_AN_INTR_EN				BIT(0)
#define MII_CTRL				BIT(8)

#define AN_ENABLE				BIT(12)
#define AN_RESTART				BIT(9)
#define SS5					BIT(5)
#define SS6					BIT(6)
#define SS13					BIT(13)
#define DUPLEX_MODE				BIT(8)

#define VR_MII_AN_INTR_STS_CHANNEL1_ADDRESS	0x1a8002
#define VR_MII_AN_INTR_STS_CHANNEL2_ADDRESS	0x1b8002
#define VR_MII_AN_INTR_STS_CHANNEL3_ADDRESS	0x1c8002
#define VR_MII_AN_INTR_STS			0x1f8002
#define CL37_ANCMPLT_INTR			BIT(0)

#define UNIPHYQP_USXG_OPITON1			0x584
#define GMII_SRC_SEL				BIT(0)

/*
 * QP_USXG_RESET - UNIPHY functional reset register (RESET_VERSION_V2)
 * Address: 0x630 (direct CSR0 access)
 *
 * Bit layout:
 *   bit[0] = mmd1_reg_usxg_func_rst_n    (active-LOW, main USXG reset)
 *   bit[1] = mmd1_reg_usxg_func_rst_n_p1 (active-LOW, port 1 reset)
 *   bit[2] = mmd1_reg_usxg_func_rst_n_p2 (active-LOW, port 2 reset)
 *   bit[3] = mmd1_reg_usxg_func_rst_n_p3 (active-LOW, port 3 reset)
 *   bit[4] = mmd1_reg_pqsgmii_func_rst_n (active-LOW)
 *   bit[5] = mmd1_reg_qsgmii_enable
 *   bit[6] = mmd1_reg_xpcs_enable
 *   Default: 0x7F (all resets de-asserted, XPCS+QSGMII enabled)
 *
 * Active-LOW bits require explicit assert (clear) then de-assert (set).
 * Used by RESET_VERSION_V2 path in ppe_uniphy_usxgmii_port_reset().
 */
#define QP_USXG_RESET_ADDRESS			0x630
#define QP_USXG_RST_N_MAIN			BIT(0)
#define QP_USXG_RST_N_P1			BIT(1)
#define QP_USXG_RST_N_P2			BIT(2)
#define QP_USXG_RST_N_P3			BIT(3)

#define VR_XAUI_MODE_CTRL_CHANNEL1_ADDRESS      0x1a8004
#define VR_XAUI_MODE_CTRL_CHANNEL2_ADDRESS      0x1b8004
#define VR_XAUI_MODE_CTRL_CHANNEL3_ADDRESS      0x1c8004
#define VR_XAUI_MODE_CTRL_ADDRESS               0x1f8004
#define IPG_CHECK                               0x1

#define UNIPHY_XPCS_TSL_TIMER				(0xa << 0)
#define SIGN_BIT					BIT(6)
#define MULT_FACT_100NS					BIT(8)
#define UNIPHY_XPCS_TLU_TIMER				(0x3 << 6)
#define UNIPHY_XPCS_TWL_TIMER				(0x16 << 8)
#define UNIPHY_XPCS_100US_TIMER				(0xc8 << 0)
#define UNIPHY_XPCS_TWR_TIMER				(0x1c << 8)
#define LRX_EN						BIT(0)
#define LTX_EN						BIT(1)
#define TRN_LPI						BIT(0)
#define TRN_RXLPI					BIT(8)

#define CLK_1_25_MHZ					(1250000UL)
#define CLK_2_5_MHZ					(2500000UL)
#define CLK_12_5_MHZ					(12500000UL)
#define CLK_25_MHZ					(25000000UL)
#define CLK_78_125_MHZ					(78125000UL)
#define CLK_50_MHZ					(50000000UL)
#define CLK_125_MHZ					(125000000UL)
#define CLK_156_25_MHZ					(156250000UL)
#define CLK_312_5_MHZ					(312500000UL)

enum uniphy_reset_type {
	UNIPHY0_SOFT_RESET = 0,
	UNIPHY0_XPCS_RESET,
	UNIPHY1_SOFT_RESET,
	UNIPHY1_XPCS_RESET,
	UNIPHY2_SOFT_RESET,
	UNIPHY2_XPCS_RESET,
	UNIPHY_RST_MAX
};

enum {
	GMAC = 0,
	XGMAC,
};

enum ipq_edma_tx {
	EDMA_TX_OK = 0,			/* Tx success */
	EDMA_TX_DESC = 1,		/* Not enough descriptors */
	EDMA_TX_FAIL = 2,		/* Tx failure */
};

enum port_wrapper_cfg {
	PORT_WRAPPER_PSGMII = 0,              /* 0 - PSGMII mode */
	PORT_WRAPPER_PSGMII_RGMII5,           /* 1 - PSGMII + RGMII5 */
	PORT_WRAPPER_SGMII0_RGMII5,           /* 2 - SGMII0 + RGMII5 */
	PORT_WRAPPER_SGMII1_RGMII5,           /* 3 - SGMII1 + RGMII5 */
	PORT_WRAPPER_PSGMII_RMII0,            /* 4 - PSGMII + RMII0 */
	PORT_WRAPPER_PSGMII_RMII1,            /* 5 - PSGMII + RMII1 */
	PORT_WRAPPER_PSGMII_RMII0_RMII1,      /* 6 - PSGMII + RMII0 + RMII1 */
	PORT_WRAPPER_PSGMII_RGMII4,           /* 7 - PSGMII + RGMII4 */
	PORT_WRAPPER_SGMII0_RGMII4,           /* 8 - SGMII0 + RGMII4 */
	PORT_WRAPPER_SGMII1_RGMII4,           /* 9 - SGMII1 + RGMII4 */
	PORT_WRAPPER_SGMII4_RGMII4,           /* 10 - SGMII4 + RGMII4 */
	PORT_WRAPPER_QSGMII,                  /* 11 - QSGMII mode */
	PORT_WRAPPER_SGMII_PLUS,              /* 12 - SGMII+ mode */
	PORT_WRAPPER_USXGMII,                 /* 13 - USXGMII mode */
	PORT_WRAPPER_10GBASE_R,               /* 14 - 10GBASE-R mode */
	PORT_WRAPPER_SGMII_CHANNEL0,          /* 15 - SGMII Channel 0 */
	PORT_WRAPPER_SGMII_CHANNEL1,          /* 16 - SGMII Channel 1 */
	PORT_WRAPPER_SGMII_CHANNEL4,          /* 17 - SGMII Channel 4 */
	PORT_WRAPPER_RGMII,                   /* 18 - RGMII mode */
	PORT_WRAPPER_PSGMII_FIBER,            /* 19 - PSGMII Fiber */
	PORT_WRAPPER_SGMII_FIBER,             /* 20 - SGMII Fiber */
	PORT_WRAPPER_UQXGMII,                 /* 21 - UQXGMII (4 channels PHY) */
	PORT_WRAPPER_UDXGMII,                 /* 22 - UDXGMII mode */
	PORT_WRAPPER_UQXGMII_3CHANNELS,       /* 23 - UQXGMII (3 channels PHY) */
	PORT_WRAPPER_25GBASE_R,               /* 24 - 25GBASE-R mode */
	PORT_WRAPPER_GPON,                    /* 25 - GPON mode */
	PORT_WRAPPER_XGPON,                   /* 26 - XGPON mode */
	PORT_WRAPPER_XGSPON,                  /* 27 - XGSPON mode */
	PORT_WRAPPER_EMULATION,
	PORT_WRAPPER_NA,
	PORT_WRAPPER_MAX = 0xFF               /* Maximum value marker */
};

/*
 * Helper macros for mode checking
 */
#define IS_USXGMII_MODE(mode)     ((mode) == PORT_WRAPPER_USXGMII)
#define IS_UQXGMII_MODE(mode)     ((mode) == PORT_WRAPPER_UQXGMII || \
                                   (mode) == PORT_WRAPPER_UQXGMII_3CHANNELS)
#define IS_UDXGMII_MODE(mode)     ((mode) == PORT_WRAPPER_UDXGMII)
#define IS_10GBASE_R_MODE(mode)   ((mode) == PORT_WRAPPER_10GBASE_R)
#define IS_25GBASE_R_MODE(mode)   ((mode) == PORT_WRAPPER_25GBASE_R)
#define IS_SGMII_MODE(mode)       ((mode) == PORT_WRAPPER_SGMII_PLUS || \
                                   (mode) == PORT_WRAPPER_SGMII_CHANNEL0 || \
                                   (mode) == PORT_WRAPPER_SGMII_CHANNEL1 || \
                                   (mode) == PORT_WRAPPER_SGMII_CHANNEL4 || \
                                   (mode) == PORT_WRAPPER_SGMII_FIBER)
#define IS_QSGMII_MODE(mode)      ((mode) == PORT_WRAPPER_QSGMII)
#define IS_PSGMII_MODE(mode)      ((mode) == PORT_WRAPPER_PSGMII || \
                                   (mode) == PORT_WRAPPER_PSGMII_RGMII5 || \
                                   (mode) == PORT_WRAPPER_PSGMII_RMII0 || \
                                   (mode) == PORT_WRAPPER_PSGMII_RMII1 || \
                                   (mode) == PORT_WRAPPER_PSGMII_RMII0_RMII1 || \
                                   (mode) == PORT_WRAPPER_PSGMII_RGMII4 || \
                                   (mode) == PORT_WRAPPER_PSGMII_FIBER)
#define IS_RGMII_MODE(mode)       ((mode) == PORT_WRAPPER_RGMII || \
                                   (mode) == PORT_WRAPPER_PSGMII_RGMII5 || \
                                   (mode) == PORT_WRAPPER_SGMII0_RGMII5 || \
                                   (mode) == PORT_WRAPPER_SGMII1_RGMII5 || \
                                   (mode) == PORT_WRAPPER_PSGMII_RGMII4 || \
                                   (mode) == PORT_WRAPPER_SGMII0_RGMII4 || \
                                   (mode) == PORT_WRAPPER_SGMII1_RGMII4 || \
                                   (mode) == PORT_WRAPPER_SGMII4_RGMII4)
#define IS_PON_MODE(mode)         ((mode) == PORT_WRAPPER_GPON || \
                                   (mode) == PORT_WRAPPER_XGPON || \
                                   (mode) == PORT_WRAPPER_XGSPON)


enum {
	TCP_PKT,
	UDP_PKT,
};

/* RxDesc descriptor */
struct ipq_edma_rxdesc_desc {
	u32 rdes0; /* Contains lower 32-bit of buffer address */
	u32 rdes1;
	/* v1: Contains more bit, priority bit, service code
	 * v2: Contains more bit, priority bit, service code & higher 8-bit of
	 * buffer address.
	 */
	u32 rdes2; /* Contains opaque */
	u32 rdes3; /* Contains opaque high bits */
	u32 rdes4; /* Contains destination and source information */
	u32 rdes5; /* Contains WiFi QoS, data length */
	u32 rdes6; /* Contains hash value, check sum status */
	u32 rdes7; /* Contains DSCP, packet offsets */
};

/* EDMA Rx Secondary Descriptor */
struct ipq_edma_rx_sec_desc {
	u32 rx_sec0; /* Contains timestamp */
	u32 rx_sec1; /* Contains secondary checksum status */
	u32 rx_sec2; /* Contains QoS tag */
	u32 rx_sec3; /* Contains flow index details */
	u32 rx_sec4; /* Contains secondary packet offsets */
	u32 rx_sec5; /* Contains multicast bit, checksum */
	u32 rx_sec6; /* Contains SVLAN, CVLAN */
	u32 rx_sec7; /* Contains secondary SVLAN, CVLAN */
};

/* RxFill descriptor */
struct ipq_edma_rxfill_desc {
	u32 rdes0; /* Contains Lower 32-bit of buffer address */
	u32 rdes1;
	/* v1: Contains buffer size
	 * v2: Contains buffer & higher 8-bit of buffer address.
	 */
	u32 rdes2; /* Contains opaque */
	u32 rdes3; /* Contains opaque high bits */
};

/* TxDesc descriptor */
struct ipq_edma_txdesc_desc {
	u32 tdes0; /* Lower 32-bit of buffer address */
	u32 tdes1;
	/* v1: Buffer recycling, PTP tag flag, PRI valid flag
	 * v2: Buffer recycling, PTP tag flag, PRI valid flag & higher 8-bit
	 * of buffer address
	 */
	u32 tdes2; /* Low 32-bit of opaque value */
	u32 tdes3; /* High 32-bit of opaque value */
	u32 tdes4; /* Source/Destination port info */
	u32 tdes5; /* VLAN offload, csum_mode, ip_csum_en, tso_en, data length */
	u32 tdes6; /* MSS/hash_value/PTP tag, data offset */
	u32 tdes7; /* L4/L3 offset, PROT type, L2 type, CVLAN/SVLAN tag, service code */
};

/* EDMA Tx Secondary Descriptor */
struct ipq_edma_tx_sec_desc {
	u32 tx_sec0; /* Reserved */
	u32 tx_sec1; /* Custom csum offset, payload offset, TTL/NAT action */
	u32 rx_sec2; /* NAPT translated port, DSCP value, TTL value */
	u32 rx_sec3; /* Flow index value and valid flag */
	u32 rx_sec4; /* Reserved */
	u32 rx_sec5; /* Reserved */
	u32 rx_sec6; /* CVLAN/SVLAN command */
	u32 rx_sec7; /* CVLAN/SVLAN tag value */
};

/* TxCmpl descriptor */
struct ipq_edma_txcmpl_desc {
	u32 tdes0; /* Low 32-bit opaque value */
	u32 tdes1; /* High 32-bit opaque value */
	u32 tdes2; /* More fragment, transmit ring id, pool id */
	u32 tdes3; /* Error indications */
};

/* Tx descriptor ring */
struct ipq_edma_txdesc_ring {
	u32 prod_idx;		/* Producer index */
	u32 avail_desc;		/* Number of available descriptor to process */
	u32 id;			/* TXDESC ring number */
	struct ipq_edma_txdesc_desc *desc;
					/* descriptor ring virtual address */
	dma_addr_t dma;			/* descriptor ring physical address */
	struct ipq_edma_tx_sec_desc *sdesc;
					/* Secondary descriptor ring virtual addr */
	dma_addr_t sdma;		/* Secondary descriptor ring physical address */
	u16 count;			/* number of descriptors */
};

/* TxCmpl ring */
struct ipq_edma_txcmpl_ring {
	u32 cons_idx;		/* Consumer index */
	u32 avail_pkt;		/* Number of available packets to process */
	struct ipq_edma_txcmpl_desc *desc;
					/* descriptor ring virtual address */
	u32 id;			/* TXCMPL ring number */
	dma_addr_t dma;			/* descriptor ring physical address */
	u32 count;			/* Number of descriptors in the ring */
};

/* RxFill ring */
struct ipq_edma_rxfill_ring {
	u32 id;			/* RXFILL ring number */
	u32 count;			/* number of descriptors in the ring */
	u32 prod_idx;		/* Ring producer index */
	struct ipq_edma_rxfill_desc *desc;
					/* descriptor ring virtual address */
	dma_addr_t dma;			/* descriptor ring physical address */
};

/* RxDesc ring */
struct ipq_edma_rxdesc_ring {
	u32 id;			/* RXDESC ring number */
	u32 count;			/* number of descriptors in the ring */
	u32 cons_idx;		/* Ring consumer index */
	struct ipq_edma_rxdesc_desc *desc;
					/* Primary descriptor ring virtual addr */
	struct ipq_edma_sec_rxdesc_ring *sdesc;
					/* Secondary desc ring VA */
	struct ipq_edma_rxfill_ring *rxfill;
					/* RXFILL ring used */
	dma_addr_t dma;			/* Primary descriptor ring physical address */
	dma_addr_t sdma;		/* Secondary descriptor ring physical address */
};

struct port_mux_ctrl {
	u32 port1_pcs_sel:1;
	u32 port2_pcs_sel:1;
	u32 port3_pcs_sel:1;
	u32 port4_pcs_sel:1;
	u32 port5_pcs_sel:1;
	u32 port6_pcs_sel:1;
	u32 _reserved0:2;
	u32 port1_mac_sel:1;
	u32 port2_mac_sel:1;
	u32 port3_mac_sel:1;
	u32 port4_mac_sel:1;
	u32 port5_mac_sel:1;
	u32 port6_mac_sel:1;
	u32 _reserved1:18;
};

union port_mux_ctrl_u {
	u32 val;
	struct port_mux_ctrl bf;
};

struct ipo_rule_reg {
	u32  rule_field_0:32;
	u32  rule_field_1:20;
	u32  fake_mac_header:1;
	u32  range_en:1;
	u32  inverse_en:1;
	u32  rule_type:5;
	u32  src_type:3;
	u32  src_0:1;
	u32  src_1:7;
	u32  pri:9;
	u32  res_chain:1;
	u32  post_routing_en:1;
	u32  _reserved0:14;
};

union ipo_rule_reg_u {
	u32 val[3];
	struct ipo_rule_reg bf;
};

struct ipo_mask_reg {
	u32  maskfield_0:32;
	u32  maskfield_1:21;
	u32  _reserved0:11;
};

union ipo_mask_reg_u {
	u32 val[2];
	struct ipo_mask_reg bf;
};

struct ipo_action {
	u32  dest_info_change_en:1;
	u32  fwd_cmd:2;
	u32  _reserved0:15;
	u32 bypass_bitmap_0:14;
	u32 bypass_bitmap_1:18;
	u32  _reserved1:14;
	u32  _reserved2:32;
	u32  _reserved3:32;
	u32  _reserved4:32;
	u32  _reserved5:32;
};

union ipo_action_u {
	u32 val[6];
	struct ipo_action bf;
};

struct ppe_acl_set {
	phys_addr_t reg_base;
	u32 rule_id;
	u32 rule_type;
	u32 field0;
	u32 field1;
	u32 mask;
	u32 permit;
	u32 deny;
	u32 ipo_cnt;
};

struct ipq_eth_port_config {
	u8 id;
	u32 clk_rate[6];
	u8 mac_mode[6];
	u8 mode[6];
} __aligned(8);

/* Generic PPE Table Address Structure
 * Contains base addresses, offsets, and increments for PPE hardware tables
 */
struct ppe_table_addr {
	u32 base_addr;		/* Base address of the table region */
	u32 offset;		/* Offset within the base region */
	u32 increment;		/* Bytes per table entry */
};

/* PPE Table Address Configuration
 * Centralized configuration for all PPE table addresses
 */
struct ppe_table_addr_config {
	struct ppe_table_addr vsi_tbl;			/* VSI Table */
	struct ppe_table_addr vp_port_tbl;		/* VP Port Table */
	struct ppe_table_addr port_fc_cfg;		/* Port Flow Control Config */
	struct ppe_table_addr mru_mtu_ctrl_tbl;	/* MRU/MTU Control Table */
	struct ppe_table_addr mc_mtu_ctrl_tbl;		/* MC MTU Control Table */
	struct ppe_table_addr port_bridge_ctrl;	/* Port Bridge Control */
	struct ppe_table_addr cst_state;		/* CST (Common Spanning Tree) State */
	struct ppe_table_addr l2_global_conf;		/* L2 Global Configuration */
	struct ppe_table_addr ipo_action;		/* IPO Action Table */
	struct ppe_table_addr port_eg_vlan_tbl;	/* Port Egress VLAN Table */
	struct ppe_table_addr eg_bridge_config;	/* Egress Bridge Config */
	struct ppe_table_addr ipo_rule_reg;		/* IPO Rule Register */
	struct ppe_table_addr ipo_mask_reg;		/* IPO Mask Register */
	struct ppe_table_addr tl_port_vp_tbl;		/* TL Port VP Table */
	struct ppe_table_addr ac_uni_queue_cfg_tbl;	/* AC Unicast Queue Config Table */
	struct ppe_table_addr ac_mul_queue_cfg_tbl;	/* AC Multicast Queue Config Table */
	struct ppe_table_addr ac_grp_cfg_tbl;		/* AC Group Config Table */
	struct ppe_table_addr ucast_queue_map_tbl;	/* Unicast Queue Map Table */
	struct ppe_table_addr ucast_priority_map_tbl;	/* Unicast Priority Map Table */
	struct ppe_table_addr l0_flow_map_tbl;		/* L0 Flow Map Table */
	struct ppe_table_addr l0_flow_port_map_tbl;	/* L0 Flow Port Map Table */
	struct ppe_table_addr l1_flow_map_tbl;		/* L1 Flow Map Table */
	struct ppe_table_addr l1_flow_port_map_tbl;	/* L1 Flow Port Map Table */
	struct ppe_table_addr psch_tdm_cfg_tbl;	/* PSCH TDM Config Table */
	struct ppe_table_addr port_bufgrp_cfg;		/* Port Buffer Group Config */
	struct ppe_table_addr port_shp_cfg;		/* Port Shaper Config */
	struct ppe_table_addr port_cnt_cfg;		/* Port Counter Config */
	struct ppe_table_addr l0_comp_cfg_tbl;		/* L0 Component Config Table */
	struct ppe_table_addr l1_comp_cfg_tbl;		/* L1 Component Config Table */
	struct ppe_table_addr ac_mcast_queue_en_tbl;	/* AC Multicast Queue Enable Table */
};

/* TDM Configuration Structure
 * Supports multiple TDM data configurations with shared address settings
 */
struct ipq_tdm_config {
	u32 val[256];			/* TDM data values (32-bit register values) */
	u32 depth;			/* Number of TDM register entries to configure */
} __aligned(8);

/* TDM Address Configuration
 * Shared across all TDM configurations for a chip
 */
struct ipq_tdm_addr_config {
	struct ppe_table_addr tdm_addr;	/* TDM table address configuration */
	u32 tdm_ctrl_offset;		/* TDM control register offset (e.g., 0xb000) */
} __aligned(8);

struct ipq_sch_config {
	u32 val[256];
	u32 depth;	/* Number of scheduler register entries to configure */
};

struct ipq_eth_sku {
	phys_addr_t reg;
	u8 bit;
	u8 status;
} __aligned(8);

extern struct ipq_tdm_config *tdm_config;
extern struct ipq_tdm_addr_config *tdm_addr_config;
extern struct ipq_sch_config *sch_config;
extern struct ipq_eth_port_config *port_config;
extern struct ipq_eth_sku *ipq_uniphy;
extern u32 nb_vsi_config[CONFIG_ETH_MAX_MAC];
extern struct ppe_table_addr_config *ppe_table_addrs;

/* Helper function for PPE table address calculation */
static inline phys_addr_t ppe_table_addr(const struct ppe_table_addr *tbl, u32 index)
{
	return tbl->base_addr + tbl->offset + (index * tbl->increment);
}

struct edma_config {
	struct ipq_eth_port_config *pconfig;
	struct edma_hw_cfg *hw_cfg;			/* Unified configuration */
	u32 tdm_ctrl_val;
	u32 sw_version;
	u32 vsi;
	u8 txdesc_ring_start;
	u8 txdesc_rings;
	u8 txdesc_ring_end;
	u8 txcmpl_ring_start;
	u8 txcmpl_rings;
	u8 txcmpl_ring_end;
	u8 rxfill_ring_start;
	u8 rxfill_rings;
	u8 rxfill_ring_end;
	u8 rxdesc_ring_start;
	u8 rxdesc_rings;
	u8 rxdesc_ring_end;
	u8 max_txcmpl_rings;
	u8 max_txdesc_rings;
	u8 max_rxdesc_rings;
	u8 max_rxfill_rings;
	u8 iports;
	u8 ports;
	u8 start_ports;
	u8 tx_map;
	u8 rx_map;
	u8 ipo_action;
	bool  hw_reset;
	/*
	 * Optional chip-specific port-to-XGMAC-ID mapping.
	 * If NULL, the generic formula (gmacid = portid - 1) is used.
	 * Returns (u32)-1 to indicate the port has no XGMAC on this SoC.
	 */
	u32 (*port_to_gmacid)(u32 portid);
};

extern struct edma_config ipq_edma_config;

/* edma hw specific data */
struct ipq_edma_hw {
	phys_addr_t iobase;		/* EDMA base address */
	struct ipq_edma_txdesc_ring *txdesc_ring;
					/* Tx Desc Ring, SW is producer */
	struct ipq_edma_txcmpl_ring *txcmpl_ring;
					/* Tx Compl Ring, SW is consumer */
	struct ipq_edma_rxdesc_ring *rxdesc_ring;
					/* Rx Desc Ring, SW is consumer */
	struct ipq_edma_rxfill_ring *rxfill_ring;
					/* Rx Fill Ring, SW is producer */
	struct edma_hw_cfg *hw_cfg;		/* Unified hardware configuration */
	u32 sw_version;		/* EDMA SW version */
	u32 rxfill_intr_mask;	/* Rx fill ring interrupt mask */
	u32 rxdesc_intr_mask;	/* Rx Desc ring interrupt mask */
	u32 txcmpl_intr_mask;	/* Tx Cmpl ring interrupt mask */
	u32 misc_intr_mask;	/* misc interrupt mask */
	u32 flags;			/* internal flags */
	u16 rx_payload_offset;	/* start of the payload offset */
	u16 rx_buff_size;		/* To do chk type Rx buffer size */
	u8 intr_clear_type;	/* interrupt clear */
	u8 intr_sw_idx_w;		/* To do chk type intr sw index */
	u8 rss_type;		/* rss protocol type */
	u8 txdesc_rings;		/* Number of TxDesc rings */
	u8 txdesc_ring_start;	/* Id of first TXDESC ring */
	u8 txdesc_ring_end;	/* Id of the last TXDESC ring */
	u8 txcmpl_rings;		/* Number of TxCmpl rings */
	u8 txcmpl_ring_start;	/* Id of first TXCMPL ring */
	u8 txcmpl_ring_end;	/* Id of last TXCMPL ring */
	u8 rxfill_rings;		/* Number of RxFill rings */
	u8 rxfill_ring_start;	/* Id of first RxFill ring */
	u8 rxfill_ring_end;	/* Id of last RxFill ring */
	u8 rxdesc_rings;		/* Number of RxDesc rings */
	u8 rxdesc_ring_start;	/* Id of first RxDesc ring */
	u8 rxdesc_ring_end;	/* Id of last RxDesc ring */
	u8 tx_intr_mask;		/* Tx intr mask */
	u8 rx_intr_mask;		/* Rx intr MAsk */
	u8 max_txcmpl_rings;	/* Max tx Completion rings */
	u8 max_txdesc_rings;	/* Max tx Descriptor rings */
	u8 max_rxdesc_rings;	/* Max rx Descriptor rings */
	u8 max_rxfill_rings;	/* Max rx fill rings */
	u8 max_ports;		/* Max ports index  */
	u8 start_ports;		/* Initial  ports index switch */
	u8 tx_map;		/* Number of TXDESC2CMPL map registers */
	u8 rx_map;		/* Number of RXDESC2FILL map registers */
};

struct port_info {
	struct phy_device *phydev;
	struct mii_dev *bus;
	struct udevice *dev;
	const char *label;
	const char *dt_phy_mode;
	const char *package_mode;
	phys_addr_t uniphy_base;
	phy_interface_t interface;
	struct gpio_desc rst_gpio;
	u32 reset_assert_us;
	u32 reset_deassert_us;
	ofnode port_node;
	ofnode pcs_node;
	ofnode node;
	u32 max_speed;
	u32 cur_speed;
	u8 phyaddr;
	u8 mac_mode;
	u8 mac_speed;
	u8 duplex;
	u8 id;
	u8 phy_id;
	u8 uniphy_id;
	u8 uniphy_mode;
	u8 uniphy_type;
	u8 pcs_channel;
	u8 gmac_type;
	u8 cur_uniphy_mode;
	u8 cur_gmac_type;
	bool isforce_speed;
	bool xgmac;
	bool isconfigured;
	bool from_dt_port;
	bool phy_25mhz;
	bool fw_loaded;
	int (*calibrate)(struct port_info *port); /* SoC-specific calibration */
	int i2c_bus;
	struct clk rx_clk_rate;
	struct clk tx_clk_rate;
	struct clk rx_clk;
	struct clk tx_clk;
} __aligned(8);

/* Port Scheduler Configuration */
struct port_scheduler_cfg {
	/* L1 Scheduler */
	u8  l1_sp_id;
	u8  l1_c_pri;
	u8  l1_c_drr_id;
	u8  l1_e_pri;
	u8  l1_e_drr_id;
	u16 l1_c_drr_wt;
	u16 l1_e_drr_wt;
	u32 l1_map_index;

	/* L0 Scheduler (common config used for both uc/mc) */
	u8  l0_sp_id;
	u8  l0_c_pri;
	u8  l0_c_drr_id;
	u8  l0_e_pri;
	u8  l0_e_drr_id;
	u16 l0_c_drr_wt;
	u16 l0_e_drr_wt;

	/* L0 Queue IDs */
	u32 l0_uc_queue_id; /* Unicast queue id */
	u32 l0_mc_queue_id; /* Multicast queue id */

	/* Resource ranges from port_scheduler_resource */
	u32 ucast_queue_start;
	u32 ucast_queue_end;
	u32 mcast_queue_start;
	u32 mcast_queue_end;
	u32 l0cdrr_start;
	u32 l0cdrr_end;

	/* Parsing validity flag */
	bool valid;
};

/* PPE port configuration using bit fields */
struct ppe_port_config {
	u32 port_cpu:4;
	u32 port_eth_start:4;
	u32 port_eth_end:4;
	u32 port_loopback:4;
	u32 port_eip:4;
	u32 num_ports:4;
	u32 reserved:8;
} __aligned(4);

struct ppe_info {
	phys_addr_t base;
	u32 tdm_offset;
	u32 tdm_ctrl_val;
	u32 vsi;
	u8 no_ports;
	u8 nos_iports;
	u8 tdm_mode;
	u8 no_reg;
	u8 nbport; /* non bridge port*/
	u8 ipo_action;
	bool tm;
	bool bridge_mode;
	/* Bridge-mode VSI members; zero selects the hardware default. */
	u32 bridge_port_mask;
	/* Per-port scheduler configuration (passed via ppe_info) */
	struct port_scheduler_cfg *port_sched_cfg;
	u32 port_sched_cfg_len;
	/* Port configuration structure (32-bit packed) */
	struct ppe_port_config port_cfg;
} __aligned(8);

struct ipq_eth_dev {
	phys_addr_t uniphy_base;
	struct udevice *dev;
	struct ppe_info ppe;
	struct port_info *port[CONFIG_ETH_MAX_MAC];
	struct ipq_edma_hw hw;
	size_t uniphy_size;
	int uniphy_50mhz;
	bool emulation;
};

/* Generic register read/write functions */
static inline void reg_read(phys_addr_t base_addr, size_t size, size_t increment, u32 *val_ptr)
{
	size_t i;

	for (i = 0; i < size; i++)
		val_ptr[i] = readl(base_addr + (i * increment));
}

/* Generic macros that work for both single values and arrays */
#define REG_READ(addr, var) \
	reg_read((addr), sizeof(var) / sizeof(u32), sizeof(u32), \
		 (u32 *)(void *)&(var))

#define REG_WRITE(addr, var) \
	reg_write((addr), sizeof(var) / sizeof(u32), sizeof(u32), \
		  (u32 *)(void *)&(var))

/* Legacy array-specific macros (kept for backward compatibility) */
#define REG_READ_ARRAY(addr, arr) REG_READ(addr, arr)
#define REG_WRITE_ARRAY(addr, arr) REG_WRITE(addr, arr)

/* reg_write() - Write one or more registers with configurable increment */
static inline void reg_write(phys_addr_t base_addr, size_t size,
			      size_t increment, const u32 *val_ptr)
{
	size_t i;

	for (i = 0; i < size; i++)
		writel(val_ptr[i], base_addr + (i * increment));
}

/* TDM Direction Definitions */
#define FAL_PORT_TDB_DIR_INGRESS    2
#define FAL_PORT_TDB_DIR_EGRESS     3

/* TDM control register constants */
#define TDM_CTRL_TDM_EN_SHIFT		31
#define TDM_CTRL_TDM_EN_MASK		BIT(TDM_CTRL_TDM_EN_SHIFT)
#define TDM_CTRL_TDM_OFFSET_SHIFT	8
#define TDM_CTRL_TDM_OFFSET_MASK	(0x7F << TDM_CTRL_TDM_OFFSET_SHIFT)
#define TDM_CTRL_TDM_DEPTH_SHIFT	0
#define TDM_CTRL_TDM_DEPTH_MASK		(0xFF << TDM_CTRL_TDM_DEPTH_SHIFT)

#ifndef A_TRUE
#define A_TRUE  1
#define A_FALSE 0
#endif

/* TDM Tick Conversion Macro */
#define TDM(valid, dir, port, port1, port2)  (((dir) << 4) | ((port) & 0xF))

/* Port Scheduler Configuration Macros */
#define PSCH(ens_bmp, ens_port, des_port, des_sec_en, des_sec_port) \
	(((des_port) & 0xF) | \
	(((ens_port) & 0xF) << 4) | \
	(((ens_bmp) & 0x1FF) << 8) | \
	(((des_sec_en) & 0x1) << 17) | \
	(((des_sec_port) & 0xF) << 18))

/* Port Scheduler Extraction */
#define PSCH_GET_DES_PORT(val)       ((val) & 0xF)
#define PSCH_GET_ENS_PORT(val)       (((val) >> 4) & 0xF)
#define PSCH_GET_ENS_BMP(val)        (((val) >> 8) & 0x1FF)
#define PSCH_GET_DES_SEC_EN(val)     (((val) >> 17) & 0x1)
#define PSCH_GET_DES_SEC_PORT(val)   (((val) >> 18) & 0xF)

/* Layout V2 compatibility aliases */
#define PPE_PORT_BRIDGE_CTRL_PORT_ISOLATION_BMP_V2	\
		PPE_PORT_BRIDGE_CTRL_PORT_ISOLATION_BMP
#define PPE_PORT_BRIDGE_CTRL_TXMAC_EN_V2		\
		PPE_PORT_BRIDGE_CTRL_TXMAC_EN
#define PPE_PORT_BRIDGE_CTRL_PROMISC_EN_V2		\
		PPE_PORT_BRIDGE_CTRL_PROMISC_EN

/* Layout V4 - used by IPQ95xx/HMSS PPE */
#ifndef PPE_PORT_BRIDGE_CTRL_PORT_ISOLATION_BMP_V4
#define PPE_PORT_BRIDGE_CTRL_PORT_ISOLATION_BMP_V4	0x1ff00
#endif
#ifndef PPE_PORT_BRIDGE_CTRL_TXMAC_EN_V4
#define PPE_PORT_BRIDGE_CTRL_TXMAC_EN_V4		0x20000
#endif
#ifndef PPE_PORT_BRIDGE_CTRL_PROMISC_EN_V4
#define PPE_PORT_BRIDGE_CTRL_PROMISC_EN_V4		0x40000
#endif

/* CST Port State Values */
#define CST_STATE_DISABLED				0x0
#define CST_STATE_BLOCKING				0x1
#define CST_STATE_LEARNING				0x2
#define CST_STATE_FORWARDING				0x3

void ipq_config_cmn_clock(void);
u16 ipq_get_group_buf(void);
u16 ipq_get_ac_group_total_buf(void);

/* Per-SoC dynamic masks provided by nss-switch.c */
u32 ppe_port_bridge_txmac_mask(void);
u32 ppe_port_bridge_promisc_mask(void);
u32 ppe_port_bridge_isolation_mask(unsigned int nos_iports);
/* ========================================================================
 * Main API - Works for Both V1 and V2
 * ========================================================================
 */
/**
 * uniphy_get_csr_version - Get current CSR version
 * Return: CSR_VERSION_V1 or CSR_VERSION_V2
 */
enum csr_version uniphy_get_csr_version(void);

/**
 * uniphy_get_reset_version - Get SoC-specific USXGMII port reset version
 *
 * Weak default returns RESET_VERSION_V1 (VR_XS_PCS_DIG_CTRL1 USRA_RST path).
 * SoCs that use the QP_USXG_RESET active-LOW path provide a strong override
 * in their *_port_config.c returning RESET_VERSION_V2.
 *
 * Return: RESET_VERSION_V1 or RESET_VERSION_V2
 */
enum reset_version uniphy_get_reset_version(void);

/**
 * uniphy_set_base_addr - Set UNIPHY base address
 * @base_addr: Physical base address for UNIPHY registers
 *
 * Must be called to configure the base address before CSR access
 */
void uniphy_set_base_addr(phys_addr_t base_addr);

/**
 * uniphy_get_base_addr - Get current UNIPHY base address
 * Return: Physical base address for UNIPHY registers
 */
phys_addr_t uniphy_get_base_addr(void);

/**
 * csr_write - Write UNIPHY CSR register
 * @uniphy_index: UNIPHY instance (0, 1, or 2)
 * @addr: Register address (should be CSR1 encoded)
 * @value: Value to write
 *
 * Works for both V1 and V2 CSR methods. Address should be encoded with CSR1_ADDR().
 * V2: Uses csr_write_v2() with type detection
 * V1: Strips encoding and uses csr_write_v1()
 */
void csr_write(int uniphy_index, u32 addr, u32 value);

/**
 * csr_read - Read UNIPHY CSR register
 * @uniphy_index: UNIPHY instance (0, 1, or 2)
 * @addr: Register address (should be CSR1 encoded)
 * Return: Register value
 *
 * Works for both V1 and V2 CSR methods. Address should be encoded with CSR1_ADDR().
 * V2: Uses csr_read_v2() with type detection
 * V1: Strips encoding and uses csr_read_v1()
 */
u32 csr_read(int uniphy_index, u32 addr);

int uniphy_pma_init_setting(struct port_info *port, u32 uniphy_mode,
			    u32 dfe_mode, bool is_long);
int ppe_uniphy_serdes_calibration(struct port_info *port);
int ppe_uniphy_calibration(struct port_info *port);
/* SoC-specific port initialization - implemented in each *_port_config.c */
void port_init(struct port_info *port);

/**
 * ppe_uniphy_uxgmii_mode_ctrl_val - Return UQXGMII/UDXGMII MODE_CONTROL value
 *
 * Weak default in nss_switch_v2.c returns 0x1021 (XPCS_MODE | CH0_25M | AUTONEG).
 * SoCs that also need USXG_EN (bit 13) provide a strong override in their
 * *_port_config.c (e.g. IPQ9650 returns 0x3021).
 *
 * Return: 32-bit value to write to PPE_UNIPHY_MODE_CONTROL
 */
u32 ppe_uniphy_uxgmii_mode_ctrl_val(void);
#endif /* __NSS_SWITCH_H__ */
