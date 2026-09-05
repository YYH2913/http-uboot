/*
 * Copyright (c) 2016-2019, 2021, The Linux Foundation. All rights reserved.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "nss-switch.h"
#include <blk.h>
#include <command.h>
#include <console.h>
#include <cpu_func.h>
#include <dm/device.h>
#include <dm/device-internal.h>
#include <env.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <part.h>
#include <u-boot/crc.h>

DECLARE_GLOBAL_DATA_PTR;

#define REG_DELAY			1
#define RESET_DELAY			10
#define USXGMII_RESET_DELAY		100
#define USXGMII_PLL_DELAY		100
#define NSS_PORT_CLK_DATA_UNIPHY_MASK	0xff
#define NSS_PORT_CLK_DATA_PARENT_312_5	BIT(8)

static int tftp_acl_our_port;
static enum ppe_bridge_ctrl_layout current_bridge_layout =
	PPE_BRIDGE_CTRL_LAYOUT_UNKNOWN;
static struct ppe_info *ipq_active_ppe;
static phys_addr_t uniphy_base_addr = 0x07a00000;
static enum csr_version current_csr_version = CSR_VERSION_V1;
static enum reset_version current_reset_version = RESET_VERSION_V1;

uchar ipq_def_enetaddr[6] = {0x00, 0x03, 0x7F, 0xBA, 0xDB, 0xAD};
int mac_speed_config[] = {10, 100, 1000, 10000, 2500, 5000};
static u32 ipq_edma_rx_log_count;
static u32 ipq_edma_tx_log_count;
static u32 ipq_edma_txc_log_count;
static u32 ipq_edma_rx_frame_log_count;
static u32 ipq_edma_tx_frame_log_count;
static u32 ipq_edma_tx_stall_dump_count;
static ulong ipq_edma_debug_last_ms;
static ulong ipq_eth_link_refresh_last_ms;
static u32 ipq_edma_start_generation;

enum ipq_edma_debug_tdes4_mode {
	IPQ_EDMA_TDES4_LEGACY = 0,
	IPQ_EDMA_TDES4_SRCDST,
	IPQ_EDMA_TDES4_DST,
	IPQ_EDMA_TDES4_RAW,
};

static u32 ipq_edma_debug_port_ctrl_override;
static u32 ipq_edma_debug_dmar_mode;
static u32 ipq_edma_debug_tdes4_mode = IPQ_EDMA_TDES4_LEGACY;
static u32 ipq_edma_debug_txdesc_ctrl_base;
static u32 ipq_edma_debug_tdes4_raw;
static int ipq_edma_debug_passthrough;
static bool ipq_edma_debug_trace_enabled;

static int ipq_eth_get_port_reset(struct udevice *dev, struct port_info *port,
				  const char *port_name, const char *dev_name,
				  struct reset_ctl *rst);
static int ipq_eth_get_pcs_channel_clk(struct udevice *dev,
				       struct port_info *port,
				       const char *pcs_name,
				       const char *dev_name,
				       struct clk *clk);
static int ipq_eth_reset_edma_node(struct udevice *dev);
static ulong ipq_eth_uniphy_parent_rate(struct port_info *port);
static ulong ipq_eth_port_clk_data(struct port_info *port);

__weak enum csr_version uniphy_get_csr_version(void)
{
	return CSR_VERSION_V1;
}

__weak enum reset_version uniphy_get_reset_version(void)
{
	return RESET_VERSION_V1;
}

void uniphy_set_base_addr(phys_addr_t base_addr)
{
	uniphy_base_addr = base_addr;
}

phys_addr_t uniphy_get_base_addr(void)
{
	return uniphy_base_addr;
}

static const char *ipq_edma_tdes4_mode_name(void)
{
	switch (ipq_edma_debug_tdes4_mode) {
	case IPQ_EDMA_TDES4_SRCDST:
		return "srcdst";
	case IPQ_EDMA_TDES4_DST:
		return "dst";
	case IPQ_EDMA_TDES4_RAW:
		return "raw";
	case IPQ_EDMA_TDES4_LEGACY:
	default:
		return "legacy";
	}
}

static bool ipq_edma_debug_trace(void)
{
	return ipq_edma_debug_trace_enabled;
}

static bool ipq_edma_txdesc_passthrough_enabled(void)
{
	return ipq_edma_debug_passthrough == 1;
}

static void ipq_edma_debug_refresh_runtime(void)
{
	ipq_edma_debug_trace_enabled = env_get_yesno("nss_debug_log") == 1;
}

static void ipq_edma_debug_reset_defaults(void)
{
	current_bridge_layout = PPE_BRIDGE_CTRL_LAYOUT_UNKNOWN;

	ipq_edma_debug_port_ctrl_override = 0;
	ipq_edma_debug_dmar_mode = 1;
	ipq_edma_debug_tdes4_mode = IPQ_EDMA_TDES4_LEGACY;
	ipq_edma_debug_txdesc_ctrl_base = 0;
	ipq_edma_debug_tdes4_raw = 0;
	ipq_edma_debug_passthrough = 0;
	ipq_edma_debug_refresh_runtime();
}

static int ipq_eth_clk_set_rate(struct clk *clk, ulong rate)
{
	ulong new_rate;

	new_rate = clk_set_rate(clk, rate);
	if (IS_ERR_VALUE(new_rate))
		return (int)new_rate;

	return 0;
}

static enum ppe_bridge_ctrl_layout ppe_detect_bridge_ctrl_layout(void)
{
	const char *layout;

	layout = env_get("nss_bridge_layout");
	if (layout && !strcmp(layout, "v4")) {
		current_bridge_layout = PPE_V4;
		return current_bridge_layout;
	}

	if (layout && !strcmp(layout, "v2")) {
		current_bridge_layout = PPE_V2;
		return current_bridge_layout;
	}

	if (current_bridge_layout != PPE_BRIDGE_CTRL_LAYOUT_UNKNOWN)
		return current_bridge_layout;

	/*
	 * Keep the default bridge control layout identical to QSDK.  The V4
	 * masks are useful for experiments, but they must be selected
	 * explicitly with nss_debug bridge_layout v4.
	 */
	current_bridge_layout = PPE_V2;

	return current_bridge_layout;
}

static const char *ppe_bridge_ctrl_layout_name(void)
{
	switch (ppe_detect_bridge_ctrl_layout()) {
	case PPE_V4:
		return "v4";
	case PPE_V2:
	default:
		return "v2";
	}
}

u32 ppe_port_bridge_txmac_mask(void)
{
	switch (ppe_detect_bridge_ctrl_layout()) {
	case PPE_V4:
		return PPE_PORT_BRIDGE_CTRL_TXMAC_EN_V4;
	case PPE_V2:
	default:
		return PPE_PORT_BRIDGE_CTRL_TXMAC_EN_V2;
	}
}

u32 ppe_port_bridge_promisc_mask(void)
{
	switch (ppe_detect_bridge_ctrl_layout()) {
	case PPE_V4:
		return PPE_PORT_BRIDGE_CTRL_PROMISC_EN_V4;
	case PPE_V2:
	default:
		return PPE_PORT_BRIDGE_CTRL_PROMISC_EN_V2;
	}
}

u32 ppe_port_bridge_isolation_mask(unsigned int nos_iports)
{
	(void)nos_iports;

	switch (ppe_detect_bridge_ctrl_layout()) {
	case PPE_V4:
		return PPE_PORT_BRIDGE_CTRL_PORT_ISOLATION_BMP_V4;
	case PPE_V2:
	default:
		return PPE_PORT_BRIDGE_CTRL_PORT_ISOLATION_BMP_V2;
	}
}

#define QCA8075_LANES				5
#define QCA8075_PSGMII_ADDR_OFFSET		5
#define QCA8075_PHY_CHIP_CONFIG			0x1f
#define QCA8075_PHY_SPEC_STATUS			0x11
#define QCA8075_PSGMII_TX_DRIVER_1_CTRL		0x0b
#define QCA8075_MMD1_PSGMII_MODE_CTRL		0x6d
#define QCA8075_MMD1_PSGMII_FIFO_CTRL		0x6e
#define QCA8075_MMD3_REMOTE_LOOPBACK_CTRL	0x805a
#define QCA8075_MMD3_REMOTE_LOOPBACK_EN		BIT(0)
#define QCA8075_MMD3_REMOTE_LOOPBACK_CUT_RX	BIT(1)
#define QCA8075_MMD7_EEE_CTRL			0x003c
#define QCA8075_MMD7_DAC_CTRL			0x801a
#define QCA8075_MMD7_CRC_PACKET_COUNTER		0x8029
#define QCA8075_MMD7_VALID_INGRESS_COUNTER_HI	0x802a
#define QCA8075_MMD7_VALID_INGRESS_COUNTER_LO	0x802b
#define QCA8075_MMD7_ERRED_INGRESS_COUNTER	0x802c
#define QCA8075_MMD7_VALID_EGRESS_COUNTER_HI	0x802d
#define QCA8075_MMD7_VALID_EGRESS_COUNTER_LO	0x802e
#define QCA8075_MMD7_ERRED_EGRESS_COUNTER	0x802f
#define QCA8075_MMD7_FRAME_CHK_EN		BIT(0)
#define QCA8075_MMD7_COUNTER_SELFCLR		BIT(1)

#define QCA8081_PHY_SPEC_STATUS		0x11
#define QCA8081_STATUS_LINK_PASS		BIT(10)
#define QCA8081_STATUS_FULL_DUPLEX		BIT(13)
#define QCA8081_STATUS_SPEED_MASK		0x0380
#define QCA8081_STATUS_SPEED_2500MBS		0x0200
#define QCA8081_STATUS_SPEED_1000MBS		0x0100
#define QCA8081_STATUS_SPEED_100MBS		0x0080
#define QCA8081_STATUS_SPEED_10MBS		0x0000
#define QCA8081_MMD3_CLD_CTRL7			0x8007
#define QCA8081_MMD3_AZ_TRAINING_CTRL		0x8008
#define QCA8081_MMD7_CHIP_TYPE			0x901d
#define QCA8081_CHIP_TYPE_1G			BIT(0)
#define QCA8081_SERDES_ADDR_OFFSET		1
#define QCA8081_SERDES_MMD1_FIFO_CTRL		0x9072
#define QCA8081_FIFO_RSTN			BIT(11)

#ifdef CONFIG_MDIO_QCOM_I2C
extern struct mii_dev *qcom_mdio_i2c_alloc(struct udevice *i2c_bus,
					   int phy_addr);
#endif /* CONFIG_MDIO_QCOM_I2C */
#ifdef CONFIG_PHY_AQUANTIA
static int ipq_aquantia_load_fw(struct phy_device *phydev);
#endif

#if defined(CONFIG_CMD_NET) && defined(CONFIG_ETH_SKIP_INIT_R)
extern int initr_net(void);
#endif

#ifdef CONFIG_PHY_AQUANTIA
struct mbn_header {
	unsigned int image_type;
	unsigned int header_vsn_num;
	unsigned int image_src;
	unsigned int image_dest_ptr;
	unsigned int image_size;
	unsigned int code_size;
	unsigned int signature_ptr;
	unsigned int signature_size;
	unsigned int cert_chain_ptr;
	unsigned int cert_chain_size;
};

static int ipq_aquantia_load_memory(struct phy_device *phydev, u32 addr,
				    const u8 *data, size_t len)
{
	size_t pos;
	u16 crc = 0, up_crc;

	phy_write(phydev, MDIO_MMD_VEND1, 0x200, BIT(12));
	phy_write(phydev, MDIO_MMD_VEND1, 0x202, addr >> 16);
	phy_write(phydev, MDIO_MMD_VEND1, 0x203, addr & 0xfffc);

	for (pos = 0; pos < len; pos += min(sizeof(u32), len - pos)) {
		u32 word = 0;

		memcpy(&word, &data[pos], min(sizeof(u32), len - pos));

		phy_write(phydev, MDIO_MMD_VEND1, 0x204, (word >> 16));
		phy_write(phydev, MDIO_MMD_VEND1, 0x205, word & 0xffff);

		phy_write(phydev, MDIO_MMD_VEND1, 0x200,
			  BIT(15) | BIT(14));

		word = cpu_to_be32(word);
		crc = crc16_ccitt(crc, (u8 *)&word, sizeof(word));
	}

	up_crc = phy_read(phydev, MDIO_MMD_VEND1, 0x201);
	if (crc != up_crc) {
		printf("%s CRC Mismatch: Calculated 0x%04x PHY 0x%04x\n",
		       phydev->dev->name, crc, up_crc);
		return -EINVAL;
	}

	return 0;
}

static bool ipq_aquantia_range_valid(u32 file_size, u32 offset, u32 size)
{
	return offset <= file_size && size <= file_size - offset;
}

static int ipq_aquantia_upload_firmware(struct phy_device *phydev,
					u8 *addr, u32 file_size)
{
	u8 *buf = addr;
	u32 primary_header_ptr = 0x00000000;
	u32 primary_iram_ptr = 0x00000000;
	u32 primary_dram_ptr = 0x00000000;
	u32 primary_iram_sz = 0x00000000;
	u32 primary_dram_sz = 0x00000000;
	u32 phy_img_hdr_off = 0x300;
	u16 recorded_ggp8_val, daisy_chain_dis;
	u16 computed_crc, file_crc;
	int ret;

	if (file_size < 2 || file_size < 0xa) {
		printf("bad PHY FW size 0x%x\n", file_size);
		return -EINVAL;
	}

	phy_write(phydev, MDIO_MMD_VEND1, 0x300, 0xdead);
	phy_write(phydev, MDIO_MMD_VEND1, 0x301, 0xbeaf);
	if ((phy_read(phydev, MDIO_MMD_VEND1, 0x300) != 0xdead) &&
	    (phy_read(phydev, MDIO_MMD_VEND1, 0x301) != 0xbeaf)) {
		printf("PHY::Scratchpad Read/Write test fail\n");
		return -EIO;
	}

	file_crc = buf[file_size - 2] << 8 | buf[file_size - 1];
	computed_crc = crc16_ccitt(0, buf, file_size - 2);
	if (file_crc != computed_crc) {
		printf("CRC check failed on phy fw file\n");
		return -EIO;
	}

	printf("CRC check good on PHY FW (0x%04X)\n", computed_crc);
	daisy_chain_dis = phy_read(phydev, MDIO_MMD_VEND1, 0xc452);
	if (!(daisy_chain_dis & 0x1))
		phy_write(phydev, MDIO_MMD_VEND1, 0xc452, 0x1);

	phy_write(phydev, MDIO_MMD_VEND1, 0xc471, 0x40);
	recorded_ggp8_val = phy_read(phydev, MDIO_MMD_VEND1, 0xc447);
	if ((recorded_ggp8_val & 0x1f) != phydev->addr)
		phy_write(phydev, MDIO_MMD_VEND1, 0xc447, phydev->addr);

	phy_write(phydev, MDIO_MMD_VEND1, 0xc441, 0x4000);
	phy_write(phydev, MDIO_MMD_VEND1, 0xc001, 0x41);

	primary_header_ptr = (((buf[0x9] & 0x0F) << 8) | buf[0x8]) << 12;
	if (!ipq_aquantia_range_valid(file_size,
				      primary_header_ptr + phy_img_hdr_off,
				      0x10)) {
		printf("bad PHY FW image header offset 0x%x size 0x%x\n",
		       primary_header_ptr + phy_img_hdr_off, file_size);
		return -EINVAL;
	}

	primary_iram_ptr = (buf[primary_header_ptr + phy_img_hdr_off + 0x4 + 2] << 16) |
		(buf[primary_header_ptr + phy_img_hdr_off + 0x4 + 1] << 8) |
		buf[primary_header_ptr + phy_img_hdr_off + 0x4];
	primary_iram_sz = (buf[primary_header_ptr + phy_img_hdr_off + 0x7 + 2] << 16) |
		(buf[primary_header_ptr + phy_img_hdr_off + 0x7 + 1] << 8) |
		buf[primary_header_ptr + phy_img_hdr_off + 0x7];
	primary_dram_ptr = (buf[primary_header_ptr + phy_img_hdr_off + 0xA + 2] << 16) |
		(buf[primary_header_ptr + phy_img_hdr_off + 0xA + 1] << 8) |
		buf[primary_header_ptr + phy_img_hdr_off + 0xA];
	primary_dram_sz = (buf[primary_header_ptr + phy_img_hdr_off + 0xD + 2] << 16) |
		(buf[primary_header_ptr + phy_img_hdr_off + 0xD + 1] << 8) |
		buf[primary_header_ptr + phy_img_hdr_off + 0xD];
	primary_iram_ptr += primary_header_ptr;
	primary_dram_ptr += primary_header_ptr;
	if (!ipq_aquantia_range_valid(file_size, primary_iram_ptr,
				      primary_iram_sz) ||
	    !ipq_aquantia_range_valid(file_size, primary_dram_ptr,
				      primary_dram_sz)) {
		printf("bad PHY FW payload ranges iram=0x%x/0x%x dram=0x%x/0x%x file=0x%x\n",
		       primary_iram_ptr, primary_iram_sz,
		       primary_dram_ptr, primary_dram_sz, file_size);
		return -EINVAL;
	}

	phy_write(phydev, MDIO_MMD_VEND1, 0x200, 0x1000);
	phy_write(phydev, MDIO_MMD_VEND1, 0x200, 0x0);
	computed_crc = 0;

	printf("PHYFW:Loading IRAM...........");
	ret = ipq_aquantia_load_memory(phydev, 0x40000000,
				       &buf[primary_iram_ptr], primary_iram_sz);
	if (ret < 0)
		return ret;
	printf("done.\n");

	printf("PHYFW:Loading DRAM..............");
	ret = ipq_aquantia_load_memory(phydev, 0x3ffe0000,
				       &buf[primary_dram_ptr], primary_dram_sz);
	if (ret < 0)
		return ret;
	printf("done.\n");

	phy_write(phydev, MDIO_MMD_VEND1, 0x0, 0x0);
	phy_write(phydev, MDIO_MMD_VEND1, 0xc001, 0x41);
	phy_write(phydev, MDIO_MMD_VEND1, 0xc001, 0x8041);
	mdelay(100);

	phy_write(phydev, MDIO_MMD_VEND1, 0xc001, 0x40);
	mdelay(100);
	printf("PHYFW loading done.\n");

	return 0;
}

static int ipq_aquantia_load_fw(struct phy_device *phydev)
{
	struct blk_desc *desc;
	struct disk_partition part;
	const char *part_spec = env_get("aq_fw_part") ?: "0#ETHPHYFW";
	struct mbn_header *fwimg_header;
	u8 *fw_load_addr;
	size_t fw_size;
	int ret;

	ret = part_get_info_by_dev_and_name_or_num("mmc", part_spec,
						   &desc, &part, false);
	if (ret < 0) {
		printf("%s: failed to resolve AQ firmware partition '%s': %d\n",
		       __func__, part_spec, ret);
		return ret;
	}

	fw_size = (size_t)part.size * (size_t)part.blksz;
	fw_load_addr = malloc_cache_aligned(fw_size);
	if (!fw_load_addr)
		return -ENOMEM;

	memset(fw_load_addr, 0, fw_size);
	if (blk_dread(desc, part.start, part.size, fw_load_addr) != part.size) {
		printf("%s: failed to read AQ firmware partition '%s'\n",
		       __func__, part_spec);
		ret = -EIO;
		goto out_free;
	}

	fwimg_header = (struct mbn_header *)fw_load_addr;
	if (fwimg_header->image_type == 0x13 &&
	    fwimg_header->header_vsn_num == 0x3) {
		if (fwimg_header->image_size > fw_size - sizeof(*fwimg_header)) {
			printf("bad ETHPHYFW image size 0x%x on '%s'\n",
			       fwimg_header->image_size, part_spec);
			ret = -EINVAL;
			goto out_free;
		}

		ret = ipq_aquantia_upload_firmware(phydev,
				fw_load_addr + sizeof(struct mbn_header),
				(u32)fwimg_header->image_size);
	} else {
		printf("bad magic on ETHPHYFW partition '%s'\n", part_spec);
		ret = -EINVAL;
	}

out_free:
	free(fw_load_addr);
	return ret;
}
#endif /* CONFIG_PHY_AQUANTIA */

#ifdef CONFIG_ETH_LOW_MEM
/* Explicit DMA cache maintenance keeps heap-backed rings coherent. */
#define mem_init()
#define mem_alloc(size, align)		malloc_cache_aligned(size)
#else

static unsigned long nc_end;
static unsigned long nc_next;

/*
 * Non-Cached Memory APIs
 */
static int mem_init(void)
{
	unsigned long nc_start = NONCACHED_MEM_REGION_ADDR;

	nc_end = nc_start + NONCACHED_MEM_REGION_SIZE;
	nc_next = nc_start;

	mmu_set_region_dcache_behaviour(nc_start, nc_end - nc_start,
					DCACHE_OFF);
	return 0;
}

static phys_addr_t mem_alloc(size_t size, size_t align)
{
	phys_addr_t next = ALIGN(nc_next, align);

	if (next >= nc_end || (nc_end - next) < size)
		return 0;

	nc_next = next + size;
	return next;
}
#endif

static void ipq_edma_flush_range(void *addr, size_t size)
{
	unsigned long start = ALIGN_DOWN((uintptr_t)addr, ARCH_DMA_MINALIGN);
	unsigned long end = ALIGN((uintptr_t)addr + size, ARCH_DMA_MINALIGN);

	flush_dcache_range(start, end);
}

static void ipq_edma_invalidate_range(void *addr, size_t size)
{
	unsigned long start = ALIGN_DOWN((uintptr_t)addr, ARCH_DMA_MINALIGN);
	unsigned long end = ALIGN((uintptr_t)addr + size, ARCH_DMA_MINALIGN);

	invalidate_dcache_range(start, end);
}

/*
 * Uniphy configuration
 */
void csr1_write(int phy_id, int addr, int  value)
{
	uintptr_t addr_h, addr_l, ahb_h, ahb_l,  phy;

	phy = phy_id << UNIPHY_PHY_SHIFT;
	addr_h = (addr & 0xffffff) >> 8;
	addr_l = ((addr & CSR_INDIRECT_LOW_ADDR) << 2) |
		 (V1_CSR_DATA_OFFSET << 0xa);
	ahb_l = (addr_l & 0xffff) | (uniphy_base_addr | phy);
	ahb_h = (uniphy_base_addr | phy) + V1_CSR_INDIRECT_REG;
	writel(addr_h, ahb_h);
	writel(value, ahb_l);
}

int  csr1_read(int phy_id, int  addr)
{
	uintptr_t addr_h, addr_l, ahb_h, ahb_l, phy;

	phy = phy_id << UNIPHY_PHY_SHIFT;
	addr_h = (addr & 0xffffff) >> 8;
	addr_l = ((addr & CSR_INDIRECT_LOW_ADDR) << 2) |
		 (V1_CSR_DATA_OFFSET << 0xa);
	ahb_l = (addr_l & 0xffff) | (uniphy_base_addr | phy);
	ahb_h = (uniphy_base_addr | phy) + V1_CSR_INDIRECT_REG;
	writel(addr_h, ahb_h);
	return  readl(ahb_l);
}

static void csr_write_v2(int uniphy_index, u32 reg_addr, u32 value)
{
	uintptr_t addr_h, addr_l, ahb_h, ahb_l, phy;
	u32 csr_type, actual_addr, indirect_reg, data_offset;

	csr_type = (reg_addr & UNIPHY_CSR_BLOCK_MASK) >>
		   UNIPHY_CSR_BLOCK_SHIFT;
	actual_addr = reg_addr & UNIPHY_REG_ADDR_MASK;
	phy = uniphy_index << UNIPHY_PHY_SHIFT;

	switch (csr_type) {
	case 0:
		writel(value, (uniphy_base_addr | phy) + actual_addr);
		return;
	case 1:
		indirect_reg = V2_CSR1_INDIRECT_REG;
		data_offset = V2_CSR1_DATA_OFFSET;
		break;
	case 2:
		indirect_reg = V2_CSR2_INDIRECT_REG;
		data_offset = V2_CSR2_DATA_OFFSET;
		break;
	default:
		printf("UNIPHY CSR: invalid CSR type %u\n", csr_type);
		return;
	}

	addr_h = (actual_addr & UNIPHY_REG_ADDR_MASK) >> 8;
	addr_l = ((actual_addr & CSR_INDIRECT_LOW_ADDR) << 2) |
		 (data_offset << 0xa);
	ahb_h = (uniphy_base_addr | phy) + indirect_reg;
	ahb_l = (addr_l & 0xffff) | (uniphy_base_addr | phy);
	writel(addr_h, ahb_h);
	writel(value, ahb_l);
}

static u32 csr_read_v2(int uniphy_index, u32 reg_addr)
{
	uintptr_t addr_h, addr_l, ahb_h, ahb_l, phy;
	u32 csr_type, actual_addr, indirect_reg, data_offset;

	csr_type = (reg_addr & UNIPHY_CSR_BLOCK_MASK) >>
		   UNIPHY_CSR_BLOCK_SHIFT;
	actual_addr = reg_addr & UNIPHY_REG_ADDR_MASK;
	phy = uniphy_index << UNIPHY_PHY_SHIFT;

	switch (csr_type) {
	case 0:
		return readl((uniphy_base_addr | phy) + actual_addr);
	case 1:
		indirect_reg = V2_CSR1_INDIRECT_REG;
		data_offset = V2_CSR1_DATA_OFFSET;
		break;
	case 2:
		indirect_reg = V2_CSR2_INDIRECT_REG;
		data_offset = V2_CSR2_DATA_OFFSET;
		break;
	default:
		printf("UNIPHY CSR: invalid CSR type %u\n", csr_type);
		return 0;
	}

	addr_h = (actual_addr & UNIPHY_REG_ADDR_MASK) >> 8;
	addr_l = ((actual_addr & CSR_INDIRECT_LOW_ADDR) << 2) |
		 (data_offset << 0xa);
	ahb_h = (uniphy_base_addr | phy) + indirect_reg;
	ahb_l = (addr_l & 0xffff) | (uniphy_base_addr | phy);
	writel(addr_h, ahb_h);
	return readl(ahb_l);
}

void csr_write(int uniphy_index, u32 addr, u32 value)
{
	if (current_csr_version == CSR_VERSION_V2)
		csr_write_v2(uniphy_index, addr, value);
	else
		csr1_write(uniphy_index, addr & UNIPHY_REG_ADDR_MASK, value);
}

u32 csr_read(int uniphy_index, u32 addr)
{
	if (current_csr_version == CSR_VERSION_V2)
		return csr_read_v2(uniphy_index, addr);

	return csr1_read(uniphy_index, addr & UNIPHY_REG_ADDR_MASK);
}

int ppe_uniphy_calibration(struct port_info *port)
{
	int retries = 100, calibration_done = 0;
	u32 reg_value = 0;
	uintptr_t reg = port->uniphy_base + PPE_UNIPHY_OFFSET_CALIB_4;

	while (calibration_done != UNIPHY_CALIBRATION_DONE) {
		mdelay(1);
		if (retries-- == 0) {
			printf("uniphy calibration time out!\n");
			return -ETIMEDOUT;
		}
		reg_value = readl(reg);
		calibration_done = (reg_value >> 0x7) & 0x1;
	}

	return 0;
}

void ipq_port_reset(struct reset_ctl *rst, bool set)
{
	if (set)
		reset_assert(rst);
	else
		reset_deassert(rst);
}

static int ipq_eth_get_node_reset(ofnode node, const char *name,
				  struct reset_ctl *rst)
{
	int index;

	if (!ofnode_valid(node))
		return -ENOENT;

	index = ofnode_stringlist_search(node, "reset-names", name);
	if (index < 0)
		return -ENOENT;

	return reset_get_by_index_nodev(node, index, rst);
}

static void ppe_uniphy_reset(struct port_info *port, bool issoft, bool set)
{
	struct udevice *dev;
	struct reset_ctl rst;
	int ret;
	char name[64];

	if (!port || !port->dev)
		return;

	dev = port->dev;

	if (!issoft && ofnode_valid(port->pcs_node)) {
		ofnode parent = ofnode_get_parent(port->pcs_node);

		ret = ipq_eth_get_node_reset(parent, "xpcs", &rst);
		if (ret)
			ret = reset_get_by_index_nodev(parent, 0, &rst);
		if (!ret) {
			ipq_port_reset(&rst, set);
			reset_free(&rst);
			return;
		}
	}

	snprintf(name, sizeof(name), "uniphy%d_%s", port->uniphy_id,
		 (issoft) ? "sys_rst" : "xpcs_rst");

	ret = reset_get_by_name(dev, name, &rst);
	if (ret) {
		snprintf(name, sizeof(name), "uniphy%d_%s", port->uniphy_id,
			 (issoft) ? "srst" : "xrst");
		ret = reset_get_by_name(dev, name, &rst);
	}

	if (!ret)
		ipq_port_reset(&rst, set);

	if (issoft) {
		snprintf(name, sizeof(name), "uniphy_port%d_tx_rst", port->id);

		ret = reset_get_by_name(dev, name, &rst);
		if (ret) {
			snprintf(name, sizeof(name), "uniphy_port%d_tx", port->id);
			ret = reset_get_by_name(dev, name, &rst);
		}

		if (!ret)
			ipq_port_reset(&rst, set);

		snprintf(name, sizeof(name), "uniphy_port%d_rx_rst", port->id);

		ret = reset_get_by_name(dev, name, &rst);
		if (ret) {
			snprintf(name, sizeof(name), "uniphy_port%d_rx", port->id);
			ret = reset_get_by_name(dev, name, &rst);
		}

		if (!ret)
			ipq_port_reset(&rst, set);
	}
}

static void ppe_uniphy_psgmii_mode_set(struct port_info *port)
{
	if (ipq_edma_debug_trace())
		printf("UNIPHY%u: set PSGMII mode for port%u\n",
		       port->uniphy_id, port->id);

	ppe_uniphy_reset(port, false, true);

	writel(0x220, port->uniphy_base + PPE_UNIPHY_MODE_CONTROL);

	ppe_uniphy_reset(port, true, true);
	mdelay(RESET_DELAY);
	ppe_uniphy_reset(port, true, false);
	mdelay(RESET_DELAY);

	ppe_uniphy_calibration(port);
}

static void ppe_uniphy_qsgmii_mode_set(struct port_info *port)
{
	if (ipq_edma_debug_trace())
		printf("UNIPHY%u: set QSGMII mode for port%u\n",
		       port->uniphy_id, port->id);

	ppe_uniphy_reset(port, false, true);

	writel(0x120, port->uniphy_base + PPE_UNIPHY_MODE_CONTROL);

	ppe_uniphy_reset(port, true, true);
	mdelay(RESET_DELAY);
	ppe_uniphy_reset(port, true, false);
	mdelay(RESET_DELAY);
}

void ppe_uniphy_set_forcemode(struct port_info *port)
{
	u32 reg_value;

	reg_value = readl(port->uniphy_base +
				UNIPHY_DEC_CHANNEL_0_INPUT_OUTPUT_4);

	reg_value |= UNIPHY_FORCE_SPEED_25M;

	writel(reg_value, port->uniphy_base +
				UNIPHY_DEC_CHANNEL_0_INPUT_OUTPUT_4);
}

void ppe_uniphy_refclk_set_25M(struct port_info *port)
{
	u32 reg_value;

	reg_value = readl(port->uniphy_base + UNIPHY1_CLKOUT_50M_CTRL_OPTION);

	reg_value |= (UNIPHY1_CLKOUT_50M_CTRL_CLK50M_DIV2_SEL |
				UNIPHY1_CLKOUT_50M_CTRL_50M_25M_EN);

	writel(reg_value, port->uniphy_base + UNIPHY1_CLKOUT_50M_CTRL_OPTION);
}

static bool ppe_uniphy_port_is_sfp(struct port_info *port)
{
	return port && (port->phy_id == SFP10G_PHY_TYPE ||
			port->phy_id == SFP2_5G_PHY_TYPE ||
			port->phy_id == SFP1G_PHY_TYPE);
}

static u32 ppe_uniphy_mii_ctrl_addr(u32 portid, u32 uniphy_index)
{
	if (uniphy_index != 0 || portid == 1)
		return SR_MII_CTRL_ADDRESS;

	switch (portid) {
	case 2:
		return SR_MII_CTRL_CHANNEL1_ADDRESS;
	case 3:
		return SR_MII_CTRL_CHANNEL2_ADDRESS;
	case 4:
		return SR_MII_CTRL_CHANNEL3_ADDRESS;
	default:
		return SR_MII_CTRL_ADDRESS;
	}
}

static u32 ppe_uniphy_an_intr_sts_addr(u32 portid, u32 uniphy_index)
{
	if (uniphy_index != 0 || portid == 1)
		return VR_MII_AN_INTR_STS;

	switch (portid) {
	case 2:
		return VR_MII_AN_INTR_STS_CHANNEL1_ADDRESS;
	case 3:
		return VR_MII_AN_INTR_STS_CHANNEL2_ADDRESS;
	case 4:
		return VR_MII_AN_INTR_STS_CHANNEL3_ADDRESS;
	default:
		return VR_MII_AN_INTR_STS;
	}
}

static u32 ppe_uniphy_an_ctrl_addr(u32 portid, u32 uniphy_index)
{
	if (uniphy_index != 0 || portid == 1)
		return VR_MII_AN_CTRL_ADDRESS;

	switch (portid) {
	case 2:
		return VR_MII_AN_CTRL_CHANNEL1_ADDRESS;
	case 3:
		return VR_MII_AN_CTRL_CHANNEL2_ADDRESS;
	case 4:
		return VR_MII_AN_CTRL_CHANNEL3_ADDRESS;
	default:
		return VR_MII_AN_CTRL_ADDRESS;
	}
}

static void ppe_uniphy_usxgmii_restart_autoneg(struct port_info *port)
{
	u32 mii_ctrl_address, reg_value;

	mii_ctrl_address = ppe_uniphy_mii_ctrl_addr(port->id, port->uniphy_id);
	reg_value = csr1_read(port->uniphy_id, mii_ctrl_address);
	reg_value |= AN_RESTART;
	csr1_write(port->uniphy_id, mii_ctrl_address, reg_value);
}

static void ppe_uniphy_sgmii_mode_set(struct port_info *port)
{
	phys_addr_t base = port->uniphy_base;
	u32 reg_value;

	ppe_uniphy_reset(port, false, true);

	writel(UNIPHY_MISC_SRC_PHY_MODE, base +
			UNIPHY_MISC_SOURCE_SELECTION_REG_OFFSET);

	if (port->uniphy_mode == PORT_WRAPPER_SGMII_PLUS)
		reg_value = UNIPHY_MISC2_REG_SGMII_PLUS_MODE;
	else
		reg_value = UNIPHY_MISC2_REG_SGMII_MODE;

	writel(reg_value, base + UNIPHY_MISC2_REG_OFFSET);

	writel(UNIPHY_PLL_RESET_REG_VALUE, base + UNIPHY_PLL_RESET_REG_OFFSET);

	mdelay(REG_DELAY);

	writel(UNIPHY_PLL_RESET_REG_DEFAULT_VALUE,
	       base + UNIPHY_PLL_RESET_REG_OFFSET);

	mdelay(REG_DELAY);

	switch (port->uniphy_mode) {
	case PORT_WRAPPER_SGMII_FIBER:
		writel(0x400, base + PPE_UNIPHY_MODE_CONTROL);
		break;
	case PORT_WRAPPER_SGMII0_RGMII4:
	case PORT_WRAPPER_SGMII1_RGMII4:
	case PORT_WRAPPER_SGMII4_RGMII4:
		writel(0x420, base + PPE_UNIPHY_MODE_CONTROL);
		break;

	case PORT_WRAPPER_SGMII_PLUS:
		writel(0x820, base + PPE_UNIPHY_MODE_CONTROL);
		break;
	default:
		printf("SGMII Config. wrongly");
		break;
	}

	ppe_uniphy_reset(port, true, true);
	mdelay(RESET_DELAY);
	ppe_uniphy_reset(port, true, false);
	mdelay(RESET_DELAY);

	ppe_uniphy_calibration(port);
}

static int ppe_uniphy_10g_r_linkup(u32 uniphy_index)
{
	u32 reg_value = 0;
	u32 retries = 100, linkup = 0;

	while (linkup != UNIPHY_10GR_LINKUP) {
		mdelay(1);
		if (retries-- == 0)
			return -ETIMEDOUT;
		reg_value = csr1_read(uniphy_index, SR_XS_PCS_KR_STS1_ADDRESS);
		linkup = (reg_value >> 12) & UNIPHY_10GR_LINKUP;
	}
	mdelay(REG_DELAY);
	return 0;
}

static int ppe_uniphy_port_clocks_set(struct port_info *port, bool enable)
{
	struct clk rx_clk, tx_clk;
	char clk_name[64];
	int ret;

	snprintf(clk_name, sizeof(clk_name),
		 "nss_cc_uniphy_port%d_rx_clk", port->id);
	ret = ipq_eth_get_pcs_channel_clk(port->dev, port, "rx", clk_name,
					  &rx_clk);
	if (ret)
		return ret;

	snprintf(clk_name, sizeof(clk_name),
		 "nss_cc_uniphy_port%d_tx_clk", port->id);
	ret = ipq_eth_get_pcs_channel_clk(port->dev, port, "tx", clk_name,
					  &tx_clk);
	if (ret)
		return ret;

	ret = enable ? clk_enable(&rx_clk) : clk_disable(&rx_clk);
	if (ret)
		return ret;

	ret = enable ? clk_enable(&tx_clk) : clk_disable(&tx_clk);
	if (ret) {
		if (enable)
			clk_disable(&rx_clk);
		else
			clk_enable(&rx_clk);
		return ret;
	}

	return 0;
}

static void ppe_uniphy_10g_r_mode_set(struct port_info *port)
{
	ppe_uniphy_reset(port, false, true);

	writel(0x1021, port->uniphy_base + PPE_UNIPHY_MODE_CONTROL);

	writel(0x1C0, port->uniphy_base + UNIPHY_INSTANCE_LINK_DETECT);

	ppe_uniphy_reset(port, true, true);
	mdelay(RESET_DELAY);
	ppe_uniphy_reset(port, true, false);
	mdelay(RESET_DELAY);

	ppe_uniphy_calibration(port);

	ppe_uniphy_reset(port, false, false);
}

static int ppe_uniphy_usxgmii_mode_set(struct port_info *port)
{
	u32 index = port->uniphy_id;
	phys_addr_t base = port->uniphy_base;
	u32 reg_value;
	int ret, restore_ret;

	/* Keep the PCS channel clocks off throughout mode reconfiguration. */
	ret = ppe_uniphy_port_clocks_set(port, false);
	if (ret) {
		printf("UNIPHY%u port%u clock disable failed: %d\n",
		       port->uniphy_id, port->id, ret);
		return ret;
	}
	ppe_uniphy_reset(port, false, true);

	writel(UNIPHY_MISC2_REG_VALUE, base + UNIPHY_MISC2_REG_OFFSET);
	writel(0x1021, base + PPE_UNIPHY_MODE_CONTROL);

	if (!ppe_uniphy_port_is_sfp(port)) {
		reg_value = readl(base + UNIPHY_INSTANCE_LINK_DETECT);
		reg_value &= ~UNIPHY_DETECT_LOS_FROM_SFP;
		writel(reg_value, base + UNIPHY_INSTANCE_LINK_DETECT);
	}

	ppe_uniphy_reset(port, true, true);
	mdelay(USXGMII_RESET_DELAY);
	ppe_uniphy_reset(port, true, false);
	mdelay(USXGMII_RESET_DELAY);

	/* Program the complete analog PLL reset values used by OpenWrt. */
	writel(UNIPHY_PLL_RESET_REG_VALUE, base + UNIPHY_PLL_RESET_REG_OFFSET);
	mdelay(USXGMII_PLL_DELAY);
	writel(UNIPHY_PLL_RESET_REG_DEFAULT_VALUE,
	       base + UNIPHY_PLL_RESET_REG_OFFSET);
	mdelay(USXGMII_PLL_DELAY);

	ret = ppe_uniphy_calibration(port);
	if (ret)
		goto out_restore;

	ret = ppe_uniphy_port_clocks_set(port, true);
	if (ret) {
		printf("UNIPHY%u port%u clock restore failed: %d\n",
		       port->uniphy_id, port->id, ret);
		goto out_release;
	}
	if (port->phy_25mhz)
		ppe_uniphy_refclk_set_25M(port);

	ppe_uniphy_reset(port, false, false);

	mdelay(RESET_DELAY);

	ret = ppe_uniphy_10g_r_linkup(index);
	if (ret)
		printf("UNIPHY%u port%u 10GBASE-R link not up before USXG_EN\n",
		       port->uniphy_id, port->id);

	reg_value = csr1_read(index, VR_XS_PCS_DIG_CTRL1_ADDRESS);
	reg_value |= USXG_EN;
	csr1_write(index, VR_XS_PCS_DIG_CTRL1_ADDRESS, reg_value);
	reg_value = csr1_read(index, VR_MII_AN_CTRL_ADDRESS);
	reg_value |= MII_AN_INTR_EN;
	reg_value |= MII_CTRL;
	csr1_write(index, VR_MII_AN_CTRL_ADDRESS, reg_value);
	reg_value = csr1_read(index, SR_MII_CTRL_ADDRESS);
	reg_value |= AN_ENABLE;
	reg_value &= ~SS5;
	reg_value |= SS6 | SS13 | DUPLEX_MODE;
	csr1_write(index, SR_MII_CTRL_ADDRESS, reg_value);

	return 0;

out_restore:
	restore_ret = ppe_uniphy_port_clocks_set(port, true);
	if (restore_ret)
		printf("UNIPHY%u port%u clock restore after failure failed: %d\n",
		       port->uniphy_id, port->id, restore_ret);
out_release:
	ppe_uniphy_reset(port, false, false);
	return ret;
}

static void ppe_uniphy_uqxgmii_mode_set(struct port_info *port)
{
	u32 index = port->uniphy_id;
	phys_addr_t base = port->uniphy_base;
	u32 reg_value = 0;

	writel(UNIPHY_MISC2_REG_VALUE, base + UNIPHY_MISC2_REG_OFFSET);

	writel(UNIPHY_PLL_RESET_REG_VALUE, base + UNIPHY_PLL_RESET_REG_OFFSET);
	mdelay(REG_DELAY);

	writel(UNIPHY_PLL_RESET_REG_DEFAULT_VALUE,
	       base + UNIPHY_PLL_RESET_REG_OFFSET);
	mdelay(REG_DELAY);

	ppe_uniphy_reset(port, false, true);
	mdelay(RESET_DELAY);

	writel(0x1021, base + PPE_UNIPHY_MODE_CONTROL);

	reg_value = readl(base + UNIPHYQP_USXG_OPITON1);
	reg_value |= GMII_SRC_SEL;
	writel(reg_value, base + UNIPHYQP_USXG_OPITON1);

	ppe_uniphy_reset(port, true, true);
	mdelay(RESET_DELAY);
	ppe_uniphy_reset(port, true, false);
	mdelay(RESET_DELAY);

	ppe_uniphy_calibration(port);

	ppe_uniphy_reset(port, false, false);
	mdelay(RESET_DELAY);

	ppe_uniphy_10g_r_linkup(index);

	reg_value = csr1_read(index, VR_XS_PCS_DIG_CTRL1_ADDRESS);
	reg_value |= USXG_EN;
	csr1_write(index, VR_XS_PCS_DIG_CTRL1_ADDRESS, reg_value);

	/* set QXGMII mode */
	reg_value = csr1_read(index, VR_XS_PCS_KR_CTRL_ADDRESS);
	reg_value |= USXG_MODE;
	csr1_write(index, VR_XS_PCS_KR_CTRL_ADDRESS, reg_value);

	/* set AM interval mode */
	reg_value = csr1_read(index, VR_XS_PCS_DIG_STS_ADDRESS);
	reg_value |= AM_COUNT;
	csr1_write(index, VR_XS_PCS_DIG_STS_ADDRESS, reg_value);

	reg_value = csr1_read(index, VR_XS_PCS_DIG_CTRL1_ADDRESS);
	reg_value |= VR_RST;
	csr1_write(index, VR_XS_PCS_DIG_CTRL1_ADDRESS, reg_value);

	reg_value = csr1_read(index, VR_MII_AN_CTRL_ADDRESS);
	reg_value |= MII_AN_INTR_EN;
	reg_value |= MII_CTRL;
	csr1_write(index, VR_MII_AN_CTRL_ADDRESS, reg_value);
	csr1_write(index, VR_MII_AN_CTRL_CHANNEL1_ADDRESS, reg_value);
	csr1_write(index, VR_MII_AN_CTRL_CHANNEL2_ADDRESS, reg_value);
	csr1_write(index, VR_MII_AN_CTRL_CHANNEL3_ADDRESS, reg_value);

	/* disable TICD */
	reg_value = csr1_read(index, VR_XAUI_MODE_CTRL_ADDRESS);
	reg_value |= IPG_CHECK;
	csr1_write(index, VR_XAUI_MODE_CTRL_ADDRESS, reg_value);
	csr1_write(index, VR_XAUI_MODE_CTRL_CHANNEL1_ADDRESS, reg_value);
	csr1_write(index, VR_XAUI_MODE_CTRL_CHANNEL2_ADDRESS, reg_value);
	csr1_write(index, VR_XAUI_MODE_CTRL_CHANNEL3_ADDRESS, reg_value);

	/* enable uniphy autoneg ability and usxgmii 10g speed
	 * and full duplex
	 */
	reg_value = csr1_read(index, SR_MII_CTRL_ADDRESS);
	reg_value |= AN_ENABLE;
	reg_value &= ~SS5;
	reg_value |= SS6 | SS13 | DUPLEX_MODE;
	csr1_write(index, SR_MII_CTRL_ADDRESS, reg_value);
	csr1_write(index, SR_MII_CTRL_CHANNEL1_ADDRESS, reg_value);
	csr1_write(index, SR_MII_CTRL_CHANNEL2_ADDRESS, reg_value);
	csr1_write(index, SR_MII_CTRL_CHANNEL3_ADDRESS, reg_value);

	/* enable uniphy eee transparent mode and configure eee
	 * related timer value
	 */
	reg_value = csr1_read(index, VR_XS_PCS_EEE_MCTRL0_ADDRESS);
	reg_value |= SIGN_BIT | MULT_FACT_100NS;
	csr1_write(index, VR_XS_PCS_EEE_MCTRL0_ADDRESS, reg_value);

	reg_value = csr1_read(index, VR_XS_PCS_EEE_TXTIMER_ADDRESS);
	reg_value |= UNIPHY_XPCS_TSL_TIMER | UNIPHY_XPCS_TLU_TIMER
			| UNIPHY_XPCS_TWL_TIMER;
	csr1_write(index, VR_XS_PCS_EEE_TXTIMER_ADDRESS, reg_value);

	reg_value = csr1_read(index, VR_XS_PCS_EEE_RXTIMER_ADDRESS);
	reg_value |= UNIPHY_XPCS_100US_TIMER | UNIPHY_XPCS_TWR_TIMER;
	csr1_write(index, VR_XS_PCS_EEE_RXTIMER_ADDRESS, reg_value);

	/* Transparent LPI mode and LPI pattern enable */
	reg_value = csr1_read(index, VR_XS_PCS_EEE_MCTRL1_ADDRESS);
	reg_value |= TRN_LPI | TRN_RXLPI;
	csr1_write(index, VR_XS_PCS_EEE_MCTRL1_ADDRESS, reg_value);

	reg_value = csr1_read(index, VR_XS_PCS_EEE_MCTRL0_ADDRESS);
	reg_value |= LRX_EN | LTX_EN;
	csr1_write(index, VR_XS_PCS_EEE_MCTRL0_ADDRESS, reg_value);
}

static int ppe_uniphy_mode_set(struct port_info *port)
{
	switch (port->uniphy_mode) {
	case PORT_WRAPPER_PSGMII:
		ppe_uniphy_psgmii_mode_set(port);
		break;
	case PORT_WRAPPER_QSGMII:
		ppe_uniphy_qsgmii_mode_set(port);
		break;
	case PORT_WRAPPER_SGMII0_RGMII4:
	case PORT_WRAPPER_SGMII1_RGMII4:
	case PORT_WRAPPER_SGMII4_RGMII4:
	case PORT_WRAPPER_SGMII_PLUS:
	case PORT_WRAPPER_SGMII_FIBER:
		ppe_uniphy_sgmii_mode_set(port);
		break;
	case PORT_WRAPPER_USXGMII:
		return ppe_uniphy_usxgmii_mode_set(port);
	case PORT_WRAPPER_10GBASE_R:
		ppe_uniphy_10g_r_mode_set(port);
		break;
	case PORT_WRAPPER_UQXGMII:
		ppe_uniphy_uqxgmii_mode_set(port);
		break;
	default:
		break;
	}

	return 0;
}

void ppe_uniphy_usxgmii_autoneg_completed(struct port_info *port)
{
	u32 autoneg_complete = 0, retries = 100;
	u32 reg_value = 0;
	u32 an_intr_sts;

	an_intr_sts = ppe_uniphy_an_intr_sts_addr(port->id, port->uniphy_id);

	while (autoneg_complete != 0x1) {
		mdelay(1);
		if (retries-- == 0) {
			if (ipq_edma_debug_trace())
				printf("UNIPHY%u port%u USXGMII AN timeout sts=%08x\n",
				       port->uniphy_id, port->id, reg_value);
			return;
		}

		reg_value = csr1_read(port->uniphy_id, an_intr_sts);
		autoneg_complete = reg_value & 0x1;
	}
	reg_value &= ~CL37_ANCMPLT_INTR;
	csr1_write(port->uniphy_id, an_intr_sts, reg_value);
}

void ppe_uniphy_usxgmii_speed_set(int portid, int uniphy_index, int speed)
{
	u32 reg_value = 0;
	u32 mii_ctrl_aadress = ppe_uniphy_mii_ctrl_addr(portid, uniphy_index);

	reg_value = csr1_read(uniphy_index, mii_ctrl_aadress);
	reg_value |= DUPLEX_MODE;

	switch (speed) {
	case 0:
		reg_value &=  ~SS5;
		reg_value &=  ~SS6;
		reg_value &=  ~SS13;
		break;
	case 1:
		reg_value &=  ~SS5;
		reg_value &=  ~SS6;
		reg_value |= SS13;
		break;
	case 2:
		reg_value &=  ~SS5;
		reg_value |= SS6;
		reg_value &=  ~SS13;
		break;
	case 3:
		reg_value &=  ~SS5;
		reg_value |= SS6;
		reg_value |= SS13;
		break;
	case 4:
		reg_value |= SS5;
		reg_value &=  ~SS6;
		reg_value &=  ~SS13;
		break;
	case 5:
		reg_value |= SS5;
		reg_value &=  ~SS6;
		reg_value |= SS13;
		break;
	}

	csr1_write(uniphy_index, mii_ctrl_aadress, reg_value);
}

void ppe_uniphy_usxgmii_duplex_set(int portid, int uniphy_index, int duplex)
{
	u32 reg_value = 0;
	u32 mii_ctrl_address = ppe_uniphy_mii_ctrl_addr(portid, uniphy_index);

	reg_value = csr1_read(uniphy_index, mii_ctrl_address);

	if (duplex & 0x1)
		reg_value |= DUPLEX_MODE;
	else
		reg_value &= ~DUPLEX_MODE;

	csr1_write(uniphy_index, mii_ctrl_address, reg_value);
}

void ppe_uniphy_usxgmii_port_reset(int uniphy_index, int port_id,
				   int uniphy_mode)
{
	phys_addr_t base = uniphy_base_addr |
			   (uniphy_index << UNIPHY_PHY_SHIFT);
	u32 reg_value, rst_bit, ch_addr;

	if (current_reset_version == RESET_VERSION_V2) {
		switch (port_id) {
		case 1:
			rst_bit = QP_USXG_RST_N_MAIN;
			break;
		case 2:
			rst_bit = QP_USXG_RST_N_P1;
			break;
		case 3:
			rst_bit = QP_USXG_RST_N_P2;
			break;
		case 4:
			rst_bit = QP_USXG_RST_N_P3;
			break;
		default:
			return;
		}

		reg_value = readl(base + QP_USXG_RESET_ADDRESS);
		writel(reg_value & ~rst_bit, base + QP_USXG_RESET_ADDRESS);
		mdelay(1);
		writel(reg_value | rst_bit, base + QP_USXG_RESET_ADDRESS);
		mdelay(10);
		return;
	}

	reg_value = csr_read(uniphy_index,
			     CSR1_ADDR(VR_XS_PCS_DIG_CTRL1_ADDRESS));
	csr_write(uniphy_index, CSR1_ADDR(VR_XS_PCS_DIG_CTRL1_ADDRESS),
		  reg_value | USRA_RST);
	mdelay(10);

	if (uniphy_mode == PORT_WRAPPER_UQXGMII ||
	    uniphy_mode == PORT_WRAPPER_UQXGMII_3CHANNELS ||
	    uniphy_mode == PORT_WRAPPER_UDXGMII) {
		switch (port_id) {
		case 2:
			ch_addr = VR_MII_DIG_CTRL1_CHANNEL1_ADDRESS;
			break;
		case 3:
			ch_addr = VR_MII_DIG_CTRL1_CHANNEL2_ADDRESS;
			break;
		case 4:
			ch_addr = VR_MII_DIG_CTRL1_CHANNEL3_ADDRESS;
			break;
		default:
			ch_addr = 0;
			break;
		}

		if (ch_addr) {
			reg_value = csr_read(uniphy_index, CSR1_ADDR(ch_addr));
			csr_write(uniphy_index, CSR1_ADDR(ch_addr),
				  reg_value | USRA_RST_MII);
		}
	}

	mdelay(10);
}

static void ppe_uniphy_sgmii_adapter_reset(struct port_info *port)
{
	phys_addr_t reg = port->uniphy_base +
			  UNIPHY_DEC_CHANNEL_0_INPUT_OUTPUT_4;
	u32 value = readl(reg);

	writel(value & ~UNIPHY_MII_ADPT_RESET, reg);
	udelay(1);
	writel(value | UNIPHY_MII_ADPT_RESET, reg);
}

/*
 * PPE configuration
 */

void ppe_xgmac_configuration(phys_addr_t reg_base, u32 portid,
			     u32 speed, bool uxsgmii)
{
	u32 reg_value, gmacid = portid - 1;

	uintptr_t base = reg_base + PPE_SWITCH_NSS_SWITCH_XGMAC0 +
			 (gmacid * NSS_SWITCH_XGMAC_MAC_TX_CONFIGURATION);

	reg_value = readl(base);
	/*
	 * Jabber Disable.
	 */
	reg_value |= JD;
	/*
	 * Transmitter Enable.
	 */
	reg_value |= TE;
	/*
	 * Speed set
	 */
	reg_value &= ~XGMAC_SPEED_SELECT_MASK;
	switch (speed) {
	case 0:
	case 1:
	case 2:
		reg_value |= SS(XGMAC_SPEED_SELECT_1000M);
		break;
	case 3:
		reg_value |= SS(XGMAC_SPEED_SELECT_10000M);
		break;
	case 4:
		reg_value |= SS(XGMAC_SPEED_SELECT_2500M);
		break;
	case 5:
		reg_value |= SS(XGMAC_SPEED_SELECT_5000M);
		break;
	default:
		/* Not supported in 10G_R mode*/
		break;
	}

	if (uxsgmii && speed > 2)
		reg_value |= USS;
	else
		reg_value &=  ~USS;

	writel(reg_value, base);
	mdelay(1);
	/*
	 * Rx configuration
	 */
	reg_value = readl(base + MAC_RX_CONFIGURATION_ADDRESS);
	reg_value |= 0x300000c0;
	reg_value |= RE;
	reg_value |= ACS;
	reg_value |= CST;
	writel(reg_value, base + MAC_RX_CONFIGURATION_ADDRESS);
	mdelay(1);
	/*
	 * set up mac filter
	 */
	writel(0x80000081, base + MAC_PACKET_FILTER_ADDRESS);
}

/*
 * ppe_port_bridge_txmac_set()
 * TXMAC should be disabled for all ports by default
 * TXMAC should be enabled for all ports that are link up alone
 */
void ppe_port_bridge_txmac_set(phys_addr_t reg_base, u32 port,
			       bool isenable)
{
	u32 reg_value = readl(reg_base + IPE_L2_BASE_ADDR +
				PORT_BRIDGE_CTRL_ADDRESS +
				(port * PORT_BRIDGE_CTRL_INC));
	u32 txmac_mask = ppe_port_bridge_txmac_mask();

	if (isenable)
		reg_value |= txmac_mask;
	else
		reg_value &= ~txmac_mask;

	writel(reg_value, reg_base + IPE_L2_BASE_ADDR +
			PORT_BRIDGE_CTRL_ADDRESS +
			(port * PORT_BRIDGE_CTRL_INC));
}

u8 phy_status_get_from_ppe(phys_addr_t reg_base, u32 port_id)
{
	u32 reg_field = readl((reg_base + ((port_id > 4) ?
				PORT_PHY_STATUS_ADDRESS1 :
				PORT_PHY_STATUS_ADDRESS)));

	switch (port_id) {
	case 2:
		reg_field >>= PORT_PHY_STATUS_PORT2_OFFSET;
		break;
	case 3:
		reg_field >>= PORT_PHY_STATUS_PORT3_OFFSET;
		break;
	case 4:
		reg_field >>= PORT_PHY_STATUS_PORT4_OFFSET;
		break;
	case 5:
		reg_field >>= PORT_PHY_STATUS_PORT5_1_OFFSET;
		break;
	case 6:
		reg_field >>= PORT_PHY_STATUS_PORT6_OFFSET;
		break;
	default:
		/* case port 1 */
		break;
	}

	return (u8)reg_field;
}

static void ppe_port_mux_set(phys_addr_t reg_base, struct port_info *port)
{
	union port_mux_ctrl_u port_mux_ctrl;
	u8 id = port->id;
	u8 pcs_sel = port->uniphy_type;
	u8 mac_type = port->gmac_type;

	pr_debug("port id is: %d, mac_type is %d, uniphy_type is %d\n",
		 id, mac_type, pcs_sel);

	port_mux_ctrl.val = 0;

	port_mux_ctrl.val = readl(reg_base + PORT_MUX_CTRL);

	pr_debug("\nBEFORE UPDATE: Port MUX CTRL value is %u",
		 port_mux_ctrl.val);

	switch (id) {
	case 1:
		port_mux_ctrl.bf.port1_mac_sel = mac_type;
		port_mux_ctrl.bf.port1_pcs_sel = pcs_sel;
		break;
	case 2:
		port_mux_ctrl.bf.port2_mac_sel = mac_type;
		port_mux_ctrl.bf.port2_pcs_sel = pcs_sel;
		break;
	case 3:
		port_mux_ctrl.bf.port3_mac_sel = mac_type;
		port_mux_ctrl.bf.port3_pcs_sel = pcs_sel;
		break;
	case 4:
		port_mux_ctrl.bf.port4_mac_sel = mac_type;
		port_mux_ctrl.bf.port4_pcs_sel = pcs_sel;
		break;
	case 5:
		port_mux_ctrl.bf.port5_mac_sel = mac_type;
		port_mux_ctrl.bf.port5_pcs_sel = pcs_sel;
		break;
	case 6:
		port_mux_ctrl.bf.port6_mac_sel = mac_type;
		port_mux_ctrl.bf.port6_pcs_sel = pcs_sel;
		break;
	default:
		break;
	}

	writel(port_mux_ctrl.val, (reg_base + PORT_MUX_CTRL));
}

void ppe_port_speed_set(phys_addr_t reg_base, struct port_info *port)
{
	bool usxgmii = false;
	u32 gmacid, speed = -1;
	uintptr_t base;

	ppe_port_bridge_txmac_set(reg_base, port->id, false);

	if (port->cur_gmac_type != port->gmac_type) {
		ppe_port_mux_set(reg_base, port);
		port->cur_gmac_type = port->gmac_type;
	}

	switch (port->uniphy_mode) {
	case PORT_WRAPPER_10GBASE_R:
		break;
	case PORT_WRAPPER_UQXGMII:
	case PORT_WRAPPER_USXGMII:
		ppe_uniphy_usxgmii_restart_autoneg(port);
		ppe_uniphy_usxgmii_autoneg_completed(port);
		ppe_uniphy_usxgmii_speed_set(port->id, port->uniphy_id,
					     port->mac_speed);
		ppe_uniphy_usxgmii_duplex_set(port->id, port->uniphy_id,
					       port->duplex);
		ppe_uniphy_usxgmii_port_reset(port->uniphy_id, port->id,
					      port->uniphy_mode);
		usxgmii = true;
		if (port->gmac_type == XGMAC)
			speed = port->mac_speed;
		break;
	case PORT_WRAPPER_SGMII_PLUS:
		ppe_uniphy_sgmii_adapter_reset(port);
		if (port->gmac_type == XGMAC)
			speed = port->mac_speed;
		break;
	case PORT_WRAPPER_SGMII0_RGMII4:
	case PORT_WRAPPER_SGMII1_RGMII4:
	case PORT_WRAPPER_SGMII4_RGMII4:
	case PORT_WRAPPER_SGMII_FIBER:
		ppe_uniphy_sgmii_adapter_reset(port);
		break;
	case PORT_WRAPPER_QSGMII:
	case PORT_WRAPPER_PSGMII:
		break;
	case PORT_WRAPPER_EMULATION:
		usxgmii = true;
		speed = port->mac_speed;
		break;
	default:
		break;
	}

	ppe_port_bridge_txmac_set(reg_base, port->id, true);

	if (port->gmac_type == XGMAC) {
		ppe_xgmac_configuration(reg_base, port->id, speed, usxgmii);
	} else {
		gmacid = port->id - 1;
		base = reg_base + PPE_MAC_ENABLE + (0x200 * gmacid);

		writel(0x73, base);

		writel(port->mac_speed, base + PPE_MAC_SPEED_OFF);

		writel(0x1, base + PPE_MAC_MIB_CTL_OFF);
	}
}

/*
 * ipq_ppe_enable_port_counter
 */
static void ipq_ppe_enable_port_counter(phys_addr_t reg_base)
{
	u32 i;
	u32 reg = 0;

	for (i = 0; i < 7; i++) {
		/* MRU_MTU_CTRL_TBL.rx_cnt_en, MRU_MTU_CTRL_TBL.tx_cnt_en */
		reg = readl(reg_base + PPE_MRU_MTU_CTRL_TBL_ADDR
					+ (i * 0x10));
		writel(reg, reg_base + PPE_MRU_MTU_CTRL_TBL_ADDR
					+ (i * 0x10));
		reg = readl(reg_base + PPE_MRU_MTU_CTRL_TBL_ADDR
					+ (i * 0x10) + 0x4);
		writel(reg | 0x284303,
		       reg_base + PPE_MRU_MTU_CTRL_TBL_ADDR +
			(i * 0x10) + 0x4);
		reg = readl(reg_base + PPE_MRU_MTU_CTRL_TBL_ADDR
					+ (i * 0x10) + 0x8);
		writel(reg, reg_base + PPE_MRU_MTU_CTRL_TBL_ADDR
					+ (i * 0x10) + 0x8);
		reg = readl(reg_base + PPE_MRU_MTU_CTRL_TBL_ADDR
					+ (i * 0x10) + 0xc);
		writel(reg, reg_base + PPE_MRU_MTU_CTRL_TBL_ADDR
					+ (i * 0x10) + 0xc);

		/* MC_MTU_CTRL_TBL.tx_cnt_en */
		reg = readl(reg_base + PPE_MC_MTU_CTRL_TBL_ADDR
					+ (i * 0x4));
		writel(reg | 0x10000,
		       reg_base + PPE_MC_MTU_CTRL_TBL_ADDR +
			(i * 0x4));

		/* PORT_EG_VLAN.tx_counting_en */
		reg = readl(reg_base + PPE_PORT_EG_VLAN_TBL_ADDR
					+ (i * 0x4));
		writel(reg | 0x100,
		       reg_base + PPE_PORT_EG_VLAN_TBL_ADDR +
			(i * 0x4));

		/* TL_PORT_VP_TBL.rx_cnt_en */
		reg = readl(reg_base + PPE_TL_PORT_VP_TBL_ADDR
					+ (i * 0x10));
		writel(reg, reg_base + PPE_TL_PORT_VP_TBL_ADDR
					+ (i * 0x10));
		reg = readl(reg_base + PPE_TL_PORT_VP_TBL_ADDR
					+ (i * 0x10) + 0x4);
		writel(reg, reg_base + PPE_TL_PORT_VP_TBL_ADDR
					+ (i * 0x10) + 0x4);
		reg = readl(reg_base + PPE_TL_PORT_VP_TBL_ADDR
					+ (i * 0x10) + 0x8);
		writel(reg | 0x20000,
		       reg_base + PPE_TL_PORT_VP_TBL_ADDR +
			(i * 0x10) + 0x8);
		reg = readl(reg_base + PPE_TL_PORT_VP_TBL_ADDR
					+ (i * 0x10) + 0xc);
		writel(reg, reg_base + PPE_TL_PORT_VP_TBL_ADDR
					+ (i * 0x10) + 0xc);
	}
}

/*
 * ipq_vsi_setup()
 */
static void ipq_vsi_setup(phys_addr_t reg_base, u32 vsi,
			  u32 group_mask)
{
	u32 val = (group_mask << 24 | group_mask << 16 |
				group_mask << 8 | group_mask);

	/* Set mask */
	writel(val, reg_base + 0x063800 + (vsi * 0x10));

	/*  new addr lrn en | station move lrn en */
	writel(0x9, reg_base + 0x063804 + (vsi * 0x10));
}

static u32 ipq_ppe_bridge_member_mask(const struct ppe_info *info)
{
	u32 mask;

	mask = info->bridge_port_mask ? info->bridge_port_mask : info->vsi;

	/* The CPU port must remain reachable from the recovery bridge. */
	mask |= BIT(0);
	mask &= BIT(info->no_ports + 1) - 1;

	return mask;
}

static u32 ipq_ppe_bridge_isolation_value(const struct ppe_info *info,
						  unsigned int port)
{
	u32 member_mask = ipq_ppe_bridge_member_mask(info);
	u32 allowed = member_mask;

	/* A port outside the recovery bridge must not reach the CPU or LAN. */
	if (!(member_mask & BIT(port)))
		allowed = BIT(port);

	return (allowed << 8) & ppe_port_bridge_isolation_mask(info->nos_iports);
}

void ipq_ppe_tdm_configuration(struct ppe_info *ppe)
{
	u32 i;
	u32 *config_values = &tdm_config[ppe->tdm_mode].val[0];

	for (i = 0; i < ppe->no_reg; ++i) {
		writel(config_values[i],
		       ppe->base + ppe->tdm_offset + (i * 0x10));
	}

	writel(ppe->tdm_ctrl_val, ppe->base + 0xb000);

	if (ppe->tm) {
		writel(0x20, (void *)0x3a47a000);
		writel(0x12, (void *)0x3a47a010);
		writel(0x1, (void *)0x3a47a020);
		writel(0x2, (void *)0x3a47a030);
		writel(0x10, (void *)0x3a47a040);
		writel(0x21, (void *)0x3a47a050);
		writel(0x2, (void *)0x3a47a060);
		writel(0x10, (void *)0x3a47a070);
		writel(0x12, (void *)0x3a47a080);
		writel(0x1, (void *)0x3a47a090);
		writel(0xa, (void *)0x3a400000);
		writel(0x303, (void *)0x3a026100);
		writel(0x303, (void *)0x3a026104);
		writel(0x303, (void *)0x3a026108);
	}
}

void ppe_ipo_rule_reg_set(phys_addr_t reg_base, union ipo_rule_reg_u *hw_reg,
			  u32 rule_id)
{
	u32 i;

	for (i = 0; i < 3; i++) {
		writel(hw_reg->val[i], reg_base + IPO_CSR_BASE_ADDR +
			IPO_RULE_REG_ADDRESS + (rule_id * IPO_RULE_REG_INC) +
			(i * 4));
	}
}

void ppe_ipo_mask_reg_set(phys_addr_t reg_base, union ipo_mask_reg_u *hw_mask,
			  u32 rule_id)
{
	u32 i;

	for (i = 0; i < 2; i++) {
		writel(hw_mask->val[i], reg_base + (IPO_CSR_BASE_ADDR +
			IPO_MASK_REG_ADDRESS + (rule_id * IPO_MASK_REG_INC) +
			(i * 4)));
	}
}

void ppe_ipo_action_set(phys_addr_t reg_base, union ipo_action_u *hw_act,
			u32 rule_id, u32 ipo_cnt)
{
	u32 i;

	for (i = 0; i < ipo_cnt; i++) {
		writel(hw_act->val[i], reg_base + (IPE_L2_BASE_ADDR +
			IPO_ACTION_ADDRESS + (rule_id * IPO_ACTION_INC) +
			(i * 4)));
	}
}

void ipq_ppe_acl_set(struct ppe_acl_set *acl_set)
{
	union ipo_rule_reg_u hw_reg = {0};
	union ipo_mask_reg_u hw_mask = {0};
	union ipo_action_u hw_act = {0};

	memset(&hw_reg, 0, sizeof(hw_reg));
	memset(&hw_mask, 0, sizeof(hw_mask));
	memset(&hw_act, 0, sizeof(hw_act));

	if (acl_set->rule_id < MAX_RULE) {
		hw_act.bf.dest_info_change_en = 1;
		hw_mask.bf.maskfield_0 = acl_set->mask;
		hw_reg.bf.rule_type = acl_set->rule_type;
		if (acl_set->rule_type == ADPT_ACL_HPPE_IPV4_DIP_RULE) {
			hw_reg.bf.rule_field_0 = acl_set->field1;
			hw_reg.bf.rule_field_1 = acl_set->field0 << 17;
			hw_mask.bf.maskfield_1 = 7 << 17;
			if (acl_set->permit == 0x0) {
				hw_act.bf.fwd_cmd = 0;/* forward */
				hw_reg.bf.pri = 0x1;
			}
			if (acl_set->deny == 0x1) {
				hw_act.bf.fwd_cmd = 1;/* drop */
				hw_reg.bf.pri = 0x0;
			}
		} else if (acl_set->rule_type == ADPT_ACL_HPPE_MAC_SA_RULE) {
			/* src mac AC rule */
			hw_reg.bf.rule_field_0 = acl_set->field1;
			hw_reg.bf.rule_field_1 = acl_set->field0;
			hw_mask.bf.maskfield_1 = 0xffff;
			hw_act.bf.fwd_cmd = 1;/* drop */
			hw_reg.bf.pri = 0x2;
			/* bypass fdb lean and fdb freash */
			hw_act.bf.bypass_bitmap_0 = 0x1800;
		} else if (acl_set->rule_type == ADPT_ACL_HPPE_MAC_DA_RULE) {
			/* dest mac AC rule */
			hw_reg.bf.rule_field_0 = acl_set->field1;
			hw_reg.bf.rule_field_1 = acl_set->field0;
			hw_mask.bf.maskfield_1 = 0xffff;
			hw_act.bf.fwd_cmd = 1;/* drop */
			hw_reg.bf.pri = 0x2;
		}
		/* bind port1-port6 */
		hw_reg.bf.src_0 = 0x0;
		hw_reg.bf.src_1 = 0x3F;
		ppe_ipo_rule_reg_set(acl_set->reg_base, &hw_reg,
				     acl_set->rule_id);
		ppe_ipo_mask_reg_set(acl_set->reg_base, &hw_mask,
				     acl_set->rule_id);
		ppe_ipo_action_set(acl_set->reg_base, &hw_act,
				   acl_set->rule_id, acl_set->ipo_cnt);
	}
}

static void ipq_ppe_udp_fallback_acl_set(struct ppe_info *info, bool permit)
{
	struct ppe_acl_set acl_set;

	if (!info || !info->base)
		return;

	UPDATE_ACL_SET(acl_set, info->base, 2, ADPT_ACL_HPPE_IPV4_DIP_RULE,
		       UDP_PKT, 0, 0, 0, permit ? 0 : 1,
		       info->ipo_action);
	ipq_ppe_acl_set(&acl_set);
}

void recovery_board_http_acl(bool enable)
{
	ipq_ppe_udp_fallback_acl_set(ipq_active_ppe, enable);

	if (ipq_edma_debug_trace())
		printf("PPE ACL: UDP fallback drop %s for HTTP recovery\n",
		       enable ? "disabled" : "restored");
}

/*
 * ipq_ppe_vp_port_tbl_set()
 */
static void ipq_ppe_vp_port_tbl_set(phys_addr_t reg_base, u32 port,
				    u32 vsi)
{
	u32 addr = PPE_L3_VP_PORT_TBL_ADDR +
		 (port * PPE_L3_VP_PORT_TBL_INC);
	writel(0x0, reg_base + addr);
	writel(1 << 9 | vsi << 10, reg_base + addr + 0x4);
	writel(0x0, reg_base + addr + 0x8);
	writel(0x0, reg_base + addr + 0xc);
}

void ipq_port_mac_clock_setclear(struct udevice *dev, struct port_info *port,
				 bool set)
{
	struct reset_ctl rst;
	int ret;
	char name[64];

	if (!dev || !port)
		return;

	snprintf(name, sizeof(name), "nss_cc_port%d_mac", port->id);

	ret = ipq_eth_get_port_reset(dev, port, "mac", name, &rst);

	if (!ret)
		ipq_port_reset(&rst, set);

	snprintf(name, sizeof(name), "nss_cc_port%d_tx", port->id);

	ret = ipq_eth_get_port_reset(dev, port, "tx", name, &rst);

	if (!ret)
		ipq_port_reset(&rst, set);

	snprintf(name, sizeof(name), "nss_cc_port%d_rx", port->id);

	ret = ipq_eth_get_port_reset(dev, port, "rx", name, &rst);

	if (!ret)
		ipq_port_reset(&rst, set);
}

/*
 * ipq_port_mac_clock_reset()
 */
void ipq_port_mac_clock_reset(struct udevice *dev, struct port_info *port)
{
	ipq_port_mac_clock_setclear(dev, port, true);
	mdelay(10);
	ipq_port_mac_clock_setclear(dev, port, false);
	mdelay(10);
}

/*
 * ipq_ppe_provision_init()
 */
void ipq_ppe_provision_init(struct ppe_info *info)
{
	phys_addr_t reg_base = info->base;
	u32 queue, bridge_ctrl, val, bridge_members;
	int i, j, port;
	struct ppe_acl_set acl_set;

	bridge_members = info->bridge_mode ?
		ipq_ppe_bridge_member_mask(info) : 0;

	/* tdm/sched configuration */
	ipq_ppe_tdm_configuration(info);

	if (info->bridge_mode)
		/* Add CPU port 0 to VSI 2 */
		ipq_ppe_vp_port_tbl_set(reg_base, 0, 2);

	for (i = 1, j = 2; i <= info->no_ports; ++i) {
		/* Keep recovery WAN/out-of-bridge ports in private VSIs. */
		if (info->bridge_mode && !(bridge_members & BIT(i)))
			ipq_ppe_vp_port_tbl_set(reg_base, i, 2 + i);
		else
			/* Add the selected physical ports to the recovery VSI. */
			ipq_ppe_vp_port_tbl_set(reg_base, i, j);

		if (!info->bridge_mode)
			++j;
	}

	/* Unicast priority map */
	writel(0, reg_base + PPE_QM_UPM_TBL);

	/* Port0 - 8 unicast queue settings */
	for (port = 0; port < info->nos_iports; ++port) {
		if (port == 0)
			queue = 0;
		else
			queue = ((port * 0x10) + 0x70);
		/*
		 * ppe_ucast_queue_map_tbl_queue_id_set
		 */
		val = readl(reg_base + PPE_QM_UQM_TBL +
				(port * PPE_UCAST_QUEUE_MAP_TBL_INC));

		val |= queue << 4;

		writel(val, reg_base + PPE_QM_UQM_TBL +
			(port * PPE_UCAST_QUEUE_MAP_TBL_INC));

		/*
		 * ppe_flow_port_map_tbl_port_num_set
		 */
		writel(port, reg_base + PPE_L0_FLOW_PORT_MAP_TBL +
			queue * PPE_L0_FLOW_PORT_MAP_TBL_INC);

		writel(port, reg_base + PPE_L1_FLOW_PORT_MAP_TBL +
			port * PPE_L1_FLOW_PORT_MAP_TBL_INC);

		/*
		 * ppe_flow_map_tbl_set
		 */
		val = port | 0x401000; /* c_drr_wt = 1, e_drr_wt = 1 */

		writel(val, reg_base + PPE_L0_FLOW_MAP_TBL + queue *
				PPE_L0_FLOW_MAP_TBL_INC);

		val = port | 0x100400; /* c_drr_wt = 1, e_drr_wt = 1 */

		writel(val, reg_base + PPE_L1_FLOW_MAP_TBL + port *
				PPE_L1_FLOW_MAP_TBL_INC);

		/*
		 * ppe_c_sp_cfg_tbl_drr_id_set
		 */
		writel(port * 2, reg_base + PPE_L0_C_SP_CFG_TBL +
				(port * 0x80));
		writel(port * 2, reg_base + PPE_L1_C_SP_CFG_TBL +
				(port * 0x80));

		/*
		 * ppe_e_sp_cfg_tbl_drr_id_set
		 */
		writel(port * 2 + 1, reg_base + PPE_L0_E_SP_CFG_TBL +
				(port * 0x80));
		writel(port * 2 + 1, reg_base + PPE_L1_E_SP_CFG_TBL +
				(port * 0x80));
	}

	/* Port0 multicast queue */
	writel(0x00000000, reg_base + 0x409000);
	writel(0x00401000, reg_base + 0x403000);

	/* Port1 - 7 multicast queue */
	for (i = 1; i < info->nos_iports; i++) {
		writel(i, reg_base + 0x409100 + ((i - 1) * 0x40));
		writel(0x401000 | i, reg_base + 0x403100 + ((i - 1) * 0x40));
	}

	/* ac enable for queues - disable queue tail drop */
	/* ucast queue */
	for (i = 0; i < 256; i++) {
		writel(0x32120001, reg_base + PPE_UCAST_QUEUE_AC_EN_BASE_ADDR
			+ (i * 0x10));
		writel(0x0, reg_base + PPE_UCAST_QUEUE_AC_EN_BASE_ADDR
					+ (i * 0x10) + 0x4);
		writel(0x0, reg_base + PPE_UCAST_QUEUE_AC_EN_BASE_ADDR
					+ (i * 0x10) + 0x8);
		writel(0x48000, reg_base + PPE_UCAST_QUEUE_AC_EN_BASE_ADDR
					+ (i * 0x10) + 0xc);
	}

	/* mcast queue */
	for (i = 0; i < 44; i++) {
		writel(0x00fa0001, reg_base + PPE_MCAST_QUEUE_AC_EN_BASE_ADDR
					+ (i * 0x10));
		writel(0x0, reg_base + PPE_MCAST_QUEUE_AC_EN_BASE_ADDR
					+ (i * 0x10) + 0x4);
		writel(0x1200, reg_base + PPE_MCAST_QUEUE_AC_EN_BASE_ADDR
					+ (i * 0x10) + 0x8);
	}

	/* enable queue counter */
	writel(0x4, reg_base + 0x020044);

	/* assign the ac group 0 with buffer number */
	writel(0x0, reg_base + 0x84c000);
	writel(0x7D00, reg_base + 0x84c004);
	writel(0x0, reg_base + 0x84c008);
	writel(0x0, reg_base + 0x84c00c);

	/* enable physical/virtual port TX/RX counters for all ports (0-6) */
	ipq_ppe_enable_port_counter(reg_base);

	/*
	 * Port0 - TX_EN is set by default, Port1 - LRN_EN is set
	 * Port0 -> CPU Port
	 * Port1-6 -> Ethernet Ports
	 * Port7 -> EIP197
	 * IPQ5332 ==> 1-3 ports
	 * IPQ9574 ==> 1-8 ports
	 */

	for (i = 0; i < info->nos_iports; i++) {
		bridge_ctrl = PPE_PORT_BRIDGE_CTRL_OFFSET;
		val = info->bridge_mode ?
			ipq_ppe_bridge_isolation_value(info, i) :
			ppe_port_bridge_isolation_mask(info->nos_iports);
		if (i == 0) {
			val |= ppe_port_bridge_promisc_mask() |
				ppe_port_bridge_txmac_mask() |
				PPE_PORT_BRIDGE_CTRL_STATION_LRN_EN |
				PPE_PORT_BRIDGE_CTRL_NEW_ADDR_LRN_EN;
		} else if (i == 7) {
			val |= ppe_port_bridge_promisc_mask() |
				PPE_PORT_BRIDGE_CTRL_STATION_LRN_EN |
				PPE_PORT_BRIDGE_CTRL_NEW_ADDR_LRN_EN;
		} else {
			val |= ppe_port_bridge_promisc_mask();
		}
		writel(val, reg_base + bridge_ctrl + (i * 4));
	}

	/* Global learning */
	writel(0xc0, reg_base + 0x060038);

	if (info->bridge_mode) {
		ipq_vsi_setup(reg_base, 2, bridge_members);
		for (i = 1; i <= info->no_ports; ++i) {
			if (!(bridge_members & BIT(i)))
				ipq_vsi_setup(reg_base, 2 + i, BIT(i));
		}
	} else {
		for (i = 0, j = 2;
			i <= info->no_ports && i < CONFIG_ETH_MAX_MAC;
			++i, ++j)
			ipq_vsi_setup(reg_base, j, nb_vsi_config[i]);
	}
	/*
	 * STP
	 * For IPQ5332 ==> Port 0-3
	 * For IPQ9574 ==> Ports 0-7
	 */
	for (i = 0; i < info->nos_iports; i++)
		writel(0x3, reg_base + PPE_STP_BASE + (0x4 * i));

	UPDATE_ACL_SET(acl_set, reg_base, 0, ADPT_ACL_HPPE_IPV4_DIP_RULE,
		       UDP_PKT, 67, 0xffff, 0, 0, info->ipo_action);
	/* Allowing DHCP packets */
	ipq_ppe_acl_set(&acl_set);

	UPDATE_ACL_SET(acl_set, reg_base, 1, ADPT_ACL_HPPE_IPV4_DIP_RULE,
		       UDP_PKT, 68, 0xffff, 0, 0, info->ipo_action);

	ipq_ppe_acl_set(&acl_set);

	UPDATE_ACL_SET(acl_set, reg_base, 2, ADPT_ACL_HPPE_IPV4_DIP_RULE,
		       UDP_PKT, 0, 0, 0, 1, info->ipo_action);

	/* Dropping all the UDP packets */
	ipq_ppe_acl_set(&acl_set);

	if (IS_ENABLED(CONFIG_CMD_TFTPBOOT)) {
		tftp_acl_our_port = 1024 + (get_timer(0) % 3072);

		UPDATE_ACL_SET(acl_set, reg_base, 3, 0x4, 0x1,
			       tftp_acl_our_port, 0xffff, 0, 0,
				info->ipo_action);

		/* Allowing tftp packets */
		ipq_ppe_acl_set(&acl_set);
	}
}

/*
 * EDMA configuration
 */
static u16 ipq_edma_ring_index(u32 idx, u32 count)
{
	if (count && !(count & (count - 1)))
		return idx & (count - 1);

	return count ? idx % count : 0;
}

static u32 ipq_edma_ring_next(u32 idx, u32 count)
{
	return ipq_edma_ring_index(idx + 1, count);
}

static u32 ipq_edma_ring_add(u32 idx, u32 add, u32 count)
{
	return ipq_edma_ring_index(idx + add, count);
}

static const char *ipq_edma_misc_status_str(u32 status, char *buf, size_t len)
{
	struct {
		u32 mask;
		const char *name;
	} bits[] = {
		{ EDMA_MISC_AXI_RD_ERR_MASK, "AXI_RD_ERR" },
		{ EDMA_MISC_AXI_WR_ERR_MASK, "AXI_WR_ERR" },
		{ EDMA_MISC_RX_DESC_FIFO_FULL_MASK, "RX_DESC_FIFO_FULL" },
		{ EDMA_MISC_RX_ERR_BUF_SIZE_MASK, "RX_ERR_BUF_SIZE" },
		{ EDMA_MISC_TX_SRAM_FULL_MASK, "TX_SRAM_FULL" },
		{ EDMA_MISC_TX_CMPL_BUF_FULL_MASK, "TX_CMPL_BUF_FULL" },
		{ EDMA_MISC_DATA_LEN_ERR_MASK, "DATA_LEN_ERR" },
		{ EDMA_MISC_TX_TIMEOUT_MASK, "TX_TIMEOUT" },
	};
	u32 unknown = status;
	int i;

	if (!len)
		return "";

	buf[0] = '\0';

	for (i = 0; i < ARRAY_SIZE(bits); i++) {
		if (!(status & bits[i].mask))
			continue;

		if (buf[0])
			strlcat(buf, "|", len);
		strlcat(buf, bits[i].name, len);
		unknown &= ~bits[i].mask;
	}

	if (unknown) {
		char tmp[16];

		snprintf(tmp, sizeof(tmp), "%s0x%x", buf[0] ? "|" : "",
			 unknown);
		strlcat(buf, tmp, len);
	}

	if (!buf[0])
		strlcat(buf, "none", len);

	return buf;
}

static const char *ipq_edma_txcmpl_error_str(u32 errors, char *buf,
					     size_t len)
{
	static const char * const names[] = {
		"IP_HDR",
		"TSO",
		"IP6_DATA_LEN",
		"TCP_DATA_LEN",
		"TCP_HDR_TOO_LONG",
		"TCP_DATA_OFFSET_FIELD",
		"UDP_DATA_LEN",
		"UDP_HDR_TOO_LONG",
		"UDP_LEN_FIELD",
		"UDPLITE_DATA_LEN",
		"UDPLITE_HDR_TOO_LONG",
		"UDPLITE_CSUM_COV",
		"IP_VERSION",
		"L4_OFFSET",
		"L4_OFFSET_LARGE",
		"L3_OFFSET_LARGE",
		"PAYLOAD_OFFSET_LARGE",
		"CUST_CSUM_OFFSET_LARGE",
		"RESERVE0",
		"RESERVE1",
		"RESERVE2",
		"TSO_MSS",
		"TSO_TCP",
	};
	u32 unknown = errors;
	int i;

	if (!len)
		return "";

	buf[0] = '\0';

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		if (!(errors & BIT(i)))
			continue;

		if (buf[0])
			strlcat(buf, "|", len);
		strlcat(buf, names[i], len);
		unknown &= ~BIT(i);
	}

	if (unknown) {
		char tmp[16];

		snprintf(tmp, sizeof(tmp), "%s0x%x", buf[0] ? "|" : "",
			 unknown);
		strlcat(buf, tmp, len);
	}

	if (!buf[0])
		strlcat(buf, "none", len);

	return buf;
}

static void ipq_eth_debug_dump(struct ipq_eth_dev *priv, const char *tag);
static void ipq_eth_debug_dump_queue_detail(struct ipq_eth_dev *priv, u32 qid);
static const char *ipq_eth_interface_name(phy_interface_t interface);
static const char *ipq_eth_phy_type_name(u32 phy_id);
static const char *ipq_eth_uniphy_mode_name(u32 mode);

static u32 ipq_eth_debug_queue_for_port(u32 port_id)
{
	return port_id ? ((port_id * 0x10) + 0x70) : 0;
}

struct ipq_eth_debug_counter_sample {
	u32 epoch;
	u32 port_id;
	u32 queue_id;
	u32 uniphy_id;
	u32 speed;
	u32 bridge;
	u32 txd_prod;
	u32 txd_cons;
	u32 txd_count;
	u32 txc_prod;
	u32 txc_cons;
	u32 txc_count;
	u32 rxd_prod;
	u32 rxd_cons;
	u32 rxd_count;
	u32 rxf_prod;
	u32 rxf_cons;
	u32 rxf_count;
	u32 misc;
	u32 ppe_port_packets;
	u32 ppe_queue_packets;
	u64 ppe_port_bytes;
	u64 ppe_queue_bytes;
	u64 mac_tx_frames;
	u64 mac_tx_bytes;
	u64 mac_rx_frames;
	u64 mac_rx_bytes;
	u32 mac_tx_cfg;
	u32 mac_rx_cfg;
	u32 mac_mib_cfg;
	u32 uniphy_mode;
	u32 uniphy_link;
	bool xgmac;
};

static struct ipq_eth_debug_counter_sample ipq_eth_debug_previous_counters;
static bool ipq_eth_debug_previous_counters_valid;

static u16 ipq_eth_read_be16(const uchar *p)
{
	return ((u16)p[0] << 8) | p[1];
}

static void ipq_eth_print_ipv4(const uchar *addr)
{
	printf("%u.%u.%u.%u", addr[0], addr[1], addr[2], addr[3]);
}

static void ipq_eth_debug_log_frame(const char *dir, const void *packet,
				    int length, u32 port, u32 desc4, u32 desc5)
{
	const uchar *pkt = packet;
	u32 *counter;
	u16 eth_type;
	int off = ETHER_HDR_SIZE;
	bool interesting;

	if (!ipq_edma_debug_trace())
		return;

	if (!packet || length < ETHER_HDR_SIZE)
		return;

	counter = (dir && dir[0] == 'R') ? &ipq_edma_rx_frame_log_count :
					   &ipq_edma_tx_frame_log_count;

	eth_type = ipq_eth_read_be16(pkt + 12);
	if (eth_type == PROT_VLAN && length >= ETHER_HDR_SIZE + 4) {
		eth_type = ipq_eth_read_be16(pkt + 16);
		off += 4;
	}

	interesting = eth_type == PROT_ARP || eth_type == PROT_IP;
	if ((*counter >= 64) || (!interesting && *counter >= 8))
		return;

	printf("EDMA %s frame port=%u len=%d eth=0x%04x dst=%pM src=%pM desc4=0x%08x desc5=0x%08x",
	       dir, port, length, eth_type, pkt, pkt + ARP_HLEN, desc4, desc5);

	if (eth_type == PROT_ARP && length >= off + ARP_HDR_SIZE) {
		const uchar *arp = pkt + off;

		printf(" arp op=%u spa=", ipq_eth_read_be16(arp + 6));
		ipq_eth_print_ipv4(arp + 14);
		printf(" tpa=");
		ipq_eth_print_ipv4(arp + 24);
	} else if (eth_type == PROT_IP && length >= off + IP_HDR_SIZE) {
		const uchar *ip = pkt + off;
		u8 ihl = (ip[0] & 0xf) * 4;

		if (ihl >= IP_HDR_SIZE && length >= off + ihl) {
			printf(" ip proto=%u ", ip[9]);
			ipq_eth_print_ipv4(ip + 12);
			printf("->");
			ipq_eth_print_ipv4(ip + 16);

			if (ip[9] == IPPROTO_UDP && length >= off + ihl + UDP_HDR_SIZE) {
				const uchar *udp = ip + ihl;
				u16 sport = ipq_eth_read_be16(udp);
				u16 dport = ipq_eth_read_be16(udp + 2);

				printf(" udp %u->%u", sport, dport);
				if ((sport == 68 && dport == 67) ||
				    (sport == 67 && dport == 68))
					printf(" dhcp");
			}
		}
	} else if (eth_type == PROT_IPV6) {
		printf(" ipv6");
	}

	printf("\n");
	(*counter)++;
}

static struct port_info *ipq_eth_debug_find_port(struct ipq_eth_dev *priv,
						 u32 port_id)
{
	int i;

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		if (priv->port[i] && priv->port[i]->id == port_id)
			return priv->port[i];
	}

	return NULL;
}

static u32 ipq_eth_debug_active_port(struct ipq_eth_dev *priv)
{
	int i;

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		if (priv->port[i] && priv->port[i]->cur_speed)
			return priv->port[i]->id;
	}

	return 0;
}

static void ipq_eth_debug_dump_words(const char *name, phys_addr_t base,
				     u32 off, u32 words)
{
	u32 i;

	printf(" %s @0x%08x:", name, off);
	for (i = 0; i < words; i++)
		printf(" %08x", readl(base + off + (i * 4)));
	printf("\n");
}

static void ipq_eth_debug_dump_counter3(const char *name, phys_addr_t base,
					u32 off)
{
	u32 packets = readl(base + off);
	u32 bytes_lo = readl(base + off + 4);
	u32 bytes_hi = readl(base + off + 8) & 0xff;

	printf(" %s @0x%08x: packets=%u bytes=0x%02x%08x raw=%08x/%08x/%08x\n",
	       name, off, packets, bytes_hi, bytes_lo, packets, bytes_lo,
		       readl(base + off + 8));
}

static u64 ipq_eth_debug_read_counter64(phys_addr_t base, u32 off)
{
	u32 high, high_again, low;

	do {
		high = readl(base + off + sizeof(u32));
		low = readl(base + off);
		high_again = readl(base + off + sizeof(u32));
	} while (high != high_again);

	return low | ((u64)high << 32);
}

static void ipq_eth_debug_read_counter3(phys_addr_t base, u32 off,
					u32 *packets, u64 *bytes)
{
	*packets = readl(base + off);
	*bytes = readl(base + off + 4) |
		((u64)(readl(base + off + 8) & 0xff) << 32);
}

static u32 ipq_ring_delta(u32 current, u32 previous, u32 count)
{
	if (!count)
		return 0;
	if (current >= previous)
		return current - previous;

	return count - previous + current;
}

static u64 counter_delta(u64 current, u64 previous, bool *reset)
{
	if (current < previous) {
		*reset = true;
		return current;
	}

	return current - previous;
}

static int
ipq_eth_debug_collect_counters(struct ipq_eth_dev *priv, u32 port_id,
			       struct ipq_eth_debug_counter_sample *sample)
{
	struct ipq_edma_hw *ehw = &priv->hw;
	struct ipq_edma_rxdesc_ring *rxdesc;
	struct ipq_edma_rxfill_ring *rxfill;
	struct ipq_edma_txdesc_ring *txdesc;
	struct ipq_edma_txcmpl_ring *txcmpl;
	struct port_info *port;
	phys_addr_t ppe_base = priv->ppe.base;
	phys_addr_t uniphy_base;
	u32 macid, mac_base, off;

	memset(sample, 0, sizeof(*sample));

	port = ipq_eth_debug_find_port(priv, port_id);
	if (!port) {
		printf("NSS counters: PPE port%u is not parsed in DT\n", port_id);
		return -ENOENT;
	}
	if (!ehw->txdesc_ring || !ehw->txcmpl_ring ||
	    !ehw->rxdesc_ring || !ehw->rxfill_ring) {
		printf("NSS counters: EDMA rings are not initialized\n");
		return -ENODEV;
	}

	txdesc = &ehw->txdesc_ring[0];
	txcmpl = &ehw->txcmpl_ring[0];
	rxdesc = &ehw->rxdesc_ring[0];
	rxfill = &ehw->rxfill_ring[0];

	sample->epoch = ipq_edma_start_generation;
	sample->port_id = port_id;
	sample->queue_id = ipq_eth_debug_queue_for_port(port_id);
	sample->uniphy_id = port->uniphy_id;
	sample->speed = port->cur_speed;
	sample->xgmac = port->xgmac;
	sample->bridge = readl(ppe_base + PPE_PORT_BRIDGE_CTRL_OFFSET +
			       port_id * PORT_BRIDGE_CTRL_INC);

	sample->txd_count = txdesc->count;
	sample->txd_prod = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_TXDESC_PROD_IDX(txdesc->id)) &
			EDMA_TXDESC_PROD_IDX_MASK, txdesc->count);
	sample->txd_cons = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_TXDESC_CONS_IDX(txdesc->id)) &
			EDMA_TXDESC_CONS_IDX_MASK, txdesc->count);

	sample->txc_count = txcmpl->count;
	sample->txc_prod = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_TXCMPL_PROD_IDX(txcmpl->id)) &
			EDMA_TXCMPL_PROD_IDX_MASK, txcmpl->count);
	sample->txc_cons = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_TXCMPL_CONS_IDX(txcmpl->id)) &
			EDMA_TXCMPL_CONS_IDX_MASK, txcmpl->count);

	sample->rxd_count = rxdesc->count;
	sample->rxd_prod = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_RXDESC_PROD_IDX(rxdesc->id)) &
			EDMA_RXDESC_PROD_IDX_MASK, rxdesc->count);
	sample->rxd_cons = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_RXDESC_CONS_IDX(rxdesc->id)) &
			EDMA_RXDESC_CONS_IDX_MASK, rxdesc->count);

	sample->rxf_count = rxfill->count;
	sample->rxf_prod = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_RXFILL_PROD_IDX(rxfill->id)) &
			EDMA_RXFILL_PROD_IDX_MASK, rxfill->count);
	sample->rxf_cons = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_RXFILL_CONS_IDX(rxfill->id)) &
			EDMA_RXFILL_CONS_IDX_MASK, rxfill->count);
	sample->misc = readl(ehw->iobase + EDMA_REG_MISC_INT_STAT);

	off = PPE_PTX_CSR_BASE_ADDR + PPE_PORT_TX_COUNTER_TBL_ADDR +
	      port_id * PPE_PORT_TX_COUNTER_TBL_INC;
	ipq_eth_debug_read_counter3(ppe_base, off, &sample->ppe_port_packets,
				    &sample->ppe_port_bytes);
	off = PPE_PTX_CSR_BASE_ADDR + PPE_QM_QUEUE_TX_COUNTER_TBL_ADDR +
	      sample->queue_id * PPE_QM_QUEUE_TX_COUNTER_TBL_INC;
	ipq_eth_debug_read_counter3(ppe_base, off, &sample->ppe_queue_packets,
				    &sample->ppe_queue_bytes);

	macid = port_id - 1;
	if (sample->xgmac) {
		mac_base = PPE_SWITCH_NSS_SWITCH_XGMAC0 +
			   macid * NSS_SWITCH_XGMAC_MAC_TX_CONFIGURATION;
		sample->mac_tx_cfg = readl(ppe_base + mac_base);
		sample->mac_rx_cfg = readl(ppe_base + mac_base +
					  MAC_RX_CONFIGURATION_ADDRESS);
		sample->mac_mib_cfg = readl(ppe_base + mac_base +
					    NSS_SWITCH_XGMAC_MMC_CONTROL);
		off = mac_base + NSS_SWITCH_XGMAC_MIB_TX_OCTETS;
		sample->mac_tx_bytes = ipq_eth_debug_read_counter64(ppe_base, off);
		off = mac_base + NSS_SWITCH_XGMAC_MIB_TX_FRAMES;
		sample->mac_tx_frames = ipq_eth_debug_read_counter64(ppe_base, off);
		off = mac_base + NSS_SWITCH_XGMAC_MIB_RX_FRAMES;
		sample->mac_rx_frames = ipq_eth_debug_read_counter64(ppe_base, off);
		off = mac_base + NSS_SWITCH_XGMAC_MIB_RX_OCTETS;
		sample->mac_rx_bytes = ipq_eth_debug_read_counter64(ppe_base, off);
	} else {
		mac_base = PPE_MAC_ENABLE + macid * 0x200;
		sample->mac_tx_cfg = readl(ppe_base + mac_base);
		sample->mac_rx_cfg = sample->mac_tx_cfg;
		sample->mac_mib_cfg = readl(ppe_base + mac_base +
					    PPE_MAC_MIB_CTL_OFF);
		sample->mac_tx_frames = (u64)readl(ppe_base + mac_base + 0xa0) +
					readl(ppe_base + mac_base + 0xa8) +
					readl(ppe_base + mac_base + 0xf0);
		sample->mac_tx_bytes =
			ipq_eth_debug_read_counter64(ppe_base, mac_base + 0xcc);
		sample->mac_rx_frames = (u64)readl(ppe_base + mac_base + 0x40) +
					readl(ppe_base + mac_base + 0x48) +
					readl(ppe_base + mac_base + 0x94);
		sample->mac_rx_bytes =
			ipq_eth_debug_read_counter64(ppe_base, mac_base + 0x84);
	}

	uniphy_base = priv->uniphy_base + port->uniphy_id * priv->uniphy_size;
	sample->uniphy_mode = readl(uniphy_base + PPE_UNIPHY_MODE_CONTROL);
	sample->uniphy_link = readl(uniphy_base + UNIPHY_INSTANCE_LINK_DETECT);

	return 0;
}

static void
ipq_eth_debug_dump_counter_delta(const struct ipq_eth_debug_counter_sample *prev,
				 const struct ipq_eth_debug_counter_sample *cur)
{
	bool reset = false;
	bool edma_reset = prev->epoch != cur->epoch;
	u32 txd_prod = 0, txd_cons = 0, txc_prod = 0, txc_cons = 0;
	u32 rxd_prod = 0, rxd_cons = 0, rxf_prod = 0, rxf_cons = 0;
	u64 ppe_port_packets, ppe_port_bytes;
	u64 ppe_queue_packets, ppe_queue_bytes;
	u64 mac_tx_frames, mac_tx_bytes, mac_rx_frames, mac_rx_bytes;

	if (!edma_reset) {
		txd_prod = ipq_ring_delta(cur->txd_prod, prev->txd_prod,
					  cur->txd_count);
		txd_cons = ipq_ring_delta(cur->txd_cons, prev->txd_cons,
					  cur->txd_count);
		txc_prod = ipq_ring_delta(cur->txc_prod, prev->txc_prod,
					  cur->txc_count);
		txc_cons = ipq_ring_delta(cur->txc_cons, prev->txc_cons,
					  cur->txc_count);
		rxd_prod = ipq_ring_delta(cur->rxd_prod, prev->rxd_prod,
					  cur->rxd_count);
		rxd_cons = ipq_ring_delta(cur->rxd_cons, prev->rxd_cons,
					  cur->rxd_count);
		rxf_prod = ipq_ring_delta(cur->rxf_prod, prev->rxf_prod,
					  cur->rxf_count);
		rxf_cons = ipq_ring_delta(cur->rxf_cons, prev->rxf_cons,
					  cur->rxf_count);
	}

	ppe_port_packets = counter_delta(cur->ppe_port_packets, prev->ppe_port_packets, &reset);
	ppe_port_bytes = counter_delta(cur->ppe_port_bytes, prev->ppe_port_bytes, &reset);
	ppe_queue_packets = counter_delta(cur->ppe_queue_packets, prev->ppe_queue_packets, &reset);
	ppe_queue_bytes = counter_delta(cur->ppe_queue_bytes, prev->ppe_queue_bytes, &reset);
	mac_tx_frames = counter_delta(cur->mac_tx_frames, prev->mac_tx_frames, &reset);
	mac_tx_bytes = counter_delta(cur->mac_tx_bytes, prev->mac_tx_bytes, &reset);
	mac_rx_frames = counter_delta(cur->mac_rx_frames, prev->mac_rx_frames, &reset);
	mac_rx_bytes = counter_delta(cur->mac_rx_bytes, prev->mac_rx_bytes, &reset);

	printf("NSS counters delta: port=%u epoch=%u->%u edma_reset=%u ",
	       cur->port_id, prev->epoch, cur->epoch, edma_reset);
	printf("txd=%u/%u txc=%u/%u rxd=%u/%u rxf=%u/%u ",
	       txd_prod, txd_cons, txc_prod, txc_cons,
	       rxd_prod, rxd_cons, rxf_prod, rxf_cons);
	printf("ppe_port=%llu/%llu ppe_queue=%llu/%llu ",
	       (unsigned long long)ppe_port_packets,
	       (unsigned long long)ppe_port_bytes,
	       (unsigned long long)ppe_queue_packets,
	       (unsigned long long)ppe_queue_bytes);
	printf("mac_tx=%llu/%llu mac_rx=%llu/%llu counter_reset=%u\n",
	       (unsigned long long)mac_tx_frames,
	       (unsigned long long)mac_tx_bytes,
	       (unsigned long long)mac_rx_frames,
	       (unsigned long long)mac_rx_bytes, reset);
}

static int ipq_eth_debug_dump_counters(struct ipq_eth_dev *priv, u32 port_id)
{
	struct ipq_eth_debug_counter_sample sample;
	int ret;

	if (!port_id)
		port_id = ipq_eth_debug_active_port(priv);
	if (!port_id) {
		printf("NSS counters: no active PPE port, pass one explicitly\n");
		return -EINVAL;
	}

	ret = ipq_eth_debug_collect_counters(priv, port_id, &sample);
	if (ret)
		return ret;

	printf("NSS counters: epoch=%u port=%u queue=%u mac=%s uniphy=%u ",
	       sample.epoch, sample.port_id, sample.queue_id,
	       sample.xgmac ? "xgmac" : "gmac", sample.uniphy_id);
	printf("speed=%u bridge=%08x txmac=%u\n",
	       sample.speed, sample.bridge,
	       !!(sample.bridge & ppe_port_bridge_txmac_mask()));
	printf("NSS counters edma: txd=%u/%u txc=%u/%u rxd=%u/%u ",
	       sample.txd_prod, sample.txd_cons,
	       sample.txc_prod, sample.txc_cons,
	       sample.rxd_prod, sample.rxd_cons);
	printf("rxf=%u/%u misc=%08x\n",
	       sample.rxf_prod, sample.rxf_cons, sample.misc);
	printf("NSS counters path: ppe_port=%u/%llu ppe_queue=%u/%llu ",
	       sample.ppe_port_packets,
	       (unsigned long long)sample.ppe_port_bytes,
	       sample.ppe_queue_packets,
	       (unsigned long long)sample.ppe_queue_bytes);
	printf("mac_tx=%llu/%llu mac_rx=%llu/%llu\n",
	       (unsigned long long)sample.mac_tx_frames,
	       (unsigned long long)sample.mac_tx_bytes,
	       (unsigned long long)sample.mac_rx_frames,
	       (unsigned long long)sample.mac_rx_bytes);
	printf("NSS counters link: mac_tx_cfg=%08x mac_rx_cfg=%08x mib=%08x ",
	       sample.mac_tx_cfg, sample.mac_rx_cfg, sample.mac_mib_cfg);
	printf("uniphy_mode=%08x uniphy_link=%08x\n",
	       sample.uniphy_mode, sample.uniphy_link);

	if (ipq_eth_debug_previous_counters_valid &&
	    ipq_eth_debug_previous_counters.port_id == sample.port_id)
		ipq_eth_debug_dump_counter_delta(&ipq_eth_debug_previous_counters,
						 &sample);
	else
		printf("NSS counters baseline: epoch=%u port=%u\n",
		       sample.epoch, sample.port_id);

	ipq_eth_debug_previous_counters = sample;
	ipq_eth_debug_previous_counters_valid = true;

	return 0;
}

static void ipq_eth_debug_dump_tx_path_for_port(struct ipq_eth_dev *priv,
						u32 port_id,
						const char *port_field)
{
	struct ppe_info *ppe = &priv->ppe;
	phys_addr_t base = ppe->base;
	u32 queue, macid, gmac_base;
	u64 gmac_tx_bytes;
	u32 bridge;

	if (!port_id || port_id >= ppe->nos_iports)
		return;

	queue = ipq_eth_debug_queue_for_port(port_id);
	macid = port_id - 1;
	gmac_base = PPE_MAC_ENABLE + macid * 0x200;
	bridge = readl(base + PPE_PORT_BRIDGE_CTRL_OFFSET +
		       port_id * PORT_BRIDGE_CTRL_INC);
	gmac_tx_bytes = readl(base + gmac_base + 0xcc) |
			((u64)readl(base + gmac_base + 0xd0) << 32);

	printf(" ppe_tx_path: %s=%u queue=%u bridge=0x%08x txmac=%u promisc=%u stp=0x%08x uqm=0x%08x l0=0x%08x ac=0x%08x gmac=0x%08x/%08x/%08x txbytes=0x%llx\n",
	       port_field ?: "port", port_id, queue, bridge,
	       !!(bridge & ppe_port_bridge_txmac_mask()),
	       !!(bridge & ppe_port_bridge_promisc_mask()),
	       readl(base + PPE_STP_BASE + port_id * 4),
	       readl(base + PPE_QM_UQM_TBL +
		     port_id * PPE_UCAST_QUEUE_MAP_TBL_INC),
	       readl(base + PPE_L0_FLOW_MAP_TBL +
		     queue * PPE_L0_FLOW_MAP_TBL_INC),
	       readl(base + PPE_QUEUE_MANAGER_BASE_ADDR +
		     PPE_QM_AC_UNI_QUEUE_CFG_TBL_ADDR +
		     queue * PPE_QM_AC_UNI_QUEUE_CFG_TBL_INC),
	       readl(base + gmac_base),
	       readl(base + gmac_base + PPE_MAC_SPEED_OFF),
	       readl(base + gmac_base + PPE_MAC_MIB_CTL_OFF),
	       (unsigned long long)gmac_tx_bytes);
	ipq_eth_debug_dump_counter3("ppe_port_tx", base,
				   PPE_PTX_CSR_BASE_ADDR +
				   PPE_PORT_TX_COUNTER_TBL_ADDR +
				   port_id * PPE_PORT_TX_COUNTER_TBL_INC);
	ipq_eth_debug_dump_counter3("ppe_queue_tx", base,
				   PPE_PTX_CSR_BASE_ADDR +
				   PPE_QM_QUEUE_TX_COUNTER_TBL_ADDR +
				   queue * PPE_QM_QUEUE_TX_COUNTER_TBL_INC);
}

static void ipq_eth_debug_dump_tx_path(struct ipq_eth_dev *priv)
{
	ipq_eth_debug_dump_tx_path_for_port(priv,
					    ipq_eth_debug_active_port(priv),
					    "active_port");
}

static const char *ipq_eth_debug_port_label(struct port_info *port)
{
	return port && port->label ? port->label : "-";
}

static u32 ipq_eth_debug_qid2rid_reg(struct ipq_edma_hw *ehw, u32 qid)
{
	return readl(ehw->iobase +
		     EDMA_QID2RID_TABLE_MEM(qid / EDMA_QID2RID_NUM_PER_REG));
}

static u32 ipq_eth_debug_qid2rid_lane(u32 reg, u32 qid)
{
	return (reg >> ((qid % EDMA_QID2RID_NUM_PER_REG) * 8)) & 0xff;
}

static void ipq_eth_debug_dump_qid2rid(struct ipq_eth_dev *priv, u32 qid,
				       u32 count)
{
	struct ipq_edma_hw *ehw = &priv->hw;
	u32 end, reg_idx, last_reg = ~0U, reg = 0;

	if (!count)
		count = EDMA_QID2RID_NUM_PER_REG;
	if (count > 128)
		count = 128;

	end = qid + count;
	while (qid < end) {
		reg_idx = qid / EDMA_QID2RID_NUM_PER_REG;
		if (reg_idx != last_reg) {
			reg = ipq_eth_debug_qid2rid_reg(ehw, qid);
			printf(" qid2rid_reg[%u] @0x%08x = 0x%08x\n",
			       reg_idx, EDMA_QID2RID_TABLE_MEM(reg_idx), reg);
			last_reg = reg_idx;
		}

		printf("  qid%u -> rid%u\n", qid,
		       ipq_eth_debug_qid2rid_lane(reg, qid));
		qid++;
	}
}

static void ipq_eth_debug_dump_summary(struct ipq_eth_dev *priv,
				       const char *tag)
{
	struct ipq_edma_hw *ehw = &priv->hw;
	struct ppe_info *ppe = &priv->ppe;
	struct ipq_edma_rxdesc_ring *rxdesc;
	struct ipq_edma_rxfill_ring *rxfill;
	struct ipq_edma_txdesc_ring *txdesc;
	struct ipq_edma_txcmpl_ring *txcmpl;
	u32 active_port, active_queue;
	u32 rxd_prod = 0, rxd_cons = 0, rxf_prod = 0, rxf_cons = 0;
	u32 txd_prod = 0, txd_cons = 0, txc_prod = 0, txc_cons = 0;
	u32 rxd_int = 0, rxf_int = 0;
	u32 q0_cfg, q0_cnt, qa_cfg, qa_cnt;
	u32 misc;
	char misc_buf[128];

	if (!ehw->rxdesc_ring || !ehw->rxfill_ring) {
		printf("NSS summary%s%s: EDMA rings are not initialized\n",
		       tag ? " " : "", tag ? tag : "");
		return;
	}

	active_port = ipq_eth_debug_active_port(priv);
	active_queue = ipq_eth_debug_queue_for_port(active_port);

	rxdesc = &ehw->rxdesc_ring[0];
	rxd_prod = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_RXDESC_PROD_IDX(rxdesc->id)) &
			EDMA_RXDESC_PROD_IDX_MASK, rxdesc->count);
	rxd_cons = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_RXDESC_CONS_IDX(rxdesc->id)) &
			EDMA_RXDESC_CONS_IDX_MASK, rxdesc->count);
	rxd_int = readl(ehw->iobase + EDMA_REG_RXDESC_INT_STAT(rxdesc->id));

	rxfill = &ehw->rxfill_ring[0];
	rxf_prod = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_RXFILL_PROD_IDX(rxfill->id)) &
			EDMA_RXFILL_PROD_IDX_MASK, rxfill->count);
	rxf_cons = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_RXFILL_CONS_IDX(rxfill->id)) &
			EDMA_RXFILL_CONS_IDX_MASK, rxfill->count);
	rxf_int = readl(ehw->iobase + EDMA_REG_RXFILL_INT_STAT(rxfill->id));

	txdesc = &ehw->txdesc_ring[0];
	txd_prod = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_TXDESC_PROD_IDX(txdesc->id)) &
			EDMA_TXDESC_PROD_IDX_MASK, txdesc->count);
	txd_cons = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_TXDESC_CONS_IDX(txdesc->id)) &
			EDMA_TXDESC_CONS_IDX_MASK, txdesc->count);

	txcmpl = &ehw->txcmpl_ring[0];
	txc_prod = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_TXCMPL_PROD_IDX(txcmpl->id)) &
			EDMA_TXCMPL_PROD_IDX_MASK, txcmpl->count);
	txc_cons = ipq_edma_ring_index(readl(ehw->iobase +
			EDMA_REG_TXCMPL_CONS_IDX(txcmpl->id)) &
			EDMA_TXCMPL_CONS_IDX_MASK, txcmpl->count);

	q0_cfg = readl(ppe->base + PPE_QUEUE_MANAGER_BASE_ADDR +
		       PPE_QM_AC_UNI_QUEUE_CFG_TBL_ADDR);
	q0_cnt = readl(ppe->base + PPE_QUEUE_MANAGER_BASE_ADDR +
		       PPE_QM_AC_UNI_QUEUE_CNT_TBL_ADDR);
	qa_cfg = readl(ppe->base + PPE_QUEUE_MANAGER_BASE_ADDR +
		       PPE_QM_AC_UNI_QUEUE_CFG_TBL_ADDR +
		       active_queue * PPE_QM_AC_UNI_QUEUE_CFG_TBL_INC);
	qa_cnt = readl(ppe->base + PPE_QUEUE_MANAGER_BASE_ADDR +
		       PPE_QM_AC_UNI_QUEUE_CNT_TBL_ADDR +
		       active_queue * PPE_QM_AC_UNI_QUEUE_CNT_TBL_INC);
	misc = readl(ehw->iobase + EDMA_REG_MISC_INT_STAT);

	printf("NSS summary%s%s: active_port=%u active_queue=%u bridge_layout=%s ",
	       tag ? " " : "", tag ? tag : "", active_port, active_queue,
	       ppe_bridge_ctrl_layout_name());
	printf("rxd%u=%u/%u int=%08x rxf%u=%u/%u int=%08x ",
	       rxdesc->id, rxd_prod, rxd_cons, rxd_int,
	       rxfill->id, rxf_prod, rxf_cons, rxf_int);
	printf("txd%u=%u/%u txc%u=%u/%u ",
	       txdesc->id, txd_prod, txd_cons,
	       txcmpl->id, txc_prod, txc_cons);
	printf("misc=%08x(%s) ", misc,
	       ipq_edma_misc_status_str(misc, misc_buf, sizeof(misc_buf)));
	printf("qid2rid0=%08x qid2rid64=%08x ",
	       readl(ehw->iobase + EDMA_QID2RID_TABLE_MEM(0)),
	       readl(ehw->iobase + EDMA_QID2RID_TABLE_MEM(64)));
	printf("bm0=%08x bm%u=%08x q0=%08x/%08x q%u=%08x/%08x\n",
	       readl(ppe->base + PPE_BM_CSR_BASE_ADDR +
		     PPE_BM_PORT_CNT_ADDR),
	       active_port,
	       readl(ppe->base + PPE_BM_CSR_BASE_ADDR +
		     PPE_BM_PORT_CNT_ADDR +
		     active_port * PPE_BM_PORT_CNT_INC),
	       q0_cfg, q0_cnt, active_queue, qa_cfg, qa_cnt);
}

static void ipq_eth_debug_dump_gmac(struct ipq_eth_dev *priv, u32 port_id)
{
	struct ppe_info *ppe = &priv->ppe;
	struct port_info *port = ipq_eth_debug_find_port(priv, port_id);
	phys_addr_t base = ppe->base;
	u32 macid, gmac_base, xgmac_base;
	u64 rx_good, rx_bad;
	u64 tx_bytes;

	if (!port_id) {
		printf("GMAC debug needs PPE port id 1..6\n");
		return;
	}

	macid = port_id - 1;
	gmac_base = PPE_MAC_ENABLE + macid * 0x200;
	xgmac_base = PPE_SWITCH_NSS_SWITCH_XGMAC0 +
		     macid * NSS_SWITCH_XGMAC_MAC_TX_CONFIGURATION;

	printf("GMAC port%u%s%s\n", port_id, port ? " drv=" : "",
	       port && port->dev ? port->dev->name : "");
	if (port)
		printf(" drv: label=%s interface=%s phyaddr=0x%x phy=%s(%u) uniphy=%u ch=%u mux=%u mode=%s(%u) gmac=%u xgmac=%d speed=%u duplex=%u cur_mode=%s(%u) cur_gmac=%u\n",
		       ipq_eth_debug_port_label(port),
		       ipq_eth_interface_name(port->interface), port->phyaddr,
		       ipq_eth_phy_type_name(port->phy_id), port->phy_id,
		       port->uniphy_id, port->pcs_channel, port->uniphy_type,
		       ipq_eth_uniphy_mode_name(port->uniphy_mode),
		       port->uniphy_mode, port->gmac_type,
		       port->xgmac, port->cur_speed, port->duplex,
		       ipq_eth_uniphy_mode_name(port->cur_uniphy_mode),
		       port->cur_uniphy_mode,
		       port->cur_gmac_type);

	ipq_eth_debug_dump_words("gmac_ctrl", base, gmac_base, 16);
	ipq_eth_debug_dump_words("gmac_rx_mib", base, gmac_base + 0x40, 8);
	ipq_eth_debug_dump_words("gmac_rx_byte_mib", base, gmac_base + 0x80, 8);
	ipq_eth_debug_dump_words("gmac_tx_mib", base, gmac_base + 0xa0, 24);

	rx_good = readl(base + gmac_base + 0x84) |
		  ((u64)readl(base + gmac_base + 0x88) << 32);
	rx_bad = readl(base + gmac_base + 0x8c) |
		 ((u64)readl(base + gmac_base + 0x90) << 32);
	tx_bytes = readl(base + gmac_base + 0xcc) |
		   ((u64)readl(base + gmac_base + 0xd0) << 32);
	printf(" gmac_mib: rxbroad=%u rxmulti=%u rxfcs=%u rxgood_bytes=0x%llx rxbad_bytes=0x%llx\n",
	       readl(base + gmac_base + 0x40),
	       readl(base + gmac_base + 0x48),
	       readl(base + gmac_base + 0x4c),
	       (unsigned long long)rx_good, (unsigned long long)rx_bad);
	printf(" gmac_tx_mib: txbroad=%u txmulti=%u txuni=%u txbytes=0x%llx txunderrun=%u txcoll=%u tx64=%u tx65_127=%u\n",
	       readl(base + gmac_base + 0xa0),
	       readl(base + gmac_base + 0xa8),
	       readl(base + gmac_base + 0xf0),
	       (unsigned long long)tx_bytes,
	       readl(base + gmac_base + 0xac),
	       readl(base + gmac_base + 0xd4),
	       readl(base + gmac_base + 0xb0),
	       readl(base + gmac_base + 0xb4));

	ipq_eth_debug_dump_words("xgmac_ctrl", base, xgmac_base, 8);
	ipq_eth_debug_dump_words("xgmac_rx_mib", base, xgmac_base + 0x900, 16);
}

static void ipq_eth_debug_dump_uniphy(struct ipq_eth_dev *priv, u32 uniphy_id)
{
	phys_addr_t base;
	bool xpcs_mode = false;
	int i;

	if (uniphy_id >= CONFIG_ETH_MAX_UNIPHY) {
		printf("UNIPHY id %u out of range\n", uniphy_id);
		return;
	}

	base = priv->uniphy_base + uniphy_id * priv->uniphy_size;
	printf("UNIPHY%u base=0x%llx size=0x%llx\n", uniphy_id,
	       (unsigned long long)base,
	       (unsigned long long)priv->uniphy_size);

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		struct port_info *port = priv->port[i];

		if (!port || port->uniphy_id != uniphy_id)
			continue;

		printf(" port[%d]: id=%u label=%s interface=%s phyaddr=0x%x phy=%s(%u) mode=%s(%u) ch=%u mux=%u gmac=%u speed=%u configured=%d\n",
		       i, port->id, ipq_eth_debug_port_label(port),
		       ipq_eth_interface_name(port->interface), port->phyaddr,
		       ipq_eth_phy_type_name(port->phy_id), port->phy_id,
		       ipq_eth_uniphy_mode_name(port->uniphy_mode),
		       port->uniphy_mode, port->pcs_channel,
		       port->uniphy_type, port->gmac_type,
		       port->cur_speed, port->isconfigured);

		if (IS_USXGMII_MODE(port->uniphy_mode) ||
		    IS_UQXGMII_MODE(port->uniphy_mode) ||
		    IS_10GBASE_R_MODE(port->uniphy_mode))
			xpcs_mode = true;
	}

	printf(" direct: mode=%08x link=%08x misc2=%08x src=%08x pll=%08x calib=%08x reset=%08x opt1=%08x ch0io4=%08x clkout=%08x\n",
	       readl(base + PPE_UNIPHY_MODE_CONTROL),
	       readl(base + UNIPHY_INSTANCE_LINK_DETECT),
	       readl(base + UNIPHY_MISC2_REG_OFFSET),
	       readl(base + UNIPHY_MISC_SOURCE_SELECTION_REG_OFFSET),
	       readl(base + UNIPHY_PLL_RESET_REG_OFFSET),
	       readl(base + PPE_UNIPHY_OFFSET_CALIB_4),
	       readl(base + QP_USXG_RESET_ADDRESS),
	       readl(base + UNIPHYQP_USXG_OPITON1),
	       readl(base + UNIPHY_DEC_CHANNEL_0_INPUT_OUTPUT_4),
	       readl(base + UNIPHY1_CLKOUT_50M_CTRL_OPTION));

	if (!xpcs_mode) {
		printf(" csr1: skipped for non-XPCS UNIPHY mode\n");
		return;
	}

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		struct port_info *port = priv->port[i];
		u32 sr_mii, an_ctrl, an_sts;

		if (!port || port->uniphy_id != uniphy_id)
			continue;
		if (!IS_USXGMII_MODE(port->uniphy_mode) &&
		    !IS_UQXGMII_MODE(port->uniphy_mode))
			continue;

		sr_mii = ppe_uniphy_mii_ctrl_addr(port->id, uniphy_id);
		an_ctrl = ppe_uniphy_an_ctrl_addr(port->id, uniphy_id);
		an_sts = ppe_uniphy_an_intr_sts_addr(port->id, uniphy_id);

		printf(" csr1 port%u: kr_sts=%08x dig_ctrl=%08x sr_mii@%06x=%08x an_ctrl@%06x=%08x an_sts@%06x=%08x\n",
		       port->id,
		       csr1_read(uniphy_id, SR_XS_PCS_KR_STS1_ADDRESS),
		       csr1_read(uniphy_id, VR_XS_PCS_DIG_CTRL1_ADDRESS),
		       sr_mii, csr1_read(uniphy_id, sr_mii),
		       an_ctrl, csr1_read(uniphy_id, an_ctrl),
		       an_sts, csr1_read(uniphy_id, an_sts));
	}
}

static void ipq_eth_debug_dump_port_detail(struct ipq_eth_dev *priv,
					   u32 port_id)
{
	struct ppe_info *ppe = &priv->ppe;
	struct port_info *port = ipq_eth_debug_find_port(priv, port_id);
	phys_addr_t base = ppe->base;
	u32 queue = ipq_eth_debug_queue_for_port(port_id);
	u32 macid = port_id ? port_id - 1 : 0;

	printf("PPE port%u detail%s%s\n", port_id,
	       port ? " drv=" : "",
	       port && port->dev ? port->dev->name : "");
	if (port) {
		printf(" drv: label=%s interface=%s phyaddr=0x%x phy=%s(%u) uniphy=%u ch=%u mux=%u mode=%s(%u) gmac=%u xgmac=%d speed=%u duplex=%u configured=%d cur_mode=%s(%u) cur_gmac=%u\n",
		       ipq_eth_debug_port_label(port),
		       ipq_eth_interface_name(port->interface), port->phyaddr,
		       ipq_eth_phy_type_name(port->phy_id), port->phy_id,
		       port->uniphy_id, port->pcs_channel, port->uniphy_type,
		       ipq_eth_uniphy_mode_name(port->uniphy_mode),
		       port->uniphy_mode, port->gmac_type,
		       port->xgmac, port->cur_speed, port->duplex,
		       port->isconfigured,
		       ipq_eth_uniphy_mode_name(port->cur_uniphy_mode),
		       port->cur_uniphy_mode, port->cur_gmac_type);
	}

	ipq_eth_debug_dump_words("bridge", base,
				 PPE_PORT_BRIDGE_CTRL_OFFSET + port_id * 4, 1);
	ipq_eth_debug_dump_words("stp", base, PPE_STP_BASE + port_id * 4, 1);
	ipq_eth_debug_dump_words("l3_vp", base,
				 PPE_L3_VP_PORT_TBL_ADDR +
				 port_id * PPE_L3_VP_PORT_TBL_INC, 4);
	ipq_eth_debug_dump_words("tl_port_vp", base,
				 PPE_TL_PORT_VP_TBL_ADDR + port_id * 0x10, 4);
	ipq_eth_debug_dump_words("mru_mtu", base,
				 PPE_MRU_MTU_CTRL_TBL_ADDR + port_id * 0x10, 4);
	ipq_eth_debug_dump_words("mc_mtu", base,
				 PPE_MC_MTU_CTRL_TBL_ADDR + port_id * 4, 1);
	ipq_eth_debug_dump_words("eg_vlan", base,
				 PPE_PORT_EG_VLAN_TBL_ADDR + port_id * 4, 1);
	ipq_eth_debug_dump_words("port_isol", base, 0x060000 + 0x1840 +
				 port_id * 4, 1);
	ipq_eth_debug_dump_words("bm_cnt", base, PPE_BM_CSR_BASE_ADDR +
				 PPE_BM_PORT_CNT_ADDR +
				 port_id * PPE_BM_PORT_CNT_INC, 1);
	ipq_eth_debug_dump_words("bm_reacted", base, PPE_BM_CSR_BASE_ADDR +
				 PPE_BM_PORT_REACTED_CNT_ADDR +
				 port_id * PPE_BM_PORT_REACTED_CNT_INC, 1);
	ipq_eth_debug_dump_words("bm_fc", base, PPE_BM_CSR_BASE_ADDR +
				 PPE_BM_PORT_FC_STATUS_ADDR +
				 port_id * PPE_BM_PORT_FC_STATUS_INC, 1);
	ipq_eth_debug_dump_counter3("port_tx", base, PPE_PTX_CSR_BASE_ADDR +
				   PPE_PORT_TX_COUNTER_TBL_ADDR +
				   port_id * PPE_PORT_TX_COUNTER_TBL_INC);

	if (port_id) {
		ipq_eth_debug_dump_words("gmac", base,
					 PPE_MAC_ENABLE + macid * 0x200, 4);
		ipq_eth_debug_dump_words("gmac_mib", base,
					 PPE_MAC_ENABLE + macid * 0x200 +
					 PPE_MAC_MIB_CTL_OFF, 1);
		ipq_eth_debug_dump_words("xgmac", base,
					 PPE_SWITCH_NSS_SWITCH_XGMAC0 +
					 macid * NSS_SWITCH_XGMAC_MAC_TX_CONFIGURATION,
					 4);
	}

	printf(" queue for egress port%u = %u\n", port_id, queue);
	ipq_eth_debug_dump_queue_detail(priv, queue);
}

static void ipq_eth_debug_dump_queue_detail(struct ipq_eth_dev *priv, u32 qid)
{
	struct ipq_edma_hw *ehw = &priv->hw;
	struct ppe_info *ppe = &priv->ppe;
	phys_addr_t base = ppe->base;
	u32 rid_reg = qid / EDMA_QID2RID_NUM_PER_REG;

	printf("PPE queue%u detail\n", qid);
	ipq_eth_debug_dump_words("l0_port_map", base,
				 PPE_L0_FLOW_PORT_MAP_TBL +
				 qid * PPE_L0_FLOW_PORT_MAP_TBL_INC, 1);
	ipq_eth_debug_dump_words("l0_flow", base,
				 PPE_L0_FLOW_MAP_TBL +
				 qid * PPE_L0_FLOW_MAP_TBL_INC, 1);
	ipq_eth_debug_dump_words("ac_cfg", base,
				 PPE_QUEUE_MANAGER_BASE_ADDR +
				 PPE_QM_AC_UNI_QUEUE_CFG_TBL_ADDR +
				 qid * PPE_QM_AC_UNI_QUEUE_CFG_TBL_INC, 4);
	ipq_eth_debug_dump_words("ac_cnt", base,
				 PPE_QUEUE_MANAGER_BASE_ADDR +
				 PPE_QM_AC_UNI_QUEUE_CNT_TBL_ADDR +
				 qid * PPE_QM_AC_UNI_QUEUE_CNT_TBL_INC, 1);
	ipq_eth_debug_dump_words("ac_drop_state", base,
				 PPE_QUEUE_MANAGER_BASE_ADDR +
				 PPE_QM_AC_UNI_QUEUE_DROP_STATE_TBL_ADDR +
				 qid * PPE_QM_AC_UNI_QUEUE_DROP_STATE_TBL_INC, 2);
	ipq_eth_debug_dump_counter3("queue_tx", base, PPE_PTX_CSR_BASE_ADDR +
				   PPE_QM_QUEUE_TX_COUNTER_TBL_ADDR +
				   qid * PPE_QM_QUEUE_TX_COUNTER_TBL_INC);
	ipq_eth_debug_dump_counter3("uni_drop", base,
				   PPE_QUEUE_MANAGER_BASE_ADDR +
				   PPE_QM_UNI_DROP_CNT_TBL_ADDR +
				   qid * PPE_QM_UNI_DROP_CNT_TBL_INC);

	if (ehw->iobase)
		printf(" qid2rid[%u] @0x%08x = 0x%08x\n", rid_reg,
		       EDMA_QID2RID_TABLE_MEM(rid_reg),
		       readl(ehw->iobase + EDMA_QID2RID_TABLE_MEM(rid_reg)));
}

static void ipq_eth_debug_dump_detail(struct ipq_eth_dev *priv)
{
	struct ppe_info *ppe = &priv->ppe;
	u32 active_port = 0;
	int i;

	ipq_eth_debug_dump(priv, "detail");

	printf("PPE summary tables\n");
	for (i = 0; i < ppe->nos_iports; i++) {
		u32 queue = ipq_eth_debug_queue_for_port(i);

		printf(" port%u: bridge=%08x stp=%08x uqm=%08x l0pm=%08x l0=%08x l1pm=%08x l1=%08x bm=%08x fc=%08x\n",
		       i,
		       readl(ppe->base + PPE_PORT_BRIDGE_CTRL_OFFSET + i * 4),
		       readl(ppe->base + PPE_STP_BASE + i * 4),
		       readl(ppe->base + PPE_QM_UQM_TBL +
			     i * PPE_UCAST_QUEUE_MAP_TBL_INC),
		       readl(ppe->base + PPE_L0_FLOW_PORT_MAP_TBL +
			     queue * PPE_L0_FLOW_PORT_MAP_TBL_INC),
		       readl(ppe->base + PPE_L0_FLOW_MAP_TBL +
			     queue * PPE_L0_FLOW_MAP_TBL_INC),
		       readl(ppe->base + PPE_L1_FLOW_PORT_MAP_TBL +
			     i * PPE_L1_FLOW_PORT_MAP_TBL_INC),
		       readl(ppe->base + PPE_L1_FLOW_MAP_TBL +
			     i * PPE_L1_FLOW_MAP_TBL_INC),
		       readl(ppe->base + PPE_BM_CSR_BASE_ADDR +
			     PPE_BM_PORT_CNT_ADDR + i * PPE_BM_PORT_CNT_INC),
		       readl(ppe->base + PPE_BM_CSR_BASE_ADDR +
			     PPE_BM_PORT_FC_STATUS_ADDR +
			     i * PPE_BM_PORT_FC_STATUS_INC));
	}

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		if (priv->port[i] && priv->port[i]->cur_speed) {
			active_port = priv->port[i]->id;
			break;
		}
	}

	ipq_eth_debug_dump_port_detail(priv, 0);
	if (active_port)
		ipq_eth_debug_dump_port_detail(priv, active_port);
}

static void ipq_eth_debug_dump(struct ipq_eth_dev *priv, const char *tag)
{
	struct ipq_edma_hw *ehw;
	struct ppe_info *ppe;
	u32 prod, cons, misc;
	char misc_buf[128];
	int i;

	if (!priv)
		return;

	ehw = &priv->hw;
	ppe = &priv->ppe;

	printf("NSS debug%s%s: bridge=%d dst_port=%u ports=%u iports=%u vsi=0x%x bridge_layout=%s txmac=0x%08x promisc=0x%08x isolate=0x%08x\n",
	       tag ? " " : "", tag ? tag : "", ppe->bridge_mode,
	       ppe->nbport, ppe->no_ports, ppe->nos_iports, ppe->vsi,
	       ppe_bridge_ctrl_layout_name(), ppe_port_bridge_txmac_mask(),
	       ppe_port_bridge_promisc_mask(),
	       ppe_port_bridge_isolation_mask(ppe->nos_iports));

	if (!ehw->rxdesc_ring || !ehw->rxfill_ring) {
		printf(" EDMA rings are not initialized\n");
		return;
	}

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		struct port_info *port = priv->port[i];
		u32 bridge = 0, stp = 0, vp = 0, phy = 0;

		if (!port)
			continue;

		if (port->id < ppe->nos_iports) {
			bridge = readl(ppe->base + PPE_PORT_BRIDGE_CTRL_OFFSET +
				       (port->id * 4));
			stp = readl(ppe->base + PPE_STP_BASE + (port->id * 4));
			vp = readl(ppe->base + PPE_L3_VP_PORT_TBL_ADDR +
				   (port->id * PPE_L3_VP_PORT_TBL_INC) + 0x4);
			phy = readl(ppe->base + ((port->id > 4) ?
				    PORT_PHY_STATUS_ADDRESS1 :
				    PORT_PHY_STATUS_ADDRESS));
		}

		printf(" port[%d]: id=%u label=%s interface=%s phyaddr=0x%x phy=%s(%u) cur=%u uniphy=%u mode=%s(%u) gmac=%u xgmac=%d bridge=0x%08x stp=0x%08x vp=0x%08x phy=0x%08x\n",
		       i, port->id, ipq_eth_debug_port_label(port),
		       ipq_eth_interface_name(port->interface), port->phyaddr,
		       ipq_eth_phy_type_name(port->phy_id), port->phy_id,
		       port->cur_speed, port->uniphy_id,
		       ipq_eth_uniphy_mode_name(port->uniphy_mode),
		       port->uniphy_mode, port->gmac_type, port->xgmac,
		       bridge, stp, vp, phy);
	}

	printf(" cpu0: bridge=0x%08x stp=0x%08x vp=0x%08x vsi=0x%08x/0x%08x\n",
	       readl(ppe->base + PPE_PORT_BRIDGE_CTRL_OFFSET),
	       readl(ppe->base + PPE_STP_BASE),
	       readl(ppe->base + PPE_L3_VP_PORT_TBL_ADDR + 0x4),
	       readl(ppe->base + 0x063800 + (2 * 0x10)),
	       readl(ppe->base + 0x063804 + (2 * 0x10)));

	misc = readl(ehw->iobase + EDMA_REG_MISC_INT_STAT);
	printf(" edma: mas=0x%08x port=0x%08x dmar=0x%08x txto=0x%08x misc=0x%08x/0x%08x bits=%s\n",
	       readl(ehw->iobase + EDMA_REG_MAS_CTRL),
	       readl(ehw->iobase + EDMA_REG_PORT_CTRL),
	       readl(ehw->iobase + EDMA_REG_DMAR_CTRL),
	       readl(ehw->iobase + EDMA_REG_TX_TIMEOUT_THRESH),
	       misc, readl(ehw->iobase + EDMA_REG_MISC_INT_MASK),
	       ipq_edma_misc_status_str(misc, misc_buf, sizeof(misc_buf)));
	printf(" map: rxd2fill=%08x/%08x/%08x txd2cmpl=%08x/%08x/%08x/%08x/%08x/%08x\n",
	       readl(ehw->iobase + EDMA_REG_RXDESC2FILL_MAP_0),
	       readl(ehw->iobase + EDMA_REG_RXDESC2FILL_MAP_1),
	       readl(ehw->iobase + EDMA_REG_RXDESC2FILL_MAP_2),
	       readl(ehw->iobase + EDMA_REG_TXDESC2CMPL_MAP_0),
	       readl(ehw->iobase + EDMA_REG_TXDESC2CMPL_MAP_1),
	       readl(ehw->iobase + EDMA_REG_TXDESC2CMPL_MAP_2),
	       readl(ehw->iobase + EDMA_REG_TXDESC2CMPL_MAP_3),
	       readl(ehw->iobase + EDMA_REG_TXDESC2CMPL_MAP_4),
	       readl(ehw->iobase + EDMA_REG_TXDESC2CMPL_MAP_5));

	for (i = 0; i < ehw->rxdesc_rings; i++) {
		struct ipq_edma_rxdesc_ring *ring = &ehw->rxdesc_ring[i];
		struct ipq_edma_rxdesc_desc *desc;
		struct ipq_edma_rx_sec_desc *sdesc;
		u64 ba, ba2;

		prod = ipq_edma_ring_index(readl(ehw->iobase +
				EDMA_REG_RXDESC_PROD_IDX(ring->id)) &
				EDMA_RXDESC_PROD_IDX_MASK, ring->count);
		cons = ipq_edma_ring_index(readl(ehw->iobase +
				EDMA_REG_RXDESC_CONS_IDX(ring->id)) &
				EDMA_RXDESC_CONS_IDX_MASK, ring->count);
		printf(" rxdesc%d prod=%u cons=%u ctrl=0x%08x int=0x%08x mask=0x%08x\n",
		       ring->id, prod, cons,
		       readl(ehw->iobase + EDMA_REG_RXDESC_CTRL(ring->id)),
		       readl(ehw->iobase + EDMA_REG_RXDESC_INT_STAT(ring->id)),
		       readl(ehw->iobase + EDMA_REG_RXDESC_INT_MASK(ring->id)));
		printf(" rxdesc%d disable=0x%08x done=0x%08x reset=0x%08x\n",
		       ring->id,
		       readl(ehw->iobase + EDMA_REG_RXDESC_DISABLE(ring->id)),
		       readl(ehw->iobase + EDMA_REG_RXDESC_DISABLE_DONE(ring->id)),
		       readl(ehw->iobase + EDMA_REG_RXDESC_RESET(ring->id)));
		ba = readl(ehw->iobase + EDMA_REG_RXDESC_BA(ring->id)) |
		     ((u64)(readl(ehw->iobase +
				  EDMA_REG_RXDESC_BA_HIGH(ring->id)) & 0xff) << 32);
		ba2 = readl(ehw->iobase + EDMA_REG_RXDESC_BA2(ring->id)) |
		      ((u64)(readl(ehw->iobase +
				   EDMA_REG_RXDESC_BA2_HIGH(ring->id)) & 0xff) << 32);
		printf(" rxdesc%d sw=0x%llx/0x%llx hw=0x%llx/0x%llx\n",
		       ring->id, (unsigned long long)ring->dma,
		       (unsigned long long)ring->sdma,
		       (unsigned long long)ba, (unsigned long long)ba2);
		desc = EDMA_RXDESC_DESC(ring, cons);
		ipq_edma_invalidate_range(desc, EDMA_RXDESC_DESC_SIZE);
		printf(" rxdesc%d[%u]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
		       ring->id, cons, desc->rdes0, desc->rdes1,
		       desc->rdes2, desc->rdes3, desc->rdes4, desc->rdes5,
		       desc->rdes6, desc->rdes7);
		sdesc = (struct ipq_edma_rx_sec_desc *)ring->sdesc + cons;
		ipq_edma_invalidate_range(sdesc, EDMA_RX_SEC_DESC_SIZE);
		printf(" rxsec%d[%u]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
		       ring->id, cons, sdesc->rx_sec0, sdesc->rx_sec1,
		       sdesc->rx_sec2, sdesc->rx_sec3, sdesc->rx_sec4,
		       sdesc->rx_sec5, sdesc->rx_sec6, sdesc->rx_sec7);
	}

	for (i = 0; i < ehw->rxfill_rings; i++) {
		struct ipq_edma_rxfill_ring *ring = &ehw->rxfill_ring[i];
		struct ipq_edma_rxfill_desc *desc;
		u64 ba;

		prod = ipq_edma_ring_index(readl(ehw->iobase +
				EDMA_REG_RXFILL_PROD_IDX(ring->id)) &
				EDMA_RXFILL_PROD_IDX_MASK, ring->count);
		cons = ipq_edma_ring_index(readl(ehw->iobase +
				EDMA_REG_RXFILL_CONS_IDX(ring->id)) &
				EDMA_RXFILL_CONS_IDX_MASK, ring->count);
		printf(" rxfill%d prod=%u cons=%u en=0x%08x int=0x%08x mask=0x%08x\n",
		       ring->id, prod, cons,
		       readl(ehw->iobase + EDMA_REG_RXFILL_RING_EN(ring->id)),
		       readl(ehw->iobase + EDMA_REG_RXFILL_INT_STAT(ring->id)),
		       readl(ehw->iobase + EDMA_REG_RXFILL_INT_MASK(ring->id)));
		printf(" rxfill%d disable=0x%08x done=0x%08x idx_reset=0x%08x\n",
		       ring->id,
		       readl(ehw->iobase + EDMA_REG_RXFILL_DISABLE(ring->id)),
		       readl(ehw->iobase + EDMA_REG_RXFILL_DISABLE_DONE(ring->id)),
		       readl(ehw->iobase + EDMA_REG_RXFILL_INDEX_RESET(ring->id)));
		ba = readl(ehw->iobase + EDMA_REG_RXFILL_BA(ring->id)) |
		     ((u64)(readl(ehw->iobase +
				  EDMA_REG_RXFILL_BA_HIGH(ring->id)) & 0xff) << 32);
		printf(" rxfill%d sw=0x%llx hw=0x%llx\n",
		       ring->id, (unsigned long long)ring->dma,
		       (unsigned long long)ba);
		desc = EDMA_RXFILL_DESC(ring, cons);
		ipq_edma_invalidate_range(desc, EDMA_RXFILL_DESC_SIZE);
		printf(" rxfill%d[%u]=%08x %08x %08x %08x\n",
		       ring->id, cons, desc->rdes0, desc->rdes1,
		       desc->rdes2, desc->rdes3);
	}

	for (i = 0; i < ehw->txdesc_rings; i++) {
		struct ipq_edma_txdesc_ring *ring = &ehw->txdesc_ring[i];
		struct ipq_edma_txdesc_desc *desc;
		u64 ba, ba2;

		prod = ipq_edma_ring_index(readl(ehw->iobase +
				EDMA_REG_TXDESC_PROD_IDX(ring->id)) &
				EDMA_TXDESC_PROD_IDX_MASK, ring->count);
		cons = ipq_edma_ring_index(readl(ehw->iobase +
				EDMA_REG_TXDESC_CONS_IDX(ring->id)) &
				EDMA_TXDESC_CONS_IDX_MASK, ring->count);
		printf(" txdesc%d prod=%u cons=%u ctrl=0x%08x\n",
		       ring->id, prod, cons,
		       readl(ehw->iobase + EDMA_REG_TXDESC_CTRL(ring->id)));
		ba = readl(ehw->iobase + EDMA_REG_TXDESC_BA(ring->id)) |
		     ((u64)(readl(ehw->iobase +
				  EDMA_REG_TXDESC_BA_HIGH(ring->id)) & 0xff) << 32);
		ba2 = readl(ehw->iobase + EDMA_REG_TXDESC_BA2(ring->id)) |
		      ((u64)(readl(ehw->iobase +
				   EDMA_REG_TXDESC_BA2_HIGH(ring->id)) & 0xff) << 32);
		printf(" txdesc%d sw=0x%llx/0x%llx hw=0x%llx/0x%llx\n",
		       ring->id, (unsigned long long)ring->dma,
		       (unsigned long long)ring->sdma,
		       (unsigned long long)ba, (unsigned long long)ba2);
		desc = EDMA_TXDESC_DESC(ring, prod);
		ipq_edma_invalidate_range(desc, EDMA_TXDESC_DESC_SIZE);
		printf(" txdesc%d[%u]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
		       ring->id, prod, desc->tdes0, desc->tdes1,
		       desc->tdes2, desc->tdes3, desc->tdes4, desc->tdes5,
		       desc->tdes6, desc->tdes7);
	}

	for (i = 0; i < ehw->txcmpl_rings; i++) {
		struct ipq_edma_txcmpl_ring *ring = &ehw->txcmpl_ring[i];
		struct ipq_edma_txcmpl_desc *desc;
		u64 ba;
		u32 errors;
		char error_buf[256];

		prod = ipq_edma_ring_index(readl(ehw->iobase +
				EDMA_REG_TXCMPL_PROD_IDX(ring->id)) &
				EDMA_TXCMPL_PROD_IDX_MASK, ring->count);
		cons = ipq_edma_ring_index(readl(ehw->iobase +
				EDMA_REG_TXCMPL_CONS_IDX(ring->id)) &
				EDMA_TXCMPL_CONS_IDX_MASK, ring->count);
		printf(" txcmpl%d prod=%u cons=%u ctrl=0x%08x int=0x%08x mask=0x%08x\n",
		       ring->id, prod, cons,
		       readl(ehw->iobase + EDMA_REG_TXCMPL_CTRL(ring->id)),
		       readl(ehw->iobase + EDMA_REG_TX_INT_STAT(ring->id)),
		       readl(ehw->iobase + EDMA_REG_TX_INT_MASK(ring->id)));
		ba = readl(ehw->iobase + EDMA_REG_TXCMPL_BA(ring->id)) |
		     ((u64)(readl(ehw->iobase +
				  EDMA_REG_TXCMPL_BA_HIGH(ring->id)) & 0xff) << 32);
		printf(" txcmpl%d sw=0x%llx hw=0x%llx\n",
		       ring->id, (unsigned long long)ring->dma,
		       (unsigned long long)ba);
		desc = EDMA_TXCMPL_DESC(ring, cons);
		ipq_edma_invalidate_range(desc, EDMA_TXCMPL_DESC_SIZE);
		errors = desc->tdes3 & EDMA_TXCOMP_RING_ERROR_MASK;
		printf(" txcmpl%d[%u]=%08x %08x %08x %08x errors=0x%08x(%s)\n",
		       ring->id, cons, desc->tdes0, desc->tdes1,
		       desc->tdes2, desc->tdes3, errors,
		       ipq_edma_txcmpl_error_str(errors, error_buf,
						 sizeof(error_buf)));
	}

	printf(" qid2rid");
	for (i = EDMA_CPU_PORT_QID_MIN / EDMA_QID2RID_NUM_PER_REG;
	     i <= EDMA_CPU_PORT_QID_MAX / EDMA_QID2RID_NUM_PER_REG; i++)
		printf(" %d=0x%08x", i,
		       readl(ehw->iobase + EDMA_QID2RID_TABLE_MEM(i)));
	for (i = EDMA_CPU_PORT_MC_QID_MIN / EDMA_QID2RID_NUM_PER_REG;
	     i <= EDMA_CPU_PORT_MC_QID_MAX / EDMA_QID2RID_NUM_PER_REG; i++)
		printf(" %d=0x%08x", i,
		       readl(ehw->iobase + EDMA_QID2RID_TABLE_MEM(i)));
	printf("\n");
}

static void ipq_eth_debug_dump_ports(struct ipq_eth_dev *priv)
{
	int i;

	if (!priv)
		return;

	printf("NSS ports: bridge=%d bridge_layout=%s active_dst_port=%u\n",
	       priv->ppe.bridge_mode, ppe_bridge_ctrl_layout_name(),
	       priv->ppe.nbport);

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		struct port_info *port = priv->port[i];

		if (!port)
			continue;

		printf(" port[%d]: id=%u label=%s phy=%s(%u) addr=0x%x interface=%s dt_phy_mode=%s package=%s pcs=uniphy%u/ch%u mux=%u runtime=%s(%u) cur=%s(%u) gmac=%u xgmac=%d speed=%u configured=%d\n",
		       i, port->id, ipq_eth_debug_port_label(port),
		       ipq_eth_phy_type_name(port->phy_id), port->phy_id,
		       port->phyaddr, ipq_eth_interface_name(port->interface),
		       port->dt_phy_mode ?: "<none>",
		       port->package_mode ?: "<none>",
		       port->uniphy_id, port->pcs_channel,
		       port->uniphy_type,
		       ipq_eth_uniphy_mode_name(port->uniphy_mode),
		       port->uniphy_mode,
		       ipq_eth_uniphy_mode_name(port->cur_uniphy_mode),
		       port->cur_uniphy_mode, port->gmac_type, port->xgmac,
		       port->cur_speed, port->isconfigured);
	}
}

static void ipq_eth_debug_dump_periodic(struct ipq_eth_dev *priv)
{
	ulong now = get_timer(0);

	if (!ipq_edma_debug_trace())
		return;

	if (ipq_edma_debug_last_ms &&
	    now - ipq_edma_debug_last_ms < 3000)
		return;

	ipq_edma_debug_last_ms = now;
	ipq_eth_debug_dump_summary(priv, "poll");
}

static phy_interface_t ipq_eth_default_interface(u32 phy_id)
{
	switch (phy_id) {
	case QCA8081_PHY_TYPE:
	case RTL8261BE_PHY_TYPE:
	case QCA81xx_PHY_TYPE:
		return PHY_INTERFACE_MODE_USXGMII;
	default:
		return PHY_INTERFACE_MODE_NA;
	}
}

static bool ipq_eth_root_compatible(const char *compat)
{
	return ofnode_device_is_compatible(ofnode_root(), compat);
}

static bool ipq_eth_is_ipq957x(void)
{
	if (ipq_eth_root_compatible("qcom,ipq9570") ||
	    ipq_eth_root_compatible("qti,ipq9570"))
		return true;

	return ipq_eth_root_compatible("qcom,ipq9574") ||
	       ipq_eth_root_compatible("qti,ipq9574");
}

static u32 ipq_eth_default_uniphy_id(u32 port_id)
{
	if (!ipq_eth_is_ipq957x())
		return (u32)-1;

	switch (port_id) {
	case 1:
	case 2:
	case 3:
	case 4:
		return 0;
	case 5:
		return 1;
	case 6:
		return 2;
	default:
		return (u32)-1;
	}
}

static u32 ipq_eth_default_uniphy_type(u32 port_id)
{
	if (!ipq_eth_is_ipq957x())
		return 0;

	return port_id == 5 ? 1 : 0;
}

static bool ipq_eth_phy_parent_compatible(ofnode phy_node, const char *compat)
{
	ofnode parent = ofnode_get_parent(phy_node);

	return ofnode_valid(parent) &&
	       ofnode_device_is_compatible(parent, compat);
}

static const char *ipq_eth_interface_name(phy_interface_t interface)
{
	const char *name = phy_string_for_interface(interface);

	return name && *name ? name : "na";
}

static const char *ipq_eth_phy_type_name(u32 phy_id)
{
	switch (phy_id) {
	case QCA8075_PHY_TYPE:
		return "qca8075";
	case QCA8081_PHY_TYPE:
		return "qca8081";
	case RTL8261BE_PHY_TYPE:
		return "rtl8261be";
	case QCA81xx_PHY_TYPE:
		return "qca81xx";
	case AQ_PHY_TYPE:
		return "aquantia";
	case QCA8x8x_PHY_TYPE:
		return "qca8x8x";
	case QCA8x8x_SWITCH_TYPE:
		return "qca8x8x-switch";
	case QCA8337_SWITCH_TYPE:
		return "qca8337";
	case QCE1204_PHY_TYPE:
		return "qce1204";
	case QCE2204_SWITCH_TYPE:
		return "qce2204";
	case SFP10G_PHY_TYPE:
		return "sfp10g";
	case SFP2_5G_PHY_TYPE:
		return "sfp2.5g";
	case SFP1G_PHY_TYPE:
		return "sfp1g";
	default:
		return "unknown";
	}
}

static const char *ipq_eth_uniphy_mode_name(u32 mode)
{
	switch (mode) {
	case PORT_WRAPPER_PSGMII:
		return "psgmii";
	case PORT_WRAPPER_QSGMII:
		return "qsgmii";
	case PORT_WRAPPER_USXGMII:
		return "usxgmii";
	case PORT_WRAPPER_10GBASE_R:
		return "10gbase-r";
	case PORT_WRAPPER_UQXGMII:
		return "uqxgmii";
	case PORT_WRAPPER_SGMII0_RGMII4:
		return "sgmii0-rgmii4";
	case PORT_WRAPPER_SGMII1_RGMII4:
		return "sgmii1-rgmii4";
	case PORT_WRAPPER_SGMII4_RGMII4:
		return "sgmii4-rgmii4";
	case PORT_WRAPPER_SGMII_PLUS:
		return "sgmii-plus";
	case PORT_WRAPPER_SGMII_FIBER:
		return "sgmii-fiber";
	case PORT_WRAPPER_EMULATION:
		return "emulation";
	case PORT_WRAPPER_NA:
		return "na";
	case PORT_WRAPPER_MAX:
		return "unset";
	default:
		return "unknown";
	}
}

static const char *ipq_eth_qca8075_package_mode(ofnode phy_node);
static phy_interface_t ipq_eth_parse_interface_string(const char *mode);

static phy_interface_t ipq_eth_qca8075_package_interface(ofnode phy_node)
{
	const char *mode;

	mode = ipq_eth_qca8075_package_mode(phy_node);

	return ipq_eth_parse_interface_string(mode);
}

static const char *ipq_eth_node_phy_mode(ofnode node)
{
	const char *mode;

	if (!ofnode_valid(node))
		return NULL;

	mode = ofnode_read_string(node, "phy-mode");
	if (!mode)
		mode = ofnode_read_string(node, "phy-connection-type");

	return mode;
}

static const char *ipq_eth_qca8075_package_mode(ofnode phy_node)
{
	ofnode package_node = ofnode_get_parent(phy_node);

	if (!ofnode_valid(package_node) ||
	    !ofnode_device_is_compatible(package_node,
					 "qcom,qca8075-package"))
		return NULL;

	return ofnode_read_string(package_node, "qcom,package-mode");
}

static phy_interface_t ipq_eth_parse_interface_string(const char *mode)
{
	if (!mode)
		return PHY_INTERFACE_MODE_NA;
	if (!strcmp(mode, "qsgmii"))
		return PHY_INTERFACE_MODE_QSGMII;
	if (!strcmp(mode, "usxgmii"))
		return PHY_INTERFACE_MODE_USXGMII;
	/*
	 * U-Boot has no PSGMII phy_interface_t. Leave it as NA so the
	 * QSDK port table keeps handling legacy PSGMII boards.
	 */
	if (!strcmp(mode, "psgmii"))
		return PHY_INTERFACE_MODE_NA;

	return PHY_INTERFACE_MODE_NA;
}

static const char *ipq_eth_port_label(ofnode port_node)
{
	const char *label;

	if (!ofnode_valid(port_node))
		return NULL;

	label = ofnode_read_string(port_node, "label");
	return label && *label ? label : NULL;
}

static bool ipq_eth_port_label_is(ofnode port_node, const char *match)
{
	const char *label;

	if (!ofnode_valid(port_node))
		return false;

	label = ipq_eth_port_label(port_node);
	return label && !strcmp(label, match);
}

static u32 ipq_eth_default_phy_id(ofnode port_node, ofnode phy_node,
				  u32 port_id)
{
	if (ipq_eth_phy_parent_compatible(phy_node, "qcom,qca8075-package"))
		return QCA8075_PHY_TYPE;

	if (ofnode_device_is_compatible(phy_node, "ethernet-phy-id004d.d100") ||
	    ofnode_device_is_compatible(phy_node, "ethernet-phy-id004d.d101"))
		return QCA8081_PHY_TYPE;

	if (ofnode_device_is_compatible(phy_node, "realtek,rtl8261be"))
		return RTL8261BE_PHY_TYPE;

	if (port_id == 6 && ipq_eth_port_label_is(port_node, "wan") &&
	    ofnode_device_is_compatible(phy_node,
					"ethernet-phy-ieee802.3-c45"))
		return RTL8261BE_PHY_TYPE;

	return (u32)-1;
}

static u32 ipq_eth_read_u32_any(ofnode node, const char *prop1,
				const char *prop2, u32 def)
{
	u32 value;

	if (ofnode_valid(node) && !ofnode_read_u32(node, prop1, &value))
		return value;
	if (ofnode_valid(node) && prop2 && !ofnode_read_u32(node, prop2, &value))
		return value;

	return def;
}

static u32 ipq_eth_uniphy_id_from_name(ofnode node)
{
	const char *name;
	int i;

	if (!ofnode_valid(node))
		return (u32)-1;

	name = ofnode_get_name(node);
	if (!name)
		return (u32)-1;

	for (i = 0; name[i]; i++) {
		if (strncmp(&name[i], "pcs", 3))
			continue;
		if (!isdigit(name[i + 3]))
			continue;

		return simple_strtoul(&name[i + 3], NULL, 10);
	}

	return (u32)-1;
}

static u32 ipq_eth_uniphy_id_from_pcs_node(ofnode node)
{
	u64 addr, base = 0x07a00000;
	const char *name, *unit;

	if (!ofnode_valid(node))
		return (u32)-1;

	addr = ofnode_get_addr(node);
	if (addr != FDT_ADDR_T_NONE && addr >= base) {
		addr -= base;
		if ((addr % 0x10000) == 0 && addr / 0x10000 < CONFIG_ETH_MAX_UNIPHY)
			return addr / 0x10000;
	}

	name = ofnode_get_name(node);
	unit = name ? strchr(name, '@') : NULL;
	if (unit) {
		addr = simple_strtoull(unit + 1, NULL, 16);
		if (addr >= base) {
			addr -= base;
			if ((addr % 0x10000) == 0 &&
			    addr / 0x10000 < CONFIG_ETH_MAX_UNIPHY)
				return addr / 0x10000;
		}
	}

	return (u32)-1;
}

static ofnode ipq_eth_read_pcs_defaults(ofnode port_node, u32 *uniphy_id,
					u32 *pcs_channel)
{
	struct ofnode_phandle_args args;
	ofnode pcs_node, parent;
	u32 channel, value;

	*uniphy_id = (u32)-1;
	*pcs_channel = (u32)-1;

	if (!ofnode_valid(port_node))
		return ofnode_null();
	if (ofnode_parse_phandle_with_args(port_node, "pcs-handle",
					   NULL, 0, 0, &args)) {
		pcs_node = ofnode_parse_phandle(port_node, "pcs-handle", 0);
		if (!ofnode_valid(pcs_node))
			return ofnode_null();
	} else {
		pcs_node = args.node;
	}

	parent = ofnode_get_parent(pcs_node);
	channel = ofnode_read_u32_default(pcs_node, "reg", (u32)-1);

	value = ipq_eth_read_u32_any(pcs_node, "uniphy_id",
				     "qcom,uniphy-id", (u32)-1);
	if (value == (u32)-1)
		value = ipq_eth_read_u32_any(parent, "uniphy_id",
					     "qcom,uniphy-id", (u32)-1);
	if (value == (u32)-1)
		value = ipq_eth_uniphy_id_from_name(pcs_node);
	if (value == (u32)-1)
		value = ipq_eth_uniphy_id_from_name(parent);
	if (value == (u32)-1)
		value = ipq_eth_uniphy_id_from_pcs_node(parent);
	*uniphy_id = value;

	*pcs_channel = channel;

	return pcs_node;
}

static bool ipq_eth_is_port_node(ofnode node)
{
	const char *name = ofnode_get_name(node);
	u32 reg;

	if (!name)
		return false;

	/*
	 * Upstream Linux/OpenWrt DTS files use DSA-style port@N nodes.  Keep
	 * ethernet-port@N for the QSDK/QTI binding spelling.
	 */
	if (!strncmp(name, "port@", 5) ||
	    !strncmp(name, "ethernet-port@", 14))
		return true;

	if (ofnode_read_u32(node, "reg", &reg))
		return false;

	return reg >= 1 && reg <= 6;
}

static void ipq_eth_apply_interface_override(struct port_info *port, int *rate);
static bool ipq_eth_port_uses_dt_interface(struct port_info *port);
static int ipq_eth_get_port_clk(struct udevice *dev, struct port_info *port,
				const char *port_name, const char *dev_name,
				struct clk *clk);
static int ipq_eth_get_pcs_channel_clk(struct udevice *dev,
				       struct port_info *port,
				       const char *pcs_name,
				       const char *dev_name,
				       struct clk *clk);
static int ipq_eth_get_optional_node_clk(ofnode node, const char *name,
					 struct clk *clk);

static u32 ipq_edma_txdesc_ctrl_base(void)
{
	return ipq_edma_debug_txdesc_ctrl_base;
}

static u32 ipq_edma_port_ctrl_value(void)
{
	if (ipq_edma_debug_port_ctrl_override)
		return ipq_edma_debug_port_ctrl_override;

	return EDMA_PORT_CTRL_EN;
}

static u32 ipq_edma_dmar_ctrl_value(void)
{
	if (ipq_edma_debug_dmar_mode == 3) {
		return EDMA_DMAR_BURST_LEN_SET(EDMA_BURST_LEN_ENABLE) |
		       EDMA_DMAR_REQ_PRI_SET(0) |
		       ((31 & 0x3f) << 4) |
		       ((7 & 0xf) << 10) |
		       ((7 & 0xf) << 14);
	}

	return EDMA_DMAR_BURST_LEN_SET(EDMA_BURST_LEN_ENABLE) |
	       EDMA_DMAR_REQ_PRI_SET(0) |
	       EDMA_DMAR_TXDATA_OUTSTANDING_NUM_SET(31) |
	       EDMA_DMAR_TXDESC_OUTSTANDING_NUM_SET(7) |
	       EDMA_DMAR_RXFILL_OUTSTANDING_NUM_SET(7);
}

static u32 ipq_edma_tx_tdes4_value(struct ppe_info *ppe)
{
	u32 dst = EDMA_DST_PORT_TYPE_SET(EDMA_DST_PORT_TYPE) |
		  EDMA_DST_PORT_ID_SET(ppe->nbport);

	switch (ipq_edma_debug_tdes4_mode) {
	case IPQ_EDMA_TDES4_LEGACY:
		if (ppe->bridge_mode)
			return 0x00002000;
		return dst;
	case IPQ_EDMA_TDES4_SRCDST:
		return EDMA_SRC_PORT_TYPE_SET(EDMA_SRC_PORT_TYPE) |
		       EDMA_SRC_PORT_ID_SET(0) |
		       EDMA_DST_PORT_TYPE_SET(EDMA_DST_PORT_TYPE) |
		       EDMA_DST_PORT_ID_SET(ppe->nbport);
	case IPQ_EDMA_TDES4_DST:
		return dst;
	case IPQ_EDMA_TDES4_RAW:
		return ipq_edma_debug_tdes4_raw;
	default:
		if (ppe->bridge_mode)
			return 0x00002000;
		return dst;
	}
}

static u32 ipq_edma_txdesc2cmpl_reg(u32 ring_id)
{
	if (ring_id <= 5)
		return EDMA_REG_TXDESC2CMPL_MAP_0;
	if (ring_id <= 11)
		return EDMA_REG_TXDESC2CMPL_MAP_1;
	if (ring_id <= 17)
		return EDMA_REG_TXDESC2CMPL_MAP_2;
	if (ring_id <= 23)
		return EDMA_REG_TXDESC2CMPL_MAP_3;
	if (ring_id <= 29)
		return EDMA_REG_TXDESC2CMPL_MAP_4;

	return EDMA_REG_TXDESC2CMPL_MAP_5;
}

static void ipq_edma_program_txdesc_ring(struct ipq_edma_hw *ehw,
					 struct ipq_edma_txdesc_ring *ring)
{
	phys_addr_t reg_base = ehw->iobase;
	u64 base;

	base = ring->dma;
	writel((u32)(base & EDMA_RING_DMA_MASK),
	       reg_base + EDMA_REG_TXDESC_BA(ring->id));
	writel((u32)(base >> 32),
	       reg_base + EDMA_REG_TXDESC_BA_HIGH(ring->id));

	base = ring->sdma;
	writel((u32)(base & EDMA_RING_DMA_MASK),
	       reg_base + EDMA_REG_TXDESC_BA2(ring->id));
	writel((u32)(base >> 32),
	       reg_base + EDMA_REG_TXDESC_BA2_HIGH(ring->id));

	writel((u32)(ring->count & EDMA_TXDESC_RING_SIZE_MASK),
	       reg_base + EDMA_REG_TXDESC_RING_SIZE(ring->id));
}

static void ipq_edma_program_txcmpl_ring(struct ipq_edma_hw *ehw,
					 struct ipq_edma_txcmpl_ring *ring)
{
	phys_addr_t reg_base = ehw->iobase;
	u64 base;

	base = ring->dma;
	writel((u32)(base & EDMA_RING_DMA_MASK),
	       reg_base + EDMA_REG_TXCMPL_BA(ring->id));
	writel((u32)(base >> 32),
	       reg_base + EDMA_REG_TXCMPL_BA_HIGH(ring->id));

	writel((u32)(ring->count & EDMA_TXDESC_RING_SIZE_MASK),
	       reg_base + EDMA_REG_TXCMPL_RING_SIZE(ring->id));
	writel(EDMA_TXCMPL_RETMODE_OPAQUE,
	       reg_base + EDMA_REG_TXCMPL_CTRL(ring->id));
	writel(EDMA_TX_NE_INT_EN,
	       reg_base + EDMA_REG_TX_INT_CTRL(ring->id));
}

static void ipq_edma_program_txdesc2cmpl_map(struct ipq_edma_hw *ehw,
					     u32 map_count)
{
	phys_addr_t reg_base = ehw->iobase;
	u32 desc_index = ehw->txcmpl_ring_start;
	u32 data, reg, i;

	WRITE_REG_ARRAY(reg_base, EDMA_REG_TXDESC2CMPL_MAP_0, 4, 0,
			map_count);

	for (i = ehw->txdesc_ring_start; i < ehw->txdesc_ring_end; i++) {
		reg = ipq_edma_txdesc2cmpl_reg(i);
		data = readl(reg_base + reg);
		data |= (desc_index & 0x1f) << ((i % 6) * 5);
		writel(data, reg_base + reg);

		desc_index++;
		if (desc_index == ehw->txcmpl_ring_end)
			desc_index = ehw->txcmpl_ring_start;
	}
}

static void ipq_edma_program_tx_globals(struct ipq_edma_hw *ehw)
{
	phys_addr_t reg_base = ehw->iobase;

	writel(ipq_edma_dmar_ctrl_value(), reg_base + EDMA_REG_DMAR_CTRL);
	writel(ipq_edma_port_ctrl_value(), reg_base + EDMA_REG_PORT_CTRL);
}

static void ipq_edma_program_qid2rid_map(struct ipq_edma_hw *ehw)
{
	phys_addr_t reg_base = ehw->iobase;
	u32 desc_index = ehw->rxdesc_ring_start & 0x1f;
	u32 i, reg, reg_idx, data;

	for (i = EDMA_CPU_PORT_QID_MIN;
	     i <= EDMA_CPU_PORT_QID_MAX;
	     i += EDMA_QID2RID_NUM_PER_REG)
		writel(0, reg_base +
		       EDMA_QID2RID_TABLE_MEM(i / EDMA_QID2RID_NUM_PER_REG));

	for (i = EDMA_CPU_PORT_MC_QID_MIN;
	     i <= EDMA_CPU_PORT_MC_QID_MAX;
	     i += EDMA_QID2RID_NUM_PER_REG)
		writel(0, reg_base +
		       EDMA_QID2RID_TABLE_MEM(i / EDMA_QID2RID_NUM_PER_REG));

	/*
	 * QSDK programs QID0 with four sequential RxDesc rings. The IPQ9574
	 * U-Boot configuration only enables one RxDesc ring, so route all CPU
	 * unicast queues to the only valid ring. If a future configuration
	 * enables enough RxDesc rings, keep the QSDK layout for QID0.
	 */
	if (ehw->rxdesc_rings >= EDMA_QID2RID_NUM_PER_REG) {
		reg = EDMA_QID2RID_TABLE_MEM(0);
		data = ((desc_index << 0) & 0xff) |
		       (((desc_index + 1) << 8) & 0xff00) |
		       (((desc_index + 2) << 16) & 0xff0000) |
		       (((desc_index + 3) << 24) & 0xff000000);

		writel(data, reg_base + reg);
		pr_debug("Configure QID2RID(0) reg:0x%x to 0x%x\n",
			 reg, data);
	} else {
		data = (desc_index << 0) | (desc_index << 8) |
		       (desc_index << 16) | (desc_index << 24);

		for (i = EDMA_CPU_PORT_QID_MIN;
		     i <= EDMA_CPU_PORT_QID_MAX;
		     i += EDMA_QID2RID_NUM_PER_REG) {
			reg_idx = i / EDMA_QID2RID_NUM_PER_REG;
			reg = EDMA_QID2RID_TABLE_MEM(reg_idx);
			writel(data, reg_base + reg);
			pr_debug("Configure QID2RID(%d) reg:0x%x to 0x%x\n",
				 reg_idx, reg, data);
		}
	}

	/*
	 * Match QSDK for multicast queues: route all PPE multicast queues to
	 * the first RxDesc ring.
	 */
	data = (desc_index << 0) | (desc_index << 8) |
	       (desc_index << 16) | (desc_index << 24);
	for (i = EDMA_CPU_PORT_MC_QID_MIN;
	     i <= EDMA_CPU_PORT_MC_QID_MAX;
	     i += EDMA_QID2RID_NUM_PER_REG) {
		reg_idx = i / EDMA_QID2RID_NUM_PER_REG;
		reg = EDMA_QID2RID_TABLE_MEM(reg_idx);
		writel(data, reg_base + reg);
		pr_debug("Configure QID2RID(%d) reg:0x%x to 0x%x\n",
			 reg_idx, reg, data);
	}
}

static void ipq_edma_program_rxdesc2fill_map(struct ipq_edma_hw *ehw)
{
	struct ipq_edma_rxdesc_ring *rxdesc_ring;
	phys_addr_t reg_base = ehw->iobase;
	u32 data, i, reg, ring_id;

	/*
	 * Set RXDESC2FILL_MAP_xx reg.
	 * There are 3 registers RXDESC2FILL_0, RXDESC2FILL_1 and RXDESC2FILL_2
	 * 3 bits holds the rx fill ring mapping for each of the
	 * rx descriptor ring.
	 */
	WRITE_REG_ARRAY(reg_base, EDMA_REG_RXDESC2FILL_MAP_0, 4, 0,
			ehw->rx_map);

	for (i = 0; i < ehw->rxdesc_rings; i++) {
		rxdesc_ring = &ehw->rxdesc_ring[i];

		ring_id = rxdesc_ring->id;
		if (ring_id <= 9)
			reg = EDMA_REG_RXDESC2FILL_MAP_0;
		else if (ring_id <= 19)
			reg = EDMA_REG_RXDESC2FILL_MAP_1;
		else
			reg = EDMA_REG_RXDESC2FILL_MAP_2;

		pr_debug("Configure RXDESC:%u to use RXFILL:%u\n",
			 ring_id, rxdesc_ring->rxfill->id);

		data = readl(reg_base + reg);
		data |= (rxdesc_ring->rxfill->id & 0x7) << ((ring_id % 10) * 3);
		writel(data, reg_base + reg);
	}
}

static void ipq_edma_program_static_maps(struct ipq_edma_hw *ehw)
{
	ipq_edma_program_txdesc2cmpl_map(ehw, ehw->tx_map);
	ipq_edma_program_qid2rid_map(ehw);
	ipq_edma_program_rxdesc2fill_map(ehw);
}

static void ipq_edma_prime_rxfill_ring(struct ipq_edma_hw *ehw,
				       struct ipq_edma_rxfill_ring *rxfill_ring)
{
	phys_addr_t reg_base = ehw->iobase;
	struct ipq_edma_rxfill_desc *rxfill_desc;
	u32 cons_raw = 0, prod_raw;
	int i;

	prod_raw = rxfill_ring->count ? rxfill_ring->count - 1 : 0;

	for (i = 0; i < rxfill_ring->count; i++) {
		rxfill_desc = EDMA_RXFILL_DESC(rxfill_ring, i);
		rxfill_desc->rdes1 &= EDMA_RXFILL_BUF_HI_ADD_MASK;
		rxfill_desc->rdes1 |= cpu_to_le32((EDMA_RX_BUFF_SIZE <<
				       EDMA_RXFILL_BUF_SIZE_SHIFT) &
				       EDMA_RXFILL_BUF_SIZE_MASK);
		rxfill_desc->rdes2 = i;
		rxfill_desc->rdes3 = 0;
	}

	ipq_edma_flush_range(rxfill_ring->desc,
			     EDMA_RXFILL_DESC_SIZE * rxfill_ring->count);
	writel(prod_raw,
	       reg_base + EDMA_REG_RXFILL_PROD_IDX(rxfill_ring->id));

	if (ipq_edma_debug_trace())
		printf("EDMA RXFILL%u prime cons=%u/%u prod=%u/%u\n",
		       rxfill_ring->id,
		       ipq_edma_ring_index(cons_raw, rxfill_ring->count), cons_raw,
		       ipq_edma_ring_index(prod_raw, rxfill_ring->count), prod_raw);
}

static void ipq_edma_rearm_tx_path(struct ipq_edma_hw *ehw)
{
	struct ipq_edma_txdesc_ring *txdesc_ring;
	struct ipq_edma_txcmpl_ring *txcmpl_ring;
	phys_addr_t reg_base = ehw->iobase;
	u32 data;
	int i;

	if (!ehw->txdesc_ring || !ehw->txcmpl_ring)
		return;

	/* Stop descriptor fetch before changing the software producer. */
	for (i = 0; i < ehw->txdesc_rings; i++) {
		txdesc_ring = &ehw->txdesc_ring[i];
		data = readl(reg_base + EDMA_REG_TXDESC_CTRL(txdesc_ring->id));
		data &= ~EDMA_TXDESC_TX_EN;
		writel(data, reg_base + EDMA_REG_TXDESC_CTRL(txdesc_ring->id));
	}

	for (i = 0; i < ehw->txcmpl_rings; i++) {
		txcmpl_ring = &ehw->txcmpl_ring[i];
		ipq_edma_program_txcmpl_ring(ehw, txcmpl_ring);
		writel(0, reg_base + EDMA_REG_TXCMPL_CONS_IDX(txcmpl_ring->id));
		writel(EDMA_TXCMPL_RETMODE_OPAQUE,
		       reg_base + EDMA_REG_TXCMPL_CTRL(txcmpl_ring->id));
		writel(ehw->txcmpl_intr_mask,
		       reg_base + EDMA_REG_TX_INT_MASK(txcmpl_ring->id));
	}

	for (i = 0; i < ehw->txdesc_rings; i++) {
		txdesc_ring = &ehw->txdesc_ring[i];
		ipq_edma_program_txdesc_ring(ehw, txdesc_ring);
		writel(EDMA_TX_INITIAL_PROD_IDX,
		       reg_base + EDMA_REG_TXDESC_PROD_IDX(txdesc_ring->id));

		data = 0;
		data |= ipq_edma_txdesc_ctrl_base();
		data |= EDMA_TXDESC_TX_EN;
		writel(data, reg_base + EDMA_REG_TXDESC_CTRL(txdesc_ring->id));

		if (ipq_edma_debug_trace())
			printf("EDMA TXDESC%u init prod=%u/%u cons=%u/%u ctrl=0x%08x\n",
			       txdesc_ring->id,
			       ipq_edma_ring_index(readl(reg_base + EDMA_REG_TXDESC_PROD_IDX(txdesc_ring->id)) &
						   EDMA_TXDESC_PROD_IDX_MASK,
						   txdesc_ring->count),
			       readl(reg_base + EDMA_REG_TXDESC_PROD_IDX(txdesc_ring->id)) &
			       EDMA_TXDESC_PROD_IDX_MASK,
			       ipq_edma_ring_index(readl(reg_base + EDMA_REG_TXDESC_CONS_IDX(txdesc_ring->id)) &
						   EDMA_TXDESC_CONS_IDX_MASK,
						   txdesc_ring->count),
			       readl(reg_base + EDMA_REG_TXDESC_CONS_IDX(txdesc_ring->id)) &
			       EDMA_TXDESC_CONS_IDX_MASK,
			       readl(reg_base + EDMA_REG_TXDESC_CTRL(txdesc_ring->id)));
	}
}

static void ipq_edma_debug_dump_tx_regs(struct ipq_eth_dev *priv,
					const char *tag)
{
	struct ipq_edma_hw *ehw = &priv->hw;
	phys_addr_t reg_base = ehw->iobase;
	u32 prod_raw, cons_raw, prod, cons, size;
	u32 misc;
	char misc_buf[128];
	u32 i;

	if (!ehw->txdesc_ring || !ehw->txcmpl_ring) {
		printf("EDMA TX%s%s: rings are not initialized\n",
		       tag ? " " : "", tag ? tag : "");
		return;
	}

	printf("EDMA TX%s%s mode: port_ctrl_override=0x%08x effective=0x%08x dmar_mode=%s effective=0x%08x txdesc_ctrl_base=0x%08x tdes4=%s raw=0x%08x passthrough=%u\n",
	       tag ? " " : "", tag ? tag : "",
	       ipq_edma_debug_port_ctrl_override,
	       ipq_edma_port_ctrl_value(),
	       ipq_edma_debug_dmar_mode == 3 ? "v3" : "v1",
	       ipq_edma_dmar_ctrl_value(),
	       ipq_edma_txdesc_ctrl_base(),
	       ipq_edma_tdes4_mode_name(),
	       ipq_edma_debug_tdes4_raw,
	       ipq_edma_txdesc_passthrough_enabled() ? 1 : 0);
	misc = readl(reg_base + EDMA_REG_MISC_INT_STAT);
	printf(" global: mas=0x%08x port=0x%08x dmar=0x%08x txto=0x%08x misc=0x%08x/0x%08x bits=%s txq=0x%08x/0x%08x rxq=0x%08x axir=0x%08x axiw=0x%08x\n",
	       readl(reg_base + EDMA_REG_MAS_CTRL),
	       readl(reg_base + EDMA_REG_PORT_CTRL),
	       readl(reg_base + EDMA_REG_DMAR_CTRL),
	       readl(reg_base + EDMA_REG_TX_TIMEOUT_THRESH),
	       misc, readl(reg_base + EDMA_REG_MISC_INT_MASK),
	       ipq_edma_misc_status_str(misc, misc_buf, sizeof(misc_buf)),
	       readl(reg_base + 0x20),
	       readl(reg_base + 0x24),
	       readl(reg_base + 0x3c),
	       readl(reg_base + 0x4c),
	       readl(reg_base + 0x50));
	printf(" map: txd2cmpl=%08x/%08x/%08x/%08x/%08x/%08x rxd2fill=%08x/%08x/%08x qid2rid0=%08x qid2rid64=%08x\n",
	       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_0),
	       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_1),
	       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_2),
	       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_3),
	       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_4),
	       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_5),
	       readl(reg_base + EDMA_REG_RXDESC2FILL_MAP_0),
	       readl(reg_base + EDMA_REG_RXDESC2FILL_MAP_1),
	       readl(reg_base + EDMA_REG_RXDESC2FILL_MAP_2),
	       readl(reg_base + EDMA_QID2RID_TABLE_MEM(0)),
	       readl(reg_base + EDMA_QID2RID_TABLE_MEM(64)));
	ipq_eth_debug_dump_tx_path(priv);

	for (i = 0; i < ehw->txdesc_rings; i++) {
		struct ipq_edma_txdesc_ring *ring = &ehw->txdesc_ring[i];
		struct ipq_edma_txdesc_desc *desc;
		u64 ba, ba2;

		prod_raw = readl(reg_base + EDMA_REG_TXDESC_PROD_IDX(ring->id)) &
			   EDMA_TXDESC_PROD_IDX_MASK;
		cons_raw = readl(reg_base + EDMA_REG_TXDESC_CONS_IDX(ring->id)) &
			   EDMA_TXDESC_CONS_IDX_MASK;
		prod = ipq_edma_ring_index(prod_raw, ring->count);
		cons = ipq_edma_ring_index(cons_raw, ring->count);
		size = readl(reg_base + EDMA_REG_TXDESC_RING_SIZE(ring->id));
		ba = readl(reg_base + EDMA_REG_TXDESC_BA(ring->id)) |
		     ((u64)(readl(reg_base +
				  EDMA_REG_TXDESC_BA_HIGH(ring->id)) & 0xff) << 32);
		ba2 = readl(reg_base + EDMA_REG_TXDESC_BA2(ring->id)) |
		      ((u64)(readl(reg_base +
				   EDMA_REG_TXDESC_BA2_HIGH(ring->id)) & 0xff) << 32);

		printf(" txdesc%u prod=%u/%u cons=%u/%u size=0x%08x ctrl=0x%08x ba=0x%llx/0x%llx sw=0x%llx/0x%llx\n",
		       ring->id, prod, prod_raw, cons, cons_raw, size,
		       readl(reg_base + EDMA_REG_TXDESC_CTRL(ring->id)),
		       (unsigned long long)ba, (unsigned long long)ba2,
		       (unsigned long long)ring->dma,
		       (unsigned long long)ring->sdma);

		desc = EDMA_TXDESC_DESC(ring, prod);
		ipq_edma_invalidate_range(desc, EDMA_TXDESC_DESC_SIZE);
		printf(" txdesc%u[prod%u]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
		       ring->id, prod, desc->tdes0, desc->tdes1,
		       desc->tdes2, desc->tdes3, desc->tdes4, desc->tdes5,
		       desc->tdes6, desc->tdes7);

		desc = EDMA_TXDESC_DESC(ring, cons);
		ipq_edma_invalidate_range(desc, EDMA_TXDESC_DESC_SIZE);
		printf(" txdesc%u[cons%u]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
		       ring->id, cons, desc->tdes0, desc->tdes1,
		       desc->tdes2, desc->tdes3, desc->tdes4, desc->tdes5,
		       desc->tdes6, desc->tdes7);
	}

	for (i = 0; i < ehw->txcmpl_rings; i++) {
		struct ipq_edma_txcmpl_ring *ring = &ehw->txcmpl_ring[i];
		struct ipq_edma_txcmpl_desc *desc;
		u64 ba;
		u32 errors;
		char error_buf[256];

		prod_raw = readl(reg_base + EDMA_REG_TXCMPL_PROD_IDX(ring->id)) &
			   EDMA_TXCMPL_PROD_IDX_MASK;
		cons_raw = readl(reg_base + EDMA_REG_TXCMPL_CONS_IDX(ring->id)) &
			   EDMA_TXCMPL_CONS_IDX_MASK;
		prod = ipq_edma_ring_index(prod_raw, ring->count);
		cons = ipq_edma_ring_index(cons_raw, ring->count);
		size = readl(reg_base + EDMA_REG_TXCMPL_RING_SIZE(ring->id));
		ba = readl(reg_base + EDMA_REG_TXCMPL_BA(ring->id)) |
		     ((u64)(readl(reg_base +
				  EDMA_REG_TXCMPL_BA_HIGH(ring->id)) & 0xff) << 32);

		printf(" txcmpl%u prod=%u/%u cons=%u/%u size=0x%08x ctrl=0x%08x int=0x%08x mask=0x%08x intctrl=0x%08x ba=0x%llx sw=0x%llx\n",
		       ring->id, prod, prod_raw, cons, cons_raw, size,
		       readl(reg_base + EDMA_REG_TXCMPL_CTRL(ring->id)),
		       readl(reg_base + EDMA_REG_TX_INT_STAT(ring->id)),
		       readl(reg_base + EDMA_REG_TX_INT_MASK(ring->id)),
		       readl(reg_base + EDMA_REG_TX_INT_CTRL(ring->id)),
		       (unsigned long long)ba,
		       (unsigned long long)ring->dma);

		desc = EDMA_TXCMPL_DESC(ring, cons);
		ipq_edma_invalidate_range(desc, EDMA_TXCMPL_DESC_SIZE);
		errors = desc->tdes3 & EDMA_TXCOMP_RING_ERROR_MASK;
		printf(" txcmpl%u[cons%u]=%08x %08x %08x %08x errors=0x%08x(%s)\n",
		       ring->id, cons, desc->tdes0, desc->tdes1,
		       desc->tdes2, desc->tdes3, errors,
		       ipq_edma_txcmpl_error_str(errors, error_buf,
						 sizeof(error_buf)));
	}
}

static void ipq_eth_debug_dump_snapshot(struct ipq_eth_dev *priv, u32 port_id)
{
	struct port_info *port;

	if (!port_id)
		port_id = ipq_eth_debug_active_port(priv);

	printf("NSS snapshot: requested_port=%u requested_queue=%u\n",
	       port_id, ipq_eth_debug_queue_for_port(port_id));
	ipq_eth_debug_dump_ports(priv);
	ipq_eth_debug_dump_summary(priv, "snapshot");
	ipq_edma_debug_dump_tx_regs(priv, "snapshot");

	if (!port_id) {
		printf("NSS snapshot: no active PPE port, pass one explicitly\n");
		return;
	}

	port = ipq_eth_debug_find_port(priv, port_id);
	if (!port) {
		printf("NSS snapshot: PPE port%u is not parsed in DT\n", port_id);
		return;
	}

	ipq_eth_debug_dump_counters(priv, port_id);
	ipq_eth_debug_dump_tx_path_for_port(priv, port_id, "snapshot_port");
	ipq_eth_debug_dump_port_detail(priv, port_id);
	ipq_eth_debug_dump_gmac(priv, port_id);
	ipq_eth_debug_dump_uniphy(priv, port->uniphy_id);
}

static void ipq_edma_debug_reprogram_tx(struct ipq_eth_dev *priv)
{
	struct ipq_edma_hw *ehw = &priv->hw;

	if (!ehw->txdesc_ring || !ehw->txcmpl_ring)
		return;

	printf("EDMA TX fix: applying port=0x%08x dmar=%s/0x%08x\n",
	       ipq_edma_port_ctrl_value(),
	       ipq_edma_debug_dmar_mode == 3 ? "v3" : "v1",
	       ipq_edma_dmar_ctrl_value());

	ipq_edma_program_tx_globals(ehw);
	ipq_edma_program_txdesc2cmpl_map(ehw, 6);
	ipq_edma_rearm_tx_path(ehw);
}

static void ipq_edma_debug_kick_tx(struct ipq_eth_dev *priv, bool toggle_en)
{
	struct ipq_edma_hw *ehw = &priv->hw;
	struct ipq_edma_txdesc_ring *txdesc_ring;
	struct ipq_edma_txcmpl_ring *txcmpl_ring;
	phys_addr_t reg_base = ehw->iobase;
	u32 prod, cons, ctrl, txc_prod, txc_cons;
	int i;

	if (!ehw->txdesc_ring || !ehw->txcmpl_ring)
		return;

	ipq_edma_program_tx_globals(ehw);
	ipq_edma_program_txdesc2cmpl_map(ehw, 6);
	txdesc_ring = &ehw->txdesc_ring[0];
	txcmpl_ring = &ehw->txcmpl_ring[0];

	prod = readl(reg_base + EDMA_REG_TXDESC_PROD_IDX(txdesc_ring->id)) &
	       EDMA_TXDESC_PROD_IDX_MASK;
	cons = readl(reg_base + EDMA_REG_TXDESC_CONS_IDX(txdesc_ring->id)) &
	       EDMA_TXDESC_CONS_IDX_MASK;
	txc_prod = readl(reg_base + EDMA_REG_TXCMPL_PROD_IDX(txcmpl_ring->id)) &
		   EDMA_TXCMPL_PROD_IDX_MASK;
	txc_cons = readl(reg_base + EDMA_REG_TXCMPL_CONS_IDX(txcmpl_ring->id)) &
		   EDMA_TXCMPL_CONS_IDX_MASK;
	ctrl = readl(reg_base + EDMA_REG_TXDESC_CTRL(txdesc_ring->id));
	printf("EDMA TX kick before: txd%u prod=%u cons=%u ctrl=0x%08x txc%u prod=%u cons=%u toggle=%u\n",
	       txdesc_ring->id, prod, cons, ctrl, txcmpl_ring->id, txc_prod,
	       txc_cons, toggle_en);

	if (toggle_en) {
		for (i = 0; i < ehw->txdesc_rings; i++) {
			txdesc_ring = &ehw->txdesc_ring[i];
			ctrl = readl(reg_base + EDMA_REG_TXDESC_CTRL(txdesc_ring->id));
			ctrl &= ~EDMA_TXDESC_TX_EN;
			writel(ctrl, reg_base + EDMA_REG_TXDESC_CTRL(txdesc_ring->id));
		}
		wmb();
		for (i = 0; i < ehw->txdesc_rings; i++) {
			txdesc_ring = &ehw->txdesc_ring[i];
			ctrl = readl(reg_base + EDMA_REG_TXDESC_CTRL(txdesc_ring->id));
			ctrl &= ~EDMA_TXDESC_TX_RESET;
			ctrl |= ipq_edma_txdesc_ctrl_base();
			ctrl |= EDMA_TXDESC_TX_EN;
			writel(ctrl, reg_base + EDMA_REG_TXDESC_CTRL(txdesc_ring->id));
		}
	}

	txdesc_ring = &ehw->txdesc_ring[0];
	wmb();
	writel(prod, reg_base + EDMA_REG_TXDESC_PROD_IDX(txdesc_ring->id));
	readl(reg_base + EDMA_REG_TXDESC_PROD_IDX(txdesc_ring->id));

	for (i = 0; i < 5; i++) {
		udelay(200);
		prod = readl(reg_base + EDMA_REG_TXDESC_PROD_IDX(txdesc_ring->id)) &
		       EDMA_TXDESC_PROD_IDX_MASK;
		cons = readl(reg_base + EDMA_REG_TXDESC_CONS_IDX(txdesc_ring->id)) &
		       EDMA_TXDESC_CONS_IDX_MASK;
		txc_prod = readl(reg_base + EDMA_REG_TXCMPL_PROD_IDX(txcmpl_ring->id)) &
			   EDMA_TXCMPL_PROD_IDX_MASK;
		txc_cons = readl(reg_base + EDMA_REG_TXCMPL_CONS_IDX(txcmpl_ring->id)) &
			   EDMA_TXCMPL_CONS_IDX_MASK;
		ctrl = readl(reg_base + EDMA_REG_TXDESC_CTRL(txdesc_ring->id));
		printf("EDMA TX kick +%uus: txd%u prod=%u cons=%u ctrl=0x%08x txc%u prod=%u cons=%u\n",
		       (i + 1) * 200, txdesc_ring->id, prod, cons, ctrl,
		       txcmpl_ring->id, txc_prod, txc_cons);
	}
}

/*
 * ipq_edma_alloc_rx_buffer()
 *	Alloc Rx buffers for one RxFill ring
 */
int ipq_edma_alloc_rx_buffer(struct ipq_edma_hw *ehw,
			     struct ipq_edma_rxfill_ring *rxfill_ring)
{
	u16 num_alloc = 0;
	u16 desc_idx;
	struct ipq_edma_rxfill_desc *rxfill_desc;
	u32 cons, next, counter;
	u32 reg_data;
	phys_addr_t reg_base = ehw->iobase;

	/*
	 * Read RXFILL ring producer index
	 */
	reg_data = readl(reg_base + EDMA_REG_RXFILL_PROD_IDX(rxfill_ring->id));

	next = ipq_edma_ring_add(reg_data & EDMA_RXFILL_PROD_IDX_MASK,
				     0, rxfill_ring->count);

	/*
	 * Read RXFILL ring consumer index
	 */
	reg_data = readl(reg_base + EDMA_REG_RXFILL_CONS_IDX(rxfill_ring->id));

	cons = ipq_edma_ring_add(reg_data & EDMA_RXFILL_CONS_IDX_MASK,
				     0, rxfill_ring->count);

	pr_debug("%s: prod_idx = %d cons_idx  = %d\n", __func__, next, cons);
	while (1) {
		counter = ipq_edma_ring_next(next, rxfill_ring->count);

		if (counter == cons) {
			pr_debug("%s: counter == cons (%u)\n", __func__, cons);
			break;
		}

		/*
		 * Get RXFILL descriptor
		 */
		desc_idx = ipq_edma_ring_index(next, rxfill_ring->count);
		rxfill_desc = EDMA_RXFILL_DESC(rxfill_ring, desc_idx);

		/*
		 * Fill the opaque value
		 */
		rxfill_desc->rdes2 = desc_idx;

		/*
		 * Save buffer size in RXFILL descriptor
		 */
		rxfill_desc->rdes1 |= cpu_to_le32((EDMA_RX_BUFF_SIZE <<
				       EDMA_RXFILL_BUF_SIZE_SHIFT) &
				       EDMA_RXFILL_BUF_SIZE_MASK);
		num_alloc++;
		next = counter;
	}

	if (num_alloc) {
		ipq_edma_flush_range(rxfill_ring->desc,
				     EDMA_RXFILL_DESC_SIZE * rxfill_ring->count);

		/*
		 * Update RXFILL ring producer index
		 */
		reg_data = next & EDMA_RXFILL_PROD_IDX_MASK;

		/*
		 * make sure the producer index updated before
		 * updating the hardware
		 */
		writel(reg_data, reg_base + EDMA_REG_RXFILL_PROD_IDX(rxfill_ring->id));

		pr_debug("%s: num_alloc = %d\n", __func__, num_alloc);
	}

	return num_alloc;
}

/*
 * ipq_edma_clean_tx()
 *	Reap Tx descriptors
 */
u32 ipq_edma_clean_tx(struct ipq_edma_hw *ehw,
		      struct ipq_edma_txcmpl_ring *txcmpl_ring)
{
	struct ipq_edma_txcmpl_desc *txcmpl_desc;
	u16 desc_idx;
	u32 prod_raw, cons_raw, prod_idx, cons_idx;
	u32 data;
	u32 txcmpl_consumed = 0;
	u32 errors;
	uchar *skb;
	char error_buf[256];
	phys_addr_t reg_base = ehw->iobase;

	/*
	 * Get TXCMPL ring producer index
	 */
	data = readl(reg_base + EDMA_REG_TXCMPL_PROD_IDX(txcmpl_ring->id));
	prod_raw = data & EDMA_TXCMPL_PROD_IDX_MASK;
	prod_idx = ipq_edma_ring_index(prod_raw, txcmpl_ring->count);

	/*
	 * Get TXCMPL ring consumer index
	 */
	data = readl(reg_base + EDMA_REG_TXCMPL_CONS_IDX(txcmpl_ring->id));
	cons_raw = data & EDMA_TXCMPL_CONS_IDX_MASK;
	cons_idx = ipq_edma_ring_index(cons_raw, txcmpl_ring->count);

	pr_debug("%s: prod_idx = %u/%u cons_idx = %u/%u\n",
		 __func__, prod_idx, prod_raw, cons_idx, cons_raw);
	while (cons_idx != prod_idx) {
		desc_idx = ipq_edma_ring_index(cons_idx, txcmpl_ring->count);
		txcmpl_desc = EDMA_TXCMPL_DESC(txcmpl_ring, desc_idx);
		ipq_edma_invalidate_range(txcmpl_desc, EDMA_TXCMPL_DESC_SIZE);

		skb = (uchar *)((uintptr_t)
				(((uint64_t)(txcmpl_desc->tdes1 &
				EDMA_TXDESC_BUF_HI_ADD_MASK) << 32) |
				txcmpl_desc->tdes0));
		errors = txcmpl_desc->tdes3 & EDMA_TXCOMP_RING_ERROR_MASK;

		if (ipq_edma_debug_trace() && ipq_edma_txc_log_count < 16) {
			printf("EDMA TXCMPL ring=%u idx=%u prod=%u opaque=0x%08x%08x word2=0x%08x more=%u word3=0x%08x errors=0x%08x(%s)\n",
			       txcmpl_ring->id, desc_idx, prod_idx,
			       txcmpl_desc->tdes1, txcmpl_desc->tdes0,
			       txcmpl_desc->tdes2,
			       !!(txcmpl_desc->tdes2 &
				  EDMA_TXCMPL_MORE_BIT_MASK),
			       txcmpl_desc->tdes3, errors,
			       ipq_edma_txcmpl_error_str(errors,
							 error_buf,
							 sizeof(error_buf)));
			ipq_edma_txc_log_count++;
		}

		if (unlikely(!skb))
			pr_debug("Dropping empty tx completion: cons_idx:%u prod_idx:%u\n",
				 cons_idx, prod_idx);

		cons_idx = ipq_edma_ring_next(cons_idx, txcmpl_ring->count);

		txcmpl_consumed++;
	}

	pr_debug("%s :%u txcmpl_consumed:%u prod_idx:%u cons_idx:%u\n",
		 __func__, txcmpl_ring->id, txcmpl_consumed, prod_idx,
		cons_idx);

	if (txcmpl_consumed == 0)
		return 0;

	/*
	 * Update TXCMPL ring consumer index
	 */
	writel(cons_idx, reg_base + EDMA_REG_TXCMPL_CONS_IDX(txcmpl_ring->id));

	return txcmpl_consumed;
}

/*
 * ipq_edma_clean_rx()
 *	Reap Rx descriptors
 */
u32 ipq_edma_clean_rx(struct ipq_edma_hw *ehw,
		      struct ipq_edma_rxdesc_ring *rxdesc_ring,
				void **buff)
{
	struct ipq_edma_rxdesc_desc *rxdesc_desc;
	u16 desc_idx;
	u32 prod_raw, cons_raw, prod_idx, cons_idx;
	int src_port_num;
	int pkt_length = 0;
	bool desc_consumed = false;
	phys_addr_t reg_base = ehw->iobase;

	if (!rxdesc_ring || !rxdesc_ring->desc || !rxdesc_ring->count)
		return 0;

	cons_raw = readl(reg_base +
			EDMA_REG_RXDESC_CONS_IDX(rxdesc_ring->id)) &
			EDMA_RXDESC_CONS_IDX_MASK;
	cons_idx = ipq_edma_ring_index(cons_raw, rxdesc_ring->count);

	/*
	 * Read Rx ring producer index
	 */
	prod_raw = readl(reg_base +
		EDMA_REG_RXDESC_PROD_IDX(rxdesc_ring->id))
		& EDMA_RXDESC_PROD_IDX_MASK;
	prod_idx = ipq_edma_ring_index(prod_raw, rxdesc_ring->count);

	pr_debug("%s: cons idx = %u/%u, prod idx = %u/%u\n",
		 __func__, cons_idx, cons_raw, prod_idx, prod_raw);
	if (cons_idx == prod_idx) {
		pr_debug("%s: cons idx == prod idx (%u)\n", __func__, prod_idx);
		goto skip;
	}

	desc_idx = ipq_edma_ring_index(cons_idx, rxdesc_ring->count);
	rxdesc_desc = EDMA_RXDESC_DESC(rxdesc_ring, desc_idx);
	ipq_edma_invalidate_range(rxdesc_desc, EDMA_RXDESC_DESC_SIZE);
	desc_consumed = true;

	/*
	 * Check src_info from Rx Descriptor
	 */
	src_port_num =
		EDMA_RXDESC_SRC_INFO_GET(rxdesc_desc->rdes4);
	if ((src_port_num & EDMA_RXDESC_SRCINFO_TYPE_MASK) ==
			EDMA_RXDESC_SRCINFO_TYPE_PORTID) {
		src_port_num &= EDMA_RXDESC_PORTNUM_BITS;
	} else {
		if (ipq_edma_debug_trace() && ipq_edma_rx_log_count < 16) {
			printf("EDMA RX drop non-port rdes4=0x%08x rdes5=0x%08x prod=%u cons=%u\n",
			       rxdesc_desc->rdes4, rxdesc_desc->rdes5,
			       prod_idx, cons_idx);
			ipq_edma_rx_log_count++;
		}
		goto next_rx_desc;
	}
	/*
	 * Get packet length
	 */
	pkt_length = (rxdesc_desc->rdes5 &
		      EDMA_RXDESC_PKT_SIZE_MASK) >>
		      EDMA_RXDESC_PKT_SIZE_SHIFT;

	if (unlikely(src_port_num < ehw->start_ports ||
		     src_port_num > ehw->max_ports)) {
		if (ipq_edma_debug_trace() && ipq_edma_rx_log_count < 16) {
			printf("EDMA RX drop invalid src port %d prod=%u cons=%u\n",
			       src_port_num, prod_idx, cons_idx);
			ipq_edma_rx_log_count++;
		}
		pkt_length = 0;
		goto next_rx_desc;
	}

	if (unlikely(!pkt_length || pkt_length > EDMA_RX_BUFF_SIZE)) {
		if (ipq_edma_debug_trace() && ipq_edma_rx_log_count < 16) {
			printf("EDMA RX drop bad len %d port=%d rdes4=0x%08x rdes5=0x%08x prod=%u cons=%u\n",
			       pkt_length, src_port_num, rxdesc_desc->rdes4,
			       rxdesc_desc->rdes5, prod_idx, cons_idx);
			ipq_edma_rx_log_count++;
		}
		pkt_length = 0;
		goto next_rx_desc;
	}

	if (ipq_edma_debug_trace() && ipq_edma_rx_log_count < 16) {
		printf("EDMA RX port=%d len=%d prod=%u cons=%u\n",
		       src_port_num, pkt_length, prod_idx, cons_idx);
		ipq_edma_rx_log_count++;
	}

	*buff = (void *)((uintptr_t)(((uint64_t)(rxdesc_desc->rdes1 &
				EDMA_TXDESC_BUF_HI_ADD_MASK) << 32)
				| rxdesc_desc->rdes0));

	if (unlikely(!*buff)) {
		if (ipq_edma_debug_trace() && ipq_edma_rx_log_count < 16) {
			printf("EDMA RX drop null buffer port=%d prod=%u cons=%u\n",
			       src_port_num, prod_idx, cons_idx);
			ipq_edma_rx_log_count++;
		}
		pkt_length = 0;
		goto next_rx_desc;
	}

	ipq_edma_invalidate_range(*buff, pkt_length);
	ipq_eth_debug_log_frame("RX", *buff, pkt_length, src_port_num,
				rxdesc_desc->rdes4, rxdesc_desc->rdes5);

next_rx_desc:
	/*
	 * Update consumer index
	 */
	cons_idx = ipq_edma_ring_next(cons_idx, rxdesc_ring->count);

skip:

	if (desc_consumed)
		writel(cons_idx, reg_base + EDMA_REG_RXDESC_CONS_IDX(rxdesc_ring->id));

	return pkt_length;
}

/*
 * ipq_edma_rx_complete()
 */
static int ipq_edma_rx_complete(struct ipq_eth_dev *priv, void **buff)
{
	struct ipq_edma_hw *ehw = &priv->hw;
	struct ipq_edma_txcmpl_ring *txcmpl_ring;
	struct ipq_edma_rxdesc_ring *rxdesc_ring;
	struct ipq_edma_rxfill_ring *rxfill_ring;
	u32 misc_intr_status, reg_data;
	int length = 0;
	int i;
	phys_addr_t reg_base = ehw->iobase;

	if (!ehw->rxdesc_ring || !ehw->rxdesc_rings)
		return 0;
	if (!ehw->txcmpl_ring || !ehw->txcmpl_rings)
		return 0;
	if (!ehw->rxfill_ring || !ehw->rxfill_rings)
		return 0;

	*buff = NULL;

	for (i = 0; i < ehw->rxdesc_rings; i++) {
		rxdesc_ring = &ehw->rxdesc_ring[i];
		length = ipq_edma_clean_rx(ehw, rxdesc_ring, buff);
		if (length > 0)
			break;
	}

	for (i = 0; i < ehw->txcmpl_rings; i++) {
		txcmpl_ring = &ehw->txcmpl_ring[i];
		ipq_edma_clean_tx(ehw, txcmpl_ring);
	}

	for (i = 0; i < ehw->rxfill_rings; i++) {
		rxfill_ring = &ehw->rxfill_ring[i];
		ipq_edma_alloc_rx_buffer(ehw, rxfill_ring);
	}

	/*
	 * Enable RXDESC EDMA ring interrupt masks
	 */
	for (i = 0; i < ehw->rxdesc_rings; i++) {
		rxdesc_ring = &ehw->rxdesc_ring[i];
		writel(ehw->rxdesc_intr_mask, reg_base +
			EDMA_REG_RXDESC_INT_MASK(rxdesc_ring->id));
	}
	/*
	 * Enable TX EDMA ring interrupt masks
	 */
	for (i = 0; i < ehw->txcmpl_rings; i++) {
		txcmpl_ring = &ehw->txcmpl_ring[i];
		writel(ehw->txcmpl_intr_mask,
		       reg_base + EDMA_REG_TX_INT_MASK(txcmpl_ring->id));
	}
	/*
	 * Enable RXFILL EDMA ring interrupt masks
	 */
	for (i = 0; i < ehw->rxfill_rings; i++) {
		rxfill_ring = &ehw->rxfill_ring[i];
		writel(ehw->rxfill_intr_mask,
		       reg_base + EDMA_REG_RXFILL_INT_MASK(rxfill_ring->id));
	}
	/*
	 * Read Misc intr status
	 */
	reg_data = readl(reg_base + EDMA_REG_MISC_INT_STAT);
	misc_intr_status = reg_data & ehw->misc_intr_mask;

	if (misc_intr_status != 0) {
		pr_info("%s: misc_intr_status = 0x%x\n", __func__,
			misc_intr_status);
		writel(EDMA_MASK_INT_DISABLE,
		       reg_base + EDMA_REG_MISC_INT_MASK);
	}

	return length;
}

static void ipq_edma_rearm_rx_path(struct ipq_edma_hw *ehw)
{
	struct ipq_edma_rxdesc_ring *rxdesc_ring;
	struct ipq_edma_rxfill_ring *rxfill_ring;
	phys_addr_t reg_base = ehw->iobase;
	u32 data;
	int i;

	if (!ehw->rxdesc_ring || !ehw->rxfill_ring)
		return;

	for (i = 0; i < ehw->rxdesc_rings; i++) {
		rxdesc_ring = &ehw->rxdesc_ring[i];
		data = readl(reg_base + EDMA_REG_RXDESC_CTRL(rxdesc_ring->id));
		data &= ~EDMA_RXDESC_RX_EN;
		writel(data, reg_base + EDMA_REG_RXDESC_CTRL(rxdesc_ring->id));
		writel(EDMA_MASK_INT_DISABLE,
		       reg_base + EDMA_REG_RXDESC_INT_MASK(rxdesc_ring->id));
	}

	for (i = 0; i < ehw->rxfill_rings; i++) {
		rxfill_ring = &ehw->rxfill_ring[i];
		data = readl(reg_base + EDMA_REG_RXFILL_RING_EN(rxfill_ring->id));
		data &= ~EDMA_RXFILL_RING_EN;
		writel(data, reg_base + EDMA_REG_RXFILL_RING_EN(rxfill_ring->id));
		writel(EDMA_MASK_INT_DISABLE,
		       reg_base + EDMA_REG_RXFILL_INT_MASK(rxfill_ring->id));
	}

	for (i = 0; i < ehw->rxdesc_rings; i++) {
		rxdesc_ring = &ehw->rxdesc_ring[i];
		writel(0, reg_base + EDMA_REG_RXDESC_CONS_IDX(rxdesc_ring->id));
		if (ipq_edma_debug_trace())
			printf("EDMA RXDESC%u init cons=%u prod=%u\n",
			       rxdesc_ring->id,
			       readl(reg_base + EDMA_REG_RXDESC_CONS_IDX(rxdesc_ring->id)) &
			       EDMA_RXDESC_CONS_IDX_MASK,
			       readl(reg_base + EDMA_REG_RXDESC_PROD_IDX(rxdesc_ring->id)) &
			       EDMA_RXDESC_PROD_IDX_MASK);
	}

	for (i = 0; i < ehw->rxfill_rings; i++) {
		rxfill_ring = &ehw->rxfill_ring[i];
		ipq_edma_prime_rxfill_ring(ehw, rxfill_ring);
		data = readl(reg_base + EDMA_REG_RXFILL_RING_EN(rxfill_ring->id));
		data |= EDMA_RXFILL_RING_EN;
		writel(data, reg_base + EDMA_REG_RXFILL_RING_EN(rxfill_ring->id));
		writel(ehw->rxfill_intr_mask,
		       reg_base + EDMA_REG_RXFILL_INT_MASK(rxfill_ring->id));
	}

	for (i = 0; i < ehw->rxdesc_rings; i++) {
		rxdesc_ring = &ehw->rxdesc_ring[i];
		data = readl(reg_base + EDMA_REG_RXDESC_CTRL(rxdesc_ring->id));
		data |= EDMA_RXDESC_RX_EN;
		writel(data, reg_base + EDMA_REG_RXDESC_CTRL(rxdesc_ring->id));
		writel(ehw->rxdesc_intr_mask,
		       reg_base + EDMA_REG_RXDESC_INT_MASK(rxdesc_ring->id));
	}
}

/*
 * ipq_edma_setup_ring_resources()
 *	Allocate/setup resources for EDMA rings
 */
static int ipq_edma_setup_ring_resources(struct ipq_edma_hw *ehw)
{
	struct ipq_edma_txcmpl_ring *txcmpl_ring;
	struct ipq_edma_txdesc_ring *txdesc_ring;
	struct ipq_edma_rxfill_ring *rxfill_ring;
	struct ipq_edma_rxdesc_ring *rxdesc_ring;
	struct ipq_edma_txdesc_desc *txdesc_desc;
	struct ipq_edma_rxfill_desc *rxfill_desc;
	int i, j, index;
	void *tx_buf;
	void *rx_buf;

	/*
	 * Allocate Rx fill ring descriptors
	 */
	for (i = 0; i < ehw->rxfill_rings; i++) {
		rxfill_ring = &ehw->rxfill_ring[i];
		rxfill_ring->count = EDMA_RX_RING_SIZE;
		rxfill_ring->id = ehw->rxfill_ring_start + i;
		rxfill_ring->desc = (void *)mem_alloc(EDMA_RXFILL_DESC_SIZE * rxfill_ring->count,
						      ARCH_DMA_MINALIGN);

		if (!rxfill_ring->desc) {
			pr_info("%s: rxfill_ring->desc alloc error\n",
				__func__);
			return -ENOMEM;
		}
		memset(rxfill_ring->desc, 0,
		       EDMA_RXFILL_DESC_SIZE * rxfill_ring->count);
		rxfill_ring->dma = virt_to_phys(rxfill_ring->desc);
		pr_debug("rxfill ring id = %d, rxfill ring ptr = %p,rxfill ring dma = %u\n",
			 rxfill_ring->id, rxfill_ring->desc, (unsigned int)
			rxfill_ring->dma);

		rx_buf = (void *)mem_alloc(EDMA_RX_BUFF_SIZE *
					rxfill_ring->count,
					ARCH_DMA_MINALIGN);

		if (!rx_buf) {
			pr_info("%s: rxfill_ring->desc buffer alloc error\n",
				__func__);
			return -ENOMEM;
		}

		/*
		 * Allocate buffers for each of the desc
		 */
		for (j = 0; j < rxfill_ring->count; j++) {
			rxfill_desc = EDMA_RXFILL_DESC(rxfill_ring, j);
			rxfill_desc->rdes0 = virt_to_phys(rx_buf);
#ifdef CONFIG_ARM64
			rxfill_desc->rdes1 =
				((u64)virt_to_phys(rx_buf) >> 32) &
				EDMA_RXFILL_BUF_HI_ADD_MASK;
#else
			rxfill_desc->rdes1 = 0;
#endif
			rxfill_desc->rdes2 = 0;
			rxfill_desc->rdes3 = 0;
			rx_buf += EDMA_RX_BUFF_SIZE;
		}
		ipq_edma_flush_range(rxfill_ring->desc,
				     EDMA_RXFILL_DESC_SIZE * rxfill_ring->count);
	}

	/*
	 * Allocate RxDesc ring descriptors
	 */
	for (i = 0; i < ehw->rxdesc_rings; i++) {
		rxdesc_ring = &ehw->rxdesc_ring[i];
		rxdesc_ring->count = EDMA_RX_RING_SIZE;
		rxdesc_ring->id = ehw->rxdesc_ring_start + i;

		/*
		 * Create a mapping between RX Desc ring and Rx fill ring.
		 * Number of fill rings are lesser than the descriptor rings
		 * Share the fill rings across descriptor rings.
		 */
		index = ehw->rxfill_ring_start + (i % ehw->rxfill_rings);
		rxdesc_ring->rxfill =
			&ehw->rxfill_ring[index - ehw->rxfill_ring_start];

		rxdesc_ring->desc = (void *)mem_alloc(EDMA_RXDESC_DESC_SIZE * rxdesc_ring->count,
						      ARCH_DMA_MINALIGN);
		if (!rxdesc_ring->desc) {
			pr_info("%s: rxdesc_ring->desc alloc error\n",
				__func__);
			return -ENOMEM;
		}
		memset(rxdesc_ring->desc, 0,
		       EDMA_RXDESC_DESC_SIZE * rxdesc_ring->count);
		rxdesc_ring->dma = virt_to_phys(rxdesc_ring->desc);

		/*
		 * Allocate secondary Rx ring descriptors
		 */
		rxdesc_ring->sdesc = (void *)mem_alloc(EDMA_RX_SEC_DESC_SIZE * rxdesc_ring->count,
				ARCH_DMA_MINALIGN);
		if (!rxdesc_ring->sdesc) {
			pr_info("%s: rxdesc_ring->sdesc alloc error\n",
				__func__);
			return -ENOMEM;
		}
		memset(rxdesc_ring->sdesc, 0,
		       EDMA_RX_SEC_DESC_SIZE * rxdesc_ring->count);
		rxdesc_ring->sdma = virt_to_phys(rxdesc_ring->sdesc);
		ipq_edma_flush_range(rxdesc_ring->desc,
				     EDMA_RXDESC_DESC_SIZE * rxdesc_ring->count);
		ipq_edma_flush_range(rxdesc_ring->sdesc,
				     EDMA_RX_SEC_DESC_SIZE * rxdesc_ring->count);
	}

	/*
	 * Allocate TxDesc ring descriptors
	 */
	for (i = 0; i < ehw->txdesc_rings; i++) {
		txdesc_ring = &ehw->txdesc_ring[i];
		txdesc_ring->count = EDMA_TX_RING_SIZE;
		txdesc_ring->id = ehw->txdesc_ring_start + i;
		txdesc_ring->desc = (void *)mem_alloc(EDMA_TXDESC_DESC_SIZE * txdesc_ring->count,
				ARCH_DMA_MINALIGN);
		if (!txdesc_ring->desc) {
			pr_info("%s: txdesc_ring->desc alloc error\n",
				__func__);
			return -ENOMEM;
		}
		memset(txdesc_ring->desc, 0,
		       EDMA_TXDESC_DESC_SIZE * txdesc_ring->count);
		txdesc_ring->dma = virt_to_phys(txdesc_ring->desc);

		tx_buf = (void *)mem_alloc(EDMA_TX_BUFF_SIZE *
					txdesc_ring->count,
					ARCH_DMA_MINALIGN);
		if (!tx_buf) {
			pr_info("%s: txdesc_ring->desc buffer alloc error\n",
				__func__);
			return -ENOMEM;
		}

		/*
		 * Allocate buffers for each of the desc
		 */
		for (j = 0; j < txdesc_ring->count; j++) {
			txdesc_desc = EDMA_TXDESC_DESC(txdesc_ring, j);
			txdesc_desc->tdes0 = virt_to_phys(tx_buf);
#ifdef CONFIG_ARM64
			txdesc_desc->tdes1 =
				((u64)virt_to_phys(tx_buf) >> 32) &
				EDMA_TXDESC_BUF_HI_ADD_MASK;
#else
			txdesc_desc->tdes1 = 0;
#endif
			if (ipq_edma_txdesc_passthrough_enabled())
				txdesc_desc->tdes1 |= EDMA_TXDESC_PASSTHROUGH_EN;
			txdesc_desc->tdes2 = 0;
			txdesc_desc->tdes3 = 0;
			txdesc_desc->tdes4 = 0;
			txdesc_desc->tdes5 = 0;
			txdesc_desc->tdes6 = 0;
			txdesc_desc->tdes7 = 0;
			tx_buf += EDMA_TX_BUFF_SIZE;
		}

		/*
		 * Allocate secondary Tx ring descriptors
		 */
		txdesc_ring->sdesc = (void *)mem_alloc(EDMA_TX_SEC_DESC_SIZE * txdesc_ring->count,
				ARCH_DMA_MINALIGN);
		if (!txdesc_ring->sdesc) {
			pr_info("%s: txdesc_ring->sdesc alloc error\n",
				__func__);
			return -ENOMEM;
		}
		memset(txdesc_ring->sdesc, 0,
		       EDMA_TX_SEC_DESC_SIZE * txdesc_ring->count);
		txdesc_ring->sdma = virt_to_phys(txdesc_ring->sdesc);
		ipq_edma_flush_range(txdesc_ring->desc,
				     EDMA_TXDESC_DESC_SIZE * txdesc_ring->count);
		ipq_edma_flush_range(txdesc_ring->sdesc,
				     EDMA_TX_SEC_DESC_SIZE * txdesc_ring->count);
	}

	/*
	 * Allocate TxCmpl ring descriptors
	 */
	for (i = 0; i < ehw->txcmpl_rings; i++) {
		txcmpl_ring = &ehw->txcmpl_ring[i];
		txcmpl_ring->count = EDMA_TX_RING_SIZE;
		txcmpl_ring->id = ehw->txcmpl_ring_start + i;
		txcmpl_ring->desc = (void *)mem_alloc(EDMA_TXCMPL_DESC_SIZE * txcmpl_ring->count,
				ARCH_DMA_MINALIGN);

		if (!txcmpl_ring->desc) {
			pr_info("%s: txcmpl_ring->desc alloc error\n",
				__func__);
			return -ENOMEM;
		}
		memset(txcmpl_ring->desc, 0,
		       EDMA_TXCMPL_DESC_SIZE * txcmpl_ring->count);
		txcmpl_ring->dma = virt_to_phys(txcmpl_ring->desc);
		ipq_edma_flush_range(txcmpl_ring->desc,
				     EDMA_TXCMPL_DESC_SIZE * txcmpl_ring->count);
	}

	pr_debug("%s: successful\n", __func__);

	return 0;
}

static void ipq_edma_disable_rings(struct ipq_edma_hw *ehw)
{
	phys_addr_t reg_base = ehw->iobase;
	int i, desc_index;
	u32 data;

	/*
	 * Disable Rx rings
	 */
	for (i = 0; i < ehw->max_rxdesc_rings; i++) {
		data = readl(reg_base + EDMA_REG_RXDESC_CTRL(i));
		data &= ~EDMA_RXDESC_RX_EN;
		writel(data, reg_base + EDMA_REG_RXDESC_CTRL(i));
	}
	/*
	 * Disable RxFill Rings
	 */
	for (i = 0; i < ehw->max_rxfill_rings; i++) {
		data = readl(reg_base +
				EDMA_REG_RXFILL_RING_EN(i));
		data &= ~EDMA_RXFILL_RING_EN;
		writel(data, reg_base + EDMA_REG_RXFILL_RING_EN(i));
	}
	/*
	 * Disable Tx rings
	 */
	for (desc_index = 0; desc_index <
			 ehw->max_txdesc_rings; desc_index++) {
		data = readl(reg_base +
				EDMA_REG_TXDESC_CTRL(desc_index));
		data &= ~EDMA_TXDESC_TX_EN;
		writel(data, reg_base + EDMA_REG_TXDESC_CTRL(desc_index));
	}
}

static void ipq_edma_disable_intr(struct ipq_edma_hw *ehw)
{
	phys_addr_t reg_base = ehw->iobase;
	int i;

	/*
	 * Disable interrupts
	 */
	for (i = 0; i < ehw->max_rxdesc_rings; i++)
		writel(0, reg_base + EDMA_REG_RX_INT_CTRL(i));

	for (i = 0; i < ehw->max_rxfill_rings; i++)
		writel(0, reg_base + EDMA_REG_RXFILL_INT_MASK(i));

	for (i = 0; i < ehw->max_txcmpl_rings; i++)
		writel(0, reg_base + EDMA_REG_TX_INT_MASK(i));

	/*
	 * Clear MISC interrupt mask
	 */
	writel(EDMA_MASK_INT_DISABLE, reg_base + EDMA_REG_MISC_INT_MASK);
}

/*
 * ipq_edma_alloc_rings()
 *	Allocate EDMA software rings
 */
static int ipq_edma_alloc_rings(struct ipq_edma_hw *ehw)
{
	ehw->rxfill_ring = (void *)mem_alloc((sizeof(struct ipq_edma_rxfill_ring) *
				ehw->rxfill_rings),
				ARCH_DMA_MINALIGN);
	if (!ehw->rxfill_ring) {
		pr_info("%s: rxfill_ring alloc error\n", __func__);
		return -ENOMEM;
	}
	memset(ehw->rxfill_ring, 0,
	       sizeof(struct ipq_edma_rxfill_ring) * ehw->rxfill_rings);

	ehw->rxdesc_ring = (void *)mem_alloc((sizeof(struct ipq_edma_rxdesc_ring) *
				ehw->rxdesc_rings),
				ARCH_DMA_MINALIGN);
	if (!ehw->rxdesc_ring) {
		pr_info("%s: rxdesc_ring alloc error\n", __func__);
		return -ENOMEM;
	}
	memset(ehw->rxdesc_ring, 0,
	       sizeof(struct ipq_edma_rxdesc_ring) * ehw->rxdesc_rings);

	ehw->txdesc_ring = (void *)mem_alloc((sizeof(struct ipq_edma_txdesc_ring) *
				ehw->txdesc_rings),
				ARCH_DMA_MINALIGN);
	if (!ehw->txdesc_ring) {
		pr_info("%s: txdesc_ring alloc error\n", __func__);
		return -ENOMEM;
	}
	memset(ehw->txdesc_ring, 0,
	       sizeof(struct ipq_edma_txdesc_ring) * ehw->txdesc_rings);

	ehw->txcmpl_ring = (void *)mem_alloc((sizeof(struct ipq_edma_txcmpl_ring) *
				ehw->txcmpl_rings),
				ARCH_DMA_MINALIGN);
	if (!ehw->txcmpl_ring) {
		pr_info("%s: txcmpl_ring alloc error\n", __func__);
		return -ENOMEM;
	}
	memset(ehw->txcmpl_ring, 0,
	       sizeof(struct ipq_edma_txcmpl_ring) * ehw->txcmpl_rings);

	pr_debug("%s: successful\n", __func__);

	return 0;
}

/*
 * ipq_edma_init_rings()
 *	Initialize EDMA rings
 */
static int ipq_edma_init_rings(struct ipq_edma_hw *ehw)
{
	int ret;

	/*
	 * Allocate desc rings
	 */
	ret = ipq_edma_alloc_rings(ehw);
	if (ret)
		return ret;

	/*
	 * Setup ring resources
	 */
	ret = ipq_edma_setup_ring_resources(ehw);
	if (ret)
		return ret;

	return 0;
}

/*
 * ipq_edma_configure_txdesc_ring()
 *	Configure one TxDesc ring
 */
static void ipq_edma_configure_txdesc_ring(struct ipq_edma_hw *ehw,
					   struct ipq_edma_txdesc_ring *txdesc_ring)
{
	phys_addr_t reg_base = ehw->iobase;

	ipq_edma_program_txdesc_ring(ehw, txdesc_ring);

	writel(EDMA_TX_INITIAL_PROD_IDX,
	       reg_base + EDMA_REG_TXDESC_PROD_IDX(txdesc_ring->id));
}

/*
 * ipq_edma_configure_txcmpl_ring()
 *	Configure one TxCmpl ring
 */
static void ipq_edma_configure_txcmpl_ring(struct ipq_edma_hw *ehw,
					   struct ipq_edma_txcmpl_ring *txcmpl_ring)
{
	phys_addr_t reg_base = ehw->iobase;

	memset(txcmpl_ring->desc, 0,
	       EDMA_TXCMPL_DESC_SIZE * txcmpl_ring->count);
	ipq_edma_flush_range(txcmpl_ring->desc,
			     EDMA_TXCMPL_DESC_SIZE * txcmpl_ring->count);
	ipq_edma_program_txcmpl_ring(ehw, txcmpl_ring);

	writel(0, reg_base + EDMA_REG_TXCMPL_CONS_IDX(txcmpl_ring->id));
}

/*
 * ipq_edma_configure_rxdesc_ring()
 *	Configure one RxDesc ring
 */
static void ipq_edma_configure_rxdesc_ring(struct ipq_edma_hw *ehw,
					   struct ipq_edma_rxdesc_ring *rxdesc_ring)
{
	phys_addr_t reg_base = ehw->iobase;
	u32 data;
	u64 base;

	base = rxdesc_ring->dma;
	writel((u32)(base & EDMA_RING_DMA_MASK),
	       reg_base + EDMA_REG_RXDESC_BA(rxdesc_ring->id));
	writel((u32)(base >> 32),
	       reg_base + EDMA_REG_RXDESC_BA_HIGH(rxdesc_ring->id));

	base = rxdesc_ring->sdma;
	writel((u32)(base & EDMA_RING_DMA_MASK),
	       reg_base + EDMA_REG_RXDESC_BA2(rxdesc_ring->id));
	writel((u32)(base >> 32),
	       reg_base + EDMA_REG_RXDESC_BA2_HIGH(rxdesc_ring->id));

	if (ehw->sw_version == EDMA_SW_VER_2_ID) {
		data = readl(reg_base +
				EDMA_REG_RXDESC_FC_THRE(rxdesc_ring->id));
		data |= (ehw->rx_payload_offset & EDMA_RXDESC_PL_OFFSET_MASK)
			<< EDMA_RXDESC_PL_OFFSET_SHIFT_V2;
		writel(data, reg_base +
				EDMA_REG_RXDESC_FC_THRE(rxdesc_ring->id));

		data = rxdesc_ring->count & EDMA_RXDESC_RING_SIZE_MASK;
	} else {
		data = rxdesc_ring->count & EDMA_RXDESC_RING_SIZE_MASK;
		data |= (ehw->rx_payload_offset & EDMA_RXDESC_PL_OFFSET_MASK)
			<< EDMA_RXDESC_PL_OFFSET_SHIFT;
	}

	writel(data, reg_base +	EDMA_REG_RXDESC_RING_SIZE(rxdesc_ring->id));
	writel(0, reg_base + EDMA_REG_RXDESC_CONS_IDX(rxdesc_ring->id));

	/*
	 * Enable ring. Set ret mode to 'opaque'.
	 */
	writel(EDMA_RX_NE_INT_EN,
	       reg_base + EDMA_REG_RX_INT_CTRL(rxdesc_ring->id));
}

/*
 * ipq_edma_configure_rxfill_ring()
 *	Configure one RxFill ring
 */
static void ipq_edma_configure_rxfill_ring(struct ipq_edma_hw *ehw,
					   struct ipq_edma_rxfill_ring *rxfill_ring)
{
	phys_addr_t reg_base = ehw->iobase;
	u32 data;
	u64 base;

	base = rxfill_ring->dma;
	writel((u32)(base & EDMA_RING_DMA_MASK),
	       reg_base + EDMA_REG_RXFILL_BA(rxfill_ring->id));
	writel((u32)(base >> 32),
	       reg_base + EDMA_REG_RXFILL_BA_HIGH(rxfill_ring->id));

	data = rxfill_ring->count & EDMA_RXFILL_RING_SIZE_MASK;

	if (ehw->sw_version == EDMA_SW_VER_2_ID)
		writel(data, reg_base +
				EDMA_REG_RXFILL_RING_SIZE_V2(rxfill_ring->id));
	else
		writel(data, reg_base +
				EDMA_REG_RXFILL_RING_SIZE(rxfill_ring->id));
}

/*
 * ipq_edma_configure_rings()
 *	Configure EDMA rings
 */
static void ipq_edma_configure_rings(struct ipq_edma_hw *ehw)
{
	int i;

	/*
	 * Configure TXDESC ring
	 */
	for (i = 0; i < ehw->txdesc_rings; i++)
		ipq_edma_configure_txdesc_ring(ehw, &ehw->txdesc_ring[i]);

	/*
	 * Configure TXCMPL ring
	 */
	for (i = 0; i < ehw->txcmpl_rings; i++)
		ipq_edma_configure_txcmpl_ring(ehw, &ehw->txcmpl_ring[i]);

	/*
	 * Configure RXFILL rings
	 */
	for (i = 0; i < ehw->rxfill_rings; i++)
		ipq_edma_configure_rxfill_ring(ehw, &ehw->rxfill_ring[i]);

	/*
	 * Configure RXDESC ring
	 */
	for (i = 0; i < ehw->rxdesc_rings; i++)
		ipq_edma_configure_rxdesc_ring(ehw, &ehw->rxdesc_ring[i]);

	pr_debug("%s: successful\n", __func__);
}

/*
 * ipq_edma_hw_init()
 *	EDMA hw init
 */
int ipq_edma_hw_init(struct udevice *dev, struct ipq_eth_dev *eth)
{
	struct edma_config *config =
			(struct edma_config *)dev_get_driver_data(dev);
	struct ipq_edma_hw *ehw = &eth->hw;
	phys_addr_t reg_base = ehw->iobase;
	struct ppe_info *ppe = &eth->ppe;
	int ret;
	u32 data;
	u32 i;

	/*
	 * PPE Init
	 */
	ppe->no_ports = config->ports;
	ppe->nos_iports = config->iports;
	ppe->vsi = config->vsi;
	ppe->tdm_ctrl_val = config->tdm_ctrl_val;
	ppe->ipo_action = config->ipo_action;

	ipq_ppe_provision_init(ppe);
	ipq_active_ppe = ppe;

	data = readl(reg_base + EDMA_REG_MAS_CTRL);
	if (ipq_edma_debug_trace())
		printf("EDMA ver %d\n", data);

	/*
	 * Setup private data structure
	 */
	ehw->sw_version = config->sw_version;
	ehw->rxfill_intr_mask = EDMA_RXFILL_INT_MASK;
	ehw->rxdesc_intr_mask = EDMA_RXDESC_INT_MASK_PKT_INT;
	ehw->txcmpl_intr_mask = EDMA_TX_INT_MASK_PKT_INT;
	ehw->misc_intr_mask = EDMA_MISC_INTR_MASK;
	ehw->rx_payload_offset = EDMA_RX_PAYLOAD_OFFSET;

	UPDATE_EDMA_CONFIG(config, ehw);

	/*
	 * Disable interrupts
	 */
	ipq_edma_disable_intr(ehw);

	/*
	 * Disable rings
	 */
	ipq_edma_disable_rings(ehw);

	ret = ipq_edma_init_rings(ehw);
	if (ret)
		return ret;

	ipq_edma_configure_rings(ehw);

	ipq_edma_program_static_maps(ehw);
	if (ipq_edma_debug_trace())
		printf("EDMA TXDESC2CMPL map: %08x/%08x/%08x/%08x/%08x/%08x\n",
		       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_0),
		       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_1),
		       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_2),
		       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_3),
		       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_4),
		       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_5));

	for (i = 0; i < ehw->rxfill_rings; i++)
		ipq_edma_prime_rxfill_ring(ehw, &ehw->rxfill_ring[i]);

	ipq_edma_program_tx_globals(ehw);
	/*
	 * Enable Rx rings
	 */
	for (i = ehw->rxdesc_ring_start; i < ehw->rxdesc_ring_end; i++) {
		data = readl(reg_base + EDMA_REG_RXDESC_CTRL(i));
		data |= EDMA_RXDESC_RX_EN;
		writel(data, reg_base + EDMA_REG_RXDESC_CTRL(i));
	}

	for (i = ehw->rxfill_ring_start; i < ehw->rxfill_ring_end; i++) {
		data = readl(reg_base + EDMA_REG_RXFILL_RING_EN(i));
		data |= EDMA_RXFILL_RING_EN;
		writel(data, reg_base + EDMA_REG_RXFILL_RING_EN(i));
	}
	/*
	 * Enable Tx rings
	 */
	for (i = ehw->txdesc_ring_start; i < ehw->txdesc_ring_end; i++) {
		data = readl(reg_base + EDMA_REG_TXDESC_CTRL(i));
		data |= EDMA_TXDESC_TX_EN;
		writel(data, reg_base + EDMA_REG_TXDESC_CTRL(i));
	}
	/*
	 * Enable MISC interrupt mask
	 */
	writel(ehw->misc_intr_mask, reg_base + EDMA_REG_MISC_INT_MASK);

	pr_debug("%s: successful\n", __func__);

	return 0;
}

static int ipq_eth_port_set_up(struct ipq_eth_dev *priv,
			       struct port_info *port)
{
	bool use_dt_interface;
	int mac_speed, i, rate = 0;
	int ret = 0;
	char clk_name[64] = "";
	ulong clk_data, parent_rate;
	struct clk clk, pclk;

	switch (port->cur_speed) {
	case 100:
		mac_speed = 1;
		break;
	case 1000:
		mac_speed = 2;
		break;
	case 10000:
		mac_speed = 3;
		break;
	case 2500:
		mac_speed = (port->phy_id == QCA8081_PHY_TYPE || port->xgmac) ?
			    4 : 2;
		break;
	case 5000:
		mac_speed = 5;
		break;
	default:
		/* speed 10Mbps */
		mac_speed = 0;
	}

	port->mac_speed = mac_speed;
	use_dt_interface = port->from_dt_port &&
			   ipq_eth_port_uses_dt_interface(port);
	if (use_dt_interface)
		ipq_eth_apply_interface_override(port, &rate);

	for (i = 0; port_config[i].id != UNUSED_PHY_TYPE; ++i) {
		if (port->phy_id != port_config[i].id)
			continue;
		if (!use_dt_interface) {
			rate = port_config[i].clk_rate[mac_speed];
			port->uniphy_mode = port_config[i].mode[mac_speed];
			port->gmac_type = port_config[i].mac_mode[mac_speed];
		}
		break;
	}

	if (port_config[i].id != UNUSED_PHY_TYPE || use_dt_interface) {
		if (ipq_edma_debug_trace())
			printf("port%u setup: phy_type=%s(%u) dt_phy_mode=%s package=%s interface=%s speed=%u mac_speed=%d rate=%d uniphy%u mode=%s(%u) cur_mode=%s(%u) gmac=%u cur_gmac=%u\n",
			       port->id, ipq_eth_phy_type_name(port->phy_id),
			       port->phy_id, port->dt_phy_mode ?: "<none>",
			       port->package_mode ?: "<none>",
			       ipq_eth_interface_name(port->interface),
			       port->cur_speed, mac_speed, rate, port->uniphy_id,
			       ipq_eth_uniphy_mode_name(port->uniphy_mode),
			       port->uniphy_mode,
			       ipq_eth_uniphy_mode_name(port->cur_uniphy_mode),
			       port->cur_uniphy_mode, port->gmac_type,
			       port->cur_gmac_type);

		if (port->cur_uniphy_mode != port->uniphy_mode) {
			ret = ppe_uniphy_mode_set(port);
			if (ret)
				goto fail;
			port->cur_uniphy_mode = port->uniphy_mode;
		}

		snprintf(clk_name, sizeof(clk_name), "nss_cc_port%d_mac_clk",
			 port->id);

		ret = ipq_eth_get_port_clk(priv->dev, port, "mac",
					   clk_name, &clk);
		if (ret)
			goto fail;

		ret = clk_enable(&clk);
		if (ret && ret != -ENOSYS)
			goto fail;

		clk_data = ipq_eth_port_clk_data(port);
		parent_rate = ipq_eth_uniphy_parent_rate(port);

		snprintf(clk_name, sizeof(clk_name), "uniphy%d_nss_rx_clk",
			 port->uniphy_id);

		ret = clk_get_by_name(priv->dev, clk_name, &pclk);
		if (ret && ret != -ENOENT && ret != -ENODATA && ret != -EINVAL)
			goto fail;

		if (!ret)
			clk_set_rate(&pclk, parent_rate);

		snprintf(clk_name, sizeof(clk_name), "nss_cc_port%d_rx_clk",
			 port->id);

		ret = ipq_eth_get_port_clk(priv->dev, port, "rx",
					   clk_name, &clk);
		if (ret)
			goto fail;

		clk.data = clk_data;
		ret = ipq_eth_clk_set_rate(&clk, rate);
		if (ret && ret != -ENOSYS)
			goto fail;
		ret = clk_enable(&clk);
		if (ret && ret != -ENOSYS)
			goto fail;

		snprintf(clk_name, sizeof(clk_name), "uniphy%d_nss_tx_clk",
			 port->uniphy_id);

		ret = clk_get_by_name(priv->dev, clk_name, &pclk);
		if (ret && ret != -ENOENT && ret != -ENODATA && ret != -EINVAL)
			goto fail;

		if (!ret)
			clk_set_rate(&pclk, parent_rate);

		snprintf(clk_name, sizeof(clk_name), "nss_cc_port%d_tx_clk",
			 port->id);

		ret = ipq_eth_get_port_clk(priv->dev, port, "tx",
					   clk_name, &clk);
		if (ret)
			goto fail;

		clk.data = clk_data;
		ret = ipq_eth_clk_set_rate(&clk, rate);
		if (ret && ret != -ENOSYS)
			goto fail;
		ret = clk_enable(&clk);
		if (ret && ret != -ENOSYS)
			goto fail;

		snprintf(clk_name, sizeof(clk_name),
			 "nss_cc_uniphy_port%d_rx_clk", port->id);

		ret = ipq_eth_get_pcs_channel_clk(priv->dev, port, "rx",
						  clk_name, &clk);
		if (ret)
			goto fail;

		clk_enable(&clk);

		snprintf(clk_name, sizeof(clk_name),
			 "nss_cc_uniphy_port%d_tx_clk", port->id);

		ret = ipq_eth_get_pcs_channel_clk(priv->dev, port, "tx",
						  clk_name, &clk);
		if (ret)
			goto fail;

		clk_enable(&clk);

		ipq_port_mac_clock_reset(priv->dev, port);

		ppe_port_speed_set(priv->ppe.base, port);

	} else if (priv->emulation) {
		port->gmac_type = XGMAC;
		port->mac_speed = mac_speed;
		port->uniphy_mode = PORT_WRAPPER_EMULATION;

		ppe_port_speed_set(priv->ppe.base, port);
	}
fail:
	if (ret)
		printf("port%d setup failed: ret=%d last_clk=%s speed=%u phy_type=%u mode=%u gmac=%u\n",
		       port->id, ret, clk_name, port->cur_speed,
		       port->phy_id, port->uniphy_mode, port->gmac_type);
	return ret;
}

static int ipq_eth_get_port_clk(struct udevice *dev, struct port_info *port,
				const char *port_name, const char *dev_name,
				struct clk *clk)
{
	int ret;

	if (ofnode_valid(port->port_node)) {
		ret = clk_get_by_name_nodev(port->port_node, port_name, clk);
		if (!ret)
			return 0;
	}

	return clk_get_by_name(dev, dev_name, clk);
}

static int ipq_eth_get_pcs_channel_clk(struct udevice *dev,
				       struct port_info *port,
				       const char *pcs_name,
				       const char *dev_name,
				       struct clk *clk)
{
	int ret;

	ret = ipq_eth_get_optional_node_clk(port->pcs_node, pcs_name, clk);
	if (!ret)
		return 0;
	if (ret != -ENOENT)
		return ret;

	return clk_get_by_name(dev, dev_name, clk);
}

static int ipq_eth_get_port_reset(struct udevice *dev, struct port_info *port,
				  const char *port_name, const char *dev_name,
				  struct reset_ctl *rst)
{
	int index, ret;

	if (ofnode_valid(port->port_node)) {
		index = ofnode_stringlist_search(port->port_node,
						 "reset-names", port_name);
		if (index >= 0) {
			ret = reset_get_by_index_nodev(port->port_node,
						       index, rst);
			if (!ret)
				return 0;
		}
	}

	return reset_get_by_name(dev, dev_name, rst);
}

static bool ipq_eth_port_selected(struct port_info *port, int slot,
				  bool active_port_set,
				  ulong active_port, bool active_port_id_set,
				  ulong active_port_id)
{
	if (!port)
		return false;

	if (active_port_set) {
		if (port->from_dt_port) {
			if (active_port != port->id)
				return false;
		} else if (active_port != CONFIG_ETH_MAX_MAC &&
			   active_port != slot) {
			return false;
		}
	}

	if (active_port_id_set && active_port_id != port->id)
		return false;

	return true;
}

static int ipq_eth_refresh_link(struct ipq_eth_dev *priv, bool quiet)
{
	struct phy_device *phydev;
	struct port_info *port = NULL;
	bool active_port_set = env_get("active_port") != NULL;
	bool active_port_id_set = env_get("active_port_id") != NULL;
	ulong active_port = env_get_ulong("active_port", 10,
					  CONFIG_ETH_MAX_MAC);
	ulong active_port_id = active_port_id_set ?
			       env_get_ulong("active_port_id", 10, 0) : 0;
	int i, ret, link, speed, duplex, linkup = -1;

	for (i = 0; i < CONFIG_ETH_MAX_MAC; ++i) {
		port = priv->port[i];

		if (!port)
			continue;

		if (!port->phydev && !priv->emulation)
			continue;

		if (!ipq_eth_port_selected(port, i, active_port_set,
					   active_port,
					   active_port_id_set, active_port_id)) {
			ppe_port_bridge_txmac_set(priv->ppe.base,
						  port->id, false);
			port->cur_speed = 0;
			continue;
		}
		/*
		 * set default value before read phy status
		 */
		link = 0;
		speed = 10;
		duplex = 0;

		if (port->phy_id == SFP10G_PHY_TYPE ||
		    port->phy_id == SFP2_5G_PHY_TYPE ||
		    port->phy_id == SFP1G_PHY_TYPE) {
			ret = phy_status_get_from_ppe(priv->ppe.base,
						      port->id);
			link = ((ret & LINK_STATUS) != 0) ? 1 : 0;
			if (link) {
				++linkup;
				duplex = ((ret & DUPLEX) != 0) ? 1 : 0;
				speed = mac_speed_config[ret & SPEED];
			}
		} else {
			phydev = port->phydev;

			if (phydev && port->isconfigured) {
				/* Start up the PHY */
				ret = phy_startup(phydev);
				if (ret < 0) {
					if (!quiet || ipq_edma_debug_trace())
						printf("PHY%u addr 0x%x (%s): status read failed: %d\n",
						       port->id, port->phyaddr,
						       ipq_eth_phy_type_name(port->phy_id), ret);
					continue;
				}
				if (phydev->link) {
					++linkup;
					link = phydev->link;
					duplex = phydev->duplex;
					speed = phydev->speed;
					if (port->phy_id == QCA8081_PHY_TYPE)
						port->interface = phydev->interface;
				}
			} else if (priv->emulation) {
				/*
				 * Clock rate will be like 1/100 or 1/150
				 * in the emulation platform. So, configuring
				 * MAC with higher speed, but in actuall it is
				 * running with lower PHY speed.
				 *
				 * Eg: 10G (MAC) <==> 100M (PHY)
				 */
				++linkup;
				link = 1;
				duplex = 1;
				speed = 10000;
			} else {
				continue;
			}
		}

		if ((port->phy_id != QCA8x8x_SWITCH_TYPE) &&
			(port->phy_id != QCA8337_SWITCH_TYPE) &&
			(port->phy_id != QCE2204_SWITCH_TYPE) &&
			(!quiet || ipq_edma_debug_trace()))
			printf("PHY%d %s Speed : %d %s\n", port->id,
			       (link ? "Up" : "Down"), speed,
				duplex ? "Full duplex" : "Half duplex");

		if (!link) {
			ppe_port_bridge_txmac_set(priv->ppe.base,
						  port->id, false);
			port->cur_speed = 0;
			continue;
		}

		if (port->cur_speed != speed ||
		    !(readl(priv->ppe.base + PPE_PORT_BRIDGE_CTRL_OFFSET +
			    (port->id * PORT_BRIDGE_CTRL_INC)) &
		      ppe_port_bridge_txmac_mask())) {
			port->cur_speed = speed;
			port->duplex = duplex;
			ret = ipq_eth_port_set_up(priv, port);
			if (ret) {
				port->cur_speed = 0;
				continue;
			}
			ppe_port_bridge_txmac_set(priv->ppe.base,
						  port->id, true);
		} else {
			port->duplex = duplex;
		}

		/*
		 * In bridge mode packets emitted by the CPU enter the shared VSI
		 * through PPE port 0 and the hardware bridge selects the physical
		 * egress port.  Do not replace that CPU-port destination with the
		 * last PHY seen during a link refresh; when a 10G WAN comes up after
		 * a LAN port, DHCP/HTTP replies would otherwise be directed at the
		 * WAN port.  Non-bridge mode still needs the selected physical port.
		 */
		if (!priv->ppe.bridge_mode)
			priv->ppe.nbport = port->id;
	}

	return linkup;
}

static bool ipq_eth_has_active_link(struct ipq_eth_dev *priv)
{
	struct port_info *port;
	u32 bridge;
	int i;

	for (i = 0; i < CONFIG_ETH_MAX_MAC; ++i) {
		port = priv->port[i];
		if (!port || !port->cur_speed)
			continue;

		bridge = readl(priv->ppe.base + PPE_PORT_BRIDGE_CTRL_OFFSET +
			       port->id * PORT_BRIDGE_CTRL_INC);
		if (bridge & ppe_port_bridge_txmac_mask())
			return true;
	}

	return false;
}

static void ipq_eth_refresh_link_if_needed(struct ipq_eth_dev *priv,
					   bool force)
{
	ulong now;

	if (env_get_yesno("eth_allow_no_link") != 1)
		return;

	/*
	 * HTTP recovery may start with a link on one port and be moved to
	 * another port while the server is running.  Do not trust the cached
	 * active-link bit in that mode; it would prevent the new PHY from ever
	 * being sampled.  The timer below keeps the MDIO work bounded.
	 */

	now = get_timer(0);
	if (!force && ipq_eth_link_refresh_last_ms &&
	    now - ipq_eth_link_refresh_last_ms < 500)
		return;

	ipq_eth_link_refresh_last_ms = now;
	ipq_eth_refresh_link(priv, true);
}

static int ipq_eth_start(struct udevice *dev)
{
	struct ipq_eth_dev *priv = dev_get_priv(dev);
	bool allow_no_link = env_get_yesno("eth_allow_no_link") == 1;
	int linkup, ret;

	ipq_edma_rx_log_count = 0;
	ipq_edma_tx_log_count = 0;
	ipq_edma_txc_log_count = 0;
	ipq_edma_rx_frame_log_count = 0;
	ipq_edma_tx_frame_log_count = 0;
	ipq_edma_tx_stall_dump_count = 0;
	ipq_edma_debug_last_ms = 0;
	ipq_eth_link_refresh_last_ms = 0;
	ipq_edma_debug_refresh_runtime();

	/*
	 * Chainloader boot can inherit a live EDMA block from the previous
	 * firmware stage. Reset the EDMA child block on every net start, then
	 * restore all ring registers and maps before enabling traffic.
	 */
	ret = ipq_eth_reset_edma_node(dev);
	if (ret && ret != -ENOENT)
		return ret;
	ipq_edma_start_generation++;

	ipq_edma_configure_rings(&priv->hw);
	ipq_edma_program_static_maps(&priv->hw);
	ipq_edma_rearm_rx_path(&priv->hw);
	ipq_edma_rearm_tx_path(&priv->hw);
	ipq_edma_program_tx_globals(&priv->hw);
	writel(priv->hw.misc_intr_mask,
	       priv->hw.iobase + EDMA_REG_MISC_INT_MASK);

	if (priv->ppe.bridge_mode)
		priv->ppe.nbport = 0;

	if (IS_ENABLED(CONFIG_CMD_TFTPBOOT))
		env_set_ulong("tftpsrcp", tftp_acl_our_port);

	/* HTTP recovery sets eth_allow_no_link and does not need link chatter. */
	linkup = ipq_eth_refresh_link(priv, allow_no_link);
	if (ipq_edma_debug_trace()) {
		u32 port_id = 0;

		if (ipq_eth_debug_previous_counters_valid) {
			u32 previous_port = ipq_eth_debug_previous_counters.port_id;
			struct port_info *port;

			port = ipq_eth_debug_find_port(priv, previous_port);

			if (port && port->cur_speed)
				port_id = port->id;
		}
		if (!port_id)
			port_id = ipq_eth_debug_active_port(priv);

		if (port_id)
			ipq_eth_debug_dump_counters(priv, port_id);
	}
	if (linkup >= 0)
		return linkup;

	return allow_no_link ? 0 : -1;
}

static int ipq_eth_send(struct udevice *dev, void *packet, int length)
{
	struct ipq_eth_dev *priv = dev_get_priv(dev);
	struct ipq_edma_hw *ehw = &priv->hw;
	struct ppe_info *ppe = &priv->ppe;
	struct ipq_edma_txdesc_desc *txdesc;
	struct ipq_edma_txdesc_ring *txdesc_ring;
	struct ipq_edma_txcmpl_ring *txcmpl_ring;
	u16 desc_idx, clean_idx;
	u32 data, prod_raw, cons_raw, hw_next_to_use, hw_next_to_clean, hw_next;
	u32 txc_prod_before = 0, txd_cons_after = 0, txc_prod_after = 0;
	uchar *skb;
	phys_addr_t reg_base = ehw->iobase;
	int i, tx_length;

	if (!ipq_eth_has_active_link(priv))
		ipq_eth_refresh_link_if_needed(priv, true);

	if (length <= 0 || length > EDMA_TX_BUFF_SIZE) {
		printf("EDMA TX invalid len=%d\n", length);
		return -EINVAL;
	}

	tx_length = length;
	if (tx_length < EDMA_TX_MIN_PKT_SIZE)
		tx_length = EDMA_TX_MIN_PKT_SIZE;

	txdesc_ring = ehw->txdesc_ring;
	txcmpl_ring = ehw->txcmpl_ring;
	/*
	 * Read TXDESC ring producer index
	 */
	data = readl(reg_base + EDMA_REG_TXDESC_PROD_IDX(txdesc_ring->id));

	prod_raw = data & EDMA_TXDESC_PROD_IDX_MASK;
	hw_next_to_use = ipq_edma_ring_index(prod_raw, txdesc_ring->count);
	desc_idx = hw_next_to_use;

	pr_debug("%s: txdesc_ring->id = %d\n", __func__, txdesc_ring->id);

	/*
	 * Read TXDESC ring consumer index
	 */
	/*
	 * TODO - read to local variable to optimize uncached access
	 */
	data = readl(reg_base +
			EDMA_REG_TXDESC_CONS_IDX(txdesc_ring->id));

	cons_raw = data & EDMA_TXDESC_CONS_IDX_MASK;
	hw_next_to_clean = ipq_edma_ring_index(cons_raw, txdesc_ring->count);
	clean_idx = hw_next_to_clean;
	pr_debug("%s: use=%u/%u clean=%u/%u\n", __func__,
		 desc_idx, prod_raw, clean_idx, cons_raw);

	/* EDMA ring indices are valid only inside the configured ring size. */
	hw_next = ipq_edma_ring_next(hw_next_to_use,
					 txdesc_ring->count);
	if (hw_next == hw_next_to_clean) {
		pr_info("netdev tx busy");
		return -EBUSY;
	}

	if (ipq_edma_debug_trace() && txcmpl_ring)
		txc_prod_before = readl(reg_base +
					EDMA_REG_TXCMPL_PROD_IDX(txcmpl_ring->id)) &
				  EDMA_TXCMPL_PROD_IDX_MASK;

	/*
	 * Get Tx descriptor
	 */
	txdesc = EDMA_TXDESC_DESC(txdesc_ring, desc_idx);

	txdesc->tdes2 = 0;
	txdesc->tdes3 = 0;
	txdesc->tdes4 = 0;
	txdesc->tdes5 = 0;
	txdesc->tdes6 = 0;
	txdesc->tdes7 = 0;
	txdesc->tdes1 &= EDMA_TXDESC_BUF_HI_ADD_MASK;
	if (ipq_edma_txdesc_passthrough_enabled())
		txdesc->tdes1 |= EDMA_TXDESC_PASSTHROUGH_EN;
	skb = (uchar *)((uintptr_t)(((uint64_t)(txdesc->tdes1 &
				EDMA_TXDESC_BUF_HI_ADD_MASK) << 32) |
				txdesc->tdes0));

	pr_debug("%s: txdesc->tdes0 (buffer addr) = 0x%lx ", __func__, (uintptr_t)txdesc->tdes0);
	pr_debug("txdesc->tdes1 (buffer addr) = 0x%lx ", (uintptr_t)txdesc->tdes1);
	pr_debug("length = %d prod_idx = %u/%u cons_idx = %u/%u\n", length,
		 desc_idx, hw_next_to_use, clean_idx, hw_next_to_clean);

	txdesc->tdes4 = ipq_edma_tx_tdes4_value(ppe);

	/*
	 * Set opaque field
	 */
	txdesc->tdes2 = cpu_to_le32(txdesc->tdes0);
	txdesc->tdes3 = (cpu_to_le32(txdesc->tdes1) &
			EDMA_TXDESC_BUF_HI_ADD_MASK);

	/*
	 * copy the packet
	 */
	memcpy(skb, packet, length);
	if (tx_length > length)
		memset(skb + length, 0, tx_length - length);

	/*
	 * Populate Tx descriptor
	 */
	txdesc->tdes5 = ((tx_length << EDMA_TXDESC_DATA_LENGTH_SHIFT) &
			 EDMA_TXDESC_DATA_LENGTH_MASK);

	if (ipq_edma_debug_trace() && ipq_edma_tx_log_count < 16) {
		printf("EDMA TX len=%d hw_len=%d dst_port=%u bridge=%d tdes4_mode=%s passthrough=%u tdes1=0x%08x tdes4=0x%08x tdes5=0x%08x tdes6=0x%08x ctrl=0x%08x prod=%u cons=%u\n",
		       length, tx_length, ppe->nbport, ppe->bridge_mode,
		       ipq_edma_tdes4_mode_name(),
		       ipq_edma_txdesc_passthrough_enabled() ? 1 : 0,
		       txdesc->tdes1,
		       txdesc->tdes4, txdesc->tdes5, txdesc->tdes6,
		       readl(reg_base + EDMA_REG_TXDESC_CTRL(txdesc_ring->id)),
		       desc_idx, clean_idx);
		ipq_edma_tx_log_count++;
	}
	ipq_eth_debug_log_frame("TX", packet, length, ppe->nbport,
				txdesc->tdes4, txdesc->tdes5);

	ipq_edma_flush_range(skb, tx_length);
	ipq_edma_flush_range(txdesc, EDMA_TXDESC_DESC_SIZE);
	wmb();

	/*
	 * Update producer index
	 */
	hw_next_to_use = hw_next;

	/*
	 * make sure the hw_next_to_use is updated before the
	 * write to hardware
	 */

	writel(hw_next_to_use & EDMA_TXDESC_PROD_IDX_MASK,
	       reg_base + EDMA_REG_TXDESC_PROD_IDX(txdesc_ring->id));

	if (ipq_edma_debug_trace() && txcmpl_ring &&
	    ipq_edma_tx_stall_dump_count < 2) {
		for (i = 0; i < 10; i++) {
			udelay(20);
			txd_cons_after = readl(reg_base +
					       EDMA_REG_TXDESC_CONS_IDX(txdesc_ring->id)) &
					 EDMA_TXDESC_CONS_IDX_MASK;
			txc_prod_after = readl(reg_base +
					       EDMA_REG_TXCMPL_PROD_IDX(txcmpl_ring->id)) &
					 EDMA_TXCMPL_PROD_IDX_MASK;
			if (ipq_edma_ring_index(txd_cons_after,
						txdesc_ring->count) !=
			    hw_next_to_clean ||
			    txc_prod_after != txc_prod_before)
				break;
		}

		if (ipq_edma_ring_index(txd_cons_after,
					txdesc_ring->count) ==
		    hw_next_to_clean &&
		    txc_prod_after == txc_prod_before) {
			printf("EDMA TX stall: txd%u prod=%u cons=%u txc%u prod=%u after submit\n",
			       txdesc_ring->id, hw_next_to_use, txd_cons_after,
			       txcmpl_ring->id, txc_prod_after);
			ipq_edma_debug_dump_tx_regs(priv, "stall");
			ipq_edma_tx_stall_dump_count++;
		}
	}

	pr_debug("%s: successful\n", __func__);

	return 0;
}

static int ipq_eth_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct ipq_eth_dev *priv = dev_get_priv(dev);
	struct ipq_edma_rxdesc_ring *rxdesc_ring;
	struct ipq_edma_txcmpl_ring *txcmpl_ring;
	struct ipq_edma_rxfill_ring *rxfill_ring;
	struct ipq_edma_hw *ehw = &priv->hw;
	phys_addr_t reg_base = ehw->iobase;
	u32 reg_data;
	u32 rxdesc_intr_status = 0;
	u32 txcmpl_intr_status = 0, rxfill_intr_status = 0;
	int i, length = 0;

	ipq_eth_refresh_link_if_needed(priv, false);

	/*
	 * Read RxDesc intr status
	 */
	for (i = 0; i < ehw->rxdesc_rings; i++) {
		rxdesc_ring = &ehw->rxdesc_ring[i];

		reg_data = readl(reg_base +
				EDMA_REG_RXDESC_INT_STAT(rxdesc_ring->id));
		rxdesc_intr_status |= reg_data &
				EDMA_RXDESC_RING_INT_STATUS_MASK;

		/*
		 * Disable RxDesc intr
		 */
		writel(EDMA_MASK_INT_DISABLE,
		       reg_base + EDMA_REG_RXDESC_INT_MASK(rxdesc_ring->id));
	}

	/*
	 * Read TxCmpl intr status
	 */
	for (i = 0; i < ehw->txcmpl_rings; i++) {
		txcmpl_ring = &ehw->txcmpl_ring[i];

		reg_data = readl(reg_base +
				EDMA_REG_TX_INT_STAT(txcmpl_ring->id));
		txcmpl_intr_status |= reg_data &
				EDMA_TXCMPL_RING_INT_STATUS_MASK;

		/*
		 * Disable TxCmpl intr
		 */
		writel(EDMA_MASK_INT_DISABLE,
		       reg_base + EDMA_REG_TX_INT_MASK(txcmpl_ring->id));
	}

	/*
	 * Read RxFill intr status
	 */
	for (i = 0; i < ehw->rxfill_rings; i++) {
		rxfill_ring = &ehw->rxfill_ring[i];

		reg_data = readl(reg_base +
				EDMA_REG_RXFILL_INT_STAT(rxfill_ring->id));
		rxfill_intr_status |= reg_data &
				EDMA_RXFILL_RING_INT_STATUS_MASK;

		/*
		 * Disable RxFill intr
		 */
		writel(EDMA_MASK_INT_DISABLE,
		       reg_base + EDMA_REG_RXFILL_INT_MASK(rxfill_ring->id));
	}

	if (rxdesc_intr_status != 0 || txcmpl_intr_status != 0 || rxfill_intr_status != 0) {
		pr_debug("%s: rxdesc_intr_status = %d txcmpl_intr_status = %d rxfill_intr_status = %d\n",
			 __func__,
				rxdesc_intr_status, txcmpl_intr_status,
				rxfill_intr_status);
		for (i = 0; i < ehw->rxdesc_rings; i++) {
			rxdesc_ring = &ehw->rxdesc_ring[i];
			writel(EDMA_MASK_INT_DISABLE,
			       reg_base + EDMA_REG_RXDESC_INT_MASK(rxdesc_ring->id));
		}
	}

	length = ipq_edma_rx_complete(priv, (void **)packetp);
	if (length <= 0)
		ipq_eth_debug_dump_periodic(priv);

	return length;
}

static int ipq_eth_free_pkt(struct udevice *dev, uchar *packet, int length)
{
	return 0;
}

static void ipq_eth_stop(struct udevice *dev)
{
	struct ipq_eth_dev *priv = dev_get_priv(dev);
	struct phy_device *phydev;
	int i;

	for (i = 0; i < CONFIG_ETH_MAX_MAC; ++i) {
		if (!priv->port[i])
			continue;

		phydev = priv->port[i]->phydev;
		if (phydev && priv->port[i]->isconfigured)
			phy_shutdown(phydev);
		priv->port[i]->cur_speed = 0;
	}
}

static int ipq_eth_write_hwaddr(struct udevice *dev)
{
	return 0;
}

struct ipq_env_image {
	u32 crc;
	u8 data[CONFIG_ENV_SIZE - sizeof(u32)];
};

static bool ipq_eth_mac_from_env_blob(const char *blob, size_t blob_len,
				      const char *name, u8 mac[6])
{
	const char *p = blob;
	size_t name_len = strlen(name);

	while (p < blob + blob_len && *p) {
		if (!strncmp(p, name, name_len) && p[name_len] == '=') {
			string_to_enetaddr(p + name_len + 1, mac);
			return is_valid_ethaddr(mac);
		}
		p += strlen(p) + 1;
	}

	return false;
}

static bool ipq_eth_read_appsblenv_mac(u8 mac[6])
{
	struct blk_desc *desc;
	struct disk_partition part;
	struct ipq_env_image *env;
	size_t blks;
	uint32_t crc;
	bool found = false;

	/* Factory MAC lives in the stock APPSBLENV partition on this board. */
	if (part_get_info_by_dev_and_name_or_num("mmc", "0#0:APPSBLENV",
						 &desc, &part, false))
		return false;

	blks = CONFIG_ENV_SIZE / part.blksz;
	if (!blks || blks > part.size)
		return false;

	env = malloc_cache_aligned(CONFIG_ENV_SIZE);
	if (!env)
		return false;

	memset(env, 0, CONFIG_ENV_SIZE);
	if (blk_dread(desc, part.start, blks, env) != blks)
		goto out;

	memcpy(&crc, &env->crc, sizeof(crc));
	if (crc32(0, env->data, sizeof(env->data)) != crc)
		goto out;

	found = ipq_eth_mac_from_env_blob((const char *)env->data,
					  sizeof(env->data),
					  "ethaddr", mac);
out:
	free(env);
	return found;
}

static ofnode ipq_eth_mac_node(struct ipq_eth_dev *priv)
{
	ofnode fallback = ofnode_null();
	int i;

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		if (priv->port[i] && priv->port[i]->cur_speed &&
		    ofnode_valid(priv->port[i]->port_node))
			return priv->port[i]->port_node;
	}

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		if (!priv->port[i] || !ofnode_valid(priv->port[i]->port_node))
			continue;
		if (!ofnode_valid(fallback))
			fallback = priv->port[i]->port_node;
		if ((priv->port[i]->label &&
		     !strcmp(priv->port[i]->label, "wan")) ||
		    priv->port[i]->id == 6)
			return priv->port[i]->port_node;
	}

	if (ofnode_valid(fallback))
		return fallback;

	return dev_ofnode(priv->dev);
}

static bool ipq_eth_read_dt_nvmem_mac(struct ipq_eth_dev *priv, u8 mac[6])
{
	struct ofnode_phandle_args args;
	struct blk_desc *desc;
	struct disk_partition part;
	ofnode mac_node;
	ofnode cell, layout, part_node;
	fdt_addr_t offset;
	fdt_size_t size;
	const char *partname;
	char part_spec[64];
	u8 *buf;
	u8 base[6];
	s64 mac_offset = 0;
	u64 value;
	u64 block_offset;
	lbaint_t start_block, read_blks;
	int ret;

	mac_node = ipq_eth_mac_node(priv);
	ret = ofnode_parse_phandle_with_args(mac_node, "nvmem-cells",
					     "#nvmem-cell-cells", 0, 0,
					     &args);
	if (ret)
		return false;

	cell = args.node;
	layout = ofnode_get_parent(cell);
	part_node = ofnode_get_parent(layout);
	partname = ofnode_read_string(part_node, "partname");

	if (!ofnode_device_is_compatible(layout, "fixed-layout") ||
	    !ofnode_device_is_compatible(cell, "mac-base") ||
	    !partname || strlen(partname) > sizeof(part_spec) - 3)
		return false;

	offset = ofnode_get_addr_size_index_notrans(cell, 0, &size);
	if (offset == FDT_ADDR_T_NONE || size != sizeof(base))
		return false;

	if (args.args_count > 0)
		mac_offset = (s32)args.args[0];

	snprintf(part_spec, sizeof(part_spec), "0#%s", partname);

	if (part_get_info_by_dev_and_name_or_num("mmc", part_spec,
						 &desc, &part, false))
		return false;

	if ((u64)offset + sizeof(base) > (u64)part.size * part.blksz)
		return false;

	block_offset = offset % part.blksz;
	start_block = part.start + offset / part.blksz;
	read_blks = (block_offset + sizeof(base) + part.blksz - 1) / part.blksz;
	if (!read_blks || start_block < part.start ||
	    start_block + read_blks > part.start + part.size)
		return false;

	buf = malloc_cache_aligned(read_blks * part.blksz);
	if (!buf)
		return false;

	if (blk_dread(desc, start_block, read_blks, buf) != read_blks) {
		free(buf);
		return false;
	}

	memcpy(base, buf + block_offset, sizeof(base));
	free(buf);
	memcpy(mac, base, sizeof(base));
	value = ((u64)mac[0] << 40) | ((u64)mac[1] << 32) |
		((u64)mac[2] << 24) | ((u64)mac[3] << 16) |
		((u64)mac[4] << 8) | mac[5];
	if ((mac_offset < 0 && value < -mac_offset) ||
	    (mac_offset > 0 && value + mac_offset > GENMASK_ULL(47, 0)))
		return false;
	value += mac_offset;

	mac[0] = value >> 40;
	mac[1] = value >> 32;
	mac[2] = value >> 24;
	mac[3] = value >> 16;
	mac[4] = value >> 8;
	mac[5] = value;

	return is_valid_ethaddr(mac);
}

static int ipq_eth_read_hwaddr(struct udevice *dev)
{
	struct ipq_eth_dev *priv = dev_get_priv(dev);
	struct eth_pdata *pdata = dev_get_plat(dev);
	uchar enet_addr[6] = { 0 };

	if (is_valid_ethaddr(pdata->enetaddr)) {
		return 0;
	} else if (ipq_eth_read_dt_nvmem_mac(priv, enet_addr) ||
	    eth_env_get_enetaddr("ethaddr", enet_addr) ||
	    ipq_eth_read_appsblenv_mac(enet_addr)) {
		memcpy(&pdata->enetaddr[0], &enet_addr[0], 6);
	} else {
		memcpy(&pdata->enetaddr[0], &ipq_def_enetaddr[0], 6);
	}

	return 0;
}

static int ipq_eth_bind(struct udevice *dev)
{
	return 0;
}

static int ipq_eth_phy_hw_reset(struct port_info *port)
{
	struct gpio_desc *gpio = &port->rst_gpio;
	u32 assert = port->reset_assert_us;
	u32 deassert = port->reset_deassert_us;
	int ret;

	if (!gpio->dev)
		return 0;

	if (!assert)
		assert = 20000;
	if (!deassert)
		deassert = 1000;

	ret = dm_gpio_set_value(gpio, 1);
	if (ret)
		return ret;

	udelay(assert);

	ret = dm_gpio_set_value(gpio, 0);
	if (ret)
		return ret;

	udelay(deassert);

	return 0;
}

#ifdef CONFIG_PHY_QCA_8X8X
static void ipq_eth_8x8x_write(struct phy_device *phydev, u32 reg, u32 val)
{
	u16 r1, r2, page, switch_phy_id;
	u16 lo = val & 0xffff;
	u16 hi = (u16)(val >> 16);

	r1 = reg & 0x1c;
	reg >>= 5;
	r2 = reg & 0x7;
	reg >>= 3;
	page = reg & 0xffff;
	reg >>= 16;
	switch_phy_id = reg & 0xff;

	phydev->addr = (0x18 | (switch_phy_id >> 5));
	phy_write(phydev, MDIO_DEVAD_NONE, switch_phy_id & 0x1f, page);
	udelay(100);

	phydev->addr = (0x10 | r2);
	phy_write(phydev, MDIO_DEVAD_NONE, r1, lo);
	phy_write(phydev, MDIO_DEVAD_NONE, r1 + 2, hi);
}

static void ipq_eth_8x8x_pre_init(struct mii_dev *bus)
{
	struct phy_device temp_phydev;
	u32 phy_data;

	/*
	 * Buid temporary data structures that the chip reading code needs to
	 * read the ID
	 */
	temp_phydev.bus = bus;

	temp_phydev.addr = 0x18;
	phy_write(&temp_phydev, MDIO_DEVAD_NONE, 0xc, 0x90f0);
	temp_phydev.addr = 0x10;
	phy_data = phy_read(&temp_phydev, MDIO_DEVAD_NONE, 0x18);
	phy_data |= (phy_read(&temp_phydev, MDIO_DEVAD_NONE, 0x1a) << 16);
	pr_debug("%s %d phy_data: 0x%x\n", __func__, __LINE__, phy_data);

	if (phy_data == 0x20c41) {
		pr_debug("%s %d addr fixup and clk init already done!!!\n",
			 __func__, __LINE__);
		return;
	}

	/* addr fixup */
	ipq_eth_8x8x_write(&temp_phydev, 0xc90f018, 0x320c41);
	ipq_eth_8x8x_write(&temp_phydev, 0xc90f014, 0x1cc5);

	/* clk init */
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001a8, 0x80000001);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001ac, 0x80000001);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001a8, 0x5);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001a8, 0x1);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001ac, 0x5);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001ac, 0x1);
	ipq_eth_8x8x_write(&temp_phydev, 0xc800058, 0x80000000);
	ipq_eth_8x8x_write(&temp_phydev, 0xc800078, 0x80000000);
	ipq_eth_8x8x_write(&temp_phydev, 0xc800098, 0x80000000);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8000b8, 0x80000000);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8000d8, 0x80000000);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8000f8, 0x80000000);
	ipq_eth_8x8x_write(&temp_phydev, 0xc800118, 0x80000000);
	ipq_eth_8x8x_write(&temp_phydev, 0xc800138, 0x80000000);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001b0, 0x80000001);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001b4, 0x80000001);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001b8, 0x80000001);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001bc, 0x80000001);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001b0, 0x5);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001b0, 0x1);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001b4, 0x5);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001b4, 0x1);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001b8, 0x5);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001b8, 0x1);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001bc, 0x5);
	ipq_eth_8x8x_write(&temp_phydev, 0xc8001bc, 0x1);
	ipq_eth_8x8x_write(&temp_phydev, 0xc800304, 0x0);
	ipq_eth_8x8x_write(&temp_phydev, 0xc90f018, 0x20c41);
}
#endif

#ifdef CONFIG_MDIO_QCOM_I2C
#define SFP_EEPROM_I2C_ADDR			0x50

static int ipq_eth_sfp_detect(struct udevice *i2c_bus, struct port_info *port)
{
	struct udevice *dev;
	int ret = dm_i2c_probe(i2c_bus, SFP_EEPROM_I2C_ADDR, 0, &dev);

	if (!ret) {
		/*
		 * SFP detected, update port info with respect to SFP
		 */
		port->phy_id = SFP10G_PHY_TYPE;
		port->phyaddr = 0xFF;
		port->rst_gpio.dev = NULL;
	}

	return ret;
}
#endif

static int ipq_eth_get_mdio_device(ofnode phy_node, struct udevice **busdev)
{
	ofnode node;
	int ret;

	for (node = ofnode_get_parent(phy_node); ofnode_valid(node);
	     node = ofnode_get_parent(node)) {
		ret = uclass_get_device_by_ofnode(UCLASS_MDIO, node, busdev);
		if (!ret)
			return 0;
	}

	return -ENODEV;
}

static int ipq_eth_usxgmii_clk_rate(int mac_speed)
{
	switch (mac_speed) {
	case 0:
		return CLK_1_25_MHZ;
	case 1:
		return CLK_12_5_MHZ;
	case 2:
		return CLK_125_MHZ;
	case 3:
		return CLK_312_5_MHZ;
	case 4:
		return CLK_78_125_MHZ;
	case 5:
		return CLK_156_25_MHZ;
	default:
		return -1;
	}
}

static ulong ipq_eth_uniphy_parent_rate(struct port_info *port)
{
	if (port->uniphy_mode == PORT_WRAPPER_PSGMII ||
	    port->uniphy_mode == PORT_WRAPPER_QSGMII ||
	    port->uniphy_mode == PORT_WRAPPER_SGMII0_RGMII4)
		return CLK_125_MHZ;

	return CLK_312_5_MHZ;
}

static ulong ipq_eth_port_clk_data(struct port_info *port)
{
	ulong data = port->uniphy_id & NSS_PORT_CLK_DATA_UNIPHY_MASK;

	if (ipq_eth_uniphy_parent_rate(port) == CLK_312_5_MHZ)
		data |= NSS_PORT_CLK_DATA_PARENT_312_5;

	return data;
}

static void ipq_eth_apply_interface_override(struct port_info *port, int *rate)
{
	switch (port->interface) {
	case PHY_INTERFACE_MODE_QSGMII:
		port->uniphy_mode = PORT_WRAPPER_QSGMII;
		port->gmac_type = GMAC;
		*rate = CLK_125_MHZ;
		break;
	case PHY_INTERFACE_MODE_USXGMII:
		port->uniphy_mode = PORT_WRAPPER_USXGMII;
		port->gmac_type = XGMAC;
		*rate = ipq_eth_usxgmii_clk_rate(port->mac_speed);
		break;
	default:
		break;
	}
}

static bool ipq_eth_port_uses_dt_interface(struct port_info *port)
{
	/* QCA8081 switches between SGMII and 2500BASE-X after AN. */
	if (port->phy_id == QCA8081_PHY_TYPE)
		return false;

	return port->interface == PHY_INTERFACE_MODE_QSGMII ||
	       port->interface == PHY_INTERFACE_MODE_USXGMII;
}

static void ipq_eth_configure_uniphy_50m(int count, phys_addr_t uniphy_base,
					size_t size)
{
	int i = 0;
	phys_addr_t base;

	while (count) {
		base = (uniphy_base + (i * size));
		base += CLKOUT_50M_CTRL_OPTION;
		writel(readl(base) |  BIT(0), base);
		count >>= 1;
		++i;
	}
}

static int ipq_eth_reset_node(ofnode node)
{
	struct reset_ctl rst;
	int ret;

	if (!ofnode_valid(node) || !ofnode_get_property(node, "resets", NULL))
		return 0;

	ret = reset_get_by_index_nodev(node, 0, &rst);
	if (ret)
		return ret;

	ret = reset_assert(&rst);
	if (ret)
		goto out_free;

	mdelay(10);

	ret = reset_deassert(&rst);
	if (ret)
		goto out_free;

	mdelay(10);

out_free:
	reset_free(&rst);
	return ret;
}

static int ipq_eth_enable_node_clk(ofnode node, const char *name)
{
	struct clk clk;
	int ret;

	ret = clk_get_by_name_nodev(node, name, &clk);
	if (ret) {
		if (ret == -ENOENT || ret == -ENODATA || ret == -EINVAL)
			return 0;
		return ret;
	}

	ret = clk_enable(&clk);
	if (ret)
		clk_release_all(&clk, 1);

	return ret;
}

static int ipq_eth_get_optional_node_clk(ofnode node, const char *name,
					 struct clk *clk)
{
	int ret;

	if (!ofnode_valid(node))
		return -ENOENT;

	ret = clk_get_by_name_nodev(node, name, clk);
	if (ret == -ENOENT || ret == -ENODATA || ret == -EINVAL)
		return -ENOENT;

	return ret;
}

static int ipq_eth_enable_optional_node_clk(ofnode node, const char *name)
{
	struct clk clk;
	int ret;

	ret = ipq_eth_get_optional_node_clk(node, name, &clk);
	if (ret)
		return ret == -ENOENT ? 0 : ret;

	ret = clk_enable(&clk);
	if (ret)
		clk_release_all(&clk, 1);

	return ret;
}

static int ipq_eth_prepare_pcs_node(struct port_info *port, bool reset_parent)
{
	ofnode pcs_node, parent;
	int ret;

	if (!port || !ofnode_valid(port->pcs_node))
		return 0;

	pcs_node = port->pcs_node;
	parent = ofnode_get_parent(pcs_node);

	ret = ipq_eth_enable_optional_node_clk(parent, "sys");
	if (ret)
		return ret;

	ret = ipq_eth_enable_optional_node_clk(parent, "ahb");
	if (ret)
		return ret;

	if (reset_parent) {
		ret = ipq_eth_reset_node(parent);
		if (ret)
			return ret;
	}

	ret = ipq_eth_enable_optional_node_clk(pcs_node, "rx");
	if (ret)
		return ret;

	return ipq_eth_enable_optional_node_clk(pcs_node, "tx");
}

static int ipq_eth_prepare_pcs_nodes(struct ipq_eth_dev *priv)
{
	bool parent_prepared[CONFIG_ETH_MAX_UNIPHY] = {};
	struct port_info *port;
	bool reset_parent;
	int i, ret;

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		port = priv->port[i];
		if (!port || !ofnode_valid(port->pcs_node))
			continue;

		reset_parent = true;
		if (port->uniphy_id < CONFIG_ETH_MAX_UNIPHY) {
			reset_parent = !parent_prepared[port->uniphy_id];
			parent_prepared[port->uniphy_id] = true;
		}

		ret = ipq_eth_prepare_pcs_node(port, reset_parent);
		if (ret)
			return ret;
	}

	return 0;
}

static int ipq_eth_reset_edma_node(struct udevice *dev)
{
	ofnode edma_node = dev_read_subnode(dev, "ethernet-dma");

	if (!ofnode_valid(edma_node))
		return -ENOENT;

	return ipq_eth_reset_node(edma_node);
}

static int ipq_eth_prepare_edma_node(struct udevice *dev)
{
	ofnode edma_node = dev_read_subnode(dev, "ethernet-dma");
	int ret;

	if (!ofnode_valid(edma_node))
		return 0;

	ret = ipq_eth_enable_node_clk(edma_node, "sys");
	if (ret)
		return ret;

	ret = ipq_eth_enable_node_clk(edma_node, "apb");
	if (ret)
		return ret;

	return ipq_eth_reset_edma_node(dev);
}

static int ipq_eth_probe(struct udevice *dev)
{
	struct ipq_eth_dev *priv = dev_get_priv(dev);
	struct udevice *busdev;
	struct port_info *port;
	struct clk clk;
	struct reset_ctl_bulk resets;
	int clk_itr, clk_cnt, ret, i, configured = 0;
	const char **clk_names = NULL;
#if defined(CONFIG_PHY_QCA_8X8X) || defined(CONFIG_PHY_QCE_1204)
	int phy_no = 0;
#endif

	mem_init();

	/*
	 * configure CMN clock for ethernet
	 */
	ipq_config_cmn_clock();

	ret = reset_get_bulk(dev, &resets);
	if (ret && ret != -ENOENT) {
		dev_err(dev, "Can't get reset: %d\n", ret);
		return -ENODEV;
	}

	ret = reset_assert_bulk(&resets);
	if (ret)
		return ret;

	mdelay(10);

	ret = reset_deassert_bulk(&resets);
	if (ret)
		return ret;

	ret = ipq_eth_prepare_edma_node(dev);
	if (ret && ret != -ENOENT) {
		dev_err(dev, "Failed to prepare EDMA child node: %d\n", ret);
		return ret;
	}

	clk_cnt = dev_read_string_list(dev, "clock-names", &clk_names);
	if (clk_cnt <= 0) {
		dev_err(dev, "Failed to get clock names (ret=%d)\n", ret);
		goto fail;
	}

	for (clk_itr = 0 ; clk_itr < clk_cnt ; clk_itr++) {
		if (!strncmp(clk_names[clk_itr], "gcc_uniphy", 10)) {
			int uniphy_id = clk_names[clk_itr][10] - 48; //Extract uniphy_id

			if (ipq_uniphy && ipq_uniphy[uniphy_id].status == SKU_ENABLED) {
				ret = clk_get_by_name(dev, clk_names[clk_itr], &clk);
				if (ret) {
					if (ret != -ENOENT) {
						dev_err(dev, "Failed to get clock by name (ret=%d)\n", ret);
						goto fail;
					}
					continue;
				}
				clk_enable(&clk);
			} else {
				ret = clk_get_by_name(dev, clk_names[clk_itr], &clk);
				if (ret) {
					if (ret != -ENOENT) {
						dev_err(dev, "Failed to get clock by name (ret=%d)\n", ret);
						goto fail;
					}
					continue;
				}
				clk_enable(&clk);
			}
			continue;
		}

		ret = clk_get_by_name(dev, clk_names[clk_itr], &clk);
		if (ret) {
			if (ret != -ENOENT) {
				dev_err(dev, "Failed to get clock by name (ret=%d)\n", ret);
				goto fail;
			}
			continue;
		}
		clk_enable(&clk);
	}

	if (priv->uniphy_50mhz)
		ipq_eth_configure_uniphy_50m(priv->uniphy_50mhz,
						priv->uniphy_base,
						priv->uniphy_size);

	ret = ipq_eth_prepare_pcs_nodes(priv);
	if (ret && ret != -ENOENT) {
		dev_err(dev, "Failed to prepare PCS nodes: %d\n", ret);
		goto fail;
	}

	uniphy_set_base_addr(priv->uniphy_base);
	current_csr_version = uniphy_get_csr_version();
	current_reset_version = uniphy_get_reset_version();

	ret = ipq_edma_hw_init(dev, priv);
	if (ret) {
		dev_err(dev, "EDMA initialization failed: %d\n", ret);
		goto fail;
	}

	for (i = 0; i < CONFIG_ETH_MAX_MAC; ++i) {
		port = priv->port[i];
		if (!port)
			continue;

		if (priv->emulation) {
			port->isconfigured = true;
			++configured;
			continue;
		}

		port->dev = dev;

		port->uniphy_base = priv->uniphy_base +
					(port->uniphy_id * priv->uniphy_size);

#ifdef CONFIG_MDIO_QCOM_I2C
		if (port->i2c_bus) {
			ret = uclass_get_device_by_phandle_id(UCLASS_I2C, port->i2c_bus, &busdev);
			if (ret) {
				printf("%s: failed to get i2c bus, err: %d\n", __func__, ret);
				continue;
			}

			port->bus = (struct mii_dev *)(uintptr_t)
					qcom_mdio_i2c_alloc(busdev,
							    port->phyaddr);
			if (!port->bus) {
				if (ipq_eth_sfp_detect(busdev, port))
					continue;
			}
		}
#endif /* CONFIG_MDIO_QCOM_I2C */

		if (port->phyaddr != 0xFF) {
			if (!port->bus) {
				ret = ipq_eth_get_mdio_device(port->node, &busdev);
				if (ret) {
					printf("PHY%u addr 0x%x (%s): MDIO bus unavailable: %d\n",
					       port->id, port->phyaddr,
					       ipq_eth_phy_type_name(port->phy_id), ret);
					continue;
				}

				port->bus = miiphy_get_dev_by_name(busdev->name);
			}

			if (!port->bus) {
				printf("PHY%u addr 0x%x (%s): MDIO bus %s is not registered\n",
				       port->id, port->phyaddr,
				       ipq_eth_phy_type_name(port->phy_id), busdev->name);
				continue;
			}
		}
		/*
		 * Create a dummy bus for the SFP module that is not connected via I2C
		 * on legacy SoCs, to prevent it from being skipped during validation checks.
		 */
		if (!port->bus) {
			port->bus = mdio_alloc();
			if (!port->bus) {
				pr_err("failed to allocate MDIO bus\n");
				return -ENOMEM;
			}

			port->bus->read = NULL;
			port->bus->write = NULL;
			port->bus->priv = NULL;
			snprintf(port->bus->name, sizeof(port->bus->name),
				"%s", "SFP-DUMMY");
		}

		if (port->rst_gpio.dev) {
			ret = ipq_eth_phy_hw_reset(port);
			if (ret) {
				printf("PHY%u addr 0x%x (%s): hardware reset failed: %d\n",
				       port->id, port->phyaddr,
				       ipq_eth_phy_type_name(port->phy_id), ret);
				continue;
			}
		}

#ifdef CONFIG_PHY_QCA_8337
		if (port->phy_id == QCA8337_SWITCH_TYPE) {
		/*
		 * The below listed configure need to perform
		 * before init switch
		 * set UNIPHY mode as SGMII
		 * Configure GMAC
		 * Disable txmac
		 * Disable GMAC
		 * configure uniphy force mode
		 */
			port->uniphy_mode = PORT_WRAPPER_SGMII0_RGMII4;
			port->cur_uniphy_mode = PORT_WRAPPER_SGMII0_RGMII4;
			port->gmac_type = port->cur_gmac_type = GMAC;
			ppe_uniphy_mode_set(port);
			ppe_port_mux_set(priv->ppe.base, port);
			ppe_port_bridge_txmac_set(priv->ppe.base, port->id, false);
			if (port->isforce_speed)
				ppe_uniphy_set_forcemode(port);
		}
#endif

#ifdef CONFIG_PHY_QCA_8X8X
		if (port->phy_id == QCA8x8x_PHY_TYPE || port->phy_id == QCA8x8x_SWITCH_TYPE)
			ipq_eth_8x8x_pre_init(port->bus);
#endif
		if (port->phy_id == SFP10G_PHY_TYPE || port->phy_id == SFP2_5G_PHY_TYPE ||
		    port->phy_id == SFP1G_PHY_TYPE) {
			port->phydev = phy_device_create(port->bus, port->phyaddr,
							 PHY_FIXED_ID, true);
			if (IS_ERR_OR_NULL(port->phydev))
				continue;

			port->phydev->dev = dev;
			port->phydev->interface = port->interface;
		} else {
			port->phydev = phy_connect(port->bus, port->phyaddr, dev, port->interface);
		}

		if (IS_ERR_OR_NULL(port->phydev)) {
			printf("PHY%u addr 0x%x (%s): connection failed\n",
			       port->id, port->phyaddr,
			       ipq_eth_phy_type_name(port->phy_id));
			continue;
		}

		if (ofnode_valid(port->node))
			port->phydev->node = port->node;

#if defined(CONFIG_PHY_QCA_8X8X) || defined(CONFIG_PHY_QCE_1204)
		/*
		 * configure UQXGMII for pure PHY mode since MHT PHY requires
		 * uniphy pre-init before configuring uniphy mode, which has
		 * to be configured by default to UQXGMII mode regardless of
		 * speed link up.
		 */
		if (port->phy_id == QCA8x8x_PHY_TYPE  ||
			port->phy_id == QCE1204_PHY_TYPE) {
			port->uniphy_mode = PORT_WRAPPER_UQXGMII;
			port->cur_uniphy_mode = PORT_WRAPPER_UQXGMII;
			port->gmac_type = XGMAC;
			port->cur_gmac_type = XGMAC;

			if (phy_no == 0)
				ppe_uniphy_mode_set(port);

			ppe_port_mux_set(priv->ppe.base, port);
			++phy_no;
		} else if (port->phy_id == QCA8x8x_SWITCH_TYPE) {
			port->uniphy_mode = PORT_WRAPPER_SGMII_PLUS;
			port->cur_uniphy_mode = PORT_WRAPPER_SGMII_PLUS;
			ppe_uniphy_mode_set(port);
		} else if (port->phy_id == QCE2204_SWITCH_TYPE) {
			port->uniphy_mode = PORT_WRAPPER_10GBASE_R;
			port->cur_uniphy_mode = PORT_WRAPPER_10GBASE_R;
			ppe_uniphy_mode_set(port);
		}
#endif

#ifdef CONFIG_PHY_AQUANTIA
		if (port->phy_id == AQ_PHY_TYPE) {
			if (!ipq_aquantia_load_fw(port->phydev)) {
				port->fw_loaded = true;
				mdelay(100);
			} else
				port->fw_loaded = false;
		}
#endif

#ifdef CONFIG_PHY_QCA_81XX
		if (port->phy_id == QCA81xx_PHY_TYPE) {
			port->uniphy_mode = PORT_WRAPPER_USXGMII;
			port->cur_uniphy_mode = PORT_WRAPPER_USXGMII;
			port->gmac_type = XGMAC;
			port->cur_gmac_type = XGMAC;
			ppe_uniphy_mode_set(port);
			ppe_port_mux_set(priv->ppe.base, port);
		}
#endif

		if (ipq_edma_debug_trace() &&
		    ipq_eth_port_uses_dt_interface(port))
			printf("port%u: label=%s phy=%s interface=%s pcs=uniphy%u/ch%u mux=%u\n",
			       port->id, port->label ?: "<none>",
			       ipq_eth_phy_type_name(port->phy_id),
			       ipq_eth_interface_name(port->interface),
			       port->uniphy_id, port->pcs_channel,
			       port->uniphy_type);

		ret = phy_config(port->phydev);
		if (ret < 0) {
			printf("PHY%u addr 0x%x (%s): configuration failed: %d\n",
			       port->id, port->phyaddr,
			       ipq_eth_phy_type_name(port->phy_id), ret);
			continue;
		}

		port->isconfigured = true;

		++configured;
	}
fail:
	free(clk_names);

	return !configured;
}

static int ipq_eth_remove(struct udevice *dev)
{
	struct ipq_eth_dev *priv = dev_get_priv(dev);
	int i;

	for (i = 0; i < CONFIG_ETH_MAX_MAC; ++i) {
		if (priv->port[i]) {
			free(priv->port[i]);
			priv->port[i] = NULL;
		}
	}

	return 0;
}

static u32 ipq_eth_read_u32_2(ofnode first, ofnode second, const char *prop,
			      u32 def)
{
	u32 value;

	if (ofnode_valid(first) && !ofnode_read_u32(first, prop, &value))
		return value;
	if (ofnode_valid(second) && !ofnode_read_u32(second, prop, &value))
		return value;

	return def;
}

static u32 ipq_eth_read_phy_id(ofnode first, ofnode second)
{
	u32 value;

	value = ipq_eth_read_u32_2(first, second, "phy_type", (u32)-1);
	if (value != (u32)-1)
		return value;

	return ipq_eth_read_u32_2(first, second, "phy_id", (u32)-1);
}

static bool ipq_eth_read_bool_2(ofnode first, ofnode second, const char *prop)
{
	return (ofnode_valid(first) && ofnode_read_bool(first, prop)) ||
	       (ofnode_valid(second) && ofnode_read_bool(second, prop));
}

static phy_interface_t ipq_eth_read_interface(ofnode port_node,
					      ofnode phy_node, u32 phy_id,
					      bool from_dt_port)
{
	phy_interface_t interface = PHY_INTERFACE_MODE_NA;
	phy_interface_t package_interface;
	const char *mode;

	if (ofnode_valid(port_node))
		interface = ofnode_read_phy_mode(port_node);
	if (interface == PHY_INTERFACE_MODE_NA) {
		mode = ipq_eth_node_phy_mode(port_node);
		interface = ipq_eth_parse_interface_string(mode);
	}
	if (interface == PHY_INTERFACE_MODE_NA)
		interface = ofnode_read_phy_mode(phy_node);
	if (interface == PHY_INTERFACE_MODE_NA) {
		mode = ipq_eth_node_phy_mode(phy_node);
		interface = ipq_eth_parse_interface_string(mode);
	}

	if (from_dt_port) {
		/*
		 * DSA-style ethernet-ports nodes are authoritative. Use the QCA8075
		 * package mode only as a fallback for older DTs which describe the
		 * package but omit the per-port link mode.
		 */
		package_interface = ipq_eth_qca8075_package_interface(phy_node);
		if (interface == PHY_INTERFACE_MODE_NA &&
		    package_interface != PHY_INTERFACE_MODE_NA)
			interface = package_interface;
	}

	if (interface == PHY_INTERFACE_MODE_NA)
		interface = ipq_eth_default_interface(phy_id);

	return interface;
}

static int ipq_eth_add_dt_port(struct ipq_eth_dev *priv, ofnode port_node,
			       ofnode phy_node, int *port_count)
{
	struct port_info *port;
	ofnode pcs_node;
	u32 pcs_uniphy_id, pcs_channel, uniphy_type;
	u32 port_id, phy_id, uniphy_id;
	int ret;

	if (*port_count >= CONFIG_ETH_MAX_MAC)
		return -ENOSPC;
	if (!ofnode_valid(phy_node))
		return -EINVAL;

	port_id = ipq_eth_read_u32_2(port_node, ofnode_null(), "reg", -1);
	if (port_id == (u32)-1)
		port_id = ofnode_read_u32_default(phy_node, "id", -1);

	pcs_node = ipq_eth_read_pcs_defaults(port_node, &pcs_uniphy_id,
					     &pcs_channel);

	uniphy_id = pcs_uniphy_id;
	if (uniphy_id == (u32)-1)
		uniphy_id = ipq_eth_read_u32_2(port_node, phy_node,
					       "uniphy_id", -1);
	if (uniphy_id == (u32)-1)
		uniphy_id = ipq_eth_default_uniphy_id(port_id);
	if (uniphy_id == (u32)-1)
		return 0;
	if (uniphy_id >= CONFIG_ETH_MAX_UNIPHY) {
		printf("Invalid UNIPHY%u in Ethernet DT\n", uniphy_id);
		return -EINVAL;
	}

	if (ipq_uniphy) {
		if (readl(ipq_uniphy[uniphy_id].reg) &
		    (1 << ipq_uniphy[uniphy_id].bit)) {
			printf("UNIPHY%d is Disabled\n", uniphy_id);
			return 0;
		}
		ipq_uniphy[uniphy_id].status = SKU_ENABLED;
	}

	port = malloc_cache_aligned(sizeof(struct port_info));
	if (!port)
		return -ENOMEM;

	memset(port, 0, sizeof(struct port_info));

	port->uniphy_mode = PORT_WRAPPER_MAX;
	port->cur_uniphy_mode = PORT_WRAPPER_MAX;
	port->cur_gmac_type = 0xff;
	port->uniphy_id = uniphy_id;
	port->port_node = port_node;
	port->pcs_node = pcs_node;
	port->node = phy_node;
	port->label = ipq_eth_port_label(port_node);
	port->dt_phy_mode = ipq_eth_node_phy_mode(port_node);
	if (!port->dt_phy_mode)
		port->dt_phy_mode = ipq_eth_node_phy_mode(phy_node);
	port->package_mode = ipq_eth_qca8075_package_mode(phy_node);
	port->from_dt_port = ofnode_valid(port_node);
	port->phyaddr = ofnode_read_u32_default(phy_node, "phy_addr",
					ofnode_read_u32_default(phy_node,
								"reg", -1));
	phy_id = ipq_eth_default_phy_id(port_node, phy_node, port_id);
	if (phy_id == (u32)-1)
		phy_id = ipq_eth_read_phy_id(phy_node, port_node);
	port->phy_id = phy_id;
	port->id = port_id;

	port->pcs_channel = pcs_channel;
	/*
	 * The PCS selector is a one-bit field in PORT_MUX_CTRL.  Keep the
	 * sentinel in a u32 until the fallback is applied; assigning it directly
	 * to the u8 member turns -1 into 0xff and defeats the port-5 PCS1
	 * default used by the upstream DTS.
	 */
	uniphy_type = ipq_eth_read_u32_2(port_node, phy_node,
					 "uniphy_type", (u32)-1);
	if (uniphy_type == (u32)-1)
		uniphy_type = ipq_eth_default_uniphy_type(port_id);
	if (uniphy_type > 1) {
		printf("Invalid PCS selector %u for Ethernet port %u\n",
		       uniphy_type, port_id);
		free(port);
		return -EINVAL;
	}
	port->uniphy_type = (u8)uniphy_type;
	port->max_speed = ipq_eth_read_u32_2(port_node, phy_node,
					     "max_speed", -1);
	if (port->max_speed == (u32)-1)
		port->max_speed = ipq_eth_read_u32_2(port_node, phy_node,
						     "max-speed", -1);
	port->isforce_speed = ipq_eth_read_bool_2(port_node, phy_node,
						  "force-speed");
	port->xgmac = ipq_eth_read_bool_2(port_node, phy_node, "xgmac");
	port->phy_25mhz = ipq_eth_read_bool_2(port_node, phy_node, "25M") ||
			  ipq_eth_read_bool_2(port_node, phy_node,
					      "qcom,uniphy-clkout-25mhz");
	port->i2c_bus = ipq_eth_read_u32_2(port_node, phy_node, "i2c-bus", 0);
	port->interface = ipq_eth_read_interface(port_node, phy_node,
						 port->phy_id,
						 port->from_dt_port);
	if (!port->xgmac && port->interface == PHY_INTERFACE_MODE_USXGMII)
		port->xgmac = true;

	ret = gpio_request_by_name_nodev(phy_node, "reset-gpios",
					 0, &port->rst_gpio,
					 GPIOD_IS_OUT);
	if (!ret) {
		port->reset_assert_us =
			ofnode_read_u32_default(phy_node,
						"reset-assert-us",
						20000);
		port->reset_deassert_us =
			ofnode_read_u32_default(phy_node,
						"reset-deassert-us",
						1000);
	} else {
		gpio_request_by_name_nodev(phy_node, "phy-reset-gpio", 0,
					   &port->rst_gpio,
					   GPIOD_IS_OUT |
					   GPIOD_ACTIVE_LOW);
		port->reset_assert_us = 20000;
		port->reset_deassert_us = 1000;
	}

	priv->port[(*port_count)++] = port;

	if (ipq_edma_debug_trace())
		printf("DT port slot=%d id=%u label=%s phyaddr=0x%x phy_type=%s(%u) dt_phy_mode=%s package=%s interface=%s pcs=uniphy%u/ch%u mux=%u\n",
		       *port_count - 1, port->id, port->label ?: "<none>",
		       port->phyaddr, ipq_eth_phy_type_name(port->phy_id),
		       port->phy_id, port->dt_phy_mode ?: "<none>",
		       port->package_mode ?: "<none>",
		       ipq_eth_interface_name(port->interface),
		       port->uniphy_id, port->pcs_channel,
		       port->uniphy_type);

	return 0;
}

static int ipq_eth_parse_ethernet_ports(struct udevice *dev,
					struct ipq_eth_dev *priv,
					int *port_count)
{
	struct ofnode_phandle_args phandle_args;
	ofnode ports_node, port_node;
	int parsed = 0, ret;

	ports_node = dev_read_subnode(dev, "ethernet-ports");
	if (!ofnode_valid(ports_node))
		return 0;

	ofnode_for_each_subnode(port_node, ports_node) {
		if (!ofnode_is_enabled(port_node))
			continue;
		if (!ipq_eth_is_port_node(port_node))
			continue;

		ret = ofnode_parse_phandle_with_args(port_node, "phy-handle",
						     NULL, 0, 0,
						     &phandle_args);
		if (ret) {
			phandle_args.node = ofnode_parse_phandle(port_node,
								 "phy-handle", 0);
			if (!ofnode_valid(phandle_args.node)) {
				printf("%s: missing phy-handle\n",
				       ofnode_get_name(port_node));
				continue;
			}
		}

		ret = ipq_eth_add_dt_port(priv, port_node,
					  phandle_args.node, port_count);
		if (ret)
			return ret;
		parsed++;
	}

	return parsed;
}

static int ipq_eth_parse_legacy_phy_handles(struct udevice *dev,
					    struct ipq_eth_dev *priv,
					    int *port_count)
{
	struct ofnode_phandle_args phandle_args;
	char phy_handle[13];
	int i, ret;

	for (i = 0; i < CONFIG_ETH_MAX_MAC; ++i) {
		snprintf(phy_handle, sizeof(phy_handle), "phy-handle%d", i);

		ret = dev_read_phandle_with_args(dev, phy_handle, NULL, 0, 0,
						 &phandle_args);
		if (ret)
			continue;

		ret = ipq_eth_add_dt_port(priv, ofnode_null(),
					  phandle_args.node, port_count);
		if (ret)
			return ret;
	}

	return 0;
}

static phys_addr_t ipq_eth_read_ppe_base(struct udevice *dev)
{
	phys_addr_t base;

	base = (phys_addr_t)dev_read_addr_name(dev, "ppe_base");
	if (base == FDT_ADDR_T_NONE)
		base = (phys_addr_t)dev_read_addr_name(dev, "ppe");
	if (base == FDT_ADDR_T_NONE)
		base = (phys_addr_t)dev_read_addr(dev);

	return base;
}

static phys_addr_t ipq_eth_read_edma_base(struct udevice *dev,
					  phys_addr_t ppe_base)
{
	static const char * const names[] = {
		"edma_hw",
		"edma",
	};
	ofnode node;
	phys_addr_t base;
	fdt_size_t size;
	int i;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		base = (phys_addr_t)dev_read_addr_name(dev, names[i]);
		if (base != FDT_ADDR_T_NONE)
			return base;
	}

	node = dev_read_subnode(dev, "ethernet-dma");
	if (ofnode_valid(node)) {
		base = ofnode_get_addr_size_index(node, 0, &size);
		if (base != FDT_ADDR_T_NONE)
			return base;
	}

	/*
	 * Upstream Linux DTS keeps EDMA as a child of the PPE block. Some
	 * snapshots do not carry a child reg yet, while QSDK/U-Boot use the
	 * fixed IPQ9574 EDMA window at PPE + 0xb00000.
	 */
	if (ppe_base != FDT_ADDR_T_NONE)
		return ppe_base + 0x00b00000;

	return FDT_ADDR_T_NONE;
}

static phys_addr_t ipq_eth_read_uniphy_base(struct udevice *dev,
					    struct ipq_eth_dev *priv)
{
	static const char * const names[] = {
		"uniphy_base",
		"uniphy",
	};
	ofnode port_node, pcs_node, parent;
	phys_addr_t base;
	fdt_size_t size;
	int i;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		base = dev_read_addr_size_name(dev, names[i], &size);
		if (base != FDT_ADDR_T_NONE) {
			if (!size || size == FDT_SIZE_T_NONE)
				size = 0x10000;
			priv->uniphy_size = size;
			return base;
		}
	}

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		if (!priv->port[i])
			continue;

		port_node = priv->port[i]->port_node;
		if (!ofnode_valid(port_node))
			continue;

		pcs_node = ofnode_parse_phandle(port_node, "pcs-handle", 0);
		if (!ofnode_valid(pcs_node))
			continue;

		parent = ofnode_get_parent(pcs_node);
		base = ofnode_get_addr_size_index(parent, 0, &size);
		if (base == FDT_ADDR_T_NONE)
			continue;

		if (!size || size == FDT_SIZE_T_NONE)
			size = 0x10000;
		priv->uniphy_size = size;
		return base - (priv->port[i]->uniphy_id * size);
	}

	return FDT_ADDR_T_NONE;
}

static const struct eth_ops ipq_eth_ops = {
	.start			= ipq_eth_start,
	.send			= ipq_eth_send,
	.recv			= ipq_eth_recv,
	.free_pkt		= ipq_eth_free_pkt,
	.stop			= ipq_eth_stop,
	.write_hwaddr		= ipq_eth_write_hwaddr,
	.read_rom_hwaddr        = ipq_eth_read_hwaddr,
};

static int ipq_eth_ofdata_to_platdata(struct udevice *dev)
{
	struct ipq_eth_dev *priv = dev_get_priv(dev);
	struct ppe_info *ppe;
	int port_count = 0, ret;

	memset(priv, 0, sizeof(struct ipq_eth_dev));

	priv->dev = dev;
	ppe = &priv->ppe;

	priv->emulation = dev_read_bool(dev, "qti,emulation");

	ppe->base = ipq_eth_read_ppe_base(dev);
	if (ppe->base == FDT_ADDR_T_NONE) {
		dev_err(dev, "ppe_base bus address not found\n");
		return -EINVAL;
	}

	priv->hw.iobase = ipq_eth_read_edma_base(dev, ppe->base);
	if (priv->hw.iobase == FDT_ADDR_T_NONE) {
		dev_err(dev, "edma_hw bus address not found\n");
		return -EINVAL;
	}

	ppe->tdm_offset = dev_read_u32_default(dev, "tdm_offset", -1);
	if (ppe->tdm_offset == -1) {
		dev_err(dev, "tdm_offset not found\n");
		return -EINVAL;
	}

	priv->uniphy_50mhz = dev_read_u32_default(dev, "50mhz", 0);

	ppe->tdm_mode = dev_read_u32_default(dev, "tdm_mode", 0);
	ppe->no_reg = dev_read_u32_default(dev, "no_tdm_reg", 0);
	ppe->tm = dev_read_bool(dev, "tdm_tm_support");
	ppe->bridge_mode = dev_read_bool(dev, "bridge_mode");
	ppe->bridge_port_mask = dev_read_u32_default(dev,
						     "bridge-port-mask", 0);
	if (!ppe->bridge_mode)
		ppe->nbport = dev_read_u32_default(dev, "port", 0);

	ret = ipq_eth_parse_ethernet_ports(dev, priv, &port_count);
	if (ret < 0)
		return ret;
	if (!ret) {
		ret = ipq_eth_parse_legacy_phy_handles(dev, priv, &port_count);
		if (ret)
			return ret;
	}

	priv->uniphy_base = ipq_eth_read_uniphy_base(dev, priv);
	if (priv->uniphy_base == FDT_ADDR_T_NONE && !priv->emulation) {
		dev_err(dev, "uniphy_base bus address not found\n");
		return -EINVAL;
	}

	return 0;
}

static const struct udevice_id ipq_eth_ids[] = {
	{ .compatible = "qcom,ipq9574-ppe",
	  .data = (ulong)&ipq_edma_config },
	{ .compatible = "qti,ipq-nss-switch",
	  .data = (ulong)&ipq_edma_config },
	{ }
};

U_BOOT_DRIVER(eth_ipq) = {
	.name	= "eth_ipq",
	.id	= UCLASS_ETH,
	.of_match = ipq_eth_ids,
	.of_to_plat = ipq_eth_ofdata_to_platdata,
	.bind	= ipq_eth_bind,
	.probe	= ipq_eth_probe,
	.remove	= ipq_eth_remove,
	.ops	= &ipq_eth_ops,
	.priv_auto = sizeof(struct ipq_eth_dev),
	.plat_auto = sizeof(struct eth_pdata),
	.flags = DM_FLAG_ALLOC_PRIV_DMA,
};

static struct ipq_eth_dev *ipq_eth_debug_get_priv_relaxed(bool require_active)
{
	struct udevice *dev = eth_get_current();

	if (!dev) {
		printf("Failed to find current nss-switch device\n");
		return NULL;
	}

	if (!eth_is_active(dev)) {
		if (!require_active) {
			printf("nss-switch device is halted; dumping retained state\n");
			return dev_get_priv(dev);
		}

		printf("Failed to find active nss-switch device\n");
		return NULL;
	}

	return dev_get_priv(dev);
}

static struct ipq_eth_dev *ipq_eth_debug_get_priv(void)
{
	return ipq_eth_debug_get_priv_relaxed(true);
}

static phys_addr_t ipq_eth_debug_base(struct ipq_eth_dev *priv,
				      const char *name)
{
	if (!strcmp(name, "ppe"))
		return priv->ppe.base;
	if (!strcmp(name, "edma"))
		return priv->hw.iobase;
	if (!strcmp(name, "uniphy"))
		return priv->uniphy_base;

	return 0;
}

static int ipq_eth_debug_peek(struct ipq_eth_dev *priv, int argc,
			      char *const argv[])
{
	phys_addr_t base;
	u32 off, count = 1, i;

	if (argc < 4 || argc > 5)
		return CMD_RET_USAGE;

	base = ipq_eth_debug_base(priv, argv[2]);
	if (!base) {
		printf("Unknown register space '%s'\n", argv[2]);
		return CMD_RET_USAGE;
	}

	off = simple_strtoul(argv[3], NULL, 0);
	if (argc == 5)
		count = simple_strtoul(argv[4], NULL, 0);
	if (!count)
		count = 1;
	if (count > 64)
		count = 64;

	printf("%s base=0x%llx off=0x%x count=%u\n", argv[2],
	       (unsigned long long)base, off, count);
	for (i = 0; i < count; i++) {
		if (!(i % 4))
			printf(" 0x%08x:", off + i * 4);
		printf(" %08x", readl(base + off + i * 4));
		if ((i % 4) == 3 || i + 1 == count)
			printf("\n");
	}

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_poke(struct ipq_eth_dev *priv, int argc,
			      char *const argv[])
{
	phys_addr_t base;
	u32 off, value, old;

	if (argc != 5)
		return CMD_RET_USAGE;

	base = ipq_eth_debug_base(priv, argv[2]);
	if (!base) {
		printf("Unknown register space '%s'\n", argv[2]);
		return CMD_RET_USAGE;
	}

	off = simple_strtoul(argv[3], NULL, 0);
	value = simple_strtoul(argv[4], NULL, 0);
	old = readl(base + off);
	writel(value, base + off);
	printf("%s[0x%08x]: 0x%08x -> 0x%08x\n",
	       argv[2], off, old, readl(base + off));

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_phy(struct ipq_eth_dev *priv, int argc,
			     char *const argv[])
{
	struct port_info *port;
	int port_arg, port_index = -1;
	int devad = MDIO_DEVAD_NONE;
	int reg = MII_BMSR;
	int i, ret;

	if (argc < 3 || argc > 5)
		return CMD_RET_USAGE;

	port_arg = simple_strtoul(argv[2], NULL, 0);
	port = ipq_eth_debug_find_port(priv, port_arg);
	if (!port && port_arg >= 0 && port_arg < CONFIG_ETH_MAX_MAC)
		port = priv->port[port_arg];

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		if (priv->port[i] == port) {
			port_index = i;
			break;
		}
	}

	if (!port) {
		printf("No PPE port id or parsed DT slot %d\n", port_arg);
		return CMD_RET_FAILURE;
	}

	if (!port->phydev) {
		printf("port id=%u slot=%d has no phydev\n",
		       port->id, port_index);
		return CMD_RET_FAILURE;
	}

	if (argc >= 4)
		devad = simple_strtol(argv[3], NULL, 0);
	if (argc == 5)
		reg = simple_strtoul(argv[4], NULL, 0);

	printf("port id=%u slot=%d phyaddr=0x%x link=%d speed=%d duplex=%d configured=%d\n",
	       port->id, port_index, port->phyaddr, port->phydev->link,
	       port->phydev->speed, port->phydev->duplex, port->isconfigured);
	ret = phy_read(port->phydev, devad, reg);
	if (ret < 0)
		printf(" phy read devad=%d reg=0x%x failed: %d\n",
		       devad, reg, ret);
	else
		printf(" phy read devad=%d reg=0x%x -> 0x%04x\n",
		       devad, reg, ret & 0xffff);

	return CMD_RET_SUCCESS;
}

static struct mii_dev *ipq_eth_debug_first_mdio(struct ipq_eth_dev *priv,
						struct udevice **mdio_dev)
{
	struct mdio_perdev_priv *pdata;
	struct udevice *busdev;
	struct mii_dev *bus;
	int i, ret;

	if (mdio_dev)
		*mdio_dev = NULL;

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		if (!priv->port[i])
			continue;

		ret = ipq_eth_get_mdio_device(priv->port[i]->node, &busdev);
		if (!ret) {
			ret = device_probe(busdev);
			if (ret) {
				printf("MDIO bus %s probe failed: %d\n",
				       busdev->name, ret);
				continue;
			}

			pdata = dev_get_uclass_priv(busdev);
			if (pdata && pdata->mii_bus &&
			    pdata->mii_bus->read && pdata->mii_bus->write) {
				if (mdio_dev)
					*mdio_dev = busdev;
				return pdata->mii_bus;
			}

			printf("MDIO bus %s has no usable legacy mii_bus\n",
			       busdev->name);
			continue;
		}

		if (priv->port[i]->phydev) {
			bus = priv->port[i]->phydev->bus;
			if (bus && bus->priv && bus->read && bus->write)
				return bus;
		}

		bus = priv->port[i]->bus;
		if (bus && bus->priv && bus->read && bus->write)
			return bus;
	}

	return NULL;
}

static struct port_info *ipq_eth_debug_qca8075_first_port(struct ipq_eth_dev *priv)
{
	int i;

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		if (priv->port[i] && priv->port[i]->phy_id == QCA8075_PHY_TYPE)
			return priv->port[i];
	}

	return NULL;
}

static int ipq_eth_debug_qca8075_base(struct ipq_eth_dev *priv, u8 *base)
{
	struct port_info *port;
	ofnode package_node;
	u32 package_reg;

	port = ipq_eth_debug_qca8075_first_port(priv);
	if (!port) {
		printf("No QCA8075 base PHY address found\n");
		return -ENODEV;
	}

	package_node = ofnode_get_parent(port->node);
	if (ofnode_valid(package_node) &&
	    !ofnode_read_u32(package_node, "reg", &package_reg) &&
	    package_reg < PHY_MAX_ADDR) {
		*base = package_reg;
		return 0;
	}

	*base = port->phyaddr & ~0x7;
	return 0;
}

static int ipq_eth_debug_qca8075_lane_addrs(struct ipq_eth_dev *priv,
					    u8 base, bool all_lanes,
					    u8 addrs[QCA8075_LANES],
					    int *count)
{
	int i;

	*count = 0;
	if (all_lanes) {
		for (i = 0; i < QCA8075_LANES; i++)
			addrs[(*count)++] = base + i;
		return 0;
	}

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		if (priv->port[i] &&
		    priv->port[i]->phy_id == QCA8075_PHY_TYPE) {
			if (*count >= QCA8075_LANES)
				return -ENOSPC;
			addrs[(*count)++] = priv->port[i]->phyaddr;
		}
	}

	if (*count)
		return 0;

	for (i = 0; i < QCA8075_LANES; i++)
		addrs[(*count)++] = base + i;

	return 0;
}

static int ipq_eth_debug_qca8075_read(struct port_info *ref_port,
				      u8 addr, int devad, int reg)
{
	struct phy_device phydev;

	if (!ref_port || !ref_port->phydev)
		return -ENODEV;

	memcpy(&phydev, ref_port->phydev, sizeof(phydev));
	phydev.addr = addr;

	if (devad == MDIO_DEVAD_NONE)
		return phy_read(&phydev, MDIO_DEVAD_NONE, reg);

	return phy_read_mmd(&phydev, devad, reg);
}

static int ipq_eth_debug_qca8075_write(struct port_info *ref_port,
				       u8 addr, int devad, int reg, u16 value)
{
	struct phy_device phydev;

	if (!ref_port || !ref_port->phydev)
		return -ENODEV;

	memcpy(&phydev, ref_port->phydev, sizeof(phydev));
	phydev.addr = addr;

	if (devad == MDIO_DEVAD_NONE)
		return phy_write(&phydev, MDIO_DEVAD_NONE, reg, value);

	return phy_write_mmd(&phydev, devad, reg, value);
}

static int ipq_eth_debug_mdio_validate(int phyaddr, int devad, int reg)
{
	if (phyaddr < 0 || phyaddr >= PHY_MAX_ADDR)
		return -EINVAL;
	if (devad != MDIO_DEVAD_NONE && (devad < 0 || devad > 31))
		return -EINVAL;
	if (reg < 0 || reg > 0xffff)
		return -EINVAL;

	return 0;
}

static int ipq_eth_debug_mdio_read_raw(struct udevice *mdio_dev,
				       struct mii_dev *bus, int phyaddr,
				       int devad, int reg)
{
	int ret;

	(void)mdio_dev;

	ret = ipq_eth_debug_mdio_validate(phyaddr, devad, reg);
	if (ret)
		return ret;

	if (!bus || !bus->priv || !bus->read)
		return -ENODEV;
	if (devad != MDIO_DEVAD_NONE && !bus->write)
		return -ENODEV;

	if (devad == MDIO_DEVAD_NONE)
		return bus->read(bus, phyaddr, MDIO_DEVAD_NONE, reg);

	ret = bus->write(bus, phyaddr, MDIO_DEVAD_NONE, MII_MMD_CTRL, devad);
	if (ret < 0)
		return ret;
	ret = bus->write(bus, phyaddr, MDIO_DEVAD_NONE, MII_MMD_DATA, reg);
	if (ret < 0)
		return ret;
	ret = bus->write(bus, phyaddr, MDIO_DEVAD_NONE, MII_MMD_CTRL,
			 devad | MII_MMD_CTRL_NOINCR);
	if (ret < 0)
		return ret;

	return bus->read(bus, phyaddr, MDIO_DEVAD_NONE, MII_MMD_DATA);
}

static int ipq_eth_debug_mdio_write_raw(struct udevice *mdio_dev,
					struct mii_dev *bus, int phyaddr,
					int devad, int reg, u16 value)
{
	int ret;

	(void)mdio_dev;

	ret = ipq_eth_debug_mdio_validate(phyaddr, devad, reg);
	if (ret)
		return ret;

	if (!bus || !bus->priv || !bus->write)
		return -ENODEV;

	if (devad == MDIO_DEVAD_NONE)
		return bus->write(bus, phyaddr, MDIO_DEVAD_NONE, reg, value);

	ret = bus->write(bus, phyaddr, MDIO_DEVAD_NONE, MII_MMD_CTRL, devad);
	if (ret < 0)
		return ret;
	ret = bus->write(bus, phyaddr, MDIO_DEVAD_NONE, MII_MMD_DATA, reg);
	if (ret < 0)
		return ret;
	ret = bus->write(bus, phyaddr, MDIO_DEVAD_NONE, MII_MMD_CTRL,
			 devad | MII_MMD_CTRL_NOINCR);
	if (ret < 0)
		return ret;

	return bus->write(bus, phyaddr, MDIO_DEVAD_NONE, MII_MMD_DATA, value);
}

static int ipq_eth_debug_mdio(struct ipq_eth_dev *priv, int argc,
			      char *const argv[])
{
	struct mii_dev *bus;
	struct udevice *mdio_dev;
	int phyaddr, devad = MDIO_DEVAD_NONE, reg;
	int old, now;
	u16 value;

	if (argc < 4 || argc > 6)
		return CMD_RET_USAGE;

	bus = ipq_eth_debug_first_mdio(priv, &mdio_dev);
	if (!bus) {
		printf("No MDIO bus found\n");
		return CMD_RET_FAILURE;
	}

	phyaddr = simple_strtoul(argv[2], NULL, 0);
	if (argc == 4) {
		reg = simple_strtoul(argv[3], NULL, 0);
	} else {
		devad = simple_strtol(argv[3], NULL, 0);
		reg = simple_strtoul(argv[4], NULL, 0);
	}

	if (argc == 6) {
		value = simple_strtoul(argv[5], NULL, 0);
		old = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, phyaddr,
						  devad, reg);
		if (old < 0) {
			printf(" mdio phy=0x%x devad=%d reg=0x%x read failed: %d\n",
			       phyaddr, devad, reg, old);
			return CMD_RET_FAILURE;
		}
		now = ipq_eth_debug_mdio_write_raw(mdio_dev, bus, phyaddr,
						   devad, reg, value);
		if (now < 0) {
			printf(" mdio phy=0x%x devad=%d reg=0x%x write failed: %d\n",
			       phyaddr, devad, reg, now);
			return CMD_RET_FAILURE;
		}
		now = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, phyaddr,
						  devad, reg);
		if (now < 0) {
			printf(" mdio phy=0x%x devad=%d reg=0x%x reread failed: %d\n",
			       phyaddr, devad, reg, now);
			return CMD_RET_FAILURE;
		}
		printf(" mdio phy=0x%x devad=%d reg=0x%x: 0x%04x -> 0x%04x\n",
		       phyaddr, devad, reg, old & 0xffff, now & 0xffff);
	} else {
		now = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, phyaddr,
						  devad, reg);
		if (now < 0) {
			printf(" mdio phy=0x%x devad=%d reg=0x%x read failed: %d\n",
			       phyaddr, devad, reg, now);
			return CMD_RET_FAILURE;
		}
		printf(" mdio phy=0x%x devad=%d reg=0x%x -> 0x%04x\n",
		       phyaddr, devad, reg, now & 0xffff);
	}

	return CMD_RET_SUCCESS;
}

static void ipq_eth_debug_qca8075_one(struct port_info *ref_port, u8 addr)
{
	int bmcr, bmsr, spec, cfg, eee, dac;

	bmcr = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_DEVAD_NONE,
					  MII_BMCR);
	bmsr = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_DEVAD_NONE,
					  MII_BMSR);
	spec = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_DEVAD_NONE,
					  QCA8075_PHY_SPEC_STATUS);
	cfg = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_DEVAD_NONE,
					 QCA8075_PHY_CHIP_CONFIG);
	eee = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_AN,
					 QCA8075_MMD7_EEE_CTRL);
	dac = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_AN,
					 QCA8075_MMD7_DAC_CTRL);

	if (bmcr < 0 || bmsr < 0 || spec < 0 || cfg < 0 ||
	    eee < 0 || dac < 0) {
		printf(" qca8075 phy=0x%x read failed bmcr=%d bmsr=%d spec=%d cfg=%d eee=%d dac=%d\n",
		       addr, bmcr, bmsr, spec, cfg, eee, dac);
		return;
	}

	printf(" qca8075 phy=0x%x bmcr=%04x bmsr=%04x spec=%04x cfg=%04x eee=%04x dac=%04x\n",
	       addr, bmcr & 0xffff, bmsr & 0xffff, spec & 0xffff,
	       cfg & 0xffff, eee & 0xffff, dac & 0xffff);
}

static int ipq_eth_debug_qca8075_cnt_one(struct port_info *ref_port, u8 addr)
{
	int ctrl, ig_hi, ig_lo, ig_err, eg_hi, eg_lo, eg_err;
	u32 ig_ok, eg_ok;

	ctrl = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_AN,
					  QCA8075_MMD7_CRC_PACKET_COUNTER);
	ig_hi = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_AN,
					   QCA8075_MMD7_VALID_INGRESS_COUNTER_HI);
	ig_lo = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_AN,
					   QCA8075_MMD7_VALID_INGRESS_COUNTER_LO);
	ig_err = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_AN,
					    QCA8075_MMD7_ERRED_INGRESS_COUNTER);
	eg_hi = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_AN,
					   QCA8075_MMD7_VALID_EGRESS_COUNTER_HI);
	eg_lo = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_AN,
					   QCA8075_MMD7_VALID_EGRESS_COUNTER_LO);
	eg_err = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_AN,
					    QCA8075_MMD7_ERRED_EGRESS_COUNTER);

	if (ctrl < 0 || ig_hi < 0 || ig_lo < 0 || ig_err < 0 ||
	    eg_hi < 0 || eg_lo < 0 || eg_err < 0) {
		printf(" qca8075 phy=0x%x counter read failed ctrl=%d ingress=%d/%d/%d egress=%d/%d/%d\n",
		       addr, ctrl, ig_hi, ig_lo, ig_err, eg_hi, eg_lo,
		       eg_err);
		return -EIO;
	}

	ig_ok = ((u32)(ig_hi & 0xffff) << 16) | (ig_lo & 0xffff);
	eg_ok = ((u32)(eg_hi & 0xffff) << 16) | (eg_lo & 0xffff);

	printf(" qca8075 phy=0x%x cnt_ctrl=%04x ingress_ok=%u ingress_err=%u egress_ok=%u egress_err=%u\n",
	       addr, ctrl & 0xffff, ig_ok, ig_err & 0xffff, eg_ok,
	       eg_err & 0xffff);

	return 0;
}

static int ipq_eth_debug_qca8075_cnt_set(struct port_info *ref_port, u8 addr,
					 const char *action)
{
	int ctrl, old, ret;

	old = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_AN,
					 QCA8075_MMD7_CRC_PACKET_COUNTER);
	if (old < 0) {
		printf(" qca8075 phy=0x%x counter control read failed: %d\n",
		       addr, old);
		return old;
	}

	ctrl = old;

	if (!strcmp(action, "enable")) {
		ctrl |= QCA8075_MMD7_FRAME_CHK_EN;
		ctrl &= ~QCA8075_MMD7_COUNTER_SELFCLR;
	} else if (!strcmp(action, "disable")) {
		ctrl &= ~(QCA8075_MMD7_FRAME_CHK_EN |
			  QCA8075_MMD7_COUNTER_SELFCLR);
	} else if (!strcmp(action, "clear")) {
		ctrl |= QCA8075_MMD7_FRAME_CHK_EN |
			QCA8075_MMD7_COUNTER_SELFCLR;
	} else {
		return -EINVAL;
	}

	ret = ipq_eth_debug_qca8075_write(ref_port, addr, MDIO_MMD_AN,
					  QCA8075_MMD7_CRC_PACKET_COUNTER,
					  ctrl);
	if (ret < 0) {
		printf(" qca8075 phy=0x%x counter control write failed: %d\n",
		       addr, ret);
		return ret;
	}

	if (!strcmp(action, "clear")) {
		ret = ipq_eth_debug_qca8075_cnt_one(ref_port, addr);
		if (ret)
			return ret;
		ctrl &= ~QCA8075_MMD7_COUNTER_SELFCLR;
		ret = ipq_eth_debug_qca8075_write(ref_port, addr,
						  MDIO_MMD_AN,
						  QCA8075_MMD7_CRC_PACKET_COUNTER,
						  ctrl);
		if (ret < 0) {
			printf(" qca8075 phy=0x%x counter self-clear restore failed: %d\n",
			       addr, ret);
			return ret;
		}
	}

	ctrl = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_AN,
					  QCA8075_MMD7_CRC_PACKET_COUNTER);
	if (ctrl < 0) {
		printf(" qca8075 phy=0x%x counter control reread failed: %d\n",
		       addr, ctrl);
		return ctrl;
	}

	printf(" qca8075 phy=0x%x cnt_ctrl: %04x -> %04x\n",
	       addr, old & 0xffff, ctrl & 0xffff);

	return 0;
}

static int ipq_eth_debug_qca8075_cnt(struct ipq_eth_dev *priv, int argc,
				     char *const argv[])
{
	struct port_info *ref_port;
	const char *action = "read";
	u8 base, addrs[QCA8075_LANES];
	int i, count;
	bool all_lanes = false;

	if (argc > 4)
		return CMD_RET_USAGE;

	ref_port = ipq_eth_debug_qca8075_first_port(priv);
	if (!ref_port || !ref_port->phydev) {
		printf("No QCA8075 PHY device found\n");
		return CMD_RET_FAILURE;
	}

	if (argc == 2) {
		if (ipq_eth_debug_qca8075_base(priv, &base))
			return CMD_RET_FAILURE;
	} else if (argc == 3) {
		if (!strcmp(argv[2], "read") || !strcmp(argv[2], "enable") ||
		    !strcmp(argv[2], "disable") || !strcmp(argv[2], "clear")) {
			action = argv[2];
			if (ipq_eth_debug_qca8075_base(priv, &base))
				return CMD_RET_FAILURE;
		} else {
			base = simple_strtoul(argv[2], NULL, 0);
			all_lanes = true;
		}
	} else {
		base = simple_strtoul(argv[2], NULL, 0);
		action = argv[3];
		all_lanes = true;
	}

	if (strcmp(action, "read") && strcmp(action, "enable") &&
	    strcmp(action, "disable") && strcmp(action, "clear"))
		return CMD_RET_USAGE;

	if (ipq_eth_debug_qca8075_lane_addrs(priv, base, all_lanes,
					     addrs, &count))
		return CMD_RET_FAILURE;

	printf("QCA8075 counters base=0x%x action=%s lanes=%s\n",
	       base, action, all_lanes ? "all" : "dt");
	for (i = 0; i < count; i++) {
		if (strcmp(action, "read") &&
		    ipq_eth_debug_qca8075_cnt_set(ref_port, addrs[i], action))
			return CMD_RET_FAILURE;
		if (ipq_eth_debug_qca8075_cnt_one(ref_port, addrs[i]))
			return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_qca8075(struct ipq_eth_dev *priv, int argc,
				 char *const argv[])
{
	struct port_info *ref_port;
	u8 base, psgmii, addrs[QCA8075_LANES];
	int count;
	int i, mode, fifo, txdrv, loop;
	bool all_lanes = false;
	bool read_ctrl = false;

	if (argc > 4)
		return CMD_RET_USAGE;

	ref_port = ipq_eth_debug_qca8075_first_port(priv);
	if (!ref_port || !ref_port->phydev) {
		printf("No QCA8075 PHY device found\n");
		return CMD_RET_FAILURE;
	}

	if (argc == 2 || (argc == 3 && !strcmp(argv[2], "ctrl"))) {
		if (ipq_eth_debug_qca8075_base(priv, &base))
			return CMD_RET_FAILURE;
		read_ctrl = argc == 3;
	} else {
		base = simple_strtoul(argv[2], NULL, 0);
		all_lanes = true;

		if (argc == 4) {
			if (strcmp(argv[3], "ctrl"))
				return CMD_RET_USAGE;
			read_ctrl = true;
		}
	}

	if (ipq_eth_debug_qca8075_lane_addrs(priv, base, all_lanes,
					     addrs, &count))
		return CMD_RET_FAILURE;

	printf("QCA8075 snapshot base=0x%x lanes=%s QSGMII/PSGMII phy=0x%x ctrl=%s\n",
	       base, all_lanes ? "all" : "dt",
	       base + QCA8075_PSGMII_ADDR_OFFSET,
	       read_ctrl ? "read" : "skipped");
	for (i = 0; i < count; i++)
		ipq_eth_debug_qca8075_one(ref_port, addrs[i]);

	if (!read_ctrl)
		return CMD_RET_SUCCESS;

	psgmii = base + QCA8075_PSGMII_ADDR_OFFSET;
	mode = ipq_eth_debug_qca8075_read(ref_port, psgmii, MDIO_MMD_PMAPMD,
					  QCA8075_MMD1_PSGMII_MODE_CTRL);
	fifo = ipq_eth_debug_qca8075_read(ref_port, psgmii, MDIO_MMD_PMAPMD,
					  QCA8075_MMD1_PSGMII_FIFO_CTRL);
	txdrv = ipq_eth_debug_qca8075_read(ref_port, psgmii, MDIO_DEVAD_NONE,
					   QCA8075_PSGMII_TX_DRIVER_1_CTRL);
	loop = ipq_eth_debug_qca8075_read(ref_port, psgmii, MDIO_MMD_PCS,
					  QCA8075_MMD3_REMOTE_LOOPBACK_CTRL);
	if (mode < 0 || fifo < 0 || txdrv < 0 || loop < 0) {
		printf(" qsgmii/psgmii phy=0x%x read failed mode=%d fifo=%d txdrv=%d loop=%d\n",
		       psgmii, mode, fifo, txdrv, loop);
		return CMD_RET_FAILURE;
	}

	printf(" qsgmii/psgmii phy=0x%x mmd1.mode=%04x mmd1.fifo=%04x txdrv=%04x mmd3.loop=%04x\n",
	       psgmii, mode & 0xffff, fifo & 0xffff, txdrv & 0xffff,
	       loop & 0xffff);

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_qca8075_loop(struct ipq_eth_dev *priv, int argc,
				      char *const argv[])
{
	struct port_info *ref_port;
	const char *action;
	u8 base, lane, addr;
	int old, ret;
	u16 value;
	bool cut_rx = false;

	if (argc < 4 || argc > 6)
		return CMD_RET_USAGE;

	ref_port = ipq_eth_debug_qca8075_first_port(priv);
	if (!ref_port || !ref_port->phydev) {
		printf("No QCA8075 PHY device found\n");
		return CMD_RET_FAILURE;
	}

	if (argc == 5 && !strcmp(argv[4], "cutrx")) {
		if (ipq_eth_debug_qca8075_base(priv, &base))
			return CMD_RET_FAILURE;
		lane = simple_strtoul(argv[2], NULL, 0);
		action = argv[3];
		cut_rx = true;
	} else if (argc >= 5) {
		base = simple_strtoul(argv[2], NULL, 0);
		lane = simple_strtoul(argv[3], NULL, 0);
		action = argv[4];
		if (argc == 6 && !strcmp(argv[5], "cutrx"))
			cut_rx = true;
	} else {
		if (ipq_eth_debug_qca8075_base(priv, &base))
			return CMD_RET_FAILURE;
		lane = simple_strtoul(argv[2], NULL, 0);
		action = argv[3];
	}

	if (lane >= QCA8075_LANES)
		return CMD_RET_USAGE;
	if (strcmp(action, "on") && strcmp(action, "off"))
		return CMD_RET_USAGE;

	addr = base + lane;
	old = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_PCS,
					 QCA8075_MMD3_REMOTE_LOOPBACK_CTRL);
	if (old < 0) {
		printf(" qca8075 phy=0x%x loopback control read failed: %d\n",
		       addr, old);
		return CMD_RET_FAILURE;
	}

	value = old;
	if (!strcmp(action, "on")) {
		value |= QCA8075_MMD3_REMOTE_LOOPBACK_EN;
		if (cut_rx)
			value |= QCA8075_MMD3_REMOTE_LOOPBACK_CUT_RX;
		else
			value &= ~QCA8075_MMD3_REMOTE_LOOPBACK_CUT_RX;
	} else {
		value &= ~(QCA8075_MMD3_REMOTE_LOOPBACK_EN |
			   QCA8075_MMD3_REMOTE_LOOPBACK_CUT_RX);
	}

	ret = ipq_eth_debug_qca8075_write(ref_port, addr, MDIO_MMD_PCS,
					  QCA8075_MMD3_REMOTE_LOOPBACK_CTRL,
					  value);
	if (ret < 0) {
		printf(" qca8075 phy=0x%x loopback control write failed: %d\n",
		       addr, ret);
		return CMD_RET_FAILURE;
	}

	ret = ipq_eth_debug_qca8075_read(ref_port, addr, MDIO_MMD_PCS,
					 QCA8075_MMD3_REMOTE_LOOPBACK_CTRL);
	if (ret < 0) {
		printf(" qca8075 phy=0x%x loopback control reread failed: %d\n",
		       addr, ret);
		return CMD_RET_FAILURE;
	}

	printf(" qca8075 phy=0x%x lane=%u mmd3.loop: %04x -> %04x\n",
	       addr, lane, old & 0xffff, ret & 0xffff);

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_qca8081_addr(struct ipq_eth_dev *priv, u8 *addr)
{
	int i;

	for (i = 0; i < CONFIG_ETH_MAX_MAC; i++) {
		if (priv->port[i] &&
		    priv->port[i]->phy_id == QCA8081_PHY_TYPE) {
			*addr = priv->port[i]->phyaddr;
			return 0;
		}
	}

	printf("No QCA8081 PHY address found\n");
	return -ENODEV;
}

static const char *ipq_eth_debug_qca8081_speed(int spec)
{
	switch (spec & QCA8081_STATUS_SPEED_MASK) {
	case QCA8081_STATUS_SPEED_2500MBS:
		return "2500";
	case QCA8081_STATUS_SPEED_1000MBS:
		return "1000";
	case QCA8081_STATUS_SPEED_100MBS:
		return "100";
	case QCA8081_STATUS_SPEED_10MBS:
		return "10";
	default:
		return "unknown";
	}
}

static int ipq_eth_debug_qca8081(struct ipq_eth_dev *priv, int argc,
				 char *const argv[])
{
	struct mii_dev *bus;
	struct udevice *mdio_dev;
	u8 addr;
	int bmcr, bmsr, id1, id2, spec, cld7, az, eee;
	int an_ctrl, an_stat, chip_type, fifo;

	if (argc > 3)
		return CMD_RET_USAGE;

	bus = ipq_eth_debug_first_mdio(priv, &mdio_dev);
	if (!bus) {
		printf("No MDIO bus found\n");
		return CMD_RET_FAILURE;
	}

	if (argc == 3)
		addr = simple_strtoul(argv[2], NULL, 0);
	else if (ipq_eth_debug_qca8081_addr(priv, &addr))
		return CMD_RET_FAILURE;

	bmcr = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, addr, MDIO_DEVAD_NONE,
					   MII_BMCR);
	bmsr = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, addr, MDIO_DEVAD_NONE,
					   MII_BMSR);
	id1 = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, addr, MDIO_DEVAD_NONE,
					  MII_PHYSID1);
	id2 = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, addr, MDIO_DEVAD_NONE,
					  MII_PHYSID2);
	spec = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, addr, MDIO_DEVAD_NONE,
					   QCA8081_PHY_SPEC_STATUS);
	cld7 = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, addr, MDIO_MMD_PCS,
					   QCA8081_MMD3_CLD_CTRL7);
	az = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, addr, MDIO_MMD_PCS,
					 QCA8081_MMD3_AZ_TRAINING_CTRL);
	eee = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, addr, MDIO_MMD_AN,
					  QCA8075_MMD7_EEE_CTRL);
	an_ctrl = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, addr, MDIO_MMD_AN,
					      MDIO_AN_10GBT_CTRL);
	an_stat = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, addr, MDIO_MMD_AN,
					      MDIO_AN_10GBT_STAT);
	chip_type = ipq_eth_debug_mdio_read_raw(mdio_dev, bus, addr,
						MDIO_MMD_AN,
						QCA8081_MMD7_CHIP_TYPE);
	fifo = ipq_eth_debug_mdio_read_raw(mdio_dev, bus,
					   addr + QCA8081_SERDES_ADDR_OFFSET,
					   MDIO_MMD_PMAPMD,
					   QCA8081_SERDES_MMD1_FIFO_CTRL);

	if (bmcr < 0 || bmsr < 0 || id1 < 0 || id2 < 0 || spec < 0) {
		printf(" qca8081 phy=0x%x read failed bmcr=%d bmsr=%d id1=%d id2=%d spec=%d\n",
		       addr, bmcr, bmsr, id1, id2, spec);
		return CMD_RET_FAILURE;
	}

	printf(" qca8081 phy=0x%x id=%04x:%04x bmcr=%04x bmsr=%04x spec=%04x link=%u speed=%s duplex=%s\n",
	       addr, id1 & 0xffff, id2 & 0xffff, bmcr & 0xffff,
	       bmsr & 0xffff, spec & 0xffff,
	       !!(spec & QCA8081_STATUS_LINK_PASS),
	       ipq_eth_debug_qca8081_speed(spec),
	       (spec & QCA8081_STATUS_FULL_DUPLEX) ? "full" : "half");

	if (cld7 < 0 || az < 0 || eee < 0) {
		printf(" qca8081 phy=0x%x mmd read failed cld7=%d az=%d eee=%d\n",
		       addr, cld7, az, eee);
		return CMD_RET_FAILURE;
	}

	printf(" qca8081 phy=0x%x mmd3.cld7=%04x mmd3.az=%04x mmd7.eee=%04x\n",
	       addr, cld7 & 0xffff, az & 0xffff, eee & 0xffff);

	if (an_ctrl < 0 || an_stat < 0 || chip_type < 0) {
		printf(" qca8081 phy=0x%x 2.5G status unavailable ctrl=%d stat=%d chip=%d\n",
		       addr, an_ctrl, an_stat, chip_type);
	} else {
		printf(" qca8081 phy=0x%x mmd7.an2500=%04x lp=%04x chip=%04x adv=%u lp_adv=%u 1g_only=%u\n",
		       addr, an_ctrl & 0xffff, an_stat & 0xffff,
		       chip_type & 0xffff,
		       !!(an_ctrl & MDIO_AN_10GBT_CTRL_ADV2_5G),
		       !!(an_stat & MDIO_AN_10GBT_STAT_LP2_5G),
		       !!(chip_type & QCA8081_CHIP_TYPE_1G));
	}

	if (fifo < 0) {
		printf(" qca8081 serdes=0x%x mmd1.fifo read failed: %d\n",
		       addr + QCA8081_SERDES_ADDR_OFFSET, fifo);
	} else {
		printf(" qca8081 serdes=0x%x mmd1.fifo=%04x rstn=%u\n",
		       addr + QCA8081_SERDES_ADDR_OFFSET, fifo & 0xffff,
		       !!(fifo & QCA8081_FIFO_RSTN));
	}

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_qid2rid(struct ipq_eth_dev *priv, int argc,
				 char *const argv[])
{
	u32 qid = EDMA_CPU_PORT_QID_MIN;
	u32 count = EDMA_CPU_PORT_QID_MAX - EDMA_CPU_PORT_QID_MIN + 1;

	if (argc > 4)
		return CMD_RET_USAGE;
	if (argc >= 3)
		qid = simple_strtoul(argv[2], NULL, 0);
	if (argc == 4)
		count = simple_strtoul(argv[3], NULL, 0);

	ipq_eth_debug_dump_qid2rid(priv, qid, count);

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_setqid(struct ipq_eth_dev *priv, int argc,
				char *const argv[])
{
	struct ipq_edma_hw *ehw = &priv->hw;
	u32 qid, rid, reg_idx, shift, old, value;

	if (argc != 4)
		return CMD_RET_USAGE;

	qid = simple_strtoul(argv[2], NULL, 0);
	rid = simple_strtoul(argv[3], NULL, 0) & 0xff;
	reg_idx = qid / EDMA_QID2RID_NUM_PER_REG;
	shift = (qid % EDMA_QID2RID_NUM_PER_REG) * 8;
	old = readl(ehw->iobase + EDMA_QID2RID_TABLE_MEM(reg_idx));
	value = (old & ~(0xffU << shift)) | (rid << shift);
	writel(value, ehw->iobase + EDMA_QID2RID_TABLE_MEM(reg_idx));
	printf("qid%u qid2rid_reg[%u]: 0x%08x -> 0x%08x\n",
	       qid, reg_idx, old,
	       readl(ehw->iobase + EDMA_QID2RID_TABLE_MEM(reg_idx)));

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_setqidreg(struct ipq_eth_dev *priv, int argc,
				   char *const argv[])
{
	struct ipq_edma_hw *ehw = &priv->hw;
	u32 reg_idx, value, old;

	if (argc != 4)
		return CMD_RET_USAGE;

	reg_idx = simple_strtoul(argv[2], NULL, 0);
	value = simple_strtoul(argv[3], NULL, 0);
	old = readl(ehw->iobase + EDMA_QID2RID_TABLE_MEM(reg_idx));
	writel(value, ehw->iobase + EDMA_QID2RID_TABLE_MEM(reg_idx));
	printf("qid2rid_reg[%u] @0x%08x: 0x%08x -> 0x%08x\n",
	       reg_idx, EDMA_QID2RID_TABLE_MEM(reg_idx), old,
	       readl(ehw->iobase + EDMA_QID2RID_TABLE_MEM(reg_idx)));

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_txctrl(struct ipq_eth_dev *priv, int argc,
				char *const argv[])
{
	struct ipq_edma_hw *ehw = &priv->hw;
	phys_addr_t reg_base = ehw->iobase;
	u32 ring_id, old, value;

	if (!ehw->txdesc_ring || !ehw->txcmpl_ring)
		return CMD_RET_FAILURE;

	if (argc == 2) {
		ring_id = 0;
		goto dump;
	}

	if (argc < 3 || argc > 4)
		return CMD_RET_USAGE;

	ring_id = simple_strtoul(argv[2], NULL, 0);
	if (ring_id >= ehw->max_txdesc_rings)
		return CMD_RET_FAILURE;

	if (argc == 4) {
		value = simple_strtoul(argv[3], NULL, 0);
		old = readl(reg_base + EDMA_REG_TXDESC_CTRL(ring_id));
		writel(value, reg_base + EDMA_REG_TXDESC_CTRL(ring_id));
		printf("txdesc%u ctrl: 0x%08x -> 0x%08x\n",
		       ring_id, old,
		       readl(reg_base + EDMA_REG_TXDESC_CTRL(ring_id)));
	}

dump:
	printf("txdesc%u ctrl=0x%08x prod=%u/%u cons=%u/%u size=0x%08x ba=0x%08x/%08x ba2=0x%08x/%08x\n",
	       ring_id,
	       readl(reg_base + EDMA_REG_TXDESC_CTRL(ring_id)),
	       readl(reg_base + EDMA_REG_TXDESC_PROD_IDX(ring_id)) &
	       EDMA_TXDESC_PROD_IDX_MASK,
	       ipq_edma_ring_index(readl(reg_base + EDMA_REG_TXDESC_PROD_IDX(ring_id)) &
				   EDMA_TXDESC_PROD_IDX_MASK,
				   ehw->txdesc_ring[0].count),
	       readl(reg_base + EDMA_REG_TXDESC_CONS_IDX(ring_id)) &
	       EDMA_TXDESC_CONS_IDX_MASK,
	       ipq_edma_ring_index(readl(reg_base + EDMA_REG_TXDESC_CONS_IDX(ring_id)) &
				   EDMA_TXDESC_CONS_IDX_MASK,
				   ehw->txdesc_ring[0].count),
	       readl(reg_base + EDMA_REG_TXDESC_RING_SIZE(ring_id)),
	       readl(reg_base + EDMA_REG_TXDESC_BA(ring_id)),
	       readl(reg_base + EDMA_REG_TXDESC_BA_HIGH(ring_id)),
	       readl(reg_base + EDMA_REG_TXDESC_BA2(ring_id)),
	       readl(reg_base + EDMA_REG_TXDESC_BA2_HIGH(ring_id)));

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_txidx(struct ipq_eth_dev *priv, int argc,
			       char *const argv[])
{
	struct ipq_edma_hw *ehw = &priv->hw;
	phys_addr_t reg_base = ehw->iobase;
	const char *kind;
	u32 ring_id = 0, value, old;
	phys_addr_t reg;

	if (!ehw->txdesc_ring || !ehw->txcmpl_ring)
		return CMD_RET_FAILURE;

	if (argc < 3 || argc > 5)
		return CMD_RET_USAGE;

	kind = argv[2];
	if (argc >= 4)
		ring_id = simple_strtoul(argv[3], NULL, 0);
	if (ring_id >= ehw->max_txdesc_rings)
		return CMD_RET_FAILURE;

	if (!strcmp(kind, "txdesc-prod"))
		reg = EDMA_REG_TXDESC_PROD_IDX(ring_id);
	else if (!strcmp(kind, "txdesc-cons"))
		reg = EDMA_REG_TXDESC_CONS_IDX(ring_id);
	else if (!strcmp(kind, "txcmpl-prod"))
		reg = EDMA_REG_TXCMPL_PROD_IDX(ring_id);
	else if (!strcmp(kind, "txcmpl-cons"))
		reg = EDMA_REG_TXCMPL_CONS_IDX(ring_id);
	else
		return CMD_RET_USAGE;

	if (argc == 5) {
		value = simple_strtoul(argv[4], NULL, 0);
		old = readl(reg_base + reg);
		writel(value, reg_base + reg);
		printf("%s%u: 0x%08x -> 0x%08x\n", kind, ring_id, old,
		       readl(reg_base + reg));
	}

	printf("%s%u = 0x%08x\n", kind, ring_id, readl(reg_base + reg));
	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_txmap(struct ipq_eth_dev *priv, int argc,
			       char *const argv[])
{
	struct ipq_edma_hw *ehw = &priv->hw;
	phys_addr_t reg_base = ehw->iobase;
	u32 txdesc_id, txcmpl_id, reg, shift, old, value, i;

	if (!ehw->txdesc_ring || !ehw->txcmpl_ring)
		return CMD_RET_FAILURE;

	if (argc == 2) {
		printf("txdesc2cmpl map: %08x/%08x/%08x/%08x/%08x/%08x\n",
		       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_0),
		       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_1),
		       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_2),
		       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_3),
		       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_4),
		       readl(reg_base + EDMA_REG_TXDESC2CMPL_MAP_5));
		for (i = 0; i < ehw->txdesc_rings; i++) {
			txdesc_id = ehw->txdesc_ring[i].id;
			reg = ipq_edma_txdesc2cmpl_reg(txdesc_id);
			shift = (txdesc_id % 6) * 5;
			printf(" txdesc%u -> txcmpl%u\n", txdesc_id,
			       (readl(reg_base + reg) >> shift) & 0x1f);
		}
		return CMD_RET_SUCCESS;
	}

	if (argc != 4)
		return CMD_RET_USAGE;

	txdesc_id = simple_strtoul(argv[2], NULL, 0);
	txcmpl_id = simple_strtoul(argv[3], NULL, 0) & 0x1f;
	if (txdesc_id >= ehw->max_txdesc_rings)
		return CMD_RET_FAILURE;

	reg = ipq_edma_txdesc2cmpl_reg(txdesc_id);
	shift = (txdesc_id % 6) * 5;
	old = readl(reg_base + reg);
	value = (old & ~(0x1fU << shift)) | (txcmpl_id << shift);
	writel(value, reg_base + reg);
	printf("txdesc%u txdesc2cmpl[0x%08x]: 0x%08x -> 0x%08x, txcmpl=%u\n",
	       txdesc_id, reg, old, readl(reg_base + reg), txcmpl_id);

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_txmode(struct ipq_eth_dev *priv, int argc,
				char *const argv[])
{
	if (argc == 3 && (!strcmp(argv[2], "defaults") ||
			  !strcmp(argv[2], "reset"))) {
		ipq_edma_debug_reset_defaults();
		printf("EDMA debug TX mode reset to QSDK defaults\n");
		return CMD_RET_SUCCESS;
	}

	if (argc != 4)
		return CMD_RET_USAGE;

	if (!strcmp(argv[2], "port")) {
		ipq_edma_debug_port_ctrl_override =
			simple_strtoul(argv[3], NULL, 0);
		printf("EDMA debug port_ctrl override set to 0x%08x\n",
		       ipq_edma_debug_port_ctrl_override);
	} else if (!strcmp(argv[2], "dmar")) {
		if (!strcmp(argv[3], "v1"))
			ipq_edma_debug_dmar_mode = 1;
		else if (!strcmp(argv[3], "v3"))
			ipq_edma_debug_dmar_mode = 3;
		else
			return CMD_RET_USAGE;
		printf("EDMA debug DMAR mode set to %s\n",
		       ipq_edma_debug_dmar_mode == 3 ? "v3" : "v1");
	} else if (!strcmp(argv[2], "txctrlbase")) {
		if (!strcmp(argv[3], "qsdk") || !strcmp(argv[3], "0")) {
			ipq_edma_debug_txdesc_ctrl_base = 0;
		} else if (!strcmp(argv[3], "tso")) {
			ipq_edma_debug_txdesc_ctrl_base =
				EDMA_TXDESC_CTRL_FC_GRP_ID_SET(0) |
				EDMA_TXDESC_TSO_IDENT_UPDATE_CTRL_SET(
					EDMA_TXDESC_TSO_IDENT_UPDATE_BY_PARSER);
		} else {
			ipq_edma_debug_txdesc_ctrl_base =
				simple_strtoul(argv[3], NULL, 0);
		}
		printf("EDMA debug TXDESC_CTRL base set to 0x%08x\n",
		       ipq_edma_debug_txdesc_ctrl_base);
	} else if (!strcmp(argv[2], "tdes4")) {
		if (!strcmp(argv[3], "legacy")) {
			ipq_edma_debug_tdes4_mode = IPQ_EDMA_TDES4_LEGACY;
		} else if (!strcmp(argv[3], "srcdst")) {
			ipq_edma_debug_tdes4_mode = IPQ_EDMA_TDES4_SRCDST;
		} else if (!strcmp(argv[3], "dst")) {
			ipq_edma_debug_tdes4_mode = IPQ_EDMA_TDES4_DST;
		} else {
			ipq_edma_debug_tdes4_mode = IPQ_EDMA_TDES4_RAW;
			ipq_edma_debug_tdes4_raw = simple_strtoul(argv[3], NULL, 0);
		}
		printf("EDMA debug tdes4 mode set to %s raw=0x%08x\n",
		       ipq_edma_tdes4_mode_name(), ipq_edma_debug_tdes4_raw);
	} else if (!strcmp(argv[2], "passthrough")) {
		if (!strcmp(argv[3], "on") || !strcmp(argv[3], "1"))
			ipq_edma_debug_passthrough = 1;
		else if (!strcmp(argv[3], "off") || !strcmp(argv[3], "0"))
			ipq_edma_debug_passthrough = 0;
		else
			return CMD_RET_USAGE;
		printf("EDMA debug passthrough effective=%u\n",
		       ipq_edma_txdesc_passthrough_enabled() ? 1 : 0);
	} else {
		return CMD_RET_USAGE;
	}

	if (priv)
		ipq_edma_debug_dump_tx_regs(priv, "mode");

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_trace_cmd(struct ipq_eth_dev *priv, int argc,
				   char *const argv[])
{
	bool enable;

	(void)priv;

	if (argc == 2) {
		printf("NSS trace is %s (nss_debug_log=%s)\n",
		       ipq_edma_debug_trace() ? "on" : "off",
		       env_get("nss_debug_log") ?: "<unset>");
		return CMD_RET_SUCCESS;
	}

	if (argc != 3)
		return CMD_RET_USAGE;

	if (!strcmp(argv[2], "on") || !strcmp(argv[2], "1"))
		enable = true;
	else if (!strcmp(argv[2], "off") || !strcmp(argv[2], "0"))
		enable = false;
	else
		return CMD_RET_USAGE;

	ipq_edma_debug_trace_enabled = enable;
	env_set("nss_debug_log", enable ? "1" : "0");
	printf("NSS trace %s\n", enable ? "enabled" : "disabled");

	return CMD_RET_SUCCESS;
}

static int ipq_eth_debug_bridge_layout(struct ipq_eth_dev *priv, int argc,
				       char *const argv[])
{
	(void)priv;

	if (argc == 2) {
		printf("NSS bridge layout is %s (nss_bridge_layout=%s)\n",
		       ppe_bridge_ctrl_layout_name(),
		       env_get("nss_bridge_layout") ?: "<auto>");
		return CMD_RET_SUCCESS;
	}

	if (argc != 3)
		return CMD_RET_USAGE;

	current_bridge_layout = PPE_BRIDGE_CTRL_LAYOUT_UNKNOWN;
	if (!strcmp(argv[2], "auto")) {
		env_set("nss_bridge_layout", NULL);
	} else if (!strcmp(argv[2], "v2")) {
		env_set("nss_bridge_layout", "v2");
	} else if (!strcmp(argv[2], "v4")) {
		env_set("nss_bridge_layout", "v4");
	} else {
		return CMD_RET_USAGE;
	}

	printf("NSS bridge layout set to %s (txmac=0x%08x promisc=0x%08x isolate=0x%08x)\n",
	       ppe_bridge_ctrl_layout_name(), ppe_port_bridge_txmac_mask(),
	       ppe_port_bridge_promisc_mask(),
	       ppe_port_bridge_isolation_mask(ipq_active_ppe ?
					      ipq_active_ppe->nos_iports : 0));

	return CMD_RET_SUCCESS;
}

static int do_nss_debug(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	struct ipq_eth_dev *priv;
	ulong seconds = 10;
	ulong start;
	ulong last = 0;
	bool do_start = false;
	bool do_poll = false;
	int ret;

	if (argc > 1) {
		if (!strcmp(argv[1], "start")) {
			do_start = true;
		} else if (!strcmp(argv[1], "detail")) {
			do_start = true;
		} else if (!strcmp(argv[1], "summary")) {
			do_start = true;
		} else if (!strcmp(argv[1], "poll")) {
			do_start = true;
			do_poll = true;
			if (argc > 2)
				seconds = simple_strtoul(argv[2], NULL, 10);
			if (!seconds)
				seconds = 10;
		} else if (!strcmp(argv[1], "stop")) {
			eth_halt();
			printf("nss-switch stopped\n");
			return CMD_RET_SUCCESS;
		} else if (strcmp(argv[1], "port") &&
			   strcmp(argv[1], "ports") &&
			   strcmp(argv[1], "queue") &&
			   strcmp(argv[1], "gmac") &&
			   strcmp(argv[1], "uniphy") &&
			   strcmp(argv[1], "peek") &&
			   strcmp(argv[1], "poke") &&
			   strcmp(argv[1], "phy") &&
			   strcmp(argv[1], "mdio") &&
			   strcmp(argv[1], "qca8075") &&
			   strcmp(argv[1], "qca8") &&
			   strcmp(argv[1], "qca8075cnt") &&
			   strcmp(argv[1], "qca8cnt") &&
			   strcmp(argv[1], "qca8075loop") &&
			   strcmp(argv[1], "qca8loop") &&
			   strcmp(argv[1], "qca8081") &&
			   strcmp(argv[1], "qca81") &&
			   strcmp(argv[1], "qid2rid") &&
			   strcmp(argv[1], "setqid") &&
			   strcmp(argv[1], "setqidreg") &&
			   strcmp(argv[1], "txctrl") &&
			   strcmp(argv[1], "txidx") &&
			   strcmp(argv[1], "txmap") &&
			   strcmp(argv[1], "txregs") &&
			   strcmp(argv[1], "counters") &&
			   strcmp(argv[1], "snapshot") &&
			   strcmp(argv[1], "rearm") &&
			   strcmp(argv[1], "txfix") &&
			   strcmp(argv[1], "txkick") &&
			   strcmp(argv[1], "txmode") &&
			   strcmp(argv[1], "trace") &&
			   strcmp(argv[1], "bridge_layout")) {
			printf("Unknown nss_debug subcommand '%s'\n", argv[1]);
			return CMD_RET_FAILURE;
		}
	}

	if (argc > 1 && !strcmp(argv[1], "bridge_layout"))
		return ipq_eth_debug_bridge_layout(NULL, argc, argv);
	if (argc > 1 && !strcmp(argv[1], "trace"))
		return ipq_eth_debug_trace_cmd(NULL, argc, argv);
	if (argc > 1 && !strcmp(argv[1], "txmode"))
		return ipq_eth_debug_txmode(NULL, argc, argv);

	if (do_start) {
		ret = net_lwip_eth_start();
		if (ret < 0) {
			printf("Failed to start Ethernet: %d\n", ret);
			return CMD_RET_FAILURE;
		}
	}

	priv = (argc > 1 && (!strcmp(argv[1], "snapshot") ||
			  !strcmp(argv[1], "counters"))) ?
	       ipq_eth_debug_get_priv_relaxed(false) :
	       ipq_eth_debug_get_priv();
	if (!priv)
		return CMD_RET_FAILURE;

	if (argc > 1) {
		if (!strcmp(argv[1], "detail")) {
			ipq_eth_debug_dump_detail(priv);
			return CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "summary")) {
			ipq_eth_debug_dump_summary(priv, "cmd");
			return CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "port")) {
			u32 port_id;

			if (argc != 3)
				return CMD_RET_USAGE;
			port_id = simple_strtoul(argv[2], NULL, 0);
			ipq_eth_debug_dump_port_detail(priv, port_id);
			return CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "ports")) {
			ipq_eth_debug_dump_ports(priv);
			return CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "queue")) {
			u32 qid;

			if (argc != 3)
				return CMD_RET_USAGE;
			qid = simple_strtoul(argv[2], NULL, 0);
			ipq_eth_debug_dump_queue_detail(priv, qid);
			return CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "gmac")) {
			u32 port_id;

			if (argc != 3)
				return CMD_RET_USAGE;
			port_id = simple_strtoul(argv[2], NULL, 0);
			ipq_eth_debug_dump_gmac(priv, port_id);
			return CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "uniphy")) {
			u32 uniphy_id;

			if (argc != 3)
				return CMD_RET_USAGE;
			uniphy_id = simple_strtoul(argv[2], NULL, 0);
			ipq_eth_debug_dump_uniphy(priv, uniphy_id);
			return CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "peek"))
			return ipq_eth_debug_peek(priv, argc, argv);
		if (!strcmp(argv[1], "poke"))
			return ipq_eth_debug_poke(priv, argc, argv);
		if (!strcmp(argv[1], "phy"))
			return ipq_eth_debug_phy(priv, argc, argv);
		if (!strcmp(argv[1], "mdio"))
			return ipq_eth_debug_mdio(priv, argc, argv);
		if (!strcmp(argv[1], "qca8075") || !strcmp(argv[1], "qca8"))
			return ipq_eth_debug_qca8075(priv, argc, argv);
		if (!strcmp(argv[1], "qca8075cnt") || !strcmp(argv[1], "qca8cnt"))
			return ipq_eth_debug_qca8075_cnt(priv, argc, argv);
		if (!strcmp(argv[1], "qca8075loop") ||
		    !strcmp(argv[1], "qca8loop"))
			return ipq_eth_debug_qca8075_loop(priv, argc, argv);
		if (!strcmp(argv[1], "qca8081") || !strcmp(argv[1], "qca81"))
			return ipq_eth_debug_qca8081(priv, argc, argv);
		if (!strcmp(argv[1], "qid2rid"))
			return ipq_eth_debug_qid2rid(priv, argc, argv);
		if (!strcmp(argv[1], "setqid"))
			return ipq_eth_debug_setqid(priv, argc, argv);
		if (!strcmp(argv[1], "setqidreg"))
			return ipq_eth_debug_setqidreg(priv, argc, argv);
		if (!strcmp(argv[1], "txctrl"))
			return ipq_eth_debug_txctrl(priv, argc, argv);
		if (!strcmp(argv[1], "txidx"))
			return ipq_eth_debug_txidx(priv, argc, argv);
		if (!strcmp(argv[1], "txmap"))
			return ipq_eth_debug_txmap(priv, argc, argv);
		if (!strcmp(argv[1], "txregs")) {
			ipq_edma_debug_dump_tx_regs(priv, "cmd");
			return CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "counters")) {
			u32 port_id = 0;

			if (argc > 3)
				return CMD_RET_USAGE;
			if (argc == 3)
				port_id = simple_strtoul(argv[2], NULL, 0);
			return ipq_eth_debug_dump_counters(priv, port_id) ?
				CMD_RET_FAILURE : CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "snapshot")) {
			u32 port_id = 0;

			if (argc > 3)
				return CMD_RET_USAGE;
			if (argc == 3)
				port_id = simple_strtoul(argv[2], NULL, 0);
			ipq_eth_debug_dump_snapshot(priv, port_id);
			return CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "rearm")) {
			ipq_eth_debug_dump_summary(priv, "before-rearm");
			ipq_edma_rearm_rx_path(&priv->hw);
			ipq_edma_rearm_tx_path(&priv->hw);
			ipq_eth_debug_dump_summary(priv, "after-rearm");
			return CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "txfix")) {
			ipq_edma_debug_dump_tx_regs(priv, "before-fix");
			ipq_edma_debug_reprogram_tx(priv);
			ipq_edma_debug_dump_tx_regs(priv, "after-fix");
			return CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "txkick")) {
			bool toggle_en = argc > 2 &&
					 (!strcmp(argv[2], "toggle") ||
					  !strcmp(argv[2], "en"));

			ipq_edma_debug_kick_tx(priv, toggle_en);
			return CMD_RET_SUCCESS;
		}
		if (!strcmp(argv[1], "txmode"))
			return ipq_eth_debug_txmode(priv, argc, argv);
		if (!strcmp(argv[1], "trace"))
			return ipq_eth_debug_trace_cmd(priv, argc, argv);
		if (!strcmp(argv[1], "bridge_layout"))
			return ipq_eth_debug_bridge_layout(priv, argc, argv);
	}

	if (do_poll)
		ipq_eth_debug_dump_summary(priv, "poll-start");
	else
		ipq_eth_debug_dump(priv, "cmd");
	if (!do_poll)
		return CMD_RET_SUCCESS;

	start = get_timer(0);
	while (get_timer(start) < seconds * 1000) {
		if (ctrlc()) {
			printf("Abort by user\n");
			break;
		}

		eth_rx();
		if (!last || get_timer(last) >= 1000) {
			last = get_timer(0);
			ipq_eth_debug_dump_summary(priv, "poll-cmd");
		}
		mdelay(10);
	}

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(nss_debug, 8, 1, do_nss_debug,
	   "Dump or poll Qualcomm NSS switch/EDMA state",
	   "[start|summary|detail|poll [seconds]|stop]\n"
	   "  start          - start Ethernet and dump state\n"
	   "  summary        - start Ethernet and dump compact counters\n"
	   "  detail         - start Ethernet and dump PPE/EDMA detail\n"
	   "  poll [seconds] - start Ethernet, poll RX, and dump state once per second\n"
	   "  stop           - stop Ethernet\n"
	   "  port <ppe-id>  - dump one PPE port and its queue\n"
	   "  ports          - dump parsed DT port/PHY/PCS mapping\n"
	   "  queue <qid>    - dump one PPE unicast queue\n"
	   "  gmac <ppe-id>  - dump GMAC/XGMAC and MAC RX/TX counters\n"
	   "  uniphy <id>    - dump UNIPHY direct/CSR status\n"
	   "  peek <ppe|edma|uniphy> <off> [count] - read raw registers\n"
	   "  poke <ppe|edma|uniphy> <off> <value> - write one raw register\n"
	   "  phy <ppe-id|slot> [devad reg] - read one PHY register\n"
	   "  mdio <phyaddr> [devad] <reg> [value] - read/write one MDIO register\n"
	   "  qca8075 [base-phyaddr] [ctrl] - dump QCA8075 lanes; ctrl also reads package registers\n"
	   "  qca8075cnt [base-phyaddr] [read|enable|disable|clear] - QCA8075 packet counters\n"
	   "  qca8075loop [base-phyaddr] <lane> <on|off> [cutrx] - QCA8075 remote loopback\n"
	   "  qca8081 [phyaddr] - dump QCA8081 status and selected MMD registers\n"
	   "  qid2rid [qid] [count] - decode EDMA QID to RX ring table\n"
	   "  setqid <qid> <rid> - set one QID2RID lane for this boot only\n"
	   "  setqidreg <reg-index> <value> - set one raw QID2RID register\n"
	   "  txctrl [ring] [value] - dump or write one TXDESC_CTRL register\n"
	   "  txidx <txdesc-prod|txdesc-cons|txcmpl-prod|txcmpl-cons> [ring] [value] - dump or write a TX ring index\n"
	   "  txmap [txdesc-ring txcmpl-ring] - dump or set TXDESC to TXCMPL mapping\n"
	   "  txregs        - dump EDMA TX ring/global registers\n"
	   "  counters [ppe-id] - sample compact EDMA/PPE/MAC counters and show deltas\n"
	   "  snapshot [ppe-id] - dump ports, summary, TX regs, PPE port, GMAC, and UNIPHY\n"
	   "  rearm         - reset EDMA RX/TX rings to ring-size-local indices\n"
	   "  txfix         - reprogram EDMA TX rings/map/global registers\n"
	   "  txkick [toggle] - rewrite TX producer and optionally toggle TX_EN\n"
	   "  txmode defaults | txmode port <value> | txmode dmar <v1|v3> | txmode txctrlbase <qsdk|tso|raw> | txmode tdes4 <legacy|srcdst|dst|raw> | txmode passthrough <on|off> - set TX debug mode; defaults match QSDK\n"
	   "  trace [on|off] - enable or disable verbose EDMA frame/ring logs\n"
	   "  bridge_layout [auto|v2|v4] - set PORT_BRIDGE_CTRL mask layout; auto defaults to QSDK v2\n"
	   "  env active_port unset selects all ports; set value filters QSDK slots or ethernet-ports reg\n"
	   "      active_port_id=<ppe-id> filters by PPE port");

#ifdef CONFIG_PHY_AQUANTIA
static int do_aqloadfw(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	static struct udevice *dev;
	static struct ipq_eth_dev *priv;
	int i;
	bool reload = false;
	bool debug = false;
	u8 phyaddr;

	if (argc < 2 || argc > 3)
		return CMD_RET_USAGE;

	if (argc == 3) {
		char opt = argv[2][0];
		if (opt == 'r')
			reload = true;
		else if (opt == 'd')
			debug = true;
		else
			return CMD_RET_USAGE;
	}

	phyaddr = simple_strtoul(argv[1], NULL, 16);

#if defined(CONFIG_CMD_NET) && defined(CONFIG_ETH_SKIP_INIT_R)
	initr_net();
#endif

	if (!dev) {
		dev = eth_get_dev_by_name("nss-switch");
		if (!dev) {
			printf("Failed to find nss-switch device\n");
			return CMD_RET_FAILURE;
		}
	}

	if (!priv) {
		priv = dev_get_priv(dev);
		if (!priv) {
			printf("Failed to find priv pointer\n");
			return CMD_RET_FAILURE;
		}
	}

	for (i = 0; i < CONFIG_ETH_MAX_MAC; ++i) {
		struct port_info *port = priv->port[i];

		if (!port)
			continue;

		if (port->phy_id != AQ_PHY_TYPE)
			continue;

		if (port->phyaddr != phyaddr)
			continue;

		if (reload)
			port->fw_loaded = false;

		if (port->fw_loaded) {
			if (debug)
				printf("AQ port %d FW already loaded\n",
					phyaddr);
			continue;
		}

		if (!ipq_aquantia_load_fw(port->phydev)) {
			port->fw_loaded = true;
			mdelay(100);
		}

		break;
	}

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(aq_load_fw, 3, 0, do_aqloadfw,
	   "Load firmware to AQ port",
	   "phy_addr --> phy address of AQ port\n"
	   "[r|d] - Optional: 'r' for reload, 'd' for debug logs\n");
#endif /* CONFIG_PHY_AQUANTIA */
