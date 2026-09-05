// SPDX-License-Identifier: GPL-2.0+
/*
 * Minimal HTTP upload recovery server using lwIP httpd
 * Serves a tiny upload page at / and handles POST to /upload/{firmware|uboot}
 */

#include <dm.h>
#include <dm/ofnode.h>
#include <blk.h>
#include <env.h>
#include <image.h>
#include <log.h>
#include <malloc.h>
#include <memalign.h>
#include <command.h>
#include <mmc.h>
#include <mtd.h>
#include <net-lwip.h>
#include <net.h>
#include <part.h>
#include <pwm.h>
#include <initcall.h>
#include <ubi_uboot.h>
#include <watchdog.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/libfdt.h>
#include <console.h>
#include <asm/global_data.h>
#include <dm/pinctrl.h>
#include <dm/uclass.h>
#include <button.h>
#include <smem.h>

#ifdef crc32
#undef crc32
#endif
#include <u-boot/crc.h>
#include <u-boot/sha256.h>

#include <lwip/netif.h>
#include <lwip/pbuf.h>
#include <lwip/ip.h>
#include <lwip/timeouts.h>
#include <lwip/udp.h>
#include <lwip/apps/httpd.h>
#include <lwip/apps/fs.h>
#include <lwip/prot/dhcp.h>
#include <lwip/prot/iana.h>
#include <version.h>
#include <timestamp.h>
#include <limits.h>
#include <vsprintf.h>
#include <miiphy.h>
#include <linux/mii.h>
#include <mtd/ubi-user.h>
#include <timer.h>
#include <asm/io.h>
#include <asm/unaligned.h>
#include "../../drivers/mtd/ubi/ubi.h"
#include <stdarg.h>

DECLARE_GLOBAL_DATA_PTR;

__weak int xr1710g_sync_factory(void)
{
	return 0;
}

__weak void recovery_board_watchdog_kick(void)
{
}

__weak void recovery_board_http_acl(bool enable)
{
}

static void recovery_watchdog_poll(void)
{
	recovery_board_watchdog_kick();
	WATCHDOG_RESET();
}

static bool recovery_debug_enabled(void)
{
	return env_get_yesno("recovery_debug") == 1;
}

static void __printf(1, 2) recovery_debug_printf(const char *fmt, ...)
{
	va_list ap;

	if (!recovery_debug_enabled())
		return;

	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
}

static bool recovery_board_is_sbe1v1k(void)
{
	return ofnode_device_is_compatible(ofnode_root(), "askey,sbe1v1k");
}

/*
 * Upload buffer
 * Use env 'recovery_addr' if set, otherwise fall back to U-Boot 'loadaddr'.
 * Avoid hard-coding a RAM address which may overlap U-Boot/lwIP memory.
 */
/* Default maximum upload size in bytes (override with env 'recovery_max') */
#define RECOVERY_UPLOAD_MAX    (32 * 1024 * 1024UL)
#define RECOVERY_MIN_FIRMWARE_SIZE (1 * 1024 * 1024UL)
#define RECOVERY_MAX_UBOOT_SIZE    (1 * 1024 * 1024UL)
#define RECOVERY_MAX_UBOOT_SIZE_MMC (4 * 1024 * 1024UL)
#define RECOVERY_KERNEL_PAD_SIZE    (7000 * 1024UL)
#define RECOVERY_MMC_WRITE_CHUNK    (1024 * 1024UL)
#define RECOVERY_MMC_STREAM_CHUNK   (1024 * 1024UL)
#define RECOVERY_MMC_BACKUP_CHUNK   (16 * 1024 * 1024UL)
#define RECOVERY_MMC_RESTORE_CHUNK  (16 * 1024 * 1024UL)
#define RECOVERY_MMC_ERASE_CHUNK    (32 * 1024 * 1024ULL)
#define RECOVERY_MMC_ERASE_PROGRESS_STEPS 1000U
#define RECOVERY_SQUASHFS_MAGIC      0x73717368U
#define RECOVERY_SQUASHFS_MAJOR      4U
#define RECOVERY_SQUASHFS_MAJOR_OFF  28U
#define RECOVERY_SQUASHFS_BYTES_OFF  40U
#define RECOVERY_SQUASHFS_HEADER_MIN 48U
#define RECOVERY_REPARTITION_MAX    64UL
#define RECOVERY_RESTORE_BODY_MAX   96UL
#define RECOVERY_RESTORE_ID_MAX     32U
#define RECOVERY_SBE1V1K_GPT_MAX    12288UL
#define RECOVERY_SBE1V1K_CHAINLOADER_START 110626ULL
#define RECOVERY_SBE1V1K_CHAINLOADER_SIZE  8192ULL
#define RECOVERY_SBE1V1K_KERNEL_START      118818ULL
#define RECOVERY_SBE1V1K_KERNEL_SIZE       65536ULL
#define RECOVERY_SBE1V1K_ROOTFS_START      184354ULL
#define RECOVERY_SBE1V1K_ROOTFS_SIZE       2097152ULL
#define RECOVERY_SBE1V1K_DATA_START        2281506ULL
#define RECOVERY_SBE1V1K_MAINLINE_KERNEL_START 81954ULL
#define RECOVERY_SBE1V1K_MAINLINE_KERNEL_SIZE  14336ULL
#define RECOVERY_SBE1V1K_MAINLINE_ROOTFS_START 110626ULL
#define RECOVERY_SBE1V1K_MAINLINE_ROOTFS_SIZE  249856ULL
#define RECOVERY_SBE1V1K_MAINLINE_DATA_START   610338ULL
#define RECOVERY_SBE1V1K_MAINLINE_DATA_SIZE    1048576ULL
#define RECOVERY_SBE1V1K_MAINLINE_UBOOT_START  5201954ULL
#define RECOVERY_SBE1V1K_MAINLINE_UBOOT_SIZE   65536ULL
#define RECOVERY_SBE1V1K_HLOS_START            81954ULL
#define RECOVERY_SBE1V1K_HLOS_SIZE             14336ULL
#define RECOVERY_SBE1V1K_HLOS_1_START          96290ULL
#define RECOVERY_SBE1V1K_HLOS_1_SIZE           14336ULL
#define RECOVERY_SBE1V1K_HLOS_1_TYPE_GUID      "A71DA577-7F81-4626-B4A2-E377F9174525"
#define RECOVERY_SBE1V1K_FIT_TFTP_ADDR         0x80000000UL
#define RECOVERY_SBE1V1K_FIT_PERSISTENT_ADDR   0x44000000UL
#define RECOVERY_GPT_TYPE_BASIC_DATA       "EBD0A0A2-B9E5-4433-87C0-68B6B72699C7"
#define RECOVERY_GPT_TYPE_LINUX_FS         "0FC63DAF-8483-4772-8E79-3D69D8477DE4"

/* Delay before reboot after flashing completes, to let browser finish reads */
#define REBOOT_DELAY_MS        3000
#define FLASH_START_DELAY_MS   3000
#define RECOVERY_STATIC_IPADDR           "192.168.255.1"
#define RECOVERY_STATIC_NETMASK          "255.255.255.0"
#define RECOVERY_STATIC_GATEWAY          "0.0.0.0"
#define RECOVERY_DHCP_CLIENT_IPADDR      "192.168.255.2"
#define RECOVERY_DHCP_BROADCAST_IPADDR   "192.168.255.255"
#define RECOVERY_DHCP_LEASE_SECS         86400U
#define RECOVERY_DHCP_MAX_MSG_LEN        1500
#define RECOVERY_LED_PORTS     2
#define RECOVERY_LED_POLL_MS   100
#define RECOVERY_STATUS_LED_MAX           8
#define RECOVERY_STATUS_PWM_PERIOD_NS     1000000U
#define RECOVERY_STATUS_HW_PWM_UPDATE_MS  20
#define RECOVERY_STATUS_BREATHE_PERIOD_MS 3200
#define RECOVERY_STATUS_BREATHE_HALF_MS   (RECOVERY_STATUS_BREATHE_PERIOD_MS / 2)
#define RECOVERY_STATUS_SWEEP_STEP_MS     1400
#define RECOVERY_STATUS_OVERLAP_FP        (2 * 256)
#define RECOVERY_STATUS_BRIGHTNESS_FP     256
#define RECOVERY_STATUS_BREATHE_MIN_FP    64
#define RECOVERY_STATUS_PWM_NODE          "/recovery-status-pwm-leds"
#define RECOVERY_PWM_POLARITY_INVERTED    BIT(0)

#define RECOVERY_GPIO_SYSCTL_BASE      0x1fbf0200
#define RECOVERY_CHIP_SCU_BASE         0x1fa20000
#define RECOVERY_REG_GPIO_DATA         0x0004
#define RECOVERY_REG_GPIO_OE           0x0014
#define RECOVERY_REG_GPIO_CTRL         0x0000
#define RECOVERY_REG_GPIO_CTRL1        0x0020
#define RECOVERY_REG_GPIO_FLASH_MODE_CFG 0x0034
#define RECOVERY_REG_GPIO_CTRL2        0x0060
#define RECOVERY_REG_GPIO_CTRL3        0x0064
#define RECOVERY_REG_GPIO_DATA1        0x0070
#define RECOVERY_REG_GPIO_OE1          0x0078
#define RECOVERY_REG_GPIO_2ND_I2C_MODE 0x0214
#define RECOVERY_REG_GPIO_FLASH_MODE_CFG_EXT 0x0068

#define RECOVERY_GPIO_LAN0_LED0_MODE_MASK BIT(3)
#define RECOVERY_GPIO_LAN0_LED1_MODE_MASK BIT(4)
#define RECOVERY_GPIO_LAN1_LED0_MODE_MASK BIT(5)
#define RECOVERY_GPIO_LAN1_LED1_MODE_MASK BIT(6)
#define RECOVERY_GPIO43_FLASH_MODE_CFG BIT(23)
#define RECOVERY_GPIO44_FLASH_MODE_CFG BIT(24)
#define RECOVERY_UBOOTENV_SIZE (1 * 1024 * 1024UL)
#define RECOVERY_FACTORY_SIZE  (1 * 1024 * 1024UL)
#define RECOVERY_UBI_WRITE_CHUNK (1024 * 1024U)
#define RECOVERY_APPSBLENV_PART "0#0:APPSBLENV"
#define RECOVERY_APPSBLENV_SIZE (256 * 1024UL)
#define RECOVERY_ENV_CRC_SIZE   4
#define RECOVERY_SBE1V1K_REPARTITION_TOKEN "SBE1V1K_REPARTITION"
#define RECOVERY_SBE1V1K_RESTORE_TOKEN "SBE1V1K_RESTORE"
#define RECOVERY_SBE1V1K_CHAINLOADER_PART "0#chainloader"
#define RECOVERY_SBE1V1K_FACTORY_UBOOT_PART "0#rsvd_2"
#define RECOVERY_SBE1V1K_LAYOUT_ENV "sbe1v1k_partition_profile"
#define RECOVERY_QWRT_AUTH_CODE_LEN (SHA256_SUM_LEN * 2U)
#define RECOVERY_QWRT_SERIAL_MAX 32U
#define RECOVERY_QWRT_ENV_SCAN_BYTES 65535U
#define RECOVERY_QWRT_SMEM_SOCINFO_ITEM 137U
#define RECOVERY_QWRT_SMEM_SERIAL_OFF 96U
#define RECOVERY_QWRT_SMEM_SERIAL_SIZE 4U
#define RECOVERY_QWRT_BUTTON_PRESSES 5U
#define RECOVERY_QWRT_BUTTON_WINDOW_MS 10000UL
#define RECOVERY_PARTITIONS_JSON_MAX (32 * 1024UL)
#define RECOVERY_BACKUP_MAGIC         0x52424b50U
#define RECOVERY_BACKUP_NAME_MAX      100U
#define RECOVERY_TAR_BLOCK_SIZE       512ULL
#define RECOVERY_TAR_END_SIZE         (2 * RECOVERY_TAR_BLOCK_SIZE)
#define RECOVERY_TAR_MAX_MEMBER_SIZE  ((8ULL * 1024 * 1024 * 1024) - 1)
#define RECOVERY_TAR_CHKSUM_OFF        148U
#define RECOVERY_TAR_CHKSUM_LEN        8U
#define RECOVERY_SBE1V1K_CONTROL_MAX   256U
#define RECOVERY_PREPARE_BODY_MAX     32U
#define RECOVERY_PREPARE_START_DELAY_MS 100U

enum recovery_backup_mode {
	RECOVERY_BACKUP_SINGLE = 0,
	RECOVERY_BACKUP_ALL_TAR,
};

enum recovery_backup_tar_phase {
	RECOVERY_BACKUP_TAR_MEMBER_HEADER = 0,
	RECOVERY_BACKUP_TAR_MEMBER_DATA,
	RECOVERY_BACKUP_TAR_MEMBER_PADDING,
	RECOVERY_BACKUP_TAR_END,
	RECOVERY_BACKUP_TAR_DONE,
};

struct recovery_backup_source {
	int hwpart;
	int number;
	lbaint_t start;
	lbaint_t blocks;
	ulong blksz;
	unsigned long long bytes;
	char name[PART_NAME_LEN + 1];
	char filename[RECOVERY_BACKUP_NAME_MAX];
};

struct recovery_backup_tar_header {
	char name[100];
	char mode[8];
	char uid[8];
	char gid[8];
	char size[12];
	char mtime[12];
	char checksum[8];
	char typeflag;
	char linkname[100];
	char magic[6];
	char version[2];
	char uname[32];
	char gname[32];
	char devmajor[8];
	char devminor[8];
	char prefix[155];
	char padding[12];
} __packed;

struct recovery_backup_file {
	u32 magic;
	enum recovery_backup_mode mode;
	struct blk_desc *desc;
	struct recovery_backup_source source;
	unsigned int source_cursor;
	lbaint_t next_lba;
	lbaint_t blocks_left;
	ulong blksz;
	u8 *cache;
	size_t cache_capacity;
	size_t cache_len;
	size_t cache_off;
	char http_header[320];
	size_t http_header_len;
	size_t http_header_off;
	struct recovery_backup_tar_header tar_header;
	size_t tar_header_off;
	unsigned long long tar_padding_left;
	unsigned long long tar_end_left;
	enum recovery_backup_tar_phase tar_phase;
	bool failed;
	bool active_counted;
	bool user_restored;
};

static u8 *recv_base;
static u32 recv_off;
static u32 recv_total;
static int post_ok;
static int upload_done;
static void *post_connection;
static struct recovery_status_led_ctrl *recovery_runtime_status_leds;
static int flash_request;
static int prepare_request;
static int restore_prepare_request;
static volatile int reboot_request;
static unsigned int recovery_backup_active;
/* Progress for /status polling */
static volatile u32 prog_total; /* combined total for backward compat */
static volatile u32 prog_done;  /* combined done for backward compat */
static volatile u32 prog_erase_total;
static volatile u32 prog_erase_done;
static volatile u32 prog_write_total;
static volatile u32 prog_write_done;
/* Phase 4 extends the original progress states with a prepared stream. */
static volatile int prog_phase; /* 0 idle, 1 erase, 2 write, 3 done, -1 error */
static volatile int prog_reboot;
static unsigned long long prog_erase_volume_base;
static unsigned long long prog_erase_volume_bytes;
static struct recovery_status_led_ctrl *prog_status_leds;

struct recovery_restore_state {
	bool active;
	bool prepared;
	bool receiving;
	struct blk_desc *desc;
	struct recovery_backup_source source;
	u8 *buf;
	size_t buf_size;
	size_t buf_used;
	unsigned long long total;
	unsigned long long written;
	unsigned long long chunk_offset;
	unsigned int chunk_received;
	char id[RECOVERY_RESTORE_ID_MAX];
};

static struct recovery_restore_state recovery_restore;
static bool current_restore_prepare;
static bool current_restore_chunk;
static char current_restore_id[RECOVERY_RESTORE_ID_MAX];
static unsigned long long current_restore_size;
static unsigned long long current_restore_offset;
static u8 current_restore_body[RECOVERY_RESTORE_BODY_MAX + 1];
static char restore_prepare_id[RECOVERY_RESTORE_ID_MAX];
static unsigned long long restore_prepare_size;

static void post_delay_cb(void *arg)
{
    (void)arg;
    flash_request = 1;
}

static void prepare_delay_cb(void *arg)
{
	(void)arg;
	prepare_request = 1;
}

static void restore_prepare_delay_cb(void *arg)
{
	(void)arg;
	restore_prepare_request = 1;
}

static void reboot_delay_cb(void *arg)
{
    (void)arg;
    reboot_request = 1;
}

static void recovery_prepare_static_network(void)
{
	env_set("ipaddr", RECOVERY_STATIC_IPADDR);
	env_set("netmask", RECOVERY_STATIC_NETMASK);
	env_set("gatewayip", RECOVERY_STATIC_GATEWAY);
}

enum upload_target {
	TARGET_FIRMWARE = 0,
	TARGET_UBOOT,
	TARGET_REPARTITION,
};

enum recovery_stream_format {
	RECOVERY_STREAM_RAW = 0,
	RECOVERY_STREAM_TAR,
};

enum recovery_sbe1v1k_layout {
	RECOVERY_SBE1V1K_LAYOUT_UNKNOWN = 0,
	RECOVERY_SBE1V1K_LAYOUT_MAINLINE,
	RECOVERY_SBE1V1K_LAYOUT_LARGE,
	RECOVERY_SBE1V1K_LAYOUT_QWRT,
};

struct recovery_sbe1v1k_layout_desc {
	enum recovery_sbe1v1k_layout id;
	const char *name;
	const char *kernel_part;
	const char *rootfs_part;
	const char *data_part;
	const char *uboot_part;
	const char *kernpart;
	const char *rootarg;
	lbaint_t kernel_start;
	lbaint_t kernel_size;
	lbaint_t rootfs_start;
	lbaint_t rootfs_size;
	lbaint_t data_start;
	lbaint_t data_size;
	lbaint_t uboot_start;
	lbaint_t uboot_size;
	size_t kernel_pad;
};

static const struct recovery_sbe1v1k_layout_desc sbe1v1k_layout_mainline = {
	.id = RECOVERY_SBE1V1K_LAYOUT_MAINLINE,
	.name = "mainline",
	.kernel_part = "0#0:HLOS",
	.rootfs_part = "0#rootfs",
	.data_part = "0#rootfs_data",
	.uboot_part = "0#rsvd_2",
	.kernpart = "0:HLOS",
	.rootarg = "/dev/mmcblk0p27",
	.kernel_start = RECOVERY_SBE1V1K_MAINLINE_KERNEL_START,
	.kernel_size = RECOVERY_SBE1V1K_MAINLINE_KERNEL_SIZE,
	.rootfs_start = RECOVERY_SBE1V1K_MAINLINE_ROOTFS_START,
	.rootfs_size = RECOVERY_SBE1V1K_MAINLINE_ROOTFS_SIZE,
	.data_start = RECOVERY_SBE1V1K_MAINLINE_DATA_START,
	.data_size = RECOVERY_SBE1V1K_MAINLINE_DATA_SIZE,
	.uboot_start = RECOVERY_SBE1V1K_MAINLINE_UBOOT_START,
	.uboot_size = RECOVERY_SBE1V1K_MAINLINE_UBOOT_SIZE,
	.kernel_pad = RECOVERY_SBE1V1K_MAINLINE_KERNEL_SIZE * 512,
};

static const struct recovery_sbe1v1k_layout_desc sbe1v1k_layout_large = {
	.id = RECOVERY_SBE1V1K_LAYOUT_LARGE,
	.name = "large",
	.kernel_part = "0#kernel",
	.rootfs_part = "0#rootfs",
	.data_part = "0#rootfs_data",
	.uboot_part = RECOVERY_SBE1V1K_CHAINLOADER_PART,
	.kernpart = "kernel",
	.rootarg = "PARTLABEL=rootfs",
	.kernel_start = RECOVERY_SBE1V1K_KERNEL_START,
	.kernel_size = RECOVERY_SBE1V1K_KERNEL_SIZE,
	.rootfs_start = RECOVERY_SBE1V1K_ROOTFS_START,
	.rootfs_size = RECOVERY_SBE1V1K_ROOTFS_SIZE,
	.data_start = RECOVERY_SBE1V1K_DATA_START,
	.data_size = 0,
	.uboot_start = RECOVERY_SBE1V1K_CHAINLOADER_START,
	.uboot_size = RECOVERY_SBE1V1K_CHAINLOADER_SIZE,
	.kernel_pad = RECOVERY_SBE1V1K_KERNEL_SIZE * 512,
};

/* QWRT follows the factory/"wipe" firmware map, with its own auth contract. */
static const struct recovery_sbe1v1k_layout_desc sbe1v1k_layout_qwrt = {
	.id = RECOVERY_SBE1V1K_LAYOUT_QWRT,
	.name = "qwrt",
	.kernel_part = "0#0:HLOS",
	.rootfs_part = "0#rootfs",
	.data_part = "0#rootfs_data",
	.uboot_part = RECOVERY_SBE1V1K_FACTORY_UBOOT_PART,
	.kernpart = "0:HLOS",
	.rootarg = "/dev/mmcblk0p27",
	.kernel_start = RECOVERY_SBE1V1K_MAINLINE_KERNEL_START,
	.kernel_size = RECOVERY_SBE1V1K_MAINLINE_KERNEL_SIZE,
	.rootfs_start = RECOVERY_SBE1V1K_MAINLINE_ROOTFS_START,
	.rootfs_size = RECOVERY_SBE1V1K_MAINLINE_ROOTFS_SIZE,
	.data_start = RECOVERY_SBE1V1K_MAINLINE_DATA_START,
	.data_size = RECOVERY_SBE1V1K_MAINLINE_DATA_SIZE,
	.uboot_start = RECOVERY_SBE1V1K_MAINLINE_UBOOT_START,
	.uboot_size = RECOVERY_SBE1V1K_MAINLINE_UBOOT_SIZE,
	.kernel_pad = RECOVERY_SBE1V1K_MAINLINE_KERNEL_SIZE * 512,
};

static enum upload_target current_target = TARGET_FIRMWARE;
static bool current_force_recreate;
static bool current_prepare_only;
static enum upload_target prepare_target = TARGET_FIRMWARE;
static size_t prepare_size;
static enum recovery_stream_format current_stream_format = RECOVERY_STREAM_RAW;
static enum recovery_stream_format prepare_stream_format = RECOVERY_STREAM_RAW;
static enum recovery_sbe1v1k_layout current_repartition_layout =
	RECOVERY_SBE1V1K_LAYOUT_LARGE;
static enum recovery_sbe1v1k_layout active_sbe1v1k_layout =
	RECOVERY_SBE1V1K_LAYOUT_UNKNOWN;
/* A recognised factory GPT is safe to migrate, but not safe to flash in place. */
static bool sbe1v1k_factory_pre_migration;

/* QWRT is an explicit, session-only capability. */
static bool qwrt_button_unlocked;
static unsigned int qwrt_button_presses;
static ulong qwrt_button_last_press;
static enum button_state_t qwrt_button_last_state;
static struct udevice *qwrt_button;
static bool qwrt_button_available;

static int recovery_read_sbe1v1k_layout_marker(
	enum recovery_sbe1v1k_layout *layout);
static int recovery_write_appsblenv(
	struct recovery_status_led_ctrl *status_leds, size_t progress_base,
	const struct recovery_sbe1v1k_layout_desc *layout);
static int recovery_refresh_qwrt_auth_code(void);

enum recovery_backend {
	RECOVERY_BACKEND_MTD = 0,
	RECOVERY_BACKEND_UBI,
	RECOVERY_BACKEND_MMC,
};

struct recovery_target {
	enum recovery_backend backend;
	const char *name;
	const char *ubi_part;
	struct mtd_info *mtd;
	struct blk_desc *blk;
	struct disk_partition part;
	loff_t ofs;
	unsigned long long cur_size;
	unsigned long long limit;
	bool ubi_needs_format;
};

struct recovery_led_ctrl {
	struct udevice *mdio_dev;
	ulong last_poll;
};

struct recovery_status_pwm_led {
	struct udevice *pwm;
	uint channel;
	uint period_ns;
	bool active_low;
	bool valid;
};

struct recovery_status_led_ctrl {
	struct recovery_status_pwm_led pwm_leds[RECOVERY_STATUS_LED_MAX];
	int pwm_count;
	bool use_pwm;
	ulong start_ms;
	ulong last_pwm_update;
};

struct recovery_dhcp_server {
	struct udp_pcb *pcb;
	struct netif *netif;
	ip4_addr_t server_ip;
	ip4_addr_t client_ip;
	ip4_addr_t netmask;
	ip4_addr_t router;
	ip4_addr_t broadcast;
	ip4_addr_t dns;
};

static const int recovery_led_phy_addrs[RECOVERY_LED_PORTS] = { 9, 10 };
static const u8 recovery_green_led_gpios[RECOVERY_LED_PORTS] = { 43, 44 };
static const u8 recovery_yellow_led_gpios[RECOVERY_LED_PORTS] = { 33, 34 };

static void recovery_led_ctrl_free(struct recovery_led_ctrl *ctrl)
{
	memset(ctrl, 0, sizeof(*ctrl));
}

static bool recovery_gpio_flash_mode_bit(u8 gpio, uintptr_t *reg, u32 *mask)
{
	if (gpio <= 15) {
		*reg = RECOVERY_GPIO_SYSCTL_BASE + RECOVERY_REG_GPIO_FLASH_MODE_CFG;
		*mask = BIT(gpio);
		return true;
	}

	if (gpio >= 16 && gpio <= 31) {
		*reg = RECOVERY_GPIO_SYSCTL_BASE + RECOVERY_REG_GPIO_FLASH_MODE_CFG_EXT;
		*mask = BIT(gpio - 16);
		return true;
	}

	if (gpio >= 36 && gpio <= 51) {
		*reg = RECOVERY_GPIO_SYSCTL_BASE + RECOVERY_REG_GPIO_FLASH_MODE_CFG_EXT;
		*mask = BIT(gpio - 20);
		return true;
	}

	return false;
}

static void recovery_clrsetbits_le32(uintptr_t addr, u32 clear, u32 set)
{
	u32 val = readl((void __iomem *)addr);

	val &= ~clear;
	val |= set;
	writel(val, (void __iomem *)addr);
}

static uintptr_t recovery_gpio_data_reg(u8 gpio)
{
	return RECOVERY_GPIO_SYSCTL_BASE +
	       (gpio < 32 ? RECOVERY_REG_GPIO_DATA : RECOVERY_REG_GPIO_DATA1);
}

static uintptr_t recovery_gpio_oe_reg(u8 gpio)
{
	return RECOVERY_GPIO_SYSCTL_BASE +
	       (gpio < 32 ? RECOVERY_REG_GPIO_OE : RECOVERY_REG_GPIO_OE1);
}

static uintptr_t recovery_gpio_dir_reg(u8 gpio)
{
	static const u16 dir_regs[] = {
		RECOVERY_REG_GPIO_CTRL,
		RECOVERY_REG_GPIO_CTRL1,
		RECOVERY_REG_GPIO_CTRL2,
		RECOVERY_REG_GPIO_CTRL3,
	};

	return RECOVERY_GPIO_SYSCTL_BASE + dir_regs[gpio / 16];
}

static void recovery_gpio_direction_output(u8 gpio)
{
	u32 bank_bit = BIT(gpio % 32);
	u32 dir_bit = BIT(2 * (gpio % 16));

	recovery_clrsetbits_le32(recovery_gpio_oe_reg(gpio), 0, bank_bit);
	recovery_clrsetbits_le32(recovery_gpio_dir_reg(gpio), 0, dir_bit);
}

static void recovery_gpio_prepare_output(u8 gpio)
{
	uintptr_t reg;
	u32 mask;

	if (recovery_gpio_flash_mode_bit(gpio, &reg, &mask))
		recovery_clrsetbits_le32(reg, mask, 0);

	recovery_gpio_direction_output(gpio);
}

static void recovery_gpio_set_value(u8 gpio, bool active_low, int active)
{
	u32 bit = BIT(gpio % 32);
	uintptr_t reg = recovery_gpio_data_reg(gpio);
	u32 set = active_low ? (active ? 0 : bit) : (active ? bit : 0);

	recovery_clrsetbits_le32(reg, bit, set);
}

static void recovery_led_set_pin(u8 gpio, int on)
{
	recovery_gpio_set_value(gpio, true, on);
}

#if CONFIG_IS_ENABLED(DM_PWM)
static void recovery_status_pwm_shutdown(struct recovery_status_led_ctrl *ctrl)
{
	int i;

	if (!ctrl->pwm_count)
		return;

	for (i = 0; i < ctrl->pwm_count; i++) {
		struct recovery_status_pwm_led *led = &ctrl->pwm_leds[i];

		if (!led->valid)
			continue;

		pwm_set_config(led->pwm, led->channel, led->period_ns, 0);
		pwm_set_enable(led->pwm, led->channel, true);
	}

	ctrl->use_pwm = false;
}

static int recovery_status_pwm_select_state(struct recovery_status_led_ctrl *ctrl)
{
	int i, j, ret;

	for (i = 0; i < ctrl->pwm_count; i++) {
		struct udevice *pwm = ctrl->pwm_leds[i].pwm;
		bool seen = false;

		for (j = 0; j < i; j++) {
			if (ctrl->pwm_leds[j].pwm == pwm) {
				seen = true;
				break;
			}
		}
		if (seen)
			continue;

		ret = pinctrl_select_state(pwm, "recovery");
		if (ret)
			return ret;
	}

	return 0;
}

static int recovery_status_pwm_parse_led(ofnode node,
					 struct recovery_status_pwm_led *led)
{
	struct ofnode_phandle_args args;
	struct udevice *pwm;
	int ret;

	memset(led, 0, sizeof(*led));

	ret = ofnode_parse_phandle_with_args(node, "pwms", "#pwm-cells", 0, 0,
					     &args);
	if (ret)
		return ret;
	if (args.args_count < 2)
		return -EINVAL;

	ret = uclass_get_device_by_ofnode(UCLASS_PWM, args.node, &pwm);
	if (ret)
		return ret;

	led->pwm = pwm;
	led->channel = args.args[0];
	led->period_ns = args.args[1] ? args.args[1] :
			 RECOVERY_STATUS_PWM_PERIOD_NS;
	led->active_low = args.args_count > 2 &&
			  (args.args[2] & RECOVERY_PWM_POLARITY_INVERTED);
	led->valid = true;

	return 0;
}

static int recovery_status_pwm_init(struct recovery_status_led_ctrl *ctrl)
{
	ofnode pwm_leds, node;
	int i, ret;

	pwm_leds = ofnode_path(RECOVERY_STATUS_PWM_NODE);
	if (!ofnode_valid(pwm_leds))
		return -ENOENT;

	ctrl->pwm_count = 0;
	ofnode_for_each_subnode(node, pwm_leds) {
		if (ctrl->pwm_count >= ARRAY_SIZE(ctrl->pwm_leds))
			break;

		ret = recovery_status_pwm_parse_led(node,
						    &ctrl->pwm_leds[ctrl->pwm_count]);
		if (ret) {
			printf("Recovery PWM LED %s parse failed: %d\n",
			       ofnode_get_name(node), ret);
			goto err;
		}

		ctrl->pwm_count++;
	}

	if (!ctrl->pwm_count)
		return -ENOENT;

	for (i = 0; i < ctrl->pwm_count; i++) {
		struct recovery_status_pwm_led *led = &ctrl->pwm_leds[i];

		ret = pwm_set_invert(led->pwm, led->channel, led->active_low);
		if (ret)
			goto err;

		ret = pwm_set_config(led->pwm, led->channel, led->period_ns, 0);
		if (ret)
			goto err;
	}

	ret = recovery_status_pwm_select_state(ctrl);
	if (ret)
		goto err;

	for (i = 0; i < ctrl->pwm_count; i++) {
		struct recovery_status_pwm_led *led = &ctrl->pwm_leds[i];

		ret = pwm_set_enable(led->pwm, led->channel, true);
		if (ret)
			goto err;
	}

	ctrl->use_pwm = true;
	ctrl->last_pwm_update = 0;

	if (recovery_debug_enabled()) {
		printf("Recovery status LEDs using hardware PWM:");
		for (i = 0; i < ctrl->pwm_count; i++)
			printf(" ch%u/%uns%s", ctrl->pwm_leds[i].channel,
			       ctrl->pwm_leds[i].period_ns,
			       ctrl->pwm_leds[i].active_low ? "(L)" : "");
		printf("\n");
	}

	return 0;

err:
	recovery_status_pwm_shutdown(ctrl);
	memset(ctrl->pwm_leds, 0, sizeof(ctrl->pwm_leds));
	ctrl->pwm_count = 0;
	ctrl->use_pwm = false;

	return ret;
}
#else
static void recovery_status_pwm_shutdown(struct recovery_status_led_ctrl *ctrl)
{
}

static int recovery_status_pwm_init(struct recovery_status_led_ctrl *ctrl)
{
	return -ENOSYS;
}
#endif

static void recovery_status_led_release(struct recovery_status_led_ctrl *ctrl)
{
	recovery_status_pwm_shutdown(ctrl);
	memset(ctrl, 0, sizeof(*ctrl));
}

static int recovery_status_led_init(struct recovery_status_led_ctrl *ctrl)
{
	int ret;

	memset(ctrl, 0, sizeof(*ctrl));
	ctrl->start_ms = get_timer(0);

	ret = recovery_status_pwm_init(ctrl);
	if (ret)
		return ret;

	return 0;
}

static u32 recovery_status_led_breath_fp(ulong elapsed)
{
	ulong phase, ramp;
	u32 delta;

	phase = elapsed % RECOVERY_STATUS_BREATHE_PERIOD_MS;
	ramp = phase < RECOVERY_STATUS_BREATHE_HALF_MS ?
	       phase : RECOVERY_STATUS_BREATHE_PERIOD_MS - phase;
	delta = RECOVERY_STATUS_BRIGHTNESS_FP - RECOVERY_STATUS_BREATHE_MIN_FP;

	return RECOVERY_STATUS_BREATHE_MIN_FP +
	       (delta * ramp * ramp +
		(RECOVERY_STATUS_BREATHE_HALF_MS *
		 RECOVERY_STATUS_BREATHE_HALF_MS) / 2) /
	       (RECOVERY_STATUS_BREATHE_HALF_MS *
		RECOVERY_STATUS_BREATHE_HALF_MS);
}

static ulong recovery_status_led_position_fp(int led_count, ulong elapsed)
{
	ulong range_fp, phase;
	int span;

	if (led_count <= 1)
		return 0;

	span = led_count - 1;
	range_fp = span * RECOVERY_STATUS_BRIGHTNESS_FP;
	phase = elapsed % (2 * span * RECOVERY_STATUS_SWEEP_STEP_MS);

	if (phase <= span * RECOVERY_STATUS_SWEEP_STEP_MS)
		return phase * RECOVERY_STATUS_BRIGHTNESS_FP /
		       RECOVERY_STATUS_SWEEP_STEP_MS;

	phase -= span * RECOVERY_STATUS_SWEEP_STEP_MS;

	return range_fp - (phase * RECOVERY_STATUS_BRIGHTNESS_FP /
			   RECOVERY_STATUS_SWEEP_STEP_MS);
}

static u32 recovery_status_led_duty_fp(int led_count, int idx, ulong elapsed)
{
	ulong center_fp, pwm_phase;
	u32 breath_fp, weight_fp, dist_fp;
	ulong led_pos_fp;

	center_fp = recovery_status_led_position_fp(led_count, elapsed);
	led_pos_fp = idx * RECOVERY_STATUS_BRIGHTNESS_FP;
	dist_fp = center_fp > led_pos_fp ? center_fp - led_pos_fp :
					  led_pos_fp - center_fp;
	if (dist_fp >= RECOVERY_STATUS_OVERLAP_FP)
		return 0;

	weight_fp = (RECOVERY_STATUS_OVERLAP_FP - dist_fp) *
		    RECOVERY_STATUS_BRIGHTNESS_FP / RECOVERY_STATUS_OVERLAP_FP;
	weight_fp = weight_fp * weight_fp / RECOVERY_STATUS_BRIGHTNESS_FP;
	breath_fp = recovery_status_led_breath_fp(elapsed);
	pwm_phase = weight_fp * breath_fp;
	pwm_phase = (pwm_phase + RECOVERY_STATUS_BRIGHTNESS_FP / 2) /
		    RECOVERY_STATUS_BRIGHTNESS_FP;

	return min_t(u32, pwm_phase, RECOVERY_STATUS_BRIGHTNESS_FP);
}

#if CONFIG_IS_ENABLED(DM_PWM)
static void recovery_status_pwm_poll(struct recovery_status_led_ctrl *ctrl,
				     ulong now, ulong elapsed)
{
	int i;

	if (!ctrl->use_pwm || !ctrl->pwm_count)
		return;

	if (ctrl->last_pwm_update &&
	    now - ctrl->last_pwm_update < RECOVERY_STATUS_HW_PWM_UPDATE_MS)
		return;

	ctrl->last_pwm_update = now;

	for (i = 0; i < ctrl->pwm_count; i++) {
		struct recovery_status_pwm_led *led = &ctrl->pwm_leds[i];
		u32 duty_fp;
		u64 duty_ns;

		if (!led->valid)
			continue;

		duty_fp = recovery_status_led_duty_fp(ctrl->pwm_count, i,
						      elapsed);
		duty_ns = (u64)led->period_ns * duty_fp /
			  RECOVERY_STATUS_BRIGHTNESS_FP;
		pwm_set_config(led->pwm, led->channel, led->period_ns, duty_ns);
	}
}
#else
static void recovery_status_pwm_poll(struct recovery_status_led_ctrl *ctrl,
				     ulong now, ulong elapsed)
{
}
#endif

static void recovery_status_led_poll(struct recovery_status_led_ctrl *ctrl)
{
	ulong elapsed, now;

	if (!ctrl->use_pwm)
		return;

	now = get_timer(0);
	elapsed = now - ctrl->start_ms;
	recovery_status_pwm_poll(ctrl, now, elapsed);
}

static void recovery_status_led_stop(struct recovery_status_led_ctrl *ctrl)
{
	if (!ctrl->use_pwm)
		return;

	recovery_status_pwm_shutdown(ctrl);
}

enum recovery_dhcp_request_verdict {
	RECOVERY_DHCP_REQUEST_ACK = 0,
	RECOVERY_DHCP_REQUEST_NAK,
	RECOVERY_DHCP_REQUEST_IGNORE,
};

static int recovery_dhcp_get_option(const u8 *pkt, int pkt_len, u8 code,
				    const u8 **value, u8 *value_len)
{
	int off = DHCP_OPTIONS_OFS;

	while (off < pkt_len) {
		u8 opt, opt_len;

		opt = pkt[off++];
		if (opt == DHCP_OPTION_PAD)
			continue;
		if (opt == DHCP_OPTION_END)
			return -ENOENT;
		if (off >= pkt_len)
			break;

		opt_len = pkt[off++];
		if (off + opt_len > pkt_len)
			break;

		if (opt == code) {
			if (value)
				*value = pkt + off;
			if (value_len)
				*value_len = opt_len;
			return 0;
		}

		off += opt_len;
	}

	return -EINVAL;
}

static int recovery_dhcp_get_u8_option(const u8 *pkt, int pkt_len, u8 code,
				       u8 *value)
{
	const u8 *opt;
	u8 opt_len;
	int ret;

	ret = recovery_dhcp_get_option(pkt, pkt_len, code, &opt, &opt_len);
	if (ret)
		return ret;
	if (opt_len != sizeof(*value))
		return -EINVAL;

	*value = opt[0];
	return 0;
}

static int recovery_dhcp_get_ip4_option(const u8 *pkt, int pkt_len, u8 code,
					ip4_addr_t *addr)
{
	const u8 *opt;
	u8 opt_len;
	int ret;

	ret = recovery_dhcp_get_option(pkt, pkt_len, code, &opt, &opt_len);
	if (ret)
		return ret;
	if (opt_len != sizeof(addr->addr))
		return -EINVAL;

	memcpy(&addr->addr, opt, sizeof(addr->addr));
	return 0;
}

static int recovery_dhcp_put_option_head(u8 *options, int off, u8 code, u8 len)
{
	if (off < 0 || off + 2 + len > DHCP_OPTIONS_LEN)
		return -ENOSPC;

	options[off++] = code;
	options[off++] = len;

	return off;
}

static int recovery_dhcp_put_u8_option(u8 *options, int off, u8 code, u8 value)
{
	off = recovery_dhcp_put_option_head(options, off, code, sizeof(value));
	if (off < 0)
		return off;

	options[off++] = value;
	return off;
}

static int recovery_dhcp_put_u32_option(u8 *options, int off, u8 code, u32 value)
{
	u32 be_value = lwip_htonl(value);

	off = recovery_dhcp_put_option_head(options, off, code,
					    sizeof(be_value));
	if (off < 0)
		return off;

	memcpy(options + off, &be_value, sizeof(be_value));
	return off + sizeof(be_value);
}

static int recovery_dhcp_put_ip4_option(u8 *options, int off, u8 code,
					const ip4_addr_t *addr)
{
	off = recovery_dhcp_put_option_head(options, off, code,
					    sizeof(addr->addr));
	if (off < 0)
		return off;

	memcpy(options + off, &addr->addr, sizeof(addr->addr));
	return off + sizeof(addr->addr);
}

static void recovery_dhcp_finalize_options(struct pbuf *p, struct dhcp_msg *msg,
					   int opt_len)
{
	msg->options[opt_len++] = DHCP_OPTION_END;

	while (((opt_len < DHCP_MIN_OPTIONS_LEN) || (opt_len & 3)) &&
	       opt_len < DHCP_OPTIONS_LEN)
		msg->options[opt_len++] = DHCP_OPTION_PAD;

	pbuf_realloc(p, sizeof(*msg) - DHCP_OPTIONS_LEN + opt_len);
}

static enum recovery_dhcp_request_verdict
recovery_dhcp_classify_request(struct recovery_dhcp_server *srv,
			       const struct dhcp_msg *req,
			       const u8 *pkt, int pkt_len)
{
	ip4_addr_t option_ip, ciaddr;
	bool has_server_id, has_requested_ip;

	has_server_id = !recovery_dhcp_get_ip4_option(pkt, pkt_len,
						      DHCP_OPTION_SERVER_ID,
						      &option_ip);
	if (has_server_id) {
		if (!ip4_addr_eq(&option_ip, &srv->server_ip))
			return RECOVERY_DHCP_REQUEST_IGNORE;
	}

	has_requested_ip = !recovery_dhcp_get_ip4_option(pkt, pkt_len,
							 DHCP_OPTION_REQUESTED_IP,
							 &option_ip);
	if (has_requested_ip) {
		return ip4_addr_eq(&option_ip, &srv->client_ip) ?
			RECOVERY_DHCP_REQUEST_ACK :
			RECOVERY_DHCP_REQUEST_NAK;
	}

	ciaddr.addr = req->ciaddr.addr;
	if (!ip4_addr_isany(&ciaddr)) {
		return ip4_addr_eq(&ciaddr, &srv->client_ip) ?
			RECOVERY_DHCP_REQUEST_ACK :
			RECOVERY_DHCP_REQUEST_NAK;
	}

	return RECOVERY_DHCP_REQUEST_ACK;
}

static int recovery_dhcp_send_reply(struct recovery_dhcp_server *srv,
				    const struct dhcp_msg *req,
				    u8 message_type)
{
	struct pbuf *p;
	struct dhcp_msg *reply;
	ip_addr_t src_addr;
	ip_addr_t reply_addr;
	int opt_len;
	err_t err;

	static u32 dhcp_tx_log_count;

	p = pbuf_alloc(PBUF_TRANSPORT, sizeof(*reply), PBUF_RAM);
	if (!p)
		return -ENOMEM;

	reply = p->payload;
	memset(reply, 0, sizeof(*reply));

	reply->op = DHCP_BOOTREPLY;
	reply->htype = req->htype;
	reply->hlen = req->hlen;
	reply->hops = req->hops;
	reply->xid = req->xid;
	reply->secs = req->secs;
	reply->flags = req->flags | lwip_htons(0x8000);
	if (message_type != DHCP_NAK)
		reply->yiaddr.addr = srv->client_ip.addr;
	reply->siaddr.addr = 0;
	reply->giaddr = req->giaddr;
	memcpy(reply->chaddr, req->chaddr, DHCP_CHADDR_LEN);
	reply->cookie = PP_HTONL(DHCP_MAGIC_COOKIE);

	opt_len = 0;
	opt_len = recovery_dhcp_put_u8_option(reply->options, opt_len,
					      DHCP_OPTION_MESSAGE_TYPE,
					      message_type);
	opt_len = recovery_dhcp_put_ip4_option(reply->options, opt_len,
					       DHCP_OPTION_SERVER_ID,
					       &srv->server_ip);
	if (message_type != DHCP_NAK) {
		opt_len = recovery_dhcp_put_u32_option(reply->options, opt_len,
						       DHCP_OPTION_LEASE_TIME,
						       RECOVERY_DHCP_LEASE_SECS);
		opt_len = recovery_dhcp_put_ip4_option(reply->options, opt_len,
						       DHCP_OPTION_SUBNET_MASK,
						       &srv->netmask);
		opt_len = recovery_dhcp_put_ip4_option(reply->options, opt_len,
						       DHCP_OPTION_ROUTER,
						       &srv->router);
		opt_len = recovery_dhcp_put_ip4_option(reply->options, opt_len,
						       DHCP_OPTION_DNS_SERVER,
						       &srv->dns);
		opt_len = recovery_dhcp_put_ip4_option(reply->options, opt_len,
						       DHCP_OPTION_BROADCAST,
						       &srv->broadcast);
	}
	if (opt_len < 0) {
		pbuf_free(p);
		return opt_len;
	}

	recovery_dhcp_finalize_options(p, reply, opt_len);

	reply_addr = *IP_ADDR_BROADCAST;

	ip_addr_copy_from_ip4(src_addr, srv->server_ip);
	err = udp_sendto_if_src(srv->pcb, p, &reply_addr,
				LWIP_IANA_PORT_DHCP_CLIENT, srv->netif,
				&src_addr);

	if (recovery_debug_enabled() && dhcp_tx_log_count < 16) {
		printf("DHCP TX type=%u xid=0x%08x to %s rc=%d\n",
		       message_type, lwip_ntohl(req->xid),
		       ip4addr_ntoa(&srv->client_ip), err);
		dhcp_tx_log_count++;
	}

	pbuf_free(p);

	return err == ERR_OK ? 0 : -EIO;
}

static void recovery_dhcp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
			       const ip_addr_t *addr, u16_t port)
{
	struct recovery_dhcp_server *srv = arg;
	const struct dhcp_msg *req;
	u8 pkt[RECOVERY_DHCP_MAX_MSG_LEN];
	u8 message_type;
	int copy_len, pkt_len;
	static u32 dhcp_rx_log_count;

	(void)pcb;

	if (!p)
		return;

	if (port != LWIP_IANA_PORT_DHCP_CLIENT)
		goto out;

	copy_len = p->tot_len;
	if (copy_len > sizeof(pkt))
		copy_len = sizeof(pkt);

	pkt_len = pbuf_copy_partial(p, pkt, copy_len, 0);
	if (pkt_len < DHCP_OPTIONS_OFS)
		goto out;

	req = (const struct dhcp_msg *)pkt;
	if (req->op != DHCP_BOOTREQUEST ||
	    req->htype != LWIP_IANA_HWTYPE_ETHERNET ||
	    req->hlen != ARP_HLEN ||
	    req->cookie != PP_HTONL(DHCP_MAGIC_COOKIE))
		goto out;

	if (recovery_dhcp_get_u8_option(pkt, pkt_len, DHCP_OPTION_MESSAGE_TYPE,
					&message_type))
		goto out;

	if (recovery_debug_enabled() && dhcp_rx_log_count < 16) {
		printf("DHCP RX type=%u xid=0x%08x from %s:%u len=%d\n",
		       message_type, lwip_ntohl(req->xid), ipaddr_ntoa(addr),
		       port, pkt_len);
		dhcp_rx_log_count++;
	}

	switch (message_type) {
	case DHCP_DISCOVER:
		recovery_dhcp_send_reply(srv, req, DHCP_OFFER);
		break;
	case DHCP_REQUEST:
		switch (recovery_dhcp_classify_request(srv, req, pkt, pkt_len)) {
		case RECOVERY_DHCP_REQUEST_ACK:
			recovery_dhcp_send_reply(srv, req, DHCP_ACK);
			break;
		case RECOVERY_DHCP_REQUEST_NAK:
			recovery_dhcp_send_reply(srv, req, DHCP_NAK);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

out:
	pbuf_free(p);
}

static int recovery_dhcp_server_init(struct recovery_dhcp_server *srv,
				     struct netif *netif)
{
	char server_ip[IP4ADDR_STRLEN_MAX];
	char client_ip[IP4ADDR_STRLEN_MAX];
	char netmask[IP4ADDR_STRLEN_MAX];
	char router[IP4ADDR_STRLEN_MAX];
	char broadcast[IP4ADDR_STRLEN_MAX];
	err_t err;

	memset(srv, 0, sizeof(*srv));

	srv->netif = netif;
	ip4_addr_copy(srv->server_ip, *netif_ip4_addr(netif));
	ip4_addr_copy(srv->netmask, *netif_ip4_netmask(netif));
	ip4_addr_copy(srv->router, *netif_ip4_gw(netif));
	ip4_addr_copy(srv->dns, *netif_ip4_addr(netif));

	if (ip4_addr_isany(&srv->router))
		ip4_addr_copy(srv->router, srv->server_ip);

	if (!ip4addr_aton(RECOVERY_DHCP_CLIENT_IPADDR, &srv->client_ip))
		return -EINVAL;

	if (ip4_addr_isany(&srv->netmask)) {
		if (!ip4addr_aton(RECOVERY_DHCP_BROADCAST_IPADDR, &srv->broadcast))
			return -EINVAL;
	} else {
		srv->broadcast.addr = (srv->server_ip.addr & srv->netmask.addr) |
				      ~srv->netmask.addr;
	}

	srv->pcb = udp_new();
	if (!srv->pcb)
		return -ENOMEM;

	ip_set_option(srv->pcb, SOF_BROADCAST);

	err = udp_bind(srv->pcb, IP4_ADDR_ANY, LWIP_IANA_PORT_DHCP_SERVER);
	if (err != ERR_OK) {
		udp_remove(srv->pcb);
		srv->pcb = NULL;
		return -EIO;
	}

	udp_bind_netif(srv->pcb, netif);
	udp_recv(srv->pcb, recovery_dhcp_recv, srv);

	recovery_debug_printf("DHCP recovery server: %s/67 -> offer %s mask %s gw %s bcast %s\n",
			      ip4addr_ntoa_r(&srv->server_ip, server_ip,
					     sizeof(server_ip)),
			      ip4addr_ntoa_r(&srv->client_ip, client_ip,
					     sizeof(client_ip)),
			      ip4addr_ntoa_r(&srv->netmask, netmask,
					     sizeof(netmask)),
			      ip4addr_ntoa_r(&srv->router, router,
					     sizeof(router)),
			      ip4addr_ntoa_r(&srv->broadcast, broadcast,
					     sizeof(broadcast)));

	return 0;
}

static void recovery_dhcp_server_stop(struct recovery_dhcp_server *srv)
{
	if (srv->pcb)
		udp_remove(srv->pcb);

	memset(srv, 0, sizeof(*srv));
}

static int recovery_led_init(struct recovery_led_ctrl *ctrl)
{
	ofnode mdio_node;
	int i, ret;

	memset(ctrl, 0, sizeof(*ctrl));

	/* Make sure PHY LED mux is disabled so software can own the lines. */
	recovery_clrsetbits_le32(RECOVERY_CHIP_SCU_BASE + RECOVERY_REG_GPIO_2ND_I2C_MODE,
				 RECOVERY_GPIO_LAN0_LED0_MODE_MASK |
				 RECOVERY_GPIO_LAN0_LED1_MODE_MASK |
				 RECOVERY_GPIO_LAN1_LED0_MODE_MASK |
				 RECOVERY_GPIO_LAN1_LED1_MODE_MASK, 0);

	for (i = 0; i < RECOVERY_LED_PORTS; i++) {
		recovery_gpio_prepare_output(recovery_green_led_gpios[i]);
		recovery_gpio_prepare_output(recovery_yellow_led_gpios[i]);
		recovery_led_set_pin(recovery_green_led_gpios[i], 0);
		recovery_led_set_pin(recovery_yellow_led_gpios[i], 0);
	}

	mdio_node = ofnode_path("/soc/switch@1fb58000/mdio");
	if (!ofnode_valid(mdio_node))
		return 0;

	ret = uclass_get_device_by_ofnode(UCLASS_MDIO, mdio_node, &ctrl->mdio_dev);
	if (ret)
		ctrl->mdio_dev = NULL;

	return 0;
}

static int recovery_led_phy_speed(struct recovery_led_ctrl *ctrl, int idx)
{
	int bmcr, bmsr, stat1000, ctrl1000, lpa;

	if (!ctrl->mdio_dev || idx >= ARRAY_SIZE(recovery_led_phy_addrs))
		return 0;

	bmsr = dm_mdio_read(ctrl->mdio_dev, recovery_led_phy_addrs[idx],
			    MDIO_DEVAD_NONE, MII_BMSR);
	if (bmsr < 0)
		return 0;

	bmsr = dm_mdio_read(ctrl->mdio_dev, recovery_led_phy_addrs[idx],
			    MDIO_DEVAD_NONE, MII_BMSR);
	if (bmsr < 0)
		return 0;

	if (!(bmsr & BMSR_LSTATUS))
		return 0;

	bmcr = dm_mdio_read(ctrl->mdio_dev, recovery_led_phy_addrs[idx],
			    MDIO_DEVAD_NONE, MII_BMCR);
	if (bmcr < 0)
		return 0;

	if (!(bmcr & BMCR_ANENABLE)) {
		if (bmcr & BMCR_SPEED1000)
			return SPEED_1000;
		if (bmcr & BMCR_SPEED100)
			return SPEED_100;

		return SPEED_10;
	}

	stat1000 = dm_mdio_read(ctrl->mdio_dev, recovery_led_phy_addrs[idx],
				MDIO_DEVAD_NONE, MII_STAT1000);
	ctrl1000 = dm_mdio_read(ctrl->mdio_dev, recovery_led_phy_addrs[idx],
				MDIO_DEVAD_NONE, MII_CTRL1000);
	if (stat1000 >= 0 && ctrl1000 >= 0) {
		stat1000 &= ctrl1000 << 2;
		if (stat1000 & (PHY_1000BTSR_1000FD | PHY_1000BTSR_1000HD))
			return SPEED_1000;
	}

	lpa = dm_mdio_read(ctrl->mdio_dev, recovery_led_phy_addrs[idx],
			   MDIO_DEVAD_NONE, MII_ADVERTISE);
	if (lpa < 0)
		return SPEED_10;

	bmsr = dm_mdio_read(ctrl->mdio_dev, recovery_led_phy_addrs[idx],
			    MDIO_DEVAD_NONE, MII_LPA);
	if (bmsr < 0)
		return SPEED_10;

	lpa &= bmsr;
	if (lpa & (LPA_100FULL | LPA_100HALF))
		return SPEED_100;

	return SPEED_10;
}

static void recovery_led_poll(struct recovery_led_ctrl *ctrl)
{
	ulong now;
	int i, speed;

	now = get_timer(0);
	if (ctrl->last_poll && now - ctrl->last_poll < RECOVERY_LED_POLL_MS)
		return;

	ctrl->last_poll = now;

	for (i = 0; i < RECOVERY_LED_PORTS; i++) {
		speed = recovery_led_phy_speed(ctrl, i);
		recovery_led_set_pin(recovery_green_led_gpios[i],
				     speed == SPEED_1000);
		recovery_led_set_pin(recovery_yellow_led_gpios[i],
				     speed == SPEED_10 || speed == SPEED_100);
	}
}

static void recovery_service_runtime(struct recovery_status_led_ctrl *status_leds);

static const char *recovery_default_target(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return "fit";
	case TARGET_UBOOT:
		return "uboot";
	case TARGET_REPARTITION:
		return "repartition";
	}

	return "fit";
}

static const char *recovery_target_env(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return "recovery_mtd";
	case TARGET_UBOOT:
		return "recovery_mtd_uboot";
	case TARGET_REPARTITION:
		return "recovery_mtd";
	}

	return "recovery_mtd";
}

static const char *recovery_raw_env(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return "recovery_dev";
	case TARGET_UBOOT:
		return "recovery_dev_uboot";
	case TARGET_REPARTITION:
		return "recovery_dev";
	}

	return "recovery_dev";
}

static const char *recovery_size_env(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return "recovery_size";
	case TARGET_UBOOT:
		return "recovery_size_uboot";
	case TARGET_REPARTITION:
		return "recovery_size";
	}

	return "recovery_size";
}

static ulong recovery_raw_offset(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return env_get_hex("recovery_ofs", 0x050000);
	case TARGET_UBOOT:
		return env_get_hex("uboot_ofs", 0x0);
	case TARGET_REPARTITION:
		return 0;
	}

	return 0;
}

static bool recovery_backend_is_mmc(void)
{
	const char *backend = env_get("recovery_backend");

	if (recovery_board_is_sbe1v1k())
		return true;

	return backend && (!strcasecmp(backend, "mmc") ||
			   !strcasecmp(backend, "emmc"));
}

static const char *recovery_mmcdev(void)
{
	const char *dev = env_get("recovery_mmcdev");

	if (!dev)
		dev = env_get("mmcdev");

	return dev ?: "0";
}

static const char *recovery_mmc_part_env(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return "recovery_part_kernel";
	case TARGET_UBOOT:
		return "recovery_part_uboot";
	case TARGET_REPARTITION:
		return "recovery_part_uboot";
	}

	return "recovery_part_kernel";
}

static unsigned long recovery_uboot_limit(void)
{
	unsigned long def = recovery_backend_is_mmc() ?
			    RECOVERY_MAX_UBOOT_SIZE_MMC :
			    RECOVERY_MAX_UBOOT_SIZE;

	return env_get_hex("recovery_uboot_max", def);
}

static bool recovery_string_is_uint(const char *s)
{
	if (!s || !*s)
		return false;

	while (*s) {
		if (*s < '0' || *s > '9')
			return false;
		s++;
	}

	return true;
}

static int recovery_format_mmc_part_spec(const char *spec, char *buf,
					 size_t buf_len)
{
	const char *sep;
	size_t dev_len;
	int len;

	if (!spec || !*spec)
		return -EINVAL;

	if (strchr(spec, '#')) {
		strlcpy(buf, spec, buf_len);
		return 0;
	}

	sep = strchr(spec, ':');
	if (sep && !recovery_string_is_uint(sep + 1)) {
		dev_len = sep - spec;
		if (dev_len >= buf_len)
			return -ENOSPC;

		memcpy(buf, spec, dev_len);
		buf[dev_len] = '#';
		strlcpy(buf + dev_len + 1, sep + 1, buf_len - dev_len - 1);
		return 0;
	}

	if (sep) {
		strlcpy(buf, spec, buf_len);
		return 0;
	}

	len = snprintf(buf, buf_len, "%s#%s", recovery_mmcdev(), spec);
	if (len < 0 || len >= buf_len)
		return -ENOSPC;

	return 0;
}

/*
 * Resolve GPT labels by walking the table directly.  Some SBE1V1K builds can
 * print entries beyond the factory prefix but fail the generic name helper
 * for those same entries.  This still uses the on-device GPT, and callers
 * validate the resulting partition geometry before any destructive action.
 */
static int recovery_get_mmc_part_by_name(const char *part_spec,
					 struct blk_desc **desc,
					 struct disk_partition *part)
{
	const char *name = strchr(part_spec, '#');
	char dev_spec[32];
	size_t dev_len;
	int part_num;
	int ret;

	if (!name || !name[1])
		return -EINVAL;

	dev_len = name - part_spec;
	if (!dev_len || dev_len >= sizeof(dev_spec))
		return -ENOSPC;
	memcpy(dev_spec, part_spec, dev_len);
	dev_spec[dev_len] = '\0';
	name++;

	ret = blk_get_device_by_str("mmc", dev_spec, desc);
	if (ret < 0)
		return ret;

	for (part_num = 1; part_num <= MAX_SEARCH_PARTITIONS; part_num++) {
		ret = part_get_info(*desc, part_num, part);
		if (ret)
			continue;
		if (!strcmp(name, (const char *)part->name))
			return 0;
	}

	return -ENOENT;
}

static int recovery_get_mmc_part_ex(const char *spec, struct blk_desc **desc,
				    struct disk_partition *part, bool quiet)
{
	char part_spec[96];
	int ret;

	ret = recovery_format_mmc_part_spec(spec, part_spec, sizeof(part_spec));
	if (ret)
		return ret;

	if (strchr(part_spec, '#'))
		ret = recovery_get_mmc_part_by_name(part_spec, desc, part);
	else
		ret = part_get_info_by_dev_and_name_or_num("mmc", part_spec, desc,
							   part, false);
	if (ret < 0 && !quiet)
		printf("Failed to resolve eMMC partition '%s': %d\n",
		       part_spec, ret);

	return ret < 0 ? ret : 0;
}

static int recovery_get_mmc_part(const char *spec, struct blk_desc **desc,
				 struct disk_partition *part)
{
	return recovery_get_mmc_part_ex(spec, desc, part, false);
}

static unsigned long long
recovery_mmc_part_bytes(const struct disk_partition *part)
{
	return (unsigned long long)part->size *
	       (unsigned long long)part->blksz;
}

static bool recovery_data_range_ok(const void *base, size_t total,
				   const void *data, size_t size)
{
	uintptr_t b = (uintptr_t)base;
	uintptr_t d = (uintptr_t)data;

	if (d < b)
		return false;
	if (size > total)
		return false;
	if (d - b > total - size)
		return false;

	return true;
}

struct recovery_image_part {
	const void *data;
	size_t size;
};

struct recovery_firmware_image {
	struct recovery_image_part kernel;
	struct recovery_image_part rootfs;
};

static bool recovery_name_has_prefix(const char *name, const char *prefix)
{
	size_t plen;

	if (!name || !prefix)
		return false;

	plen = strlen(prefix);
	return !strncasecmp(name, prefix, plen);
}

static const char *recovery_basename(const char *name)
{
	const char *base, *p;

	if (!name)
		return "";

	base = name;
	for (p = name; *p; p++) {
		if (*p == '/')
			base = p + 1;
	}

	return base;
}

static bool recovery_is_kernel_name(const char *name)
{
	const char *base = recovery_basename(name);

	return recovery_name_has_prefix(base, "hlos") ||
	       recovery_name_has_prefix(base, "kernel");
}

static bool recovery_is_rootfs_name(const char *name)
{
	const char *base = recovery_basename(name);

	return recovery_name_has_prefix(base, "rootfs") ||
	       !strcasecmp(base, "root") ||
	       !strcasecmp(base, "fs") ||
	       recovery_name_has_prefix(base, "fs-");
}

static int recovery_fit_extract_firmware(const void *fit, size_t fit_size,
					 struct recovery_firmware_image *image)
{
	bool has_kernel = false;
	size_t boot_fit_size;
	int images, node, ret;

	memset(image, 0, sizeof(*image));

	ret = fit_check_format(fit, fit_size);
	if (ret)
		return ret;

	images = fdt_path_offset(fit, FIT_IMAGES_PATH);
	if (images < 0)
		return images;

	fdt_for_each_subnode(node, fit, images) {
		const char *node_name;
		const char *desc;
		const char *type;
		const void *data;
		size_t size;

		if (fit_image_get_data(fit, node, &data, &size))
			continue;

		if (!recovery_data_range_ok(fit, fit_size, data, size))
			return -EINVAL;

		node_name = fdt_get_name(fit, node, NULL);
		desc = fdt_getprop(fit, node, FIT_DESC_PROP, NULL);
		type = fdt_getprop(fit, node, FIT_TYPE_PROP, NULL);

		if (!has_kernel &&
		    (recovery_is_kernel_name(node_name) ||
		     recovery_is_kernel_name(desc) ||
		     (type && !strcasecmp(type, "kernel")))) {
			has_kernel = true;
			continue;
		}

		if (!image->rootfs.data &&
		    (recovery_is_rootfs_name(node_name) ||
		     recovery_is_rootfs_name(desc) ||
		     (type && (!strcasecmp(type, "filesystem") ||
			       !strcasecmp(type, "rootfs"))))) {
			image->rootfs.data = data;
			image->rootfs.size = size;
		}
	}

	if (has_kernel) {
		boot_fit_size = fit_get_size(fit);
		if (!boot_fit_size || boot_fit_size > fit_size)
			return -EINVAL;

		/* The eMMC kernel partition is bootm input, not a raw kernel blob. */
		image->kernel.data = fit;
		image->kernel.size = boot_fit_size;
	}

	return image->kernel.data || image->rootfs.data ? 0 : -ENOENT;
}

struct recovery_tar_header {
	char name[100];
	char mode[8];
	char uid[8];
	char gid[8];
	char size[12];
	char mtime[12];
	char chksum[8];
	char typeflag;
	char linkname[100];
	char magic[6];
	char version[2];
	char uname[32];
	char gname[32];
	char devmajor[8];
	char devminor[8];
	char prefix[155];
	char padding[12];
};

static bool recovery_tar_header_empty(const struct recovery_tar_header *hdr)
{
	const u8 *p = (const u8 *)hdr;
	int i;

	for (i = 0; i < 512; i++) {
		if (p[i])
			return false;
	}

	return true;
}

static int recovery_tar_octal(const char *field, size_t len, size_t *value)
{
	size_t v = 0;
	size_t i;

	for (i = 0; i < len; i++) {
		char c = field[i];

		if (c == '\0' || c == ' ')
			break;
		if (c < '0' || c > '7')
			return -EINVAL;
		if (v > ((size_t)-1 - (c - '0')) / 8)
			return -EOVERFLOW;
		v = (v << 3) + c - '0';
	}

	*value = v;
	return 0;
}

static int recovery_tar_verify_checksum(const struct recovery_tar_header *hdr)
{
	const u8 *bytes = (const u8 *)hdr;
	size_t expected;
	unsigned int checksum = 0;
	size_t i;

	if (RECOVERY_TAR_CHKSUM_OFF + RECOVERY_TAR_CHKSUM_LEN >
	    sizeof(*hdr))
		return -EINVAL;
	if (recovery_tar_octal(hdr->chksum, sizeof(hdr->chksum), &expected))
		return -EINVAL;

	for (i = 0; i < sizeof(*hdr); i++) {
		if (i >= RECOVERY_TAR_CHKSUM_OFF &&
		    i < RECOVERY_TAR_CHKSUM_OFF + RECOVERY_TAR_CHKSUM_LEN)
			checksum += ' ';
		else
			checksum += bytes[i];
	}

	return checksum == expected ? 0 : -EBADMSG;
}

static void recovery_tar_name(const struct recovery_tar_header *hdr,
			      char *name, size_t name_len)
{
	size_t prefix_len = strnlen(hdr->prefix, sizeof(hdr->prefix));
	size_t file_len = strnlen(hdr->name, sizeof(hdr->name));
	size_t off = 0;

	if (!name_len)
		return;

	if (prefix_len) {
		off = prefix_len >= name_len ? name_len - 1 : prefix_len;
		memcpy(name, hdr->prefix, off);
		if (off < name_len - 1)
			name[off++] = '/';
	}

	if (off < name_len - 1) {
		size_t copy = file_len;

		if (copy > name_len - 1 - off)
			copy = name_len - 1 - off;
		memcpy(name + off, hdr->name, copy);
		off += copy;
	}

	name[off] = '\0';
}

static bool recovery_is_control_name(const char *name)
{
	return !strcasecmp(recovery_basename(name), "CONTROL");
}

static const char *
recovery_sbe1v1k_layout_name(enum recovery_sbe1v1k_layout layout)
{
	switch (layout) {
	case RECOVERY_SBE1V1K_LAYOUT_MAINLINE:
		return "mainline";
	case RECOVERY_SBE1V1K_LAYOUT_LARGE:
		return "large";
	case RECOVERY_SBE1V1K_LAYOUT_QWRT:
		return "qwrt";
	default:
		return "unknown";
	}
}

static int
recovery_sbe1v1k_tar_layout(const char *board, const char *control_layout,
			    enum recovery_sbe1v1k_layout *layout)
{
	bool spectrum;

	spectrum = !strcmp(board, "spectrum_sbe1v1k") ||
		!strcmp(board, "spectrum,sbe1v1k");
	if (spectrum) {
		/*
		 * Stock Spectrum QSDK/QWRT images identify only the board. Their
		 * platform scripts write 0:HLOS and rootfs, so an unmarked image is
		 * always a Mainline/QWRT image. Never infer Large from the active GPT:
		 * doing so would place an HLOS image at the Large kernel offset.
		 * Repacked Large images must pin their intended profile explicitly.
		 */
		if (!control_layout) {
			if (active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_QWRT)
				*layout = RECOVERY_SBE1V1K_LAYOUT_QWRT;
			else
				*layout = RECOVERY_SBE1V1K_LAYOUT_MAINLINE;
			return 0;
		}
		if (!strcmp(control_layout, "mainline")) {
			*layout = RECOVERY_SBE1V1K_LAYOUT_MAINLINE;
			return 0;
		}
		if (!strcmp(control_layout, "large")) {
			*layout = RECOVERY_SBE1V1K_LAYOUT_LARGE;
			return 0;
		}
		if (!strcmp(control_layout, "qwrt")) {
			*layout = RECOVERY_SBE1V1K_LAYOUT_QWRT;
			return 0;
		}
		return -EINVAL;
	}

	/* Non-Spectrum identifiers cannot override their fixed layout. */
	if (control_layout)
		return -EINVAL;

	/* Factory-compatible non-Spectrum images use the HLOS/rootfs pair. */
	if (!strcmp(board, "qcom,ipq9574-ap-al02-c4")) {
		*layout = RECOVERY_SBE1V1K_LAYOUT_MAINLINE;
		return 0;
	}

	if (!strcmp(board, "askey_sbe1v1k") ||
	    !strcmp(board, "askey,sbe1v1k")) {
		*layout = RECOVERY_SBE1V1K_LAYOUT_LARGE;
		return 0;
	}

	return -ENOENT;
}

static void recovery_qwrt_secure_zero(void *data, size_t size)
{
	volatile u8 *bytes = data;

	while (size--)
		*bytes++ = 0;
}

/*
 * Linux qcom_socinfo exposes serial_number from SMEM item 137.  The
 * serial_num field was added in socinfo format version 10 and is at offset
 * 96 in the little-endian item (the field immediately after foundry_id in
 * Linux's struct socinfo). Read it here so U-Boot and soc_auth use the
 * same immutable SoC identity instead of a possibly stale environment value.
 */
static int recovery_qwrt_serial_from_smem(char *serial, size_t size)
{
#if !IS_ENABLED(CONFIG_SMEM)
	(void)serial;
	(void)size;
	return -ENOSYS;
#else
	struct udevice *smem;
	const u8 *socinfo;
	size_t item_size;
	u32 serial_num;
	int length;
	int ret;

	if (!serial || size < RECOVERY_QWRT_SERIAL_MAX)
		return -EINVAL;

	ret = uclass_get_device(UCLASS_SMEM, 0, &smem);
	if (ret)
		return ret;

	socinfo = smem_get(smem, -1, RECOVERY_QWRT_SMEM_SOCINFO_ITEM,
				   &item_size);
	if (IS_ERR(socinfo))
		return PTR_ERR(socinfo);
	if (!socinfo || item_size < RECOVERY_QWRT_SMEM_SERIAL_OFF +
					RECOVERY_QWRT_SMEM_SERIAL_SIZE)
		return -ENODATA;

	serial_num = get_unaligned_le32(socinfo + RECOVERY_QWRT_SMEM_SERIAL_OFF);
	length = snprintf(serial, size, "%u", serial_num);
	if (length < 0)
		return -EINVAL;
	return (size_t)length < size ? 0 : -ENOSPC;
#endif
}

static int recovery_qwrt_serial_from_bootargs(char *serial, size_t size)
{
	ofnode chosen;
	const char *bootargs;
	const char *value;
	const char *end;
	size_t length;

	if (!serial || size < 2)
		return -EINVAL;

	chosen = ofnode_path("/chosen");
	if (!ofnode_valid(chosen))
		return -ENOENT;
	bootargs = ofnode_read_string(chosen, "bootargs");
	if (!bootargs)
		return -ENOENT;

	value = strstr(bootargs, "androidboot.serialno=");
	if (!value)
		return -ENOENT;
	value += strlen("androidboot.serialno=");
	end = value;
	while (*end && *end != ' ' && *end != '\t')
		end++;
	length = end - value;
	if (!length || length >= size)
		return -EINVAL;

	memcpy(serial, value, length);
	serial[length] = '\0';
	return 0;
}

static int recovery_qwrt_normalize_serial(const char *input,
					   char *serial, size_t size)
{
	unsigned long long value;
	const char *digits = input;
	char *end;
	int base = 10;
	int ret;

	if (!input || !*input || !serial || size < RECOVERY_QWRT_SERIAL_MAX)
		return -EINVAL;

	if (!strncmp(digits, "0x", 2) || !strncmp(digits, "0X", 2)) {
		base = 16;
		digits += 2;
	} else {
		const char *p;
		bool has_non_decimal = false;

		/* qcom_set_serialno() may leave a bare hexadecimal value in env. */
		for (p = digits; *p; p++) {
			if (*p >= '0' && *p <= '9')
				continue;
			if ((*p >= 'a' && *p <= 'f') ||
			    (*p >= 'A' && *p <= 'F')) {
				has_non_decimal = true;
				continue;
			}
			return -EINVAL;
		}
		if (has_non_decimal)
			base = 16;
	}

	if (!*digits)
		return -EINVAL;
	value = simple_strtoull(digits, &end, base);
	if (end == digits || *end)
		return -EINVAL;

	if (value > 0xffffffffULL)
		return -ERANGE;
	ret = snprintf(serial, size, "%llu", value);
	return ret < 0 || (size_t)ret >= size ? -ENOSPC : 0;
}

/* The module stores this 28-byte key XORed with 0x5a in its read-only data. */
static const u8 recovery_qwrt_key_obfuscated[] = {
	0x0b, 0x0d, 0x08, 0x0e, 0x05, 0x13, 0x0a, 0x0b,
	0x63, 0x6f, 0x6d, 0x6e, 0x05, 0x68, 0x6a, 0x68,
	0x6e, 0x05, 0x09, 0x1f, 0x19, 0x08, 0x1f, 0x0e,
	0x05, 0x11, 0x1f, 0x03,
};

static void recovery_qwrt_hex_encode(const u8 *data, size_t size, char *hex)
{
	static const char digits[] = "0123456789abcdef";
	size_t i;

	for (i = 0; i < size; i++) {
		hex[i * 2] = digits[data[i] >> 4];
		hex[i * 2 + 1] = digits[data[i] & 0xf];
	}
	hex[size * 2] = '\0';
}

/* Reproduce soc_auth.ko's non-destructive auth_code derivation. */
static int recovery_qwrt_auth_code(char auth_code[RECOVERY_QWRT_AUTH_CODE_LEN + 1])
{
	char raw_serial[RECOVERY_QWRT_SERIAL_MAX];
	char serial[RECOVERY_QWRT_SERIAL_MAX];
	const char *env_serial;
	u8 key[sizeof(recovery_qwrt_key_obfuscated)];
	u8 digest[SHA256_SUM_LEN];
	int ret;
	size_t i;

	/* The kernel's socinfo driver uses this value as the authoritative ID. */
	ret = recovery_qwrt_serial_from_smem(raw_serial, sizeof(raw_serial));
	if (ret) {
		env_serial = env_get("serial#");
		if (env_serial && *env_serial)
			strlcpy(raw_serial, env_serial, sizeof(raw_serial));
		else {
			ret = recovery_qwrt_serial_from_bootargs(raw_serial,
							 sizeof(raw_serial));
			if (ret)
				return ret;
		}
	}

	ret = recovery_qwrt_normalize_serial(raw_serial, serial,
					     sizeof(serial));
	if (ret)
		return ret;

	for (i = 0; i < sizeof(key); i++)
		key[i] = recovery_qwrt_key_obfuscated[i] ^ 0x5a;
	for (i = 0; i < min_t(size_t, strlen(serial), 8); i++)
		key[20 + i] ^= serial[i];

	ret = sha256_hmac(key, sizeof(key), (const u8 *)serial,
				  strlen(serial), digest);
	if (!ret)
		recovery_qwrt_hex_encode(digest, sizeof(digest), auth_code);

	recovery_qwrt_secure_zero(raw_serial, sizeof(raw_serial));
	recovery_qwrt_secure_zero(serial, sizeof(serial));
	recovery_qwrt_secure_zero(key, sizeof(key));
	recovery_qwrt_secure_zero(digest, sizeof(digest));
	return ret;
}

static void recovery_qwrt_button_reset(void)
{
	qwrt_button_presses = 0;
	qwrt_button_last_press = 0;
}

static void recovery_qwrt_button_init(void)
{
#if !IS_ENABLED(CONFIG_BUTTON)
	qwrt_button_unlocked = false;
	qwrt_button_available = false;
	qwrt_button = NULL;
	qwrt_button_last_state = BUTTON_OFF;
	recovery_qwrt_button_reset();
	return;
#else
	int ret;

	qwrt_button_unlocked = false;
	qwrt_button_available = false;
	qwrt_button = NULL;
	qwrt_button_last_state = BUTTON_OFF;
	recovery_qwrt_button_reset();

	if (!recovery_board_is_sbe1v1k())
		return;

	ret = button_get_by_label("reset", &qwrt_button);
	if (!ret && qwrt_button) {
		qwrt_button_available = true;
		qwrt_button_last_state = button_get_state(qwrt_button);
		if (qwrt_button_last_state < BUTTON_OFF ||
		    qwrt_button_last_state >= BUTTON_COUNT)
			qwrt_button_last_state = BUTTON_OFF;
	}
#endif
}

static void recovery_qwrt_button_poll(void)
{
#if !IS_ENABLED(CONFIG_BUTTON)
	return;
#else
	enum button_state_t state;
	ulong now;

	if (!qwrt_button_available || qwrt_button_unlocked)
		return;

	state = button_get_state(qwrt_button);
	if (state < BUTTON_OFF || state >= BUTTON_COUNT)
		return;

	if (state == BUTTON_ON && qwrt_button_last_state == BUTTON_OFF) {
		now = get_timer(0);
		if (qwrt_button_last_press &&
		    get_timer(qwrt_button_last_press) > RECOVERY_QWRT_BUTTON_WINDOW_MS)
			recovery_qwrt_button_reset();
		if (!qwrt_button_last_press)
			qwrt_button_last_press = now;
		if (qwrt_button_presses < RECOVERY_QWRT_BUTTON_PRESSES)
			qwrt_button_presses++;
		if (qwrt_button_presses >= RECOVERY_QWRT_BUTTON_PRESSES) {
			qwrt_button_unlocked = true;
			/* One-shot session capability; do not persist it in eMMC. */
			recovery_debug_printf("QWRT partition profile unlocked for this recovery session\n");
		}
	}

	qwrt_button_last_state = state;
#endif
}

/*
 * The older SBE1V1K build identifies itself as the Qualcomm AP-AL02-C4
 * board. Its own sysupgrade script writes the factory 0:HLOS/rootfs pair,
 * which is our mainline profile rather than the large chainloader layout.
 */
static int recovery_validate_sbe1v1k_tar_control(const void *data, size_t size)
{
	static const char board_prefix[] = "BOARD=";
	static const char layout_prefix[] = "SBE1V1K_LAYOUT=";
	const u8 *bytes = data;
	char board[96];
	char control_layout[16];
	enum recovery_sbe1v1k_layout image_layout;
	size_t prefix_len = strlen(board_prefix);
	size_t layout_prefix_len = strlen(layout_prefix);
	size_t off = 0;
	bool found = false;
	bool layout_found = false;
	int ret;

	if (!recovery_board_is_sbe1v1k())
		return 0;

	while (off < size) {
		size_t line_end = off;
		size_t board_len;
		size_t layout_len;

		while (line_end < size && bytes[line_end] != '\n')
			line_end++;
		if (line_end > off + prefix_len &&
		    !memcmp(bytes + off, board_prefix, prefix_len)) {
			board_len = line_end - off - prefix_len;
			if (board_len && bytes[off + prefix_len + board_len - 1] == '\r')
				board_len--;
			if (found || !board_len || board_len >= sizeof(board))
				return -EINVAL;
			memcpy(board, bytes + off + prefix_len, board_len);
			board[board_len] = '\0';
			found = true;
		} else if (line_end > off + layout_prefix_len &&
			   !memcmp(bytes + off, layout_prefix, layout_prefix_len)) {
			layout_len = line_end - off - layout_prefix_len;
			if (layout_len &&
			    bytes[off + layout_prefix_len + layout_len - 1] == '\r')
				layout_len--;
			if (layout_found || !layout_len ||
			    layout_len >= sizeof(control_layout))
				return -EINVAL;
			memcpy(control_layout, bytes + off + layout_prefix_len,
			       layout_len);
			control_layout[layout_len] = '\0';
			layout_found = true;
		}
		off = line_end + (line_end < size);
	}

	if (!found) {
		printf("SBE1V1K sysupgrade CONTROL has no BOARD entry\n");
		return -EINVAL;
	}

	ret = recovery_sbe1v1k_tar_layout(board,
					 layout_found ? control_layout : NULL,
					 &image_layout);
	if (ret) {
		if (layout_found)
			printf("SBE1V1K sysupgrade BOARD '%s' does not support layout '%s'\n",
			       board, control_layout);
		else
			printf("SBE1V1K sysupgrade BOARD '%s' is not supported\n", board);
		return ret;
	}

	if (active_sbe1v1k_layout != image_layout) {
		printf("SBE1V1K sysupgrade BOARD '%s' requires the %s partition profile\n",
		       board, recovery_sbe1v1k_layout_name(image_layout));
		return -EINVAL;
	}

	return 0;
}

static int recovery_tar_extract_firmware(const void *tar, size_t tar_size,
					 struct recovery_firmware_image *image)
{
	const u8 *base = tar;
	size_t off = 0;
	bool control_seen = false;

	memset(image, 0, sizeof(*image));

	while (off + 512 <= tar_size) {
		const struct recovery_tar_header *hdr =
			(const struct recovery_tar_header *)(base + off);
		const void *entry = base + off + 512;
		char name[256];
		size_t size;
		bool regular;
		int ret;

		if (recovery_tar_header_empty(hdr))
			break;

		if (memcmp(hdr->magic, "ustar", 5))
			return -EINVAL;
		ret = recovery_tar_verify_checksum(hdr);
		if (ret)
			return ret;

		if (recovery_tar_octal(hdr->size, sizeof(hdr->size), &size))
			return -EINVAL;

		if (off + 512 > tar_size || size > tar_size - off - 512)
			return -EINVAL;

		recovery_tar_name(hdr, name, sizeof(name));
		regular = hdr->typeflag == '\0' || hdr->typeflag == '0';

		if (recovery_board_is_sbe1v1k() && regular &&
		    recovery_is_control_name(name)) {
			if (control_seen)
				return -EINVAL;
			control_seen = true;
			ret = recovery_validate_sbe1v1k_tar_control(entry, size);
			if (ret)
				return ret;
		} else if (regular && !image->kernel.data &&
			   recovery_is_kernel_name(name)) {
			if (recovery_board_is_sbe1v1k() && !control_seen) {
				printf("SBE1V1K sysupgrade CONTROL must precede the kernel payload\n");
				return -EINVAL;
			}
			image->kernel.data = base + off + 512;
			image->kernel.size = size;
		} else if (regular && !image->rootfs.data &&
			   recovery_is_rootfs_name(name)) {
			image->rootfs.data = base + off + 512;
			image->rootfs.size = size;
		}

		off += 512 + ALIGN(size, 512);
	}

	if (recovery_board_is_sbe1v1k() && !control_seen) {
		printf("SBE1V1K sysupgrade tar has no CONTROL member\n");
		return -EINVAL;
	}

	return image->kernel.data || image->rootfs.data ? 0 : -ENOENT;
}

static int recovery_raw_extract_firmware(const void *raw, size_t raw_size,
					 struct recovery_firmware_image *image)
{
	size_t kernel_pad = env_get_hex("recovery_kernel_pad",
				       RECOVERY_KERNEL_PAD_SIZE);
	size_t kernel_size = kernel_pad;

	memset(image, 0, sizeof(*image));

	if (raw_size <= kernel_pad)
		return -ENOENT;

	if (!fit_check_format(raw, raw_size)) {
		size_t fit_size = fdt_totalsize(raw);

		if (fit_size > 0 && fit_size <= kernel_pad)
			kernel_size = fit_size;
	}

	image->kernel.data = raw;
	image->kernel.size = kernel_size;
	image->rootfs.data = (const u8 *)raw + kernel_pad;
	image->rootfs.size = raw_size - kernel_pad;

	return 0;
}

static int recovery_extract_firmware(const void *data, size_t size,
				     struct recovery_firmware_image *image)
{
	int ret;

	ret = recovery_fit_extract_firmware(data, size, image);
	if (!ret && image->kernel.data && image->rootfs.data) {
		recovery_debug_printf("Firmware image: bootable FIT=%lu rootfs=%lu\n",
				      (ulong)image->kernel.size,
				      (ulong)image->rootfs.size);
		return 0;
	}

	ret = recovery_tar_extract_firmware(data, size, image);
	if (!ret && image->kernel.data && image->rootfs.data) {
		recovery_debug_printf("Firmware image: sysupgrade tar kernel=%lu rootfs=%lu\n",
				      (ulong)image->kernel.size,
				      (ulong)image->rootfs.size);
		return 0;
	}

	ret = recovery_raw_extract_firmware(data, size, image);
	if (!ret && image->kernel.data && image->rootfs.data) {
		recovery_debug_printf("Firmware image: raw kernel/rootfs split kernel=%lu rootfs=%lu\n",
				      (ulong)image->kernel.size,
				      (ulong)image->rootfs.size);
		return 0;
	}

	printf("Firmware image must contain both kernel/hlos and root/rootfs/fs payloads\n");
	return -EINVAL;
}

static int recovery_verify_fit_payload(const void *fit, size_t payload_size)
{
	const void *data;
	size_t fit_size;
	size_t data_size;
	int images, node, ret;

	ret = fit_check_format(fit, payload_size);
	if (ret) {
		printf("Payload must be a complete FIT image: %d\n", ret);
		return ret;
	}

	fit_size = fit_get_size(fit);
	if (!fit_size || fit_size > payload_size) {
		printf("FIT payload is truncated\n");
		return -EINVAL;
	}

	images = fdt_path_offset(fit, FIT_IMAGES_PATH);
	if (images < 0)
		return images;

	fdt_for_each_subnode(node, fit, images) {
		ret = fit_image_get_data(fit, node, &data, &data_size);
		if (ret || !recovery_data_range_ok(fit, payload_size,
						   data, data_size)) {
			printf("FIT image data is missing or outside the payload\n");
			return ret ?: -EINVAL;
		}
	}

	if (!fit_all_image_verify(fit)) {
		printf("FIT image hash verification failed\n");
		return -EBADMSG;
	}

	return 0;
}

static int
recovery_validate_sbe1v1k_kernel_fit(const struct recovery_image_part *kernel)
{
	u8 type, os, arch;
	int conf, node, ret;

	ret = recovery_verify_fit_payload(kernel->data, kernel->size);
	if (ret)
		return ret;

	conf = fit_conf_get_node(kernel->data, NULL);
	if (conf < 0) {
		printf("SBE1V1K kernel FIT has no usable default configuration: %d\n",
		       conf);
		return conf;
	}

	node = fit_conf_get_prop_node(kernel->data, conf,
				      FIT_KERNEL_PROP, IH_PHASE_NONE);
	if (node < 0) {
		printf("SBE1V1K kernel FIT default configuration has no kernel: %d\n",
		       node);
		return node;
	}

	if (fit_image_get_type(kernel->data, node, &type) ||
	    fit_image_get_os(kernel->data, node, &os) ||
	    fit_image_get_arch(kernel->data, node, &arch) ||
	    type != IH_TYPE_KERNEL || os != IH_OS_LINUX ||
	    arch != IH_ARCH_ARM64) {
		printf("SBE1V1K FIT kernel must be type=kernel, os=linux, arch=arm64\n");
		return -ENOEXEC;
	}

	recovery_debug_printf("SBE1V1K boot FIT validation passed\n");
	return 0;
}

static int
recovery_validate_sbe1v1k_rootfs(const struct recovery_image_part *rootfs)
{
	const u8 *data = rootfs->data;
	u64 bytes_used;

	if (rootfs->size < RECOVERY_SQUASHFS_HEADER_MIN ||
	    get_unaligned_le32(data) != RECOVERY_SQUASHFS_MAGIC ||
	    get_unaligned_le16(data + RECOVERY_SQUASHFS_MAJOR_OFF) !=
					      RECOVERY_SQUASHFS_MAJOR) {
		printf("SBE1V1K rootfs payload must be a SquashFS v4 image\n");
		return -ENOEXEC;
	}

	bytes_used = get_unaligned_le64(data + RECOVERY_SQUASHFS_BYTES_OFF);
	if (!bytes_used || bytes_used > rootfs->size) {
		printf("SBE1V1K SquashFS payload is truncated or invalid\n");
		return -EINVAL;
	}

	recovery_debug_printf("SBE1V1K SquashFS validation passed\n");
	return 0;
}

static int recovery_mmc_check_part_size(const char *spec, size_t size)
{
	struct disk_partition part;
	struct blk_desc *desc;
	unsigned long long capacity;
	int ret;

	ret = recovery_get_mmc_part(spec, &desc, &part);
	if (ret)
		return ret;

	capacity = recovery_mmc_part_bytes(&part);
	if (size > capacity) {
		printf("Image size %lu exceeds eMMC partition '%s' size %llu\n",
		       (ulong)size, spec, capacity);
		return -EFBIG;
	}

	return 0;
}

static int recovery_mmc_write_part(const char *spec,
				   struct recovery_status_led_ctrl *status_leds,
				   const void *data, size_t size,
				   size_t progress_base)
{
	struct disk_partition part;
	struct blk_desc *desc;
	unsigned long long capacity;
	ulong blksz, chunk_size;
	u8 *chunk_buf;
	size_t written = 0;
	int ret;

	ret = recovery_get_mmc_part(spec, &desc, &part);
	if (ret)
		return ret;

	blksz = part.blksz ?: desc->blksz;
	capacity = recovery_mmc_part_bytes(&part);
	if (size > capacity) {
		printf("Image size %lu exceeds eMMC partition '%s' size %llu\n",
		       (ulong)size, spec, capacity);
		return -EFBIG;
	}

	chunk_size = ALIGN(RECOVERY_MMC_WRITE_CHUNK, blksz);
	if (chunk_size < blksz)
		chunk_size = blksz;

	chunk_buf = memalign(ARCH_DMA_MINALIGN, chunk_size);
	if (!chunk_buf)
		return -ENOMEM;

	while (written < size) {
		size_t todo = size - written;
		size_t write_len;
		lbaint_t blk;
		lbaint_t blkcnt;

		if (todo > chunk_size)
			todo = chunk_size;

		write_len = ALIGN(todo, blksz);
		memset(chunk_buf, 0, write_len);
		memcpy(chunk_buf, (const u8 *)data + written, todo);

		blk = part.start + written / blksz;
		blkcnt = write_len / blksz;
		if (blk_dwrite(desc, blk, blkcnt, chunk_buf) != blkcnt) {
			printf("eMMC write failed at partition '%s' block " LBAF "\n",
			       spec, blk);
			free(chunk_buf);
			return -EIO;
		}

		written += todo;
		prog_write_done = progress_base + written;
		prog_done = prog_erase_done + prog_write_done;
		recovery_service_runtime(status_leds);
	}

	free(chunk_buf);
	return 0;
}

static void
recovery_mmc_update_erase_progress(struct recovery_status_led_ctrl *status_leds,
				   u32 progress_base, lbaint_t done,
				   lbaint_t total)
{
	unsigned long long steps;

	if (!total)
		steps = RECOVERY_MMC_ERASE_PROGRESS_STEPS;
	else
		steps = (unsigned long long)done *
			RECOVERY_MMC_ERASE_PROGRESS_STEPS / total;
	if (steps > RECOVERY_MMC_ERASE_PROGRESS_STEPS)
		steps = RECOVERY_MMC_ERASE_PROGRESS_STEPS;

	prog_erase_done = progress_base + steps;
	prog_done = prog_erase_done + prog_write_done;
	recovery_service_runtime(status_leds);
}

static int recovery_mmc_zero_part(const char *spec,
				  struct recovery_status_led_ctrl *status_leds,
				  struct blk_desc *desc,
				  const struct disk_partition *part,
				  u32 progress_base)
{
	ulong blksz = part->blksz ?: desc->blksz;
	ulong chunk_size = ALIGN(RECOVERY_MMC_WRITE_CHUNK, blksz);
	lbaint_t chunk_blocks, cleared = 0;
	u8 *chunk_buf;

	if (chunk_size < blksz)
		chunk_size = blksz;
	chunk_blocks = chunk_size / blksz;

	chunk_buf = memalign(ARCH_DMA_MINALIGN, chunk_size);
	if (!chunk_buf)
		return -ENOMEM;

	memset(chunk_buf, 0, chunk_size);

	while (cleared < part->size) {
		lbaint_t todo = part->size - cleared;
		lbaint_t blk = part->start + cleared;

		if (todo > chunk_blocks)
			todo = chunk_blocks;

		if (blk_dwrite(desc, blk, todo, chunk_buf) != todo) {
			printf("eMMC zero-fill failed at partition '%s' block "
			       LBAF "\n", spec, blk);
			free(chunk_buf);
			return -EIO;
		}

		cleared += todo;
		recovery_mmc_update_erase_progress(status_leds, progress_base,
						   cleared, part->size);
	}

	free(chunk_buf);
	return 0;
}

static int recovery_mmc_erase_resolved(
	const char *name, struct recovery_status_led_ctrl *status_leds,
	struct blk_desc *desc, const struct disk_partition *part,
	u32 progress_base)
{
	struct mmc *mmc;
	unsigned long long capacity;
	lbaint_t chunk_blocks, erased = 0;
	u32 erase_group;
	ulong blksz;
	bool exact;

	if (!name || !*name || !desc || !part || !part->size)
		return -EINVAL;

	blksz = part->blksz ?: desc->blksz;
	if (!blksz || (unsigned long long)part->size > ~0ULL / blksz)
		return -EOVERFLOW;
	capacity = (unsigned long long)part->size * blksz;
	mmc = find_mmc_device(desc->devnum);
	if (!mmc) {
		printf("Cannot find eMMC device %d for partition '%s'\n",
		       desc->devnum, name);
		return -ENODEV;
	}

	erase_group = mmc->erase_grp_size;
	exact = erase_group &&
		(mmc->can_trim || (!(part->start % erase_group) &&
				   !(part->size % erase_group)));

	recovery_debug_printf("Erasing entire eMMC partition '%s' (%llu blocks, %llu bytes)...\n",
			      name, (unsigned long long)part->size, capacity);

	if (!exact) {
		recovery_debug_printf("eMMC partition '%s' is not erase-group aligned and exact trim is unavailable; zero-filling the complete partition to protect adjacent GPT partitions\n",
				      name);
		return recovery_mmc_zero_part(name, status_leds, desc, part,
					      progress_base);
	}

	chunk_blocks = RECOVERY_MMC_ERASE_CHUNK / blksz;
	if (!chunk_blocks)
		chunk_blocks = 1;
	if (!mmc->can_trim && chunk_blocks < erase_group)
		chunk_blocks = erase_group;
	if (!mmc->can_trim)
		chunk_blocks -= chunk_blocks % erase_group;

	while (erased < part->size) {
		lbaint_t todo = part->size - erased;
		lbaint_t blk = part->start + erased;

		if (todo > chunk_blocks)
			todo = chunk_blocks;
		if (!mmc->can_trim && todo % erase_group)
			todo -= todo % erase_group;
		if (!todo) {
			printf("Cannot safely align eMMC erase for partition '%s'\n",
			       name);
			return -EINVAL;
		}

		if (blk_derase(desc, blk, todo) != todo) {
			printf("eMMC erase failed at partition '%s' block " LBAF
			       " count " LBAF "\n", name, blk, todo);
			return -EIO;
		}

		erased += todo;
		recovery_mmc_update_erase_progress(status_leds, progress_base,
						   erased, part->size);
	}

	return 0;
}

static int recovery_mmc_erase_part(const char *spec,
				   struct recovery_status_led_ctrl *status_leds,
				   u32 progress_base)
{
	struct disk_partition part;
	struct blk_desc *desc;
	int ret;

	if (!spec || !*spec)
		return -EINVAL;

	ret = recovery_get_mmc_part(spec, &desc, &part);
	if (ret)
		return ret;

	return recovery_mmc_erase_resolved(spec, status_leds, desc, &part,
					    progress_base);
}

#define RECOVERY_MMC_STREAM_PARTS 2

struct recovery_mmc_stream_part {
	char spec[96];
	struct blk_desc *desc;
	struct disk_partition part;
	size_t expected;
	size_t written;
};

struct recovery_mmc_stream {
	struct recovery_mmc_stream_part parts[RECOVERY_MMC_STREAM_PARTS];
	u8 *buf;
	u8 tar_header[512];
	u8 tar_control[RECOVERY_SBE1V1K_CONTROL_MAX];
	size_t buf_size;
	size_t buf_used;
	size_t input_received;
	size_t tar_header_used;
	size_t tar_control_used;
	size_t tar_entry_remaining;
	size_t tar_padding_remaining;
	int part_count;
	int part_index;
	int tar_part_index;
	int error;
	enum upload_target target;
	enum recovery_stream_format format;
	size_t total_expected;
	bool tar_control_active;
	bool tar_seen_control;
	bool tar_seen_kernel;
	bool tar_seen_rootfs;
	bool tar_end;
	bool active;
	bool prepared;
};

static struct recovery_mmc_stream recovery_stream;
static bool recovery_stream_completed;

static int recovery_require_sbe1v1k_firmware_layout(void)
{
	if (recovery_board_is_sbe1v1k() &&
	    active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_QWRT &&
	    !qwrt_button_unlocked) {
		printf("QWRT firmware operations are unavailable in this recovery session\n");
		return -EPERM;
	}

	if (!recovery_board_is_sbe1v1k() ||
	    active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_MAINLINE ||
	    active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_LARGE ||
	    active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_QWRT)
		return 0;

	if (sbe1v1k_factory_pre_migration)
		printf("SBE1V1K factory GPT is recognised, but firmware uploads are locked until a verified layout migration completes\n");
	else
		printf("SBE1V1K partition profile is unknown; firmware uploads are locked\n");
	return -EPERM;
}

static void recovery_mmc_stream_reset(void)
{
	free(recovery_stream.buf);
	memset(&recovery_stream, 0, sizeof(recovery_stream));
	recovery_stream.tar_part_index = -1;
}

static int recovery_mmc_stream_add_part(const char *spec, size_t expected)
{
	struct recovery_mmc_stream_part *stream_part;
	unsigned long long capacity;
	int ret;

	if (recovery_stream.part_count >= RECOVERY_MMC_STREAM_PARTS)
		return -ENOSPC;

	stream_part = &recovery_stream.parts[recovery_stream.part_count];
	strlcpy(stream_part->spec, spec, sizeof(stream_part->spec));
	ret = recovery_get_mmc_part(stream_part->spec, &stream_part->desc,
				    &stream_part->part);
	if (ret)
		return ret;

	capacity = recovery_mmc_part_bytes(&stream_part->part);
	if (expected > capacity) {
		printf("Stream payload size %lu exceeds eMMC partition '%s' size %llu\n",
		       (ulong)expected, stream_part->spec, capacity);
		return -EFBIG;
	}

	stream_part->expected = expected;
	recovery_stream.part_count++;
	return 0;
}

static int recovery_mmc_stream_prepare(enum upload_target target, size_t size,
				       enum recovery_stream_format format)
{
	const char *kernel_part = env_get("recovery_part_kernel") ?: "0#kernel";
	const char *rootfs_part = env_get("recovery_part_rootfs") ?: "0#rootfs";
	const char *data_part = env_get("recovery_part_data") ?: "0#rootfs_data";
	const char *uboot_part = env_get("recovery_part_uboot");
	size_t kernel_pad = env_get_hex("recovery_kernel_pad",
					RECOVERY_KERNEL_PAD_SIZE);
	ulong blksz;
	int ret;

	if (target == TARGET_FIRMWARE) {
		ret = recovery_require_sbe1v1k_firmware_layout();
		if (ret)
			return ret;
		if (active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_QWRT) {
			ret = recovery_refresh_qwrt_auth_code();
			if (ret) {
				printf("Cannot refresh QWRT auth_code before erase: %d\n",
				       ret);
				return ret;
			}
		}
	}

	recovery_mmc_stream_reset();
	recovery_stream_completed = false;
	recovery_stream.target = target;
	recovery_stream.format = format;
	recovery_stream.total_expected = size;

	if (target == TARGET_FIRMWARE) {
		if (format == RECOVERY_STREAM_TAR) {
			ret = recovery_mmc_stream_add_part(kernel_part, 0);
			if (ret)
				goto err;
			ret = recovery_mmc_stream_add_part(rootfs_part, 0);
			if (ret)
				goto err;
		} else {
			if (size <= kernel_pad)
				return -EINVAL;
			ret = recovery_mmc_stream_add_part(kernel_part, kernel_pad);
			if (ret)
				goto err;
			ret = recovery_mmc_stream_add_part(rootfs_part, size - kernel_pad);
			if (ret)
				goto err;
		}
		ret = recovery_mmc_check_part_size(data_part, 0);
		if (ret)
			goto err;
	} else if (target == TARGET_UBOOT) {
		if (format != RECOVERY_STREAM_RAW) {
			ret = -EINVAL;
			goto err;
		}
		if (!uboot_part || !*uboot_part) {
			ret = -ENODEV;
			goto err;
		}
		ret = recovery_mmc_stream_add_part(uboot_part, size);
		if (ret)
			goto err;
	} else {
		ret = -EINVAL;
		goto err;
	}

	blksz = recovery_stream.parts[0].part.blksz ?:
		recovery_stream.parts[0].desc->blksz;
	recovery_stream.buf_size = ALIGN(RECOVERY_MMC_STREAM_CHUNK, blksz);
	recovery_stream.buf = memalign(ARCH_DMA_MINALIGN,
				       recovery_stream.buf_size);
	if (!recovery_stream.buf) {
		ret = -ENOMEM;
		goto err;
	}

	prog_phase = 1;
	prog_done = 0;
	prog_erase_done = 0;
	prog_erase_total = (target == TARGET_FIRMWARE ? 3 : 1) *
		RECOVERY_MMC_ERASE_PROGRESS_STEPS;
	prog_write_done = 0;
	prog_write_total = size;
	prog_total = prog_erase_total + prog_write_total;

	if (target == TARGET_FIRMWARE) {
		ret = recovery_mmc_erase_part(kernel_part,
					      recovery_runtime_status_leds, 0);
		if (ret)
			goto err;
		ret = recovery_mmc_erase_part(rootfs_part,
					      recovery_runtime_status_leds,
					      RECOVERY_MMC_ERASE_PROGRESS_STEPS);
		if (ret)
			goto err;
		ret = recovery_mmc_erase_part(data_part,
					      recovery_runtime_status_leds,
					      2 * RECOVERY_MMC_ERASE_PROGRESS_STEPS);
	} else {
		ret = recovery_mmc_erase_part(uboot_part,
					      recovery_runtime_status_leds, 0);
	}
	if (ret)
		goto err;

	prog_phase = 4;
	prog_erase_done = prog_erase_total;
	prog_done = prog_erase_done;
	recovery_stream.active = true;
	recovery_stream.prepared = true;
	recovery_debug_printf("Destructive eMMC stream prepared after complete target erase\n");
	return 0;

err:
	prog_phase = -1;
	recovery_mmc_stream_reset();
	return ret;
}

static int recovery_mmc_stream_flush(void)
{
	struct recovery_mmc_stream_part *stream_part;
	ulong blksz;
	size_t write_len;
	lbaint_t blk, blkcnt;

	if (!recovery_stream.active || recovery_stream.error)
		return recovery_stream.error ?: -EINVAL;
	if (!recovery_stream.buf_used)
		return 0;
	if (recovery_stream.part_index >= recovery_stream.part_count)
		return -EFBIG;

	stream_part = &recovery_stream.parts[recovery_stream.part_index];
	blksz = stream_part->part.blksz ?: stream_part->desc->blksz;
	write_len = ALIGN(recovery_stream.buf_used, blksz);
	if (write_len > recovery_stream.buf_used)
		memset(recovery_stream.buf + recovery_stream.buf_used, 0,
		       write_len - recovery_stream.buf_used);

	blk = stream_part->part.start + stream_part->written / blksz;
	blkcnt = write_len / blksz;
	if (blk_dwrite(stream_part->desc, blk, blkcnt,
		       recovery_stream.buf) != blkcnt) {
		printf("Destructive stream write failed at '%s' block " LBAF "\n",
		       stream_part->spec, blk);
		recovery_stream.error = -EIO;
		return -EIO;
	}

	stream_part->written += recovery_stream.buf_used;
	prog_write_done += recovery_stream.buf_used;
	prog_done = prog_erase_done + prog_write_done;
	recovery_stream.buf_used = 0;
	return 0;
}

static int recovery_mmc_stream_append_raw(const void *data, size_t size)
{
	const u8 *src = data;

	while (size) {
		struct recovery_mmc_stream_part *stream_part;
		size_t consumed, remaining, space, copy;
		int ret;

		if (recovery_stream.part_index >= recovery_stream.part_count)
			return -EFBIG;
		stream_part = &recovery_stream.parts[recovery_stream.part_index];
		consumed = stream_part->written + recovery_stream.buf_used;
		remaining = stream_part->expected - consumed;
		space = recovery_stream.buf_size - recovery_stream.buf_used;
		copy = min(size, min(remaining, space));
		if (!copy)
			return -EIO;

		memcpy(recovery_stream.buf + recovery_stream.buf_used, src, copy);
		recovery_stream.buf_used += copy;
		src += copy;
		size -= copy;
		consumed += copy;

		if (recovery_stream.buf_used == recovery_stream.buf_size ||
		    consumed == stream_part->expected) {
			ret = recovery_mmc_stream_flush();
			if (ret)
				return ret;
			if (stream_part->written == stream_part->expected)
				recovery_stream.part_index++;
		}
	}

	return 0;
}

static int recovery_mmc_stream_tar_header(void)
{
	const struct recovery_tar_header *hdr =
		(const struct recovery_tar_header *)recovery_stream.tar_header;
	struct recovery_mmc_stream_part *stream_part;
	unsigned long long capacity;
	char name[256];
	size_t entry_size;
	int part_index = -1;
	bool regular;
	int ret;

	if (recovery_tar_header_empty(hdr)) {
		recovery_stream.tar_end = true;
		return 0;
	}
	if (memcmp(hdr->magic, "ustar", 5)) {
		printf("Streamed sysupgrade has an invalid tar header\n");
		return -EINVAL;
	}
	ret = recovery_tar_verify_checksum(hdr);
	if (ret) {
		printf("Streamed sysupgrade has an invalid tar checksum\n");
		return ret;
	}
	ret = recovery_tar_octal(hdr->size, sizeof(hdr->size), &entry_size);
	if (ret)
		return ret;

	recovery_tar_name(hdr, name, sizeof(name));
	regular = hdr->typeflag == '\0' || hdr->typeflag == '0';
	recovery_stream.tar_control_active = false;
	if (recovery_board_is_sbe1v1k() && regular &&
	    recovery_is_control_name(name)) {
		if (recovery_stream.tar_seen_control ||
		    recovery_stream.tar_seen_kernel || !entry_size ||
		    entry_size > sizeof(recovery_stream.tar_control))
			return -EINVAL;
		recovery_stream.tar_seen_control = true;
		recovery_stream.tar_control_active = true;
		recovery_stream.tar_control_used = 0;
	} else if (regular && recovery_is_kernel_name(name)) {
		if (recovery_board_is_sbe1v1k() &&
		    !recovery_stream.tar_seen_control) {
			printf("SBE1V1K sysupgrade CONTROL must precede the kernel payload\n");
			return -EINVAL;
		}
		if (recovery_stream.tar_seen_kernel || recovery_stream.part_index != 0)
			return -EINVAL;
		part_index = 0;
		recovery_stream.tar_seen_kernel = true;
	} else if (regular && recovery_is_rootfs_name(name)) {
		if (!recovery_stream.tar_seen_kernel ||
		    recovery_stream.tar_seen_rootfs ||
		    recovery_stream.part_index != 1)
			return -EINVAL;
		part_index = 1;
		recovery_stream.tar_seen_rootfs = true;
	}

	if (part_index >= 0) {
		if (!entry_size)
			return -EINVAL;
		stream_part = &recovery_stream.parts[part_index];
		capacity = recovery_mmc_part_bytes(&stream_part->part);
		if (entry_size > capacity) {
			printf("Tar payload '%s' size %lu exceeds eMMC partition '%s' size %llu\n",
			       name, (ulong)entry_size, stream_part->spec, capacity);
			return -EFBIG;
		}
		stream_part->expected = entry_size;
	}

	recovery_stream.tar_part_index = part_index;
	recovery_stream.tar_entry_remaining = entry_size;
	recovery_stream.tar_padding_remaining = ALIGN(entry_size, 512) - entry_size;
	return 0;
}

static int recovery_mmc_stream_finish_control(void)
{
	const void *control = recovery_stream.tar_control;
	size_t control_size = recovery_stream.tar_control_used;
	int ret;

	ret = recovery_validate_sbe1v1k_tar_control(control, control_size);
	if (!ret)
		recovery_stream.tar_control_active = false;

	return ret;
}

static int recovery_mmc_stream_append_tar(const void *data, size_t size)
{
	const u8 *src = data;

	while (size) {
		size_t copy;
		int ret;

		if (recovery_stream.tar_end) {
			recovery_stream.input_received += size;
			break;
		}
		if (recovery_stream.tar_entry_remaining) {
			copy = min(size, recovery_stream.tar_entry_remaining);
			if (recovery_stream.tar_part_index >= 0) {
				ret = recovery_mmc_stream_append_raw(src, copy);
				if (ret)
					return ret;
			} else if (recovery_stream.tar_control_active) {
				if (copy > sizeof(recovery_stream.tar_control) -
				    recovery_stream.tar_control_used)
					return -EFBIG;
				memcpy(recovery_stream.tar_control +
				       recovery_stream.tar_control_used, src, copy);
				recovery_stream.tar_control_used += copy;
			}
			recovery_stream.tar_entry_remaining -= copy;
			recovery_stream.input_received += copy;
			src += copy;
			size -= copy;
			if (!recovery_stream.tar_entry_remaining) {
				if (recovery_stream.tar_control_active) {
					ret = recovery_mmc_stream_finish_control();
					if (ret)
						return ret;
				}
				recovery_stream.tar_part_index = -1;
			}
			continue;
		}
		if (recovery_stream.tar_padding_remaining) {
			copy = min(size, recovery_stream.tar_padding_remaining);
			recovery_stream.tar_padding_remaining -= copy;
			recovery_stream.input_received += copy;
			src += copy;
			size -= copy;
			continue;
		}

		copy = min(size, sizeof(recovery_stream.tar_header) -
			   recovery_stream.tar_header_used);
		memcpy(recovery_stream.tar_header + recovery_stream.tar_header_used,
		       src, copy);
		recovery_stream.tar_header_used += copy;
		recovery_stream.input_received += copy;
		src += copy;
		size -= copy;
		if (recovery_stream.tar_header_used ==
		    sizeof(recovery_stream.tar_header)) {
			ret = recovery_mmc_stream_tar_header();
			recovery_stream.tar_header_used = 0;
			if (ret)
				return ret;
		}
	}

	prog_write_done = recovery_stream.input_received;
	prog_done = prog_erase_done + prog_write_done;
	return 0;
}

static int recovery_mmc_stream_append(const void *data, size_t size)
{
	size_t remaining;
	int ret;

	if (recovery_stream.input_received > recovery_stream.total_expected)
		return -EFBIG;
	remaining = recovery_stream.total_expected -
		recovery_stream.input_received;
	if (size > remaining)
		return -EFBIG;

	if (recovery_stream.format == RECOVERY_STREAM_TAR) {
		ret = recovery_mmc_stream_append_tar(data, size);
	} else {
		ret = recovery_mmc_stream_append_raw(data, size);
		if (!ret)
			recovery_stream.input_received += size;
	}
	if (ret)
		recovery_stream.error = ret;
	return ret;
}

static int
recovery_mmc_stream_verify_kernel_fit(const struct recovery_mmc_stream_part *part)
{
	struct recovery_image_part kernel;
	u8 *header;
	u8 *fit;
	ulong blksz;
	size_t fit_size;
	size_t read_size;
	lbaint_t blocks;
	int ret;

	blksz = part->part.blksz ?: part->desc->blksz;
	if (!blksz || !part->expected)
		return -EINVAL;

	header = memalign(ARCH_DMA_MINALIGN, blksz);
	if (!header)
		return -ENOMEM;
	ret = 0;
	if (blk_dread(part->desc, part->part.start, 1, header) != 1) {
		ret = -EIO;
		goto out_header;
	}
	/*
	 * The first sector only contains the FDT header. With FIT_FULL_CHECK,
	 * fit_check_format() also walks the complete blob, so use the header
	 * check here and verify the complete FIT after its full readback below.
	 */
	if (fdt_check_header(header)) {
		printf("Streamed kernel is not a FIT image after eMMC write\n");
		ret = -ENOEXEC;
		goto out_header;
	}

	fit_size = fit_get_size(header);
	if (!fit_size || fit_size > part->expected ||
	    fit_size > recovery_mmc_part_bytes(&part->part)) {
		printf("Streamed FIT size is outside the written kernel partition\n");
		ret = -EFBIG;
		goto out_header;
	}
	read_size = ALIGN(fit_size, blksz);
	blocks = read_size / blksz;
	fit = memalign(ARCH_DMA_MINALIGN, read_size);
	if (!fit) {
		ret = -ENOMEM;
		goto out_header;
	}
	if (blk_dread(part->desc, part->part.start, blocks, fit) != blocks) {
		ret = -EIO;
		goto out_fit;
	}

	kernel.data = fit;
	kernel.size = fit_size;
	ret = recovery_validate_sbe1v1k_kernel_fit(&kernel);

out_fit:
	free(fit);
out_header:
	free(header);
	return ret;
}

static int
recovery_mmc_stream_verify_rootfs(const struct recovery_mmc_stream_part *part)
{
	struct recovery_image_part rootfs;
	u8 *header;
	ulong blksz;
	int ret;

	blksz = part->part.blksz ?: part->desc->blksz;
	if (!blksz || part->expected < RECOVERY_SQUASHFS_HEADER_MIN)
		return -EINVAL;

	header = memalign(ARCH_DMA_MINALIGN, blksz);
	if (!header)
		return -ENOMEM;
	if (blk_dread(part->desc, part->part.start, 1, header) != 1) {
		free(header);
		return -EIO;
	}

	rootfs.data = header;
	rootfs.size = part->expected;
	ret = recovery_validate_sbe1v1k_rootfs(&rootfs);
	free(header);

	return ret;
}

static int recovery_mmc_stream_verify_sbe1v1k_firmware(void)
{
	int ret;

	if (!recovery_board_is_sbe1v1k() ||
	    recovery_stream.target != TARGET_FIRMWARE)
		return 0;

	ret = recovery_mmc_stream_verify_kernel_fit(&recovery_stream.parts[0]);
	if (ret)
		return ret;

	return recovery_mmc_stream_verify_rootfs(&recovery_stream.parts[1]);
}

static int recovery_mmc_stream_finish(void)
{
	int ret;

	ret = recovery_mmc_stream_flush();
	if (ret)
		return ret;
	if (recovery_stream.input_received != recovery_stream.total_expected)
		return -EIO;
	if (recovery_stream.format == RECOVERY_STREAM_TAR) {
		if (!recovery_stream.tar_end ||
		    (recovery_board_is_sbe1v1k() &&
		     !recovery_stream.tar_seen_control) ||
		    !recovery_stream.tar_seen_kernel ||
		    !recovery_stream.tar_seen_rootfs ||
		    recovery_stream.parts[0].written !=
			recovery_stream.parts[0].expected ||
		    recovery_stream.parts[1].written !=
			recovery_stream.parts[1].expected)
			return -EINVAL;
		prog_write_done = prog_write_total;
		prog_done = prog_total;
	} else if (recovery_stream.part_index != recovery_stream.part_count ||
		   prog_write_done != prog_write_total) {
		return -EIO;
	}

	return recovery_mmc_stream_verify_sbe1v1k_firmware();
}

struct recovery_factory_part {
	const char *name;
	lbaint_t start;
	lbaint_t size;
};

/*
 * The first 25 entries are invariant across the supported factory, mainline,
 * and large layouts.  Do not infer a missing HLOS_1 from labels alone: QSDK
 * code can depend on these numeric partition indices.
 */
static const struct recovery_factory_part sbe1v1k_factory_parts[] = {
	{ "0:SBL1", 34, 2048 },
	{ "0:SBL1_1", 2082, 2048 },
	{ "0:BOOTCONFIG", 4130, 1024 },
	{ "0:BOOTCONFIG1", 5154, 1024 },
	{ "0:QSEE", 6178, 6144 },
	{ "0:QSEE_1", 12322, 6144 },
	{ "0:DEVCFG", 18466, 1024 },
	{ "0:DEVCFG_1", 19490, 1024 },
	{ "0:APDP", 20514, 1024 },
	{ "0:APDP_1", 21538, 1024 },
	{ "0:TME", 22562, 1024 },
	{ "0:TME_1", 23586, 1024 },
	{ "0:RPM", 24610, 1024 },
	{ "0:RPM_1", 25634, 1024 },
	{ "0:CDT", 26658, 1024 },
	{ "0:CDT_1", 27682, 1024 },
	{ "0:APPSBLENV", 28706, 512 },
	{ "0:APPSBL", 29218, 4096 },
	{ "0:APPSBL_1", 33314, 4096 },
	{ "0:ART", 37410, 2048 },
	{ "0:ETHPHYFW", 39458, 1024 },
	{ "0:LICENSE", 40482, 512 },
	{ "0:WIFIFW", 40994, 20480 },
	{ "0:WIFIFW_1", 61474, 20480 },
	{ "0:HLOS", RECOVERY_SBE1V1K_HLOS_START,
	  RECOVERY_SBE1V1K_HLOS_SIZE },
};

struct recovery_sbe1v1k_gpt_part {
	const char *name;
	lbaint_t start;
	lbaint_t size;
	const char *type_guid;
};

/* Factory-compatible tail used by the OpenWrt mainline and QWRT profiles. */
static const struct recovery_sbe1v1k_gpt_part sbe1v1k_mainline_tail[] = {
	{ "rootfs", 110626, 249856, "98D2248D-7140-449F-A954-39D67BD6C3B4" },
	{ "rootfs_1", 360482, 249856, "5647B280-DC2A-485D-9913-CF53AC40FA32" },
	{ "rootfs_data", 610338, 1048576, "AB1760DA-A8BB-4D6F-98D2-9AD3AB9009CD" },
	{ "rootfs_data_1", 1658914, 1048576, "1119CEE3-A4F1-43D0-90D1-2D226E401189" },
	{ "econfig", 2707490, 16384, "E0AAF192-CAD4-4128-9F05-D2831CA67B58" },
	{ "edata", 2723874, 32768, "A917AB5A-6C83-48AF-95DB-1004BC925C52" },
	{ "log", 2756642, 262144, "2DD625CE-BC7A-4B6A-933A-57BC0D338E9C" },
	{ "persist", 3018786, 32768, "2F4BBC39-E2DD-4882-87C9-8A402ACE35B2" },
	{ "usr_app", 3051554, 2097152, "BD65288F-D717-4C23-8F2B-EAD6A8A229EB" },
	{ "tls", 5148706, 8192, "2CBA182A-D45E-4F7D-8B40-C8E206AA227A" },
	{ "backup_tls", 5156898, 8192, "1094C6FF-3A45-4BA1-ACC5-F94717881E0F" },
	{ "bypass_cert", 5165090, 4096, "F6B84FE9-4030-44F6-8842-42CF6D6AB37A" },
	{ "rsvd_1", 5169186, 32768, "8ECB9DD2-5907-4D9E-87AF-7EDC799AC7BE" },
	{ "rsvd_2", 5201954, 65536, "5EB110B9-88EF-461F-8867-E601A9E12AC7" },
	{ "rsvd_3", 5267490, 131072, "27E70C71-5403-4CD7-8B7D-D53CBAF9BD89" },
	{ "user_data", 5398562, 9850846, "173F1230-BAB4-4505-8C1C-665E95867EA2" },
	/* Fill to the last usable LBA, leaving room for a valid secondary GPT. */
	{ "ASKEYMFC", 15249408, 0, "D45FCE56-5F25-410E-B914-87CB2C8318F4" },
};

static const struct recovery_sbe1v1k_layout_desc *
recovery_sbe1v1k_layout_desc(enum recovery_sbe1v1k_layout layout)
{
	switch (layout) {
	case RECOVERY_SBE1V1K_LAYOUT_MAINLINE:
		return &sbe1v1k_layout_mainline;
	case RECOVERY_SBE1V1K_LAYOUT_LARGE:
		return &sbe1v1k_layout_large;
	case RECOVERY_SBE1V1K_LAYOUT_QWRT:
		return &sbe1v1k_layout_qwrt;
	default:
		return NULL;
	}
}

static int recovery_appendf(char *buf, size_t size, size_t *offp,
			    const char *fmt, ...)
{
	va_list ap;
	int len;

	if (*offp >= size)
		return -ENOSPC;

	va_start(ap, fmt);
	len = vsnprintf(buf + *offp, size - *offp, fmt, ap);
	va_end(ap);

	if (len < 0)
		return len;
	if (*offp + len >= size)
		return -ENOSPC;

	*offp += len;
	return 0;
}

static int recovery_append_gpt_part(char *buf, size_t size, size_t *offp,
				    const char *name,
				    unsigned long long start_lba,
				    unsigned long long lba_count,
				    unsigned long blksz,
				    const char *type_guid,
				    const char *uuid,
				    bool bootable)
{
	int ret;

	ret = recovery_appendf(buf, size, offp, "name=%s,start=0x%llx,",
			       name, start_lba * (unsigned long long)blksz);
	if (ret)
		return ret;

	if (lba_count)
		ret = recovery_appendf(buf, size, offp, "size=0x%llx",
				       lba_count * (unsigned long long)blksz);
	else
		ret = recovery_appendf(buf, size, offp, "size=-");
	if (ret)
		return ret;

	if (type_guid && *type_guid) {
		ret = recovery_appendf(buf, size, offp, ",type=%s",
				       type_guid);
		if (ret)
			return ret;
	}

	if (uuid && *uuid) {
		ret = recovery_appendf(buf, size, offp, ",uuid=%s", uuid);
		if (ret)
			return ret;
	}

	if (bootable) {
		ret = recovery_appendf(buf, size, offp, ",bootable");
		if (ret)
			return ret;
	}

	return recovery_appendf(buf, size, offp, ";");
}

static int recovery_append_existing_gpt_part(char *buf, size_t size,
					    size_t *offp,
					    const struct disk_partition *part)
{
	const char *type_guid = NULL;
	const char *uuid = NULL;

	if (IS_ENABLED(CONFIG_PARTITION_TYPE_GUID))
		type_guid = disk_partition_type_guid(part);
	if (CONFIG_IS_ENABLED(PARTITION_UUIDS))
		uuid = disk_partition_uuid(part);

	return recovery_append_gpt_part(buf, size, offp,
					(const char *)part->name,
					part->start, part->size, part->blksz,
					type_guid, uuid,
					part->bootable & PART_BOOTABLE);
}

static int recovery_get_mmc_disk_guid(char *buf, size_t size)
{
	const char *guid;
	int ret;

	env_set("sbe1v1k_disk_guid", NULL);
	ret = run_commandf("gpt guid mmc %s sbe1v1k_disk_guid",
			   recovery_mmcdev());
	if (ret)
		return ret;

	guid = env_get("sbe1v1k_disk_guid");
	if (!guid || !*guid) {
		env_set("sbe1v1k_disk_guid", NULL);
		return -ENOENT;
	}

	strlcpy(buf, guid, size);
	env_set("sbe1v1k_disk_guid", NULL);
	return 0;
}

static int recovery_check_sbe1v1k_prefix(struct blk_desc **descp,
					  bool *has_hlos_1p)
{
	struct blk_desc *desc;
	struct disk_partition part;
	size_t prefix_count = 0;
	size_t i;
	int ret;
	int p;

	ret = blk_get_device_by_str("mmc", recovery_mmcdev(), &desc);
	if (ret < 0)
		return ret;
	ret = blk_dselect_hwpart(desc, EMMC_HWPART_DEFAULT);
	if (ret)
		return ret;
	if (desc->blksz != 512) {
		printf("SBE1V1K GPT requires 512-byte logical blocks, found %lu\n",
		       desc->blksz);
		return -EINVAL;
	}

	for (p = 1; p <= MAX_SEARCH_PARTITIONS; p++) {
		ret = part_get_info(desc, p, &part);
		if (ret || !part.size ||
		    part.start >= RECOVERY_SBE1V1K_CHAINLOADER_START)
			continue;

		if (part.size > RECOVERY_SBE1V1K_CHAINLOADER_START -
								part.start) {
			printf("Partition %d '%s' overlaps the HLOS_1/chainloader boundary\n",
			       p, (const char *)part.name);
			return -EINVAL;
		}
		prefix_count++;
	}

	if (prefix_count != ARRAY_SIZE(sbe1v1k_factory_parts) &&
	    prefix_count != ARRAY_SIZE(sbe1v1k_factory_parts) + 1) {
		printf("SBE1V1K prefix requires exactly 25 or 26 partitions before LBA %llu, found %zu\n",
		       RECOVERY_SBE1V1K_CHAINLOADER_START, prefix_count);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(sbe1v1k_factory_parts); i++) {
		const struct recovery_factory_part *expect =
			&sbe1v1k_factory_parts[i];

		ret = part_get_info(desc, i + 1, &part);
		if (ret || strcmp((const char *)part.name, expect->name) ||
		    part.start != expect->start || part.size != expect->size) {
			if (ret)
				printf("SBE1V1K prefix partition P%zu is missing\n",
				       i + 1);
			else
				printf("SBE1V1K prefix mismatch at P%zu: '%s' start "
				       LBAF " size " LBAF "\n", i + 1,
				       (const char *)part.name, part.start, part.size);
			return ret ?: -EINVAL;
		}
	}

	if (prefix_count == ARRAY_SIZE(sbe1v1k_factory_parts) + 1) {
		p = (int)ARRAY_SIZE(sbe1v1k_factory_parts) + 1;
		ret = part_get_info(desc, p, &part);
		if (ret || strcmp((const char *)part.name, "0:HLOS_1") ||
		    part.start != RECOVERY_SBE1V1K_HLOS_1_START ||
		    part.size != RECOVERY_SBE1V1K_HLOS_1_SIZE) {
			printf("SBE1V1K P26 must be the standard 0:HLOS_1 partition\n");
			return ret ?: -EINVAL;
		}
		*has_hlos_1p = true;
	} else {
		*has_hlos_1p = false;
	}

	if (descp)
		*descp = desc;
	return 0;
}

static int recovery_build_sbe1v1k_gpt(
	const struct recovery_sbe1v1k_layout_desc *layout, char **gptp)
{
	char disk_guid[UUID_STR_LEN + 1];
	struct blk_desc *desc;
	char *gpt;
	size_t off = 0;
	bool has_hlos_1 = false;
	size_t preserved = 0;
	int ret;
	int p;

	ret = recovery_check_sbe1v1k_prefix(&desc, &has_hlos_1);
	if (ret)
		return ret;

	ret = recovery_get_mmc_disk_guid(disk_guid, sizeof(disk_guid));
	if (ret)
		return ret;

	gpt = malloc(RECOVERY_SBE1V1K_GPT_MAX);
	if (!gpt)
		return -ENOMEM;
	gpt[0] = '\0';

	ret = recovery_appendf(gpt, RECOVERY_SBE1V1K_GPT_MAX, &off,
			       "uuid_disk=%s;", disk_guid);
	if (ret)
		goto err;

	for (p = 1; p <= MAX_SEARCH_PARTITIONS; p++) {
		struct disk_partition part;
		lbaint_t end;

		ret = part_get_info(desc, p, &part);
		if (ret)
			continue;
		if (!part.size)
			continue;

		end = part.start + part.size;
		if (part.start >= RECOVERY_SBE1V1K_CHAINLOADER_START)
			continue;
		if (end > RECOVERY_SBE1V1K_CHAINLOADER_START) {
			printf("Partition %d '%s' overlaps chainloader start: "
			       "start " LBAF " size " LBAF "\n",
			       p, (const char *)part.name, part.start,
			       part.size);
			ret = -EINVAL;
			goto err;
		}

		ret = recovery_append_existing_gpt_part(gpt,
							RECOVERY_SBE1V1K_GPT_MAX,
							&off, &part);
		if (ret)
			goto err;
		if (!strcmp((const char *)part.name, "0:HLOS_1"))
			has_hlos_1 = true;
		preserved++;
	}

	if (preserved != ARRAY_SIZE(sbe1v1k_factory_parts) + has_hlos_1) {
		printf("SBE1V1K prefix changed while building the target GPT\n");
		ret = -EINVAL;
		goto err;
	}

	if (!has_hlos_1) {
		ret = recovery_append_gpt_part(
			gpt, RECOVERY_SBE1V1K_GPT_MAX, &off,
			"0:HLOS_1", RECOVERY_SBE1V1K_HLOS_1_START,
			RECOVERY_SBE1V1K_HLOS_1_SIZE, desc->blksz,
			RECOVERY_SBE1V1K_HLOS_1_TYPE_GUID, NULL, false);
		if (ret)
			goto err;
	}

	if (layout->id == RECOVERY_SBE1V1K_LAYOUT_MAINLINE ||
	    layout->id == RECOVERY_SBE1V1K_LAYOUT_QWRT) {
		size_t i;

		for (i = 0; i < ARRAY_SIZE(sbe1v1k_mainline_tail); i++) {
			const struct recovery_sbe1v1k_gpt_part *part =
				&sbe1v1k_mainline_tail[i];

			ret = recovery_append_gpt_part(gpt,
						       RECOVERY_SBE1V1K_GPT_MAX,
						       &off, part->name,
						       part->start, part->size,
						       desc->blksz, part->type_guid,
						       NULL, false);
			if (ret)
				goto err;
		}
	} else {
		ret = recovery_append_gpt_part(gpt,
					       RECOVERY_SBE1V1K_GPT_MAX, &off,
					       "chainloader", layout->uboot_start,
					       layout->uboot_size, desc->blksz,
					       RECOVERY_GPT_TYPE_BASIC_DATA,
					       NULL, false);
		if (ret)
			goto err;

		ret = recovery_append_gpt_part(gpt,
					       RECOVERY_SBE1V1K_GPT_MAX, &off,
					       "kernel", layout->kernel_start,
					       layout->kernel_size, desc->blksz,
					       RECOVERY_GPT_TYPE_BASIC_DATA,
					       NULL, false);
		if (ret)
			goto err;

		ret = recovery_append_gpt_part(gpt,
					       RECOVERY_SBE1V1K_GPT_MAX, &off,
					       "rootfs", layout->rootfs_start,
					       layout->rootfs_size, desc->blksz,
					       RECOVERY_GPT_TYPE_LINUX_FS,
					       NULL, false);
		if (ret)
			goto err;

		ret = recovery_append_gpt_part(gpt,
					       RECOVERY_SBE1V1K_GPT_MAX, &off,
					       "rootfs_data", layout->data_start, 0,
					       desc->blksz,
					       RECOVERY_GPT_TYPE_LINUX_FS,
					       NULL, false);
		if (ret)
			goto err;
	}

	recovery_debug_printf("Building SBE1V1K '%s' GPT; preserving %zu partitions before LBA %llu\n",
			      layout->name, preserved,
			      RECOVERY_SBE1V1K_CHAINLOADER_START);
	*gptp = gpt;
	return 0;

err:
	free(gpt);
	return ret;
}

static int recovery_verify_factory_gpt(
	struct recovery_status_led_ctrl *status_leds, bool *has_hlos_1p)
{
	int ret;

	prog_phase = 1;
	prog_erase_done = 0;
	prog_erase_total = ARRAY_SIZE(sbe1v1k_factory_parts);
	prog_write_done = 0;
	prog_write_total = 0;
	prog_total = prog_erase_total;
	prog_done = 0;

	ret = recovery_check_sbe1v1k_prefix(NULL, has_hlos_1p);
	if (ret)
		return ret;

	prog_erase_done = prog_erase_total;
	prog_done = prog_erase_done;
	recovery_service_runtime(status_leds);

	return 0;
}

static int recovery_clone_sbe1v1k_hlos_1(
	struct recovery_status_led_ctrl *status_leds, size_t progress_base)
{
	struct disk_partition hlos;
	struct blk_desc *desc;
	lbaint_t chunk_blocks;
	lbaint_t copied = 0;
	ulong chunk_size;
	u8 *source_buf;
	u8 *verify_buf;
	int ret;

	ret = recovery_get_mmc_part("0#0:HLOS", &desc, &hlos);
	if (ret)
		return ret;
	if (hlos.start != RECOVERY_SBE1V1K_HLOS_START ||
	    hlos.size != RECOVERY_SBE1V1K_HLOS_SIZE ||
	    (hlos.blksz ?: desc->blksz) != desc->blksz) {
		printf("Refusing to clone a non-standard 0:HLOS partition\n");
		return -EINVAL;
	}

	chunk_size = ALIGN(RECOVERY_MMC_WRITE_CHUNK, desc->blksz);
	chunk_blocks = chunk_size / desc->blksz;
	source_buf = memalign(ARCH_DMA_MINALIGN, chunk_size);
	verify_buf = memalign(ARCH_DMA_MINALIGN, chunk_size);
	if (!source_buf || !verify_buf) {
		free(source_buf);
		free(verify_buf);
		return -ENOMEM;
	}

	while (copied < hlos.size) {
		lbaint_t todo = hlos.size - copied;

		if (todo > chunk_blocks)
			todo = chunk_blocks;
		if (blk_dread(desc, hlos.start + copied, todo, source_buf) != todo) {
			printf("Failed to read 0:HLOS while creating 0:HLOS_1\n");
			ret = -EIO;
			goto out;
		}
		if (blk_dwrite(desc, RECOVERY_SBE1V1K_HLOS_1_START + copied,
			       todo, source_buf) != todo) {
			printf("Failed to write the new 0:HLOS_1 data\n");
			ret = -EIO;
			goto out;
		}
		memset(verify_buf, 0, todo * desc->blksz);
		if (blk_dread(desc, RECOVERY_SBE1V1K_HLOS_1_START + copied,
			      todo, verify_buf) != todo ||
		    memcmp(source_buf, verify_buf, todo * desc->blksz)) {
			printf("0:HLOS_1 read-back verification failed\n");
			ret = -EIO;
			goto out;
		}

		copied += todo;
		prog_write_done = progress_base + (size_t)copied * desc->blksz;
		prog_done = prog_erase_done + prog_write_done;
		recovery_service_runtime(status_leds);
	}

	recovery_debug_printf("Cloned 0:HLOS into the missing 0:HLOS_1 slot\n");
	ret = 0;

out:
	free(source_buf);
	free(verify_buf);
	return ret;
}

/*
 * Some factory/QSDK images omit 0:HLOS_1 and use a different tail.  They
 * still retain the immutable boot-chain prefix and the stock rsvd_2 boot
 * location.  Recognise only that exact combination so recovery can migrate
 * it without treating the unverified tail as a writable firmware profile.
 */
static int recovery_verify_sbe1v1k_factory_boot_path(void)
{
	struct disk_partition part;
	struct blk_desc *desc;
	bool has_hlos_1;
	int ret;

	ret = recovery_check_sbe1v1k_prefix(&desc, &has_hlos_1);
	if (ret)
		return ret;
	if (has_hlos_1)
		return -EINVAL;

	ret = recovery_get_mmc_part_ex(RECOVERY_SBE1V1K_FACTORY_UBOOT_PART,
					       &desc, &part, true);
	if (ret || part.start != RECOVERY_SBE1V1K_MAINLINE_UBOOT_START ||
	    part.size != RECOVERY_SBE1V1K_MAINLINE_UBOOT_SIZE)
		return ret ?: -EINVAL;

	return 0;
}

static bool recovery_sbe1v1k_hlos_1_missing(void)
{
	struct disk_partition part;
	struct blk_desc *desc;

	return recovery_get_mmc_part_ex("0#0:HLOS_1", &desc, &part, true);
}

static int recovery_verify_sbe1v1k_gpt(
	const struct recovery_sbe1v1k_layout_desc *layout,
	bool require_hlos_1)
{
	struct disk_partition part;
	struct blk_desc *desc;
	bool has_hlos_1;
	int ret;

	if (require_hlos_1) {
		ret = recovery_check_sbe1v1k_prefix(&desc, &has_hlos_1);
		if (ret || !has_hlos_1)
			return ret ?: -EINVAL;
	}

	ret = recovery_get_mmc_part_ex("0#0:HLOS", &desc, &part, true);
	if (ret || part.start != RECOVERY_SBE1V1K_HLOS_START ||
	    part.size != RECOVERY_SBE1V1K_HLOS_SIZE)
		return ret ?: -EINVAL;
	if (require_hlos_1 ||
	    layout->id == RECOVERY_SBE1V1K_LAYOUT_MAINLINE ||
	    layout->id == RECOVERY_SBE1V1K_LAYOUT_QWRT) {
		ret = recovery_get_mmc_part_ex("0#0:HLOS_1", &desc, &part,
						   true);
		if (ret || part.start != RECOVERY_SBE1V1K_HLOS_1_START ||
		    part.size != RECOVERY_SBE1V1K_HLOS_1_SIZE)
			return ret ?: -EINVAL;
	}
	if (layout->id == RECOVERY_SBE1V1K_LAYOUT_MAINLINE ||
	    layout->id == RECOVERY_SBE1V1K_LAYOUT_QWRT) {
		ret = part_get_info(desc, 27, &part);
		if (ret || strcmp((const char *)part.name, "rootfs"))
			return ret ?: -EINVAL;
	}

	ret = recovery_get_mmc_part_ex(layout->uboot_part, &desc, &part,
					    true);
	if (ret || part.start != layout->uboot_start ||
	    part.size != layout->uboot_size)
		return ret ?: -EINVAL;

	ret = recovery_get_mmc_part_ex(layout->kernel_part, &desc, &part,
					    true);
	if (ret || part.start != layout->kernel_start ||
	    part.size != layout->kernel_size)
		return ret ?: -EINVAL;

	ret = recovery_get_mmc_part_ex(layout->rootfs_part, &desc, &part,
					    true);
	if (ret || part.start != layout->rootfs_start ||
	    part.size != layout->rootfs_size)
		return ret ?: -EINVAL;

	ret = recovery_get_mmc_part_ex(layout->data_part, &desc, &part,
					    true);
	if (ret || part.start != layout->data_start ||
	    (layout->data_size ? part.size != layout->data_size :
	     part.size < 1048576))
		return ret ?: -EINVAL;

	return 0;
}

static int recovery_apply_sbe1v1k_layout(
	const struct recovery_sbe1v1k_layout_desc *layout)
{
	char bootargs[128];
	int ret;

	if (!layout)
		return -EINVAL;

	if (snprintf(bootargs, sizeof(bootargs),
		     "console=ttyMSM0,115200n8 rootwait root=%s",
		     layout->rootarg) >= sizeof(bootargs))
		return -ENOSPC;

	ret = env_set("recovery_part_kernel", layout->kernel_part);
	ret = ret ?: env_set("recovery_part_rootfs", layout->rootfs_part);
	ret = ret ?: env_set("recovery_part_data", layout->data_part);
	ret = ret ?: env_set("recovery_part_uboot", layout->uboot_part);
	ret = ret ?: env_set("recovery_part_uboot_alt", NULL);
	ret = ret ?: env_set_hex("recovery_kernel_pad", layout->kernel_pad);
	ret = ret ?: env_set("kernpart", layout->kernpart);
	ret = ret ?: env_set("rootpart", layout->rootarg);
	ret = ret ?: env_set("bootargs", bootargs);
	if (ret)
		return ret;

	active_sbe1v1k_layout = layout->id;
	sbe1v1k_factory_pre_migration = false;
	recovery_debug_printf("SBE1V1K partition profile: %s\n", layout->name);
	return 0;
}

static int recovery_apply_sbe1v1k_factory_recovery(void)
{
	int ret;

	ret = env_set("recovery_part_uboot",
		      RECOVERY_SBE1V1K_FACTORY_UBOOT_PART);
	ret = ret ?: env_set("recovery_part_uboot_alt", NULL);
	ret = ret ?: env_set("recovery_part_kernel", NULL);
	ret = ret ?: env_set("recovery_part_rootfs", NULL);
	ret = ret ?: env_set("recovery_part_data", NULL);
	ret = ret ?: env_set("recovery_kernel_pad", NULL);
	if (ret)
		return ret;

	active_sbe1v1k_layout = RECOVERY_SBE1V1K_LAYOUT_UNKNOWN;
	sbe1v1k_factory_pre_migration = true;
	printf("SBE1V1K factory GPT recognised: chainloader target is rsvd_2; migrate the layout before uploading firmware\n");
	return 0;
}

static int recovery_detect_sbe1v1k_layout(void)
{
	sbe1v1k_factory_pre_migration = false;
	/* Avoid probing a non-existent large-layout chainloader on factory GPTs. */
	if (recovery_sbe1v1k_hlos_1_missing() &&
	    !recovery_verify_sbe1v1k_factory_boot_path())
		return recovery_apply_sbe1v1k_factory_recovery();

	if (!recovery_verify_sbe1v1k_gpt(&sbe1v1k_layout_large, false)) {
		/* QWRT is restricted to the factory-compatible geometry below. */
		return recovery_apply_sbe1v1k_layout(&sbe1v1k_layout_large);
	}

	if (!recovery_verify_sbe1v1k_gpt(&sbe1v1k_layout_mainline, false)) {
		enum recovery_sbe1v1k_layout persisted_layout;

		if (!recovery_read_sbe1v1k_layout_marker(&persisted_layout) &&
		    persisted_layout == RECOVERY_SBE1V1K_LAYOUT_QWRT)
			return recovery_apply_sbe1v1k_layout(&sbe1v1k_layout_qwrt);
		return recovery_apply_sbe1v1k_layout(&sbe1v1k_layout_mainline);
	}

	active_sbe1v1k_layout = RECOVERY_SBE1V1K_LAYOUT_UNKNOWN;
	printf("SBE1V1K partition profile is unknown; layout migration is required\n");
	return -ENOENT;
}

static bool recovery_fit_has_image_node(const void *fit, const char *name)
{
	int images;

	images = fdt_path_offset(fit, FIT_IMAGES_PATH);
	if (images < 0)
		return false;

	return fdt_subnode_offset(fit, images, name) >= 0;
}

static bool recovery_is_sbe1v1k_chainloader_fit(const void *fit,
						size_t fit_size)
{
	const char *desc;

	if (fit_check_format(fit, fit_size))
		return false;

	desc = fdt_getprop(fit, 0, FIT_DESC_PROP, NULL);
	if (!desc || !strstr(desc, "SBE1V1K") || !strstr(desc, "chainloader"))
		return false;

	return recovery_fit_has_image_node(fit, "kernel-1") &&
	       recovery_fit_has_image_node(fit, "uboot-1");
}

static bool recovery_ram_range_ok(ulong addr, size_t size)
{
	ulong ram_start = (ulong)gd->ram_base;
	ulong ram_size = (ulong)gd->ram_size;
	ulong ram_end;

	if (!ram_size || addr < ram_start)
		return false;
	if (ram_size > ULONG_MAX - ram_start)
		return false;
	ram_end = ram_start + ram_size;

	return addr <= ram_end && size <= ram_end - addr;
}

static int recovery_copy_running_chainloader_fit(void **fitp,
						 size_t *fit_sizep)
{
	const ulong candidates[] = {
		RECOVERY_SBE1V1K_FIT_PERSISTENT_ADDR,
		RECOVERY_SBE1V1K_FIT_TFTP_ADDR,
	};
	unsigned long max = recovery_uboot_limit();
	size_t i, j;

	for (i = 0; i < ARRAY_SIZE(candidates); i++) {
		const void *fit;
		void *copy;
		size_t fit_size;
		ulong addr = candidates[i];

		if (!addr || !recovery_ram_range_ok(addr, sizeof(struct fdt_header)))
			continue;

		for (j = 0; j < i; j++) {
			if (candidates[j] == addr)
				break;
		}
		if (j != i)
			continue;

		fit = (const void *)addr;
		if (fdt_check_header(fit))
			continue;

		fit_size = fit_get_size(fit);
		if (!fit_size || fit_size > max ||
		    !recovery_ram_range_ok(addr, fit_size))
			continue;

		if (!recovery_is_sbe1v1k_chainloader_fit(fit, fit_size))
			continue;

		copy = malloc(fit_size);
		if (!copy)
			return -ENOMEM;
		memcpy(copy, fit, fit_size);
		if (!recovery_is_sbe1v1k_chainloader_fit(copy, fit_size)) {
			free(copy);
			continue;
		}

		*fitp = copy;
		*fit_sizep = fit_size;
		recovery_debug_printf("Preserved running SBE1V1K chainloader FIT from 0x%08lx (%lu bytes)\n",
				      addr, (ulong)fit_size);
		return 0;
	}

	return -ENOENT;
}

static int recovery_read_installed_chainloader_fit(void **fitp,
					   size_t *fit_sizep)
{
	const struct recovery_sbe1v1k_layout_desc *layout =
		recovery_sbe1v1k_layout_desc(active_sbe1v1k_layout);
	const char *uboot_part;
	struct disk_partition part;
	struct blk_desc *desc;
	unsigned long long capacity;
	unsigned long max = recovery_uboot_limit();
	lbaint_t blocks;
	ulong blksz;
	size_t fit_size;
	size_t read_size;
	u8 *header = NULL;
	u8 *fit = NULL;
	int ret;

	uboot_part = layout ? layout->uboot_part :
		env_get("recovery_part_uboot");
	if (!uboot_part || !*uboot_part)
		return -ENOENT;

	ret = recovery_get_mmc_part(uboot_part, &desc, &part);
	if (ret)
		return ret;

	blksz = part.blksz ?: desc->blksz;
	if (!blksz || !part.size)
		return -EINVAL;
	capacity = (unsigned long long)part.size * blksz;

	header = memalign(ARCH_DMA_MINALIGN, blksz);
	if (!header)
		return -ENOMEM;
	if (blk_dread(desc, part.start, 1, header) != 1) {
		printf("Failed to read chainloader FIT header from '%s'\n",
		       uboot_part);
		ret = -EIO;
		goto out;
	}
	if (fdt_check_header(header)) {
		ret = -ENOEXEC;
		goto out;
	}

	fit_size = fit_get_size(header);
	if (!fit_size || fit_size > max || fit_size > capacity) {
		printf("Installed chainloader FIT size %lu is invalid for '%s'\n",
		       (ulong)fit_size, uboot_part);
		ret = -EFBIG;
		goto out;
	}

	blocks = (fit_size + blksz - 1) / blksz;
	read_size = blocks * blksz;
	fit = memalign(ARCH_DMA_MINALIGN, read_size);
	if (!fit) {
		ret = -ENOMEM;
		goto out;
	}
	if (blk_dread(desc, part.start, blocks, fit) != blocks) {
		printf("Failed to read chainloader FIT from '%s'\n",
		       uboot_part);
		ret = -EIO;
		goto out;
	}
	if (!recovery_is_sbe1v1k_chainloader_fit(fit, fit_size)) {
		printf("Installed image in '%s' is not an SBE1V1K chainloader FIT\n",
		       uboot_part);
		ret = -ENOEXEC;
		goto out;
	}

	*fitp = fit;
	*fit_sizep = fit_size;
	fit = NULL;
	ret = 0;
	recovery_debug_printf("Preserved installed SBE1V1K chainloader FIT from '%s' (%lu bytes)\n",
			      uboot_part, (ulong)fit_size);

out:
	free(fit);
	free(header);
	return ret;
}

static int recovery_preserve_chainloader_fit(void **fitp, size_t *fit_sizep)
{
	int ret;

	ret = recovery_copy_running_chainloader_fit(fitp, fit_sizep);
	if (ret != -ENOENT)
		return ret;

	recovery_debug_printf("No running SBE1V1K chainloader FIT was found in usable RAM; reading the current eMMC partition\n");
	ret = recovery_read_installed_chainloader_fit(fitp, fit_sizep);
	if (ret)
		printf("Cannot preserve the current SBE1V1K chainloader FIT: %d\n",
		       ret);

	return ret;
}

static int recovery_load_appsblenv(u8 **bufp, size_t *env_bytesp,
				   struct blk_desc **descp,
				   struct disk_partition *partp)
{
	struct disk_partition part;
	struct blk_desc *desc;
	unsigned long long capacity;
	u8 *buf;
	int ret;

	/* APPSBLENV is always in the eMMC user area.  A previous backup or
	 * restore request may have left the block device on boot0/boot1. */
	ret = blk_get_device_by_str("mmc", recovery_mmcdev(), &desc);
	if (ret < 0)
		return ret;
	ret = blk_dselect_hwpart(desc, EMMC_HWPART_DEFAULT);
	if (ret)
		return ret;

	ret = recovery_get_mmc_part(RECOVERY_APPSBLENV_PART, &desc, &part);
	if (ret)
		return ret;

	capacity = recovery_mmc_part_bytes(&part);
	if (capacity != RECOVERY_APPSBLENV_SIZE) {
		printf("Unexpected APPSBLENV size %llu, expected %lu\n",
		       capacity, RECOVERY_APPSBLENV_SIZE);
		return -EINVAL;
	}

	buf = memalign(ARCH_DMA_MINALIGN, RECOVERY_APPSBLENV_SIZE);
	if (!buf)
		return -ENOMEM;

	if (blk_dread(desc, part.start, part.size, buf) != part.size) {
		printf("Failed to read APPSBLENV from eMMC\n");
		free(buf);
		return -EIO;
	}

	*bufp = buf;
	*env_bytesp = RECOVERY_APPSBLENV_SIZE;
	if (descp)
		*descp = desc;
	if (partp)
		*partp = part;

	return 0;
}

static int recovery_verify_appsblenv_crc(const u8 *env_buf, size_t env_bytes)
{
	u32 stored_crc;
	u32 calc_crc;

	if (env_bytes <= RECOVERY_ENV_CRC_SIZE)
		return -EINVAL;

	memcpy(&stored_crc, env_buf, sizeof(stored_crc));
	calc_crc = crc32(0, env_buf + RECOVERY_ENV_CRC_SIZE,
			 env_bytes - RECOVERY_ENV_CRC_SIZE);
	if (stored_crc != calc_crc) {
		printf("APPSBLENV CRC mismatch: stored 0x%08x calculated 0x%08x\n",
		       stored_crc, calc_crc);
		return -EINVAL;
	}

	return 0;
}

static int recovery_check_appsblenv(void)
{
	size_t env_bytes;
	u8 *env_buf;
	int ret;

	ret = recovery_load_appsblenv(&env_buf, &env_bytes, NULL, NULL);
	if (ret)
		return ret;

	ret = recovery_verify_appsblenv_crc(env_buf, env_bytes);
	free(env_buf);

	return ret;
}

struct recovery_env_update {
	const char *key;
	const char *value;
};

static bool recovery_env_entry_is_key(const char *entry, const char *key)
{
	size_t key_len = strlen(key);

	return !strncmp(entry, key, key_len) && entry[key_len] == '=';
}

static bool recovery_env_entry_replaced(const char *entry,
					const struct recovery_env_update *updates,
					size_t update_count)
{
	size_t i;

	for (i = 0; i < update_count; i++) {
		if (recovery_env_entry_is_key(entry, updates[i].key))
			return true;
	}

	return false;
}

static int recovery_env_append(char *data, size_t data_size, size_t *offp,
			       const char *key, const char *value)
{
	int len;

	if (*offp >= data_size)
		return -ENOSPC;

	len = snprintf(data + *offp, data_size - *offp, "%s=%s", key, value);
	if (len < 0)
		return len;

	if (*offp + len + 2 > data_size)
		return -ENOSPC;

	*offp += len + 1;
	return 0;
}

static int recovery_update_env_data(u8 *env_data, size_t data_size,
				    const struct recovery_env_update *updates,
				    size_t update_count, const char *priority_key)
{
	char *new_data;
	size_t old_off = 0;
	size_t new_off = 0;
	size_t i;
	bool priority_found = false;
	int ret = 0;

	new_data = malloc(data_size);
	if (!new_data)
		return -ENOMEM;
	memset(new_data, 0, data_size);

	/*
	 * soc_auth.ko only scans the first 65535 bytes of APPSBLENV.  Keep its
	 * entry at the beginning of the serialized environment instead of
	 * appending it after an arbitrary number of vendor variables.
	 */
	if (priority_key) {
		for (i = 0; i < update_count; i++) {
			if (strcmp(updates[i].key, priority_key))
				continue;

			ret = recovery_env_append(new_data, data_size, &new_off,
						  updates[i].key, updates[i].value);
			if (ret)
				goto out;
			priority_found = true;
			break;
		}
	}

	while (old_off < data_size && env_data[old_off]) {
		const char *entry = (const char *)env_data + old_off;
		size_t remain = data_size - old_off;
		size_t entry_len = strnlen(entry, remain);

		if (entry_len == remain) {
			ret = -EINVAL;
			goto out;
		}

		if (!recovery_env_entry_replaced(entry, updates, update_count)) {
			if (new_off + entry_len + 2 > data_size) {
				ret = -ENOSPC;
				goto out;
			}
			memcpy(new_data + new_off, entry, entry_len + 1);
			new_off += entry_len + 1;
		}

		old_off += entry_len + 1;
	}

	for (i = 0; i < update_count; i++) {
		if (priority_found && !strcmp(updates[i].key, priority_key))
			continue;

		ret = recovery_env_append(new_data, data_size, &new_off,
						  updates[i].key, updates[i].value);
		if (ret)
			goto out;
	}

	memcpy(env_data, new_data, data_size);

out:
	free(new_data);
	return ret;
}

static const char *recovery_env_find_value(const u8 *env_data,
						   size_t data_size,
						   const char *key)
{
	size_t off = 0;

	while (off < data_size && env_data[off]) {
		const char *entry = (const char *)env_data + off;
		size_t remain = data_size - off;
		size_t entry_len = strnlen(entry, remain);

		if (entry_len == remain)
			return NULL;

		if (recovery_env_entry_is_key(entry, key))
			return entry + strlen(key) + 1;

		off += entry_len + 1;
	}

	return NULL;
}

/*
 * soc_auth.ko reads at most 65535 bytes from each environment partition.
 * Keep a separate bounded lookup so a valid value beyond that window cannot
 * make U-Boot believe the kernel module will find it.
 */
static const char *recovery_env_find_auth_code(const u8 *env_data,
						       size_t data_size)
{
	size_t scan_size;

	/* The vendor reads the CRC and then scans the remaining bytes. */
	scan_size = min_t(size_t, data_size,
					 RECOVERY_QWRT_ENV_SCAN_BYTES -
					 RECOVERY_ENV_CRC_SIZE);

	return recovery_env_find_value(env_data, scan_size, "auth_code");
}

static int recovery_read_sbe1v1k_layout_marker(
	enum recovery_sbe1v1k_layout *layout)
{
	const char *value;
	size_t env_bytes;
	u8 *env_buf;
	int ret;

	if (!layout)
		return -EINVAL;

	ret = recovery_load_appsblenv(&env_buf, &env_bytes, NULL, NULL);
	if (ret)
		return ret;

	ret = recovery_verify_appsblenv_crc(env_buf, env_bytes);
	if (ret)
		goto out;

	value = recovery_env_find_value(env_buf + RECOVERY_ENV_CRC_SIZE,
					env_bytes - RECOVERY_ENV_CRC_SIZE,
					RECOVERY_SBE1V1K_LAYOUT_ENV);
	if (!value) {
		ret = -ENOENT;
		goto out;
	}

	if (!strcmp(value, "qwrt"))
		*layout = RECOVERY_SBE1V1K_LAYOUT_QWRT;
	else if (!strcmp(value, "large"))
		*layout = RECOVERY_SBE1V1K_LAYOUT_LARGE;
	else if (!strcmp(value, "mainline"))
		*layout = RECOVERY_SBE1V1K_LAYOUT_MAINLINE;
	else
		ret = -EINVAL;

out:
	free(env_buf);
	return ret;
}

static int recovery_verify_env_updates(const u8 *env_data, size_t data_size,
				       const struct recovery_env_update *updates,
				       size_t update_count)
{
	const char *value;
	size_t i;

	for (i = 0; i < update_count; i++) {
		if (!strcmp(updates[i].key, "auth_code"))
			value = recovery_env_find_auth_code(env_data, data_size);
		else
			value = recovery_env_find_value(env_data, data_size,
						updates[i].key);
		if (!value) {
			printf("APPSBLENV missing '%s' after write\n",
			       updates[i].key);
			return -EINVAL;
		}

		if (strcmp(value, updates[i].value)) {
			printf("APPSBLENV mismatch for '%s' after write\n",
			       updates[i].key);
			return -EINVAL;
		}
	}

	return 0;
}

/* Refresh the QWRT identity before any firmware target is erased. */
static int recovery_refresh_qwrt_auth_code(void)
{
	const struct recovery_env_update update = { "auth_code", NULL };
	struct recovery_env_update env_update;
	struct disk_partition part;
	struct blk_desc *desc;
	char auth_code[RECOVERY_QWRT_AUTH_CODE_LEN + 1];
	const char *existing;
	u8 *env_buf = NULL;
	size_t env_bytes;
	u32 crc;
	int ret;

	if (!recovery_board_is_sbe1v1k() || !recovery_backend_is_mmc() ||
	    active_sbe1v1k_layout != RECOVERY_SBE1V1K_LAYOUT_QWRT)
		return 0;

	ret = recovery_qwrt_auth_code(auth_code);
	if (ret)
		return ret;
	env_update = update;
	env_update.value = auth_code;

	ret = recovery_load_appsblenv(&env_buf, &env_bytes, &desc, &part);
	if (ret)
		goto out;
	ret = recovery_verify_appsblenv_crc(env_buf, env_bytes);
	if (ret)
		goto out;

	/* Avoid rewriting the 256 KiB environment on every boot. */
	existing = recovery_env_find_auth_code(env_buf + RECOVERY_ENV_CRC_SIZE,
					       env_bytes - RECOVERY_ENV_CRC_SIZE);
	if (existing && !strcmp(existing, auth_code)) {
		ret = 0;
		goto out;
	}

	ret = recovery_update_env_data(env_buf + RECOVERY_ENV_CRC_SIZE,
				       env_bytes - RECOVERY_ENV_CRC_SIZE,
				       &env_update, 1, "auth_code");
	if (ret)
		goto out;

	crc = crc32(0, env_buf + RECOVERY_ENV_CRC_SIZE,
		    env_bytes - RECOVERY_ENV_CRC_SIZE);
	memcpy(env_buf, &crc, sizeof(crc));
	if (blk_dwrite(desc, part.start, part.size, env_buf) != part.size) {
		ret = -EIO;
		goto out;
	}

	memset(env_buf, 0, env_bytes);
	if (blk_dread(desc, part.start, part.size, env_buf) != part.size) {
		ret = -EIO;
		goto out;
	}
	ret = recovery_verify_appsblenv_crc(env_buf, env_bytes);
	if (!ret)
		ret = recovery_verify_env_updates(env_buf + RECOVERY_ENV_CRC_SIZE,
					  env_bytes - RECOVERY_ENV_CRC_SIZE,
					  &env_update, 1);

out:
	recovery_qwrt_secure_zero(auth_code, sizeof(auth_code));
	if (env_buf) {
		recovery_qwrt_secure_zero(env_buf, env_bytes);
		free(env_buf);
	}
	return ret;
}

/*
 * QWRT is the only profile that consumes auth_code. Detect the profile
 * first so Mainline/Large boots never modify the factory environment.
 */
int recovery_sbe1v1k_prepare_auth(void)
{
	enum recovery_sbe1v1k_layout persisted_layout;
	int ret;

	if (!recovery_board_is_sbe1v1k() || !recovery_backend_is_mmc())
		return 0;

	if (active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_UNKNOWN) {
		/*
		 * A QWRT marker is sufficient to refresh auth_code even if a
		 * vendor GPT variant cannot be fully classified. HTTP recovery
		 * performs the stricter geometry check before any flash operation.
		 */
		ret = recovery_read_sbe1v1k_layout_marker(&persisted_layout);
		if (!ret && persisted_layout == RECOVERY_SBE1V1K_LAYOUT_QWRT)
			active_sbe1v1k_layout = RECOVERY_SBE1V1K_LAYOUT_QWRT;
		else {
			ret = recovery_detect_sbe1v1k_layout();
			if (ret)
				return ret;
		}
	}
	if (active_sbe1v1k_layout != RECOVERY_SBE1V1K_LAYOUT_QWRT)
		return 0;

	return recovery_refresh_qwrt_auth_code();
}

static int recovery_write_appsblenv(struct recovery_status_led_ctrl *status_leds,
				    size_t progress_base,
				    const struct recovery_sbe1v1k_layout_desc *layout)
{
	char bootargs[128];
	char boot_chainloader[128];
	char bootcmd[384];
	char qwrt_auth_code[RECOVERY_QWRT_AUTH_CODE_LEN + 1] = { 0 };
	struct recovery_env_update updates[7] = {
		{ "bootargs", bootargs },
		{ "boot_chainloader", boot_chainloader },
		{ "do_boot", "run boot_chainloader" },
		{ "do_nothing", "true" },
		{ "bootcmd", bootcmd },
	};
	size_t update_count = 5;
	struct disk_partition part;
	struct blk_desc *desc;
	size_t env_bytes;
	u8 *env_buf;
	u32 crc;
	int ret;

	if (snprintf(bootargs, sizeof(bootargs),
		     "console=ttyMSM0,115200n8 rootwait root=%s",
		     layout->rootarg) >= sizeof(bootargs))
		return -ENOSPC;
	if (snprintf(boot_chainloader, sizeof(boot_chainloader),
		     "mmc dev 0 0; mmc read 0x44000000 0x%llx 0x2000; bootm 0x44000000",
		     (unsigned long long)layout->uboot_start) >=
	    sizeof(boot_chainloader))
		return -ENOSPC;
	if (snprintf(bootcmd, sizeof(bootcmd),
		     "echo \"Hit ctrl+c for shell...\"; if sleep 3; then setenv bootargs %s; run do_boot; else run do_nothing; fi;",
		     bootargs) >= sizeof(bootcmd))
		return -ENOSPC;
	if (layout->id == RECOVERY_SBE1V1K_LAYOUT_QWRT) {
		ret = recovery_qwrt_auth_code(qwrt_auth_code);
		if (ret) {
			printf("Cannot derive QWRT auth_code from the device serial: %d\n",
			       ret);
			return ret;
		}
		updates[update_count++] = (struct recovery_env_update){
			"auth_code", qwrt_auth_code
		};
	}
	updates[update_count++] = (struct recovery_env_update){
		RECOVERY_SBE1V1K_LAYOUT_ENV, layout->name
	};

	ret = recovery_load_appsblenv(&env_buf, &env_bytes, &desc, &part);
	if (ret)
		return ret;

	ret = recovery_verify_appsblenv_crc(env_buf, env_bytes);
	if (ret)
		goto out;

	ret = recovery_update_env_data(env_buf + RECOVERY_ENV_CRC_SIZE,
				       env_bytes - RECOVERY_ENV_CRC_SIZE,
				       updates, update_count, "auth_code");
	if (ret)
		goto out;

	crc = crc32(0, env_buf + RECOVERY_ENV_CRC_SIZE,
		    env_bytes - RECOVERY_ENV_CRC_SIZE);
	memcpy(env_buf, &crc, sizeof(crc));

	if (blk_dwrite(desc, part.start, part.size, env_buf) != part.size) {
		printf("Failed to write updated APPSBLENV to eMMC\n");
		ret = -EIO;
		goto out;
	}

	memset(env_buf, 0, env_bytes);
	if (blk_dread(desc, part.start, part.size, env_buf) != part.size) {
		printf("Failed to read back updated APPSBLENV from eMMC\n");
		ret = -EIO;
		goto out;
	}

	ret = recovery_verify_appsblenv_crc(env_buf, env_bytes);
	if (ret)
		goto out;

	ret = recovery_verify_env_updates(env_buf + RECOVERY_ENV_CRC_SIZE,
					  env_bytes - RECOVERY_ENV_CRC_SIZE,
					  updates, update_count);
	if (ret)
		goto out;

	recovery_debug_printf("APPSBLENV update verified\n");

	prog_write_done = progress_base + env_bytes;
	prog_done = prog_erase_done + prog_write_done;
	recovery_service_runtime(status_leds);

out:
	recovery_qwrt_secure_zero(qwrt_auth_code, sizeof(qwrt_auth_code));
	free(env_buf);
	return ret;
}

static int recovery_select_sbe1v1k_qwrt_profile(
	struct recovery_status_led_ctrl *status_leds)
{
	char auth_code[RECOVERY_QWRT_AUTH_CODE_LEN + 1];
	int ret;

	if (recv_off != strlen(RECOVERY_SBE1V1K_REPARTITION_TOKEN) ||
	    memcmp(recv_base, RECOVERY_SBE1V1K_REPARTITION_TOKEN,
		   strlen(RECOVERY_SBE1V1K_REPARTITION_TOKEN))) {
		printf("Invalid QWRT profile confirmation token\n");
		return -EINVAL;
	}

	if (!recovery_backend_is_mmc())
		return -EINVAL;

	/* Fail before touching eMMC when the device identity is unavailable. */
	ret = recovery_qwrt_auth_code(auth_code);
	recovery_qwrt_secure_zero(auth_code, sizeof(auth_code));
	if (ret) {
		printf("Cannot authorize QWRT profile from the device serial: %d\n",
		       ret);
		return ret;
	}

	if (recovery_verify_sbe1v1k_gpt(&sbe1v1k_layout_qwrt, false)) {
		printf("QWRT profile requires an existing verified factory-compatible GPT\n");
		return -EINVAL;
	}

	prog_phase = 2;
	prog_erase_done = 0;
	prog_erase_total = 0;
	prog_write_done = 0;
	prog_write_total = RECOVERY_APPSBLENV_SIZE;
	prog_total = prog_write_total;
	prog_done = 0;
	recovery_service_runtime(status_leds);

	ret = recovery_write_appsblenv(status_leds, 0, &sbe1v1k_layout_qwrt);
	if (ret)
		return ret;

	ret = recovery_apply_sbe1v1k_layout(&sbe1v1k_layout_qwrt);
	if (ret)
		return ret;

	prog_write_done = prog_write_total;
	prog_done = prog_total;
	recovery_service_runtime(status_leds);
	recovery_debug_printf("SBE1V1K QWRT profile selected; auth_code refreshed without repartition\n");
	return 0;
}

static int recovery_repartition_factory(
	struct recovery_status_led_ctrl *status_leds,
	enum recovery_sbe1v1k_layout layout_id)
{
	const struct recovery_sbe1v1k_layout_desc *layout =
		recovery_sbe1v1k_layout_desc(layout_id);
	char qwrt_auth_code[RECOVERY_QWRT_AUTH_CODE_LEN + 1] = { 0 };
	void *chainloader_fit;
	size_t chainloader_size;
	size_t hlos_clone_size;
	size_t write_progress;
	char *repartition_gpt;
	u32 erase_progress_base;
	bool has_hlos_1;
	int ret;

	if (!layout) {
		printf("Unknown SBE1V1K partition profile\n");
		return -EINVAL;
	}

	if (recv_off != strlen(RECOVERY_SBE1V1K_REPARTITION_TOKEN) ||
	    memcmp(recv_base, RECOVERY_SBE1V1K_REPARTITION_TOKEN,
		   strlen(RECOVERY_SBE1V1K_REPARTITION_TOKEN))) {
		printf("Invalid repartition confirmation token\n");
		return -EINVAL;
	}

	if (!recovery_backend_is_mmc()) {
		printf("Factory repartition requires eMMC recovery backend\n");
		return -EINVAL;
	}

	/* Derive QWRT authorization before changing GPT or erasing a target. */
	if (layout->id == RECOVERY_SBE1V1K_LAYOUT_QWRT) {
		ret = recovery_qwrt_auth_code(qwrt_auth_code);
		recovery_qwrt_secure_zero(qwrt_auth_code, sizeof(qwrt_auth_code));
		if (ret) {
			printf("Cannot authorize QWRT layout from the device serial: %d\n",
			       ret);
			return ret;
		}
	}

	ret = recovery_verify_factory_gpt(status_leds, &has_hlos_1);
	if (ret)
		return ret;

	ret = recovery_preserve_chainloader_fit(&chainloader_fit, &chainloader_size);
	if (ret)
		return ret;

	ret = recovery_check_appsblenv();
	if (ret)
		goto out;

	recovery_debug_printf("Factory anchors verified. Writing SBE1V1K '%s' GPT layout...\n",
			      layout->name);
	prog_phase = 2;
	prog_erase_done = prog_erase_total;
	erase_progress_base = prog_erase_total;
	prog_erase_total += RECOVERY_MMC_ERASE_PROGRESS_STEPS;
	prog_write_done = 0;
	hlos_clone_size = has_hlos_1 ? 0 :
		RECOVERY_SBE1V1K_HLOS_1_SIZE * 512ULL;
	prog_write_total = hlos_clone_size + 2 + chainloader_size +
		RECOVERY_APPSBLENV_SIZE;
	prog_total = prog_erase_total + prog_write_total;
	prog_done = prog_erase_done;
	recovery_service_runtime(status_leds);
	write_progress = 0;

	if (!has_hlos_1) {
		ret = recovery_clone_sbe1v1k_hlos_1(status_leds,
						       write_progress);
		if (ret)
			goto out;
		write_progress += hlos_clone_size;
	}

	ret = recovery_build_sbe1v1k_gpt(layout, &repartition_gpt);
	if (ret)
		goto out;

	ret = env_set("sbe1v1k_repartition_gpt", repartition_gpt);
	free(repartition_gpt);
	if (ret)
		goto out;
	prog_write_done = write_progress + 1;
	prog_done = prog_erase_done + prog_write_done;
	recovery_service_runtime(status_leds);

	ret = run_commandf("mmc dev %s", recovery_mmcdev());
	if (ret) {
		env_set("sbe1v1k_repartition_gpt", NULL);
		goto out;
	}

	ret = run_commandf("gpt write mmc %s ${sbe1v1k_repartition_gpt}",
			   recovery_mmcdev());
	env_set("sbe1v1k_repartition_gpt", NULL);
	if (ret)
		goto out;
	prog_write_done = write_progress + 2;
	prog_done = prog_erase_done + prog_write_done;
	recovery_service_runtime(status_leds);

	ret = recovery_verify_sbe1v1k_gpt(layout, true);
	if (ret) {
		printf("SBE1V1K '%s' GPT verification failed after write: %d\n",
		       layout->name, ret);
		goto out;
	}
	prog_write_done = write_progress + 2;
	prog_done = prog_erase_done + prog_write_done;
	recovery_service_runtime(status_leds);

	prog_phase = 1;
	ret = recovery_mmc_erase_part(layout->uboot_part,
					      status_leds, erase_progress_base);
	if (ret)
		goto out;
	prog_phase = 2;

	recovery_debug_printf("Writing preserved chainloader FIT to '%s'...\n",
			      layout->uboot_part);
	ret = recovery_mmc_write_part(layout->uboot_part,
					      status_leds, chainloader_fit,
					      chainloader_size,
					      write_progress + 2);
	if (ret)
		goto out;

	recovery_debug_printf("Updating factory U-Boot environment in APPSBLENV...\n");
	ret = recovery_write_appsblenv(status_leds,
					       write_progress + 2 + chainloader_size,
					       layout);
	if (ret)
		goto out;

	ret = recovery_apply_sbe1v1k_layout(layout);
	if (ret)
		goto out;

	prog_write_done = prog_write_total;
	prog_done = prog_erase_done + prog_write_done;
	recovery_service_runtime(status_leds);

	recovery_debug_printf("SBE1V1K '%s' layout written, chainloader installed, APPSBLENV updated. Upload firmware before reboot.\n",
			      layout->name);

out:
	free(chainloader_fit);
	return ret;
}

static int recovery_flash_mmc_firmware(struct recovery_status_led_ctrl *status_leds)
{
	const char *kernel_part = env_get("recovery_part_kernel") ?: "0#kernel";
	const char *rootfs_part = env_get("recovery_part_rootfs") ?: "0#rootfs";
	const char *data_part = env_get("recovery_part_data") ?: "0#rootfs_data";
	struct recovery_firmware_image image;
	int ret;

	ret = recovery_require_sbe1v1k_firmware_layout();
	if (ret)
		return ret;

	ret = recovery_extract_firmware(recv_base, recv_off, &image);
	if (ret)
		return ret;

	if (recovery_board_is_sbe1v1k()) {
		ret = recovery_validate_sbe1v1k_kernel_fit(&image.kernel);
		if (ret)
			return ret;
		ret = recovery_validate_sbe1v1k_rootfs(&image.rootfs);
		if (ret)
			return ret;
	}

	if (active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_QWRT) {
		ret = recovery_refresh_qwrt_auth_code();
		if (ret) {
			printf("Cannot refresh QWRT auth_code before erase: %d\n",
			       ret);
			return ret;
		}
	}

	ret = recovery_mmc_check_part_size(kernel_part, image.kernel.size);
	if (ret)
		return ret;
	ret = recovery_mmc_check_part_size(rootfs_part, image.rootfs.size);
	if (ret)
		return ret;
	ret = recovery_mmc_check_part_size(data_part, 0);
	if (ret)
		return ret;

	prog_phase = 1;
	prog_done = 0;
	prog_erase_done = 0;
	prog_erase_total = 3 * RECOVERY_MMC_ERASE_PROGRESS_STEPS;
	prog_write_done = 0;
	prog_write_total = image.kernel.size + image.rootfs.size;
	prog_total = prog_erase_total + prog_write_total;

	ret = recovery_mmc_erase_part(kernel_part, status_leds, 0);
	if (ret)
		return ret;

	ret = recovery_mmc_erase_part(rootfs_part, status_leds,
				      RECOVERY_MMC_ERASE_PROGRESS_STEPS);
	if (ret)
		return ret;

	ret = recovery_mmc_erase_part(data_part, status_leds,
				      2 * RECOVERY_MMC_ERASE_PROGRESS_STEPS);
	if (ret)
		return ret;

	prog_phase = 2;

	recovery_debug_printf("Writing kernel payload to eMMC partition '%s'...\n",
			      kernel_part);
	ret = recovery_mmc_write_part(kernel_part, status_leds,
				      image.kernel.data, image.kernel.size, 0);
	if (ret)
		return ret;

	recovery_debug_printf("Writing rootfs payload to eMMC partition '%s'...\n",
			      rootfs_part);
	ret = recovery_mmc_write_part(rootfs_part, status_leds,
				      image.rootfs.data, image.rootfs.size,
				      image.kernel.size);
	if (ret)
		return ret;

	return 0;
}

static int recovery_flash_mmc_uboot(struct recovery_status_led_ctrl *status_leds)
{
	const char *uboot_part = env_get("recovery_part_uboot");
	const char *uboot_alt_part = env_get("recovery_part_uboot_alt");
	unsigned long max = recovery_uboot_limit();
	u32 target_count;
	int ret;

	if (!uboot_part || !*uboot_part) {
		printf("Chainloader eMMC target is not configured. Set recovery_part_uboot explicitly.\n");
		return -EINVAL;
	}

	if (recv_off > max) {
		printf("Chainloader image size %u exceeds limit %lu\n",
		       recv_off, max);
		return -EFBIG;
	}

	ret = recovery_verify_fit_payload(recv_base, recv_off);
	if (ret) {
		printf("Chainloader upload must be a raw FIT image (.itb): %d\n",
		       ret);
		return ret;
	}

	if (recovery_board_is_sbe1v1k() &&
	    !recovery_is_sbe1v1k_chainloader_fit(recv_base, recv_off)) {
		printf("Chainloader upload is not an SBE1V1K chainloader FIT\n");
		return -ENOEXEC;
	}

	ret = recovery_mmc_check_part_size(uboot_part, recv_off);
	if (ret)
		return ret;
	if (uboot_alt_part && *uboot_alt_part) {
		ret = recovery_mmc_check_part_size(uboot_alt_part, recv_off);
		if (ret)
			return ret;
	}

	target_count = uboot_alt_part && *uboot_alt_part ? 2 : 1;
	prog_phase = 1;
	prog_done = 0;
	prog_erase_done = 0;
	prog_erase_total = target_count * RECOVERY_MMC_ERASE_PROGRESS_STEPS;
	prog_write_done = 0;
	prog_write_total = recv_off * target_count;
	prog_total = prog_erase_total + prog_write_total;

	ret = recovery_mmc_erase_part(uboot_part, status_leds, 0);
	if (ret)
		return ret;

	if (uboot_alt_part && *uboot_alt_part) {
		ret = recovery_mmc_erase_part(uboot_alt_part, status_leds,
					      RECOVERY_MMC_ERASE_PROGRESS_STEPS);
		if (ret)
			return ret;
	}

	prog_phase = 2;

	recovery_debug_printf("Writing chainloader FIT to eMMC partition '%s'...\n",
			      uboot_part);
	ret = recovery_mmc_write_part(uboot_part, status_leds,
				      recv_base, recv_off, 0);
	if (ret)
		return ret;

	if (uboot_alt_part && *uboot_alt_part) {
		recovery_debug_printf("Writing chainloader FIT to alternate eMMC partition '%s'...\n",
				      uboot_alt_part);
		ret = recovery_mmc_write_part(uboot_alt_part, status_leds,
					      recv_base, recv_off, recv_off);
		if (ret)
			return ret;
	}

	return 0;
}

static int recovery_flash_mmc_target(enum upload_target tgt,
				     struct recovery_status_led_ctrl *status_leds)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return recovery_flash_mmc_firmware(status_leds);
	case TARGET_UBOOT:
		return recovery_flash_mmc_uboot(status_leds);
	case TARGET_REPARTITION:
		return recovery_repartition_factory(status_leds,
						    current_repartition_layout);
	}

	return -EINVAL;
}

static __maybe_unused const char *recovery_ubi_part(enum upload_target tgt)
{
	const char *part;

	switch (tgt) {
	case TARGET_UBOOT:
		part = env_get("recovery_ubi_part_uboot");
		break;
	case TARGET_FIRMWARE:
	default:
		part = env_get("recovery_ubi_part");
		break;
	}

	if (!part)
		part = env_get("recovery_ubi_part");

	return part ?: "ubi";
}

static int recovery_try_ubi_target(enum upload_target tgt,
				       struct recovery_target *target)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_volume_desc *desc;
	struct ubi_device *ubi;
	struct mtd_info *mtd;
	const char *volume = env_get(recovery_target_env(tgt));
	const char *part = recovery_ubi_part(tgt);

	if (!volume)
		volume = recovery_default_target(tgt);

	if (ubi_part((char *)part, NULL)) {
		mtd_probe_devices();
		mtd = get_mtd_device_nm(part);
		if (IS_ERR_OR_NULL(mtd))
			return -ENODEV;

		target->backend = RECOVERY_BACKEND_UBI;
		target->name = volume;
		target->ubi_part = part;
		target->mtd = mtd;
		target->cur_size = 0;
		target->limit = mtd->size;
		target->ubi_needs_format = true;
		return 0;
	}

	desc = ubi_open_volume_nm(0, volume, UBI_READWRITE);
	if (IS_ERR_OR_NULL(desc)) {
		ubi = ubi_get_device(0);
		if (!ubi)
			return -ENODEV;

		target->backend = RECOVERY_BACKEND_UBI;
		target->name = volume;
		target->ubi_part = part;
		target->ubi_needs_format = false;
		target->cur_size = 0;
		target->limit = (unsigned long long)ubi->avail_pebs *
			(unsigned long long)ubi->leb_size;
		ubi_put_device(ubi);
		return 0;
	}

	target->backend = RECOVERY_BACKEND_UBI;
	target->name = volume;
	target->ubi_part = part;
	target->ubi_needs_format = false;
	target->cur_size = (unsigned long long)desc->vol->reserved_pebs *
			   (unsigned long long)desc->vol->usable_leb_size;
	target->limit = (unsigned long long)(desc->vol->reserved_pebs +
					     desc->vol->ubi->avail_pebs) *
			(unsigned long long)desc->vol->usable_leb_size;

	ubi_close_volume(desc);
	return 0;
#else
	return -ENODEV;
#endif
}

static int recovery_resolve_target(enum upload_target tgt,
				   struct recovery_target *target)
{
	const char *name = env_get(recovery_target_env(tgt));
	const char *raw;
	struct mtd_info *mtd;
	ulong ofs;

	memset(target, 0, sizeof(*target));
	if (tgt == TARGET_FIRMWARE) {
		int ret = recovery_require_sbe1v1k_firmware_layout();

		if (ret)
			return ret;
	}

	if (!name)
		name = recovery_default_target(tgt);

	if (recovery_backend_is_mmc()) {
		struct disk_partition kernel_part, rootfs_part;
		struct blk_desc *desc;
		const char *part;
		int ret;

		target->backend = RECOVERY_BACKEND_MMC;

		if (tgt == TARGET_UBOOT) {
			part = env_get(recovery_mmc_part_env(tgt));
			if (!part || !*part) {
				printf("Chainloader eMMC target is not configured. Set recovery_part_uboot explicitly.\n");
				return -ENODEV;
			}

			ret = recovery_get_mmc_part(part, &target->blk,
						    &target->part);
			if (ret)
				return ret;

			target->name = part;
			target->limit = recovery_mmc_part_bytes(&target->part);
			if (target->limit > recovery_uboot_limit())
				target->limit = recovery_uboot_limit();
			target->cur_size = target->limit;
			return 0;
		}

		part = env_get("recovery_part_kernel") ?: "0#kernel";
		ret = recovery_get_mmc_part(part, &desc, &kernel_part);
		if (ret)
			return ret;

		part = env_get("recovery_part_rootfs") ?: "0#rootfs";
		ret = recovery_get_mmc_part(part, &desc, &rootfs_part);
		if (ret)
			return ret;

		target->name = "mmc-sysupgrade";
		target->blk = desc;
		target->limit = recovery_mmc_part_bytes(&kernel_part) +
				recovery_mmc_part_bytes(&rootfs_part);
		target->cur_size = target->limit;
		return 0;
	}

	mtd_probe_devices();

	raw = env_get(recovery_raw_env(tgt));
	if (raw && *raw) {
		mtd = get_mtd_device_nm(raw);
		if (!IS_ERR_OR_NULL(mtd)) {
			ulong size_cap = env_get_hex(recovery_size_env(tgt), 0);

			ofs = recovery_raw_offset(tgt);
			target->backend = RECOVERY_BACKEND_MTD;
			target->name = raw;
			target->mtd = mtd;
			target->ofs = ofs;
			target->limit = mtd->size;
			if (ofs && target->limit > ofs)
				target->limit -= ofs;
			if (size_cap && target->limit > size_cap)
				target->limit = size_cap;
			target->cur_size = target->limit;
			return 0;
		}
	}

	mtd = get_mtd_device_nm(name);
	if (!IS_ERR_OR_NULL(mtd)) {
		target->backend = RECOVERY_BACKEND_MTD;
		target->name = name;
		target->mtd = mtd;
		target->limit = mtd->size;
		target->cur_size = target->limit;
		return 0;
	}

	if (!recovery_try_ubi_target(tgt, target))
		return 0;

	if (!raw) {
		switch (tgt) {
		case TARGET_FIRMWARE:
			raw = "nor0";
			break;
		case TARGET_UBOOT:
			raw = env_get("recovery_dev");
			if (!raw)
				raw = "nor0";
			break;
		case TARGET_REPARTITION:
			raw = "nor0";
			break;
		}
	}

	mtd = get_mtd_device_nm(raw);
	if (IS_ERR_OR_NULL(mtd))
		return -ENODEV;

	ofs = recovery_raw_offset(tgt);
	target->backend = RECOVERY_BACKEND_MTD;
	target->name = raw;
	target->mtd = mtd;
	target->ofs = ofs;
	target->limit = mtd->size;
	if (ofs && target->limit > ofs)
		target->limit -= ofs;
	{
		ulong size_cap = env_get_hex(recovery_size_env(tgt), 0);

		if (size_cap && target->limit > size_cap)
			target->limit = size_cap;
	}
	target->cur_size = target->limit;

	return 0;
}

static void recovery_release_target(struct recovery_target *target)
{
	if (target->mtd)
		put_mtd_device(target->mtd);

	target->mtd = NULL;
}

static void recovery_service_runtime(struct recovery_status_led_ctrl *status_leds)
{
	struct udevice *udev = eth_get_current();
	struct netif *netif = net_lwip_get_netif();

	if (udev && eth_is_active(udev) && netif)
		net_lwip_rx(udev, netif);
	if (status_leds)
		recovery_status_led_poll(status_leds);
	WATCHDOG_RESET();
}

static __maybe_unused void recovery_ubi_progress(struct ubi_volume *vol, int done, int total)
{
	unsigned long long bytes_done = prog_erase_volume_base;

	(void)vol;

	if (total > 0)
		bytes_done += (prog_erase_volume_bytes * (unsigned long long)done) /
			      (unsigned long long)total;

	if (bytes_done > prog_erase_total)
		bytes_done = prog_erase_total;

	prog_erase_done = bytes_done;
	prog_done = prog_erase_done + prog_write_done;

	if (prog_status_leds)
		recovery_service_runtime(prog_status_leds);
}

static int recovery_erase_mtd_region(struct mtd_info *mtd, loff_t ofs,
				     size_t len,
				     struct recovery_status_led_ctrl *status_leds)
{
	struct erase_info ei = { 0 };
	loff_t erase_len;
	int ret;

	if (!mtd || !len)
		return 0;

	erase_len = ALIGN(len, mtd->erasesize);
	ei.addr = ofs;
	ei.len = erase_len;

	ret = mtd_unlock(mtd, ei.addr, ei.len);
	if (ret && ret != -EOPNOTSUPP) {
		printf("Warning: initial mtd_unlock 0x%llx..+0x%llx failed: %d\n",
		       (unsigned long long)ei.addr,
		       (unsigned long long)ei.len, ret);
	}

	for (loff_t addr = 0; addr < erase_len; addr += mtd->erasesize) {
		struct erase_info e = {
			.addr = ofs + addr,
			.len = mtd->erasesize,
		};
		int tries = 0;

		do {
			ret = mtd_erase(mtd, &e);
			if (!ret)
				break;
			if (ret == -EROFS || ret == -EACCES)
				mtd_unlock(mtd, e.addr, e.len);
			else
				break;
		} while (++tries < 2);

		if (ret)
			return ret;

		prog_erase_done = addr + mtd->erasesize;
		if (prog_erase_done > prog_erase_total)
			prog_erase_done = prog_erase_total;
		prog_done = prog_erase_done + prog_write_done;
		recovery_service_runtime(status_leds);
	}

	return 0;
}

static __maybe_unused bool recovery_preserve_ubi_volume(const char *name)
{
	if (!name || !*name)
		return true;

	if (!strcmp(name, "ubootenv") || !strcmp(name, "ubootenv2"))
		return true;

	if (of_machine_is_compatible("econet,xr1710g") ||
	    of_machine_is_compatible("econet,xr1710g-ubi") ||
	    of_machine_is_compatible("gemtek,xr1710g") ||
	    of_machine_is_compatible("gemtek,xr1710g-ubi"))
		return !strcmp(name, "factory");

	if (of_machine_is_compatible("gemtek,w1700k") ||
	    of_machine_is_compatible("gemtek,w1700k-ubi"))
		return !strcmp(name, "factory");

	return false;
}

static unsigned long long
recovery_calc_forced_ubi_limit(struct recovery_target *target)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_device *ubi;
	unsigned long long limit;
	int i;

	if (target->backend != RECOVERY_BACKEND_UBI)
		return target->limit;

	if (ubi_part((char *)target->ubi_part, NULL))
		return target->limit;

	ubi = ubi_get_device(0);
	if (!ubi)
		return target->limit;

	limit = (unsigned long long)ubi->avail_pebs *
		(unsigned long long)ubi->leb_size;

	for (i = 0; i < ubi->vtbl_slots; i++) {
		struct ubi_volume *vol = ubi->volumes[i];

		if (!vol || vol->vol_id >= UBI_INTERNAL_VOL_START)
			continue;
		if (recovery_preserve_ubi_volume(vol->name))
			continue;

		limit += (unsigned long long)vol->reserved_pebs *
			 (unsigned long long)vol->usable_leb_size;
	}

	ubi_put_device(ubi);
	return limit;
#else
	return target->limit;
#endif
}

static __maybe_unused int recovery_create_ubi_volume(const char *name, size_t size, int vol_type)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_mkvol_req req;
	struct ubi_device *ubi;
	int ret;

	if (!name || !*name)
		return -EINVAL;

	ubi = ubi_get_device(0);
	if (!ubi)
		return -ENODEV;

	if (!size)
		size = (size_t)ubi->avail_pebs * (size_t)ubi->leb_size;

	memset(&req, 0, sizeof(req));
	req.vol_id = UBI_VOL_NUM_AUTO;
	req.alignment = 1;
	req.bytes = size;
	req.vol_type = vol_type;
	req.name_len = strlen(name);
	if (req.name_len > UBI_VOL_NAME_MAX) {
		ubi_put_device(ubi);
		return -ENAMETOOLONG;
	}
	memcpy(req.name, name, req.name_len);
	req.name[req.name_len] = '\0';

	mutex_lock(&ubi->device_mutex);
	ret = ubi_create_volume(ubi, &req);
	mutex_unlock(&ubi->device_mutex);
	ubi_put_device(ubi);

	return ret;
#else
	return -ENODEV;
#endif
}

static __maybe_unused int recovery_remove_ubi_volume(const char *name)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_volume_desc *desc;
	int ret;

	desc = ubi_open_volume_nm(0, name, UBI_EXCLUSIVE);
	if (IS_ERR_OR_NULL(desc))
		return IS_ERR(desc) ? PTR_ERR(desc) : -ENODEV;

	mutex_lock(&desc->vol->ubi->device_mutex);
	ret = ubi_remove_volume(desc, 0);
	mutex_unlock(&desc->vol->ubi->device_mutex);
	ubi_close_volume(desc);

	return ret;
#else
	return -ENODEV;
#endif
}

static int recovery_ensure_rootfs_data(struct recovery_target *target)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_volume_desc *desc;
	int ret;

	if (target->backend != RECOVERY_BACKEND_UBI)
		return 0;

	if (ubi_part((char *)target->ubi_part, NULL))
		return -ENODEV;

	desc = ubi_open_volume_nm(0, "rootfs_data", UBI_READWRITE);
	if (!IS_ERR_OR_NULL(desc)) {
		ubi_close_volume(desc);
		return 0;
	}

	ret = recovery_create_ubi_volume("rootfs_data", 0, UBI_DYNAMIC_VOLUME);
	if (ret && ret != -EEXIST) {
		printf("Failed to create UBI volume 'rootfs_data': %d\n", ret);
		return ret;
	}

	return 0;
#else
	return -ENODEV;
#endif
}

static int recovery_prepare_ubi_target(struct recovery_target *target,
				       struct recovery_status_led_ctrl *status_leds,
				       size_t image_size, bool *reformatted)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_device *ubi;
	loff_t erase_len;
	int ret;

	if (reformatted)
		*reformatted = false;

	if (target->backend != RECOVERY_BACKEND_UBI)
		return -EINVAL;

	if (!target->ubi_needs_format)
		return ubi_part((char *)target->ubi_part, NULL) ? -ENODEV : 0;

	if (!target->mtd)
		return -ENODEV;

	erase_len = ALIGN(target->mtd->size, target->mtd->erasesize);
	prog_phase = 1;
	prog_done = 0;
	prog_erase_done = 0;
	prog_erase_total = erase_len;
	prog_write_done = 0;
	prog_write_total = image_size;
	prog_total = prog_erase_total + prog_write_total;

	recovery_debug_printf("UBI partition '%s' is invalid, erasing it to recreate layout...\n",
			      target->ubi_part);
	ret = recovery_erase_mtd_region(target->mtd, 0, target->mtd->size,
					status_leds);
	if (ret)
		return ret;

	ret = ubi_part((char *)target->ubi_part, NULL);
	if (ret)
		return ret;

	ubi = ubi_get_device(0);
	if (ubi) {
		target->limit = (unsigned long long)ubi->avail_pebs *
				(unsigned long long)ubi->leb_size;
		ubi_put_device(ubi);
	}

	target->cur_size = 0;
	target->ubi_needs_format = false;
	if (reformatted)
		*reformatted = true;

	return 0;
#else
	return -ENODEV;
#endif
}

static int recovery_ensure_ubi_volume_named(const char *name, size_t size,
					    int vol_type)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_volume_desc *desc;
	int ret;

	if (!name || !*name)
		return -EINVAL;

	desc = ubi_open_volume_nm(0, name, UBI_READWRITE);
	if (!IS_ERR_OR_NULL(desc)) {
		ubi_close_volume(desc);
		return 0;
	}

	ret = recovery_create_ubi_volume(name, size, vol_type);
	if (ret && ret != -EEXIST) {
		printf("Failed to create UBI volume '%s': %d\n", name, ret);
		return ret;
	}

	return 0;
#else
	return -ENODEV;
#endif
}

static int recovery_ensure_preserved_ubi_volumes(struct recovery_target *target)
{
	int ret;

	if (target->backend != RECOVERY_BACKEND_UBI)
		return 0;

	if (ubi_part((char *)target->ubi_part, NULL))
		return -ENODEV;

	ret = recovery_ensure_ubi_volume_named("ubootenv",
					       RECOVERY_UBOOTENV_SIZE,
					       UBI_DYNAMIC_VOLUME);
	if (ret)
		return ret;

	ret = recovery_ensure_ubi_volume_named("ubootenv2",
					       RECOVERY_UBOOTENV_SIZE,
					       UBI_DYNAMIC_VOLUME);
	if (ret)
		return ret;

	if (of_machine_is_compatible("econet,xr1710g") ||
	    of_machine_is_compatible("econet,xr1710g-ubi") ||
	    of_machine_is_compatible("gemtek,xr1710g") ||
	    of_machine_is_compatible("gemtek,xr1710g-ubi") ||
	    of_machine_is_compatible("gemtek,w1700k") ||
	    of_machine_is_compatible("gemtek,w1700k-ubi")) {
		ret = recovery_ensure_ubi_volume_named("factory",
						       RECOVERY_FACTORY_SIZE,
						       UBI_STATIC_VOLUME);
		if (ret)
			return ret;
	}

	return 0;
}

static int recovery_cleanup_ubi_firmware(struct recovery_target *target,
					 struct recovery_status_led_ctrl *status_leds,
					 size_t image_size)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_device *ubi;
	unsigned long long erase_total = 0;
	unsigned long long erase_done = 0;
	int ret = 0;
	int i;

	if (target->backend != RECOVERY_BACKEND_UBI)
		return -EINVAL;

	if (ubi_part((char *)target->ubi_part, NULL))
		return -ENODEV;

	ubi = ubi_get_device(0);
	if (!ubi)
		return -ENODEV;

	for (i = 0; i < ubi->vtbl_slots; i++) {
		struct ubi_volume *vol = ubi->volumes[i];

		if (!vol || vol->vol_id >= UBI_INTERNAL_VOL_START)
			continue;
		if (recovery_preserve_ubi_volume(vol->name))
			continue;

		erase_total += (unsigned long long)vol->reserved_pebs *
			       (unsigned long long)vol->usable_leb_size;
	}

	prog_phase = 1;
	prog_done = 0;
	prog_erase_done = 0;
	prog_erase_total = erase_total > UINT_MAX ? UINT_MAX : erase_total;
	prog_write_done = 0;
	prog_write_total = image_size;
	prog_total = prog_erase_total + prog_write_total;
	prog_status_leds = status_leds;
	ubi_set_progress_callback(recovery_ubi_progress);

	for (i = 0; i < ubi->vtbl_slots; i++) {
		struct ubi_volume *vol = ubi->volumes[i];
		unsigned long long vol_size;
		char name[UBI_VOL_NAME_MAX + 1];

		if (!vol || vol->vol_id >= UBI_INTERNAL_VOL_START)
			continue;
		if (recovery_preserve_ubi_volume(vol->name))
			continue;

		vol_size = (unsigned long long)vol->reserved_pebs *
			   (unsigned long long)vol->usable_leb_size;
		strlcpy(name, vol->name, sizeof(name));
		prog_erase_volume_base = erase_done;
		prog_erase_volume_bytes = vol_size;

		recovery_debug_printf("Removing UBI volume '%s' before flashing '%s'...\n",
				      name, target->name);
		ret = recovery_remove_ubi_volume(name);
		if (ret) {
			printf("Failed to remove UBI volume '%s': %d\n", name, ret);
			break;
		}

		erase_done += vol_size;
		prog_erase_done = erase_done > prog_erase_total ?
				  prog_erase_total : erase_done;
		prog_done = prog_erase_done + prog_write_done;
		recovery_service_runtime(status_leds);
	}

	ubi_set_progress_callback(NULL);
	prog_status_leds = NULL;
	ubi_put_device(ubi);
	if (ret)
		return ret;

	return recovery_try_ubi_target(current_target, target);
#else
	return -ENODEV;
#endif
}

static int recovery_write_ubi_target(struct recovery_target *target,
				      struct recovery_status_led_ctrl *status_leds,
				      const void *image, size_t image_size)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_volume_desc *desc;
	struct ubi_volume *vol;
	struct ubi_device *ubi;
	const u8 *src = image;
	size_t reserved_bytes;
	u32 written = 0;
	int ret;

	if (target->backend != RECOVERY_BACKEND_UBI)
		return -EINVAL;

	if (ubi_part((char *)target->ubi_part, NULL))
		return -ENODEV;

	desc = ubi_open_volume_nm(0, target->name, UBI_READWRITE);
	if (IS_ERR_OR_NULL(desc))
		return IS_ERR(desc) ? PTR_ERR(desc) : -ENODEV;

	vol = desc->vol;
	ubi = vol->ubi;
	reserved_bytes = (size_t)vol->reserved_pebs *
			 (size_t)(ubi->leb_size - vol->data_pad);
	if (image_size > reserved_bytes) {
		ret = -EFBIG;
		goto out_close;
	}

	ret = ubi_start_update(ubi, vol, image_size);
	if (ret)
		goto out_close;

	/* Flush one runtime cycle so the browser can observe phase 2 before
	 * the actual UBI data write loop starts.
	 */
	recovery_service_runtime(status_leds);

	while (written < image_size) {
		u32 chunk = image_size - written;

		if (chunk > RECOVERY_UBI_WRITE_CHUNK)
			chunk = RECOVERY_UBI_WRITE_CHUNK;

		ret = ubi_more_update_data(ubi, vol, src + written, chunk);
		if (ret < 0)
			goto out_close;

		written += chunk;
		prog_write_done = written > prog_write_total ?
				  prog_write_total : written;
		prog_done = prog_erase_done + prog_write_done;
		recovery_service_runtime(status_leds);
	}

	ret = ubi_check_volume(ubi, vol->vol_id);
	if (ret < 0) {
		ret = -ret;
		goto out_close;
	}

	if (ret) {
		ubi_warn(ubi, "volume %d on UBI device %d is corrupt",
			 vol->vol_id, ubi->ubi_num);
		vol->corrupted = 1;
	}

	vol->checked = 1;
	ubi_gluebi_updated(vol);
	ret = 0;

out_close:
	ubi_close_volume(desc);
	return ret;
#else
	return -ENODEV;
#endif
}

static int recovery_resize_ubi_target(struct recovery_target *target,
				      size_t new_size)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_volume_desc *desc;
	struct ubi_volume *vol;
	int needed_pebs;
	int ret;

	if (target->backend != RECOVERY_BACKEND_UBI)
		return -EINVAL;

	if (ubi_part((char *)target->ubi_part, NULL))
		return -ENODEV;

	desc = ubi_open_volume_nm(0, target->name, UBI_EXCLUSIVE);
	if (IS_ERR_OR_NULL(desc))
		return IS_ERR(desc) ? PTR_ERR(desc) : -ENODEV;

	vol = desc->vol;
	needed_pebs = DIV_ROUND_UP(new_size, vol->usable_leb_size);

	mutex_lock(&vol->ubi->device_mutex);
	ret = ubi_resize_volume(desc, needed_pebs);
	mutex_unlock(&vol->ubi->device_mutex);

	if (!ret) {
		target->cur_size = (unsigned long long)needed_pebs *
				   (unsigned long long)vol->usable_leb_size;
		target->limit = (unsigned long long)(needed_pebs +
						     vol->ubi->avail_pebs) *
			(unsigned long long)vol->usable_leb_size;
	}

	ubi_close_volume(desc);
	return ret;
#else
	return -ENODEV;
#endif
}

static int recovery_create_ubi_target(struct recovery_target *target,
				      size_t new_size)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_mkvol_req req;
	struct ubi_device *ubi;
	int ret;

	if (target->backend != RECOVERY_BACKEND_UBI)
		return -EINVAL;

	if (ubi_part((char *)target->ubi_part, NULL))
		return -ENODEV;

	ubi = ubi_get_device(0);
	if (!ubi)
		return -ENODEV;

	memset(&req, 0, sizeof(req));
	req.vol_id = UBI_VOL_NUM_AUTO;
	req.alignment = 1;
	req.bytes = new_size;
	req.vol_type = UBI_STATIC_VOLUME;
	req.name_len = strlen(target->name);
	if (req.name_len > UBI_VOL_NAME_MAX) {
		ubi_put_device(ubi);
		return -ENAMETOOLONG;
	}
	memcpy(req.name, target->name, req.name_len);
	req.name[req.name_len] = '\0';

	mutex_lock(&ubi->device_mutex);
	ret = ubi_create_volume(ubi, &req);
	mutex_unlock(&ubi->device_mutex);
	ubi_put_device(ubi);
	if (ret)
		return ret;

	return recovery_try_ubi_target(current_target, target);
#else
	return -ENODEV;
#endif
}

/* Determine maximum payload size based on selected target and DTS-defined MTD
 * layout. Returns 0 on error. */
static unsigned long recovery_calc_target_max(enum upload_target tgt,
					      bool force_recreate, loff_t *p_ofs)
{
	struct recovery_target target;
	unsigned long limit = 0;
	unsigned long long effective_limit;

	if (recovery_resolve_target(tgt, &target))
		return 0;

	if (p_ofs)
		*p_ofs = target.ofs;

	effective_limit = target.limit;
	if (force_recreate && tgt == TARGET_FIRMWARE)
		effective_limit = recovery_calc_forced_ubi_limit(&target);

	limit = (effective_limit > ULONG_MAX) ? ULONG_MAX :
		(unsigned long)effective_limit;
	recovery_release_target(&target);

	return limit;
}

static int recovery_validate_upload_length(enum upload_target target,
					   bool force_recreate,
					   unsigned long length,
					   enum recovery_stream_format format)
{
	unsigned long env_max = env_get_hex("recovery_max", 0);
	unsigned long min = 0;
	loff_t target_ofs = 0;
	unsigned long target_max;
	unsigned long max;
	int ret;

	if (target == TARGET_FIRMWARE) {
		ret = recovery_require_sbe1v1k_firmware_layout();
		if (ret)
			return ret;
	}

	target_max = recovery_calc_target_max(target, force_recreate,
					      &target_ofs);
	max = target_max ? target_max : RECOVERY_UPLOAD_MAX;

	if (target == TARGET_FIRMWARE) {
		if (recovery_backend_is_mmc() &&
		    env_get_yesno("recovery_stream") == 1) {
			if (format == RECOVERY_STREAM_TAR)
				min = 1024;
			else
				min = env_get_hex("recovery_kernel_pad",
						  RECOVERY_KERNEL_PAD_SIZE) + 1;
		} else {
			min = RECOVERY_MIN_FIRMWARE_SIZE;
		}
	} else if (target == TARGET_UBOOT && max > recovery_uboot_limit()) {
		max = recovery_uboot_limit();
	}

	if (env_max && env_max < max)
		max = env_max;
	if (!length || length > max || (min && length < min)) {
		if (min && length < min)
			printf("httpd: upload length %lu below allowed min %lu for target %d\n",
			       length, min, target);
		else
			printf("httpd: upload length %lu exceeds allowed max %lu (target limit %lu, ofs 0x%llx)\n",
			       length, max, target_max,
			       (unsigned long long)target_ofs);
		return -EFBIG;
	}

	return 0;
}

/* Only dynamic endpoints here; static files come from fsdata */

static void recovery_http_static_file(struct fs_file *file, const char *data,
				      int len)
{
	file->data = data;
	file->len = len;
	/* Dynamic reads are enabled for backup streams. Static data is complete. */
	file->index = len;
	file->pextension = NULL;
	file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
}

static int recovery_http_error(struct fs_file *file, const char *status,
			       const char *message)
{
	static char page_error[384];
	int body_len, header_len;

	body_len = strlen(message);
	header_len = snprintf(page_error, sizeof(page_error),
			      "HTTP/1.0 %s\r\n"
			      "Content-Type: text/plain\r\n"
			      "Cache-Control: no-store\r\n"
			      "Content-Length: %d\r\n"
			      "Connection: close\r\n\r\n%s",
			      status, body_len, message);
	if (header_len < 0 || header_len >= sizeof(page_error))
		return 0;

	recovery_http_static_file(file, page_error, header_len);
	return 1;
}

static void recovery_partition_display_name(const char *name, char *safe,
					    size_t safe_len)
{
	size_t i;

	if (!safe_len)
		return;

	for (i = 0; name && name[i] && i + 1 < safe_len; i++) {
		unsigned char c = name[i];

		safe[i] = c >= 0x20 && c <= 0x7e && c != '"' && c != '\\' ?
			  c : '_';
	}
	safe[i] = '\0';
}

static void recovery_partition_filename(const char *name, char *safe,
					size_t safe_len)
{
	size_t i;

	if (!safe_len)
		return;

	for (i = 0; name && name[i] && i + 1 < safe_len; i++) {
		unsigned char c = name[i];

		if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
		    (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.')
			safe[i] = c;
		else
			safe[i] = '_';
	}
	safe[i] = '\0';
}

static int recovery_backup_source_from_partition(struct blk_desc *desc,
						 int number,
						 struct recovery_backup_source *source)
{
	struct disk_partition part;
	char raw_name[PART_NAME_LEN + 1];
	char safe_name[PART_NAME_LEN + 1];
	ulong blksz;
	int ret;

	ret = part_get_info(desc, number, &part);
	if (ret || !part.size)
		return -ENOENT;

	blksz = part.blksz ?: desc->blksz;
	if (!blksz || (unsigned long long)part.size > ~0ULL / blksz)
		return -EOVERFLOW;

	memset(source, 0, sizeof(*source));
	memcpy(raw_name, part.name, PART_NAME_LEN);
	raw_name[PART_NAME_LEN] = '\0';
	recovery_partition_display_name(raw_name, source->name,
					sizeof(source->name));
	recovery_partition_filename(raw_name, safe_name, sizeof(safe_name));

	source->hwpart = EMMC_HWPART_DEFAULT;
	source->number = number;
	source->start = part.start;
	source->blocks = part.size;
	source->blksz = blksz;
	source->bytes = (unsigned long long)part.size * blksz;
	snprintf(source->filename, sizeof(source->filename), "p%02d-%s.img",
		 number, safe_name[0] ? safe_name : "partition");
	return 0;
}

static int recovery_backup_source_from_boot(struct blk_desc *desc,
					    int boot_index,
					    struct recovery_backup_source *source)
{
	int hwpart = EMMC_HWPART_BOOT1 + boot_index;
	int ret;

	ret = blk_dselect_hwpart(desc, hwpart);
	if (ret)
		return ret;
	if (!desc->blksz || !desc->lba ||
	    (unsigned long long)desc->lba > ~0ULL / desc->blksz)
		return -EOVERFLOW;

	memset(source, 0, sizeof(*source));
	source->hwpart = hwpart;
	source->start = 0;
	source->blocks = desc->lba;
	source->blksz = desc->blksz;
	source->bytes = (unsigned long long)desc->lba * desc->blksz;
	snprintf(source->name, sizeof(source->name), "eMMC boot%d", boot_index);
	snprintf(source->filename, sizeof(source->filename), "emmc-boot%d.img",
		 boot_index);
	return 0;
}

static int recovery_restore_source_from_id(
	struct blk_desc *desc, const char *id,
	struct recovery_backup_source *source)
{
	static const char prefix[] = "partition-";
	unsigned int number = 0;
	const char *p;
	int ret;

	if (!strcmp(id, "boot0") || !strcmp(id, "boot1")) {
		ret = recovery_backup_source_from_boot(desc, id[4] - '0', source);
		if (blk_dselect_hwpart(desc, EMMC_HWPART_DEFAULT) && !ret)
			ret = -EIO;
		return ret;
	}

	if (strncmp(id, prefix, strlen(prefix)))
		return -EINVAL;
	p = id + strlen(prefix);
	if (*p < '0' || *p > '9')
		return -EINVAL;
	while (*p >= '0' && *p <= '9') {
		number = number * 10 + (*p++ - '0');
		if (number > MAX_SEARCH_PARTITIONS)
			return -EINVAL;
	}
	if (*p || !number)
		return -EINVAL;

	ret = blk_dselect_hwpart(desc, EMMC_HWPART_DEFAULT);
	if (ret)
		return ret;
	return recovery_backup_source_from_partition(desc, number, source);
}

static void recovery_restore_reset(void)
{
	if (recovery_restore.desc)
		blk_dselect_hwpart(recovery_restore.desc, EMMC_HWPART_DEFAULT);
	free(recovery_restore.buf);
	memset(&recovery_restore, 0, sizeof(recovery_restore));
}

static int recovery_restore_prepare_partition(
	const char *id, unsigned long long total,
	struct recovery_status_led_ctrl *status_leds)
{
	struct recovery_backup_source source;
	struct disk_partition part;
	struct blk_desc *desc;
	size_t buf_size;
	int restore_ret;
	int ret;

	recovery_restore_reset();
	ret = blk_get_device_by_str("mmc", recovery_mmcdev(), &desc);
	if (ret < 0)
		return ret;

	ret = recovery_restore_source_from_id(desc, id, &source);
	if (ret)
		return ret;
	if (!total || total != source.bytes) {
		printf("Partition restore size %llu does not match target '%s' size %llu\n",
		       total, id, source.bytes);
		return -EFBIG;
	}
	if (!source.blksz || source.blocks > UINT_MAX)
		return -EOVERFLOW;

	buf_size = source.bytes < RECOVERY_MMC_RESTORE_CHUNK ?
		(size_t)source.bytes : RECOVERY_MMC_RESTORE_CHUNK;
	buf_size -= buf_size % source.blksz;
	if (!buf_size)
		return -EOVERFLOW;

	recovery_restore.buf = memalign(ARCH_DMA_MINALIGN, buf_size);
	if (!recovery_restore.buf)
		return -ENOMEM;
	recovery_restore.buf_size = buf_size;
	recovery_restore.desc = desc;
	recovery_restore.source = source;
	recovery_restore.total = total;
	strlcpy(recovery_restore.id, id, sizeof(recovery_restore.id));

	memset(&part, 0, sizeof(part));
	part.start = source.start;
	part.size = source.blocks;
	part.blksz = source.blksz;
	strlcpy((char *)part.name, source.name, sizeof(part.name));

	prog_phase = 1;
	prog_done = 0;
	prog_erase_done = 0;
	prog_erase_total = RECOVERY_MMC_ERASE_PROGRESS_STEPS;
	prog_write_done = 0;
	prog_write_total = source.blocks;
	prog_total = prog_erase_total + prog_write_total;
	prog_reboot = 0;

	ret = blk_dselect_hwpart(desc, source.hwpart);
	if (!ret)
		ret = recovery_mmc_erase_resolved(id, status_leds, desc, &part, 0);
	restore_ret = blk_dselect_hwpart(desc, EMMC_HWPART_DEFAULT);
	if (!ret && restore_ret)
		ret = restore_ret;
	if (ret) {
		prog_phase = -1;
		recovery_restore_reset();
		return ret;
	}

	recovery_restore.active = true;
	recovery_restore.prepared = true;
	prog_phase = 4;
	prog_erase_done = prog_erase_total;
	prog_done = prog_erase_done;
	recovery_debug_printf("Partition restore target '%s' erased; waiting for %llu bytes\n",
			      id, total);
	return 0;
}

static int recovery_restore_flush(void)
{
	struct recovery_backup_source *source = &recovery_restore.source;
	lbaint_t blk, blocks;
	int restore_ret;
	int ret = 0;

	if (!recovery_restore.active || !recovery_restore.receiving)
		return -EINVAL;
	if (!recovery_restore.buf_used)
		return 0;
	if (recovery_restore.buf_used % source->blksz ||
	    recovery_restore.written % source->blksz)
		return -EINVAL;

	blk = source->start + recovery_restore.written / source->blksz;
	blocks = recovery_restore.buf_used / source->blksz;
	if (blocks > source->blocks - recovery_restore.written / source->blksz)
		return -EFBIG;

	ret = blk_dselect_hwpart(recovery_restore.desc, source->hwpart);
	if (!ret && blk_dwrite(recovery_restore.desc, blk, blocks,
			       recovery_restore.buf) != blocks)
		ret = -EIO;
	restore_ret = blk_dselect_hwpart(recovery_restore.desc,
					 EMMC_HWPART_DEFAULT);
	if (!ret && restore_ret)
		ret = restore_ret;
	if (ret) {
		printf("Partition restore write failed for '%s' at block " LBAF
		       " count " LBAF "\n", recovery_restore.id, blk, blocks);
		return ret;
	}

	recovery_restore.written += recovery_restore.buf_used;
	recovery_restore.buf_used = 0;
	prog_write_done = recovery_restore.written / source->blksz;
	prog_done = prog_erase_done + prog_write_done;
	if (recovery_runtime_status_leds)
		recovery_status_led_poll(recovery_runtime_status_leds);
	recovery_watchdog_poll();
	return 0;
}

static int recovery_restore_append(const void *data, size_t size)
{
	const u8 *src = data;

	if (!recovery_restore.active || !recovery_restore.receiving ||
	    recovery_restore.chunk_received > recv_total ||
	    size > recv_total - recovery_restore.chunk_received)
		return -EFBIG;

	while (size) {
		size_t space = recovery_restore.buf_size - recovery_restore.buf_used;
		size_t copy = min(size, space);
		int ret;

		memcpy(recovery_restore.buf + recovery_restore.buf_used, src, copy);
		recovery_restore.buf_used += copy;
		recovery_restore.chunk_received += copy;
		src += copy;
		size -= copy;
		if (recovery_restore.buf_used == recovery_restore.buf_size) {
			ret = recovery_restore_flush();
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int recovery_restore_finish_chunk(void)
{
	unsigned long long expected_end;
	int ret;

	if (!recovery_restore.active || !recovery_restore.receiving ||
	    recovery_restore.chunk_received != recv_total)
		return -EIO;

	ret = recovery_restore_flush();
	if (ret)
		return ret;
	expected_end = recovery_restore.chunk_offset + recv_total;
	if (recovery_restore.written != expected_end ||
	    recovery_restore.written > recovery_restore.total)
		return -EIO;

	recovery_restore.receiving = false;
	if (recovery_restore.written == recovery_restore.total) {
		recovery_restore.active = false;
		recovery_restore.prepared = false;
		free(recovery_restore.buf);
		recovery_restore.buf = NULL;
		prog_write_done = prog_write_total;
		prog_done = prog_total;
		prog_phase = 3;
		recovery_debug_printf("Partition restore '%s' complete (%llu bytes)\n",
				      recovery_restore.id, recovery_restore.total);
	} else {
		recovery_restore.prepared = true;
		prog_phase = 4;
	}

	return 0;
}

/* boot0, boot1, then every valid GPT partition in the user area. */
static int recovery_backup_next_source(struct blk_desc *desc,
				       unsigned int *cursor,
				       struct recovery_backup_source *source)
{
	unsigned int p;
	int ret;

	if (*cursor < 2) {
		unsigned int boot_index = *cursor;

		(*cursor)++;
		ret = recovery_backup_source_from_boot(desc, boot_index, source);
		return ret ? ret : 1;
	}

	ret = blk_dselect_hwpart(desc, EMMC_HWPART_DEFAULT);
	if (ret)
		return ret;

	for (p = *cursor - 1; p <= MAX_SEARCH_PARTITIONS; p++) {
		*cursor = p + 2;
		ret = recovery_backup_source_from_partition(desc, p, source);
		if (!ret)
			return 1;
		if (ret != -ENOENT)
			return ret;
	}

	return 0;
}

static int recovery_backup_restore_user(struct recovery_backup_file *backup)
{
	int ret;

	if (backup->user_restored)
		return 0;

	ret = blk_dselect_hwpart(backup->desc, EMMC_HWPART_DEFAULT);
	if (ret) {
		printf("httpd: failed to restore eMMC user area after backup: %d\n",
		       ret);
		return ret;
	}

	backup->user_restored = true;
	return 0;
}

static int recovery_tar_format_octal(char *field, size_t field_size,
				     unsigned long long value)
{
	int len;

	len = snprintf(field, field_size, "%0*llo", (int)field_size - 1, value);
	return len == field_size - 1 ? 0 : -EOVERFLOW;
}

static int recovery_backup_prepare_tar_header(struct recovery_backup_file *backup)
{
	struct recovery_backup_tar_header *header = &backup->tar_header;
	const u8 *raw = (const u8 *)header;
	unsigned int checksum = 0;
	size_t i;
	int ret;

	if (sizeof(*header) != RECOVERY_TAR_BLOCK_SIZE ||
	    strlen(backup->source.filename) >= sizeof(header->name))
		return -EOVERFLOW;

	memset(header, 0, sizeof(*header));
	strlcpy(header->name, backup->source.filename, sizeof(header->name));
	ret = recovery_tar_format_octal(header->mode, sizeof(header->mode), 0600);
	ret |= recovery_tar_format_octal(header->uid, sizeof(header->uid), 0);
	ret |= recovery_tar_format_octal(header->gid, sizeof(header->gid), 0);
	ret |= recovery_tar_format_octal(header->size, sizeof(header->size),
					 backup->source.bytes);
	ret |= recovery_tar_format_octal(header->mtime, sizeof(header->mtime), 0);
	if (ret)
		return ret;

	memset(header->checksum, ' ', sizeof(header->checksum));
	header->typeflag = '0';
	memcpy(header->magic, "ustar", sizeof("ustar"));
	memcpy(header->version, "00", sizeof(header->version));
	strlcpy(header->uname, "root", sizeof(header->uname));
	strlcpy(header->gname, "root", sizeof(header->gname));

	for (i = 0; i < sizeof(*header); i++)
		checksum += raw[i];
	if (snprintf(header->checksum, sizeof(header->checksum), "%06o",
		     checksum) != 6)
		return -EOVERFLOW;
	header->checksum[6] = '\0';
	header->checksum[7] = ' ';
	return 0;
}

static int
recovery_backup_start_next_tar_source(struct recovery_backup_file *backup)
{
	struct recovery_backup_source source;
	int ret;

	ret = recovery_backup_next_source(backup->desc, &backup->source_cursor,
					  &source);
	if (ret <= 0) {
		if (!ret) {
			backup->tar_phase = RECOVERY_BACKUP_TAR_END;
			backup->tar_end_left = RECOVERY_TAR_END_SIZE;
		}
		return ret;
	}

	backup->source = source;
	backup->next_lba = source.start;
	backup->blocks_left = source.blocks;
	backup->blksz = source.blksz;
	backup->cache_len = 0;
	backup->cache_off = 0;
	backup->tar_header_off = 0;
	backup->tar_padding_left =
		(RECOVERY_TAR_BLOCK_SIZE -
		 (source.bytes % RECOVERY_TAR_BLOCK_SIZE)) %
		RECOVERY_TAR_BLOCK_SIZE;
	ret = recovery_backup_prepare_tar_header(backup);
	if (ret)
		return ret;
	backup->tar_phase = RECOVERY_BACKUP_TAR_MEMBER_HEADER;
	return 1;
}

static int recovery_backup_scan_all(struct blk_desc *desc,
				    unsigned long long *archive_size,
				    size_t *cache_capacity,
				    unsigned int *source_count)
{
	struct recovery_backup_source source;
	unsigned long long total = RECOVERY_TAR_END_SIZE;
	unsigned int cursor = 0;
	unsigned int count = 0;
	size_t max_cache = 0;
	int restore_ret;
	int ret;

	while ((ret = recovery_backup_next_source(desc, &cursor, &source)) > 0) {
		unsigned long long padded;
		size_t source_cache;

		if (source.bytes > RECOVERY_TAR_MAX_MEMBER_SIZE) {
			ret = -EFBIG;
			break;
		}
		padded = (source.bytes + RECOVERY_TAR_BLOCK_SIZE - 1) &
			 ~((unsigned long long)RECOVERY_TAR_BLOCK_SIZE - 1);
		if (total > ~0ULL - RECOVERY_TAR_BLOCK_SIZE - padded) {
			ret = -EOVERFLOW;
			break;
		}
		total += RECOVERY_TAR_BLOCK_SIZE + padded;
		source_cache = source.bytes < RECOVERY_MMC_BACKUP_CHUNK ?
			       source.bytes : RECOVERY_MMC_BACKUP_CHUNK;
		if (source_cache > max_cache)
			max_cache = source_cache;
		count++;
	}

	restore_ret = blk_dselect_hwpart(desc, EMMC_HWPART_DEFAULT);
	if (!ret && restore_ret)
		ret = restore_ret;
	if (ret < 0)
		return ret;
	if (!count || !max_cache)
		return -ENOENT;

	*archive_size = total;
	*cache_capacity = max_cache;
	*source_count = count;
	return 0;
}

static int recovery_backup_append_source_json(char *json, size_t json_size,
					      size_t *off, bool comma,
					      const struct recovery_backup_source *source)
{
	int boot_index;
	int ret;

	if (source->number) {
		ret = recovery_appendf(json, json_size, off,
				       "%s{\"id\":\"partition-%d\",\"kind\":\"gpt\",",
				       comma ? "," : "", source->number);
		if (ret)
			return ret;
		ret = recovery_appendf(json, json_size, off,
				       "\"path\":\"/backup/partition-%d.bin\",\"number\":%d,",
				       source->number, source->number);
		if (ret)
			return ret;
		ret = recovery_appendf(json, json_size, off,
				       "\"name\":\"%s\",\"start\":%llu,\"blocks\":%llu,",
				       source->name,
				       (unsigned long long)source->start,
				       (unsigned long long)source->blocks);
		if (ret)
			return ret;
		return recovery_appendf(json, json_size, off,
			"\"block_size\":%lu,\"size\":%llu}",
			source->blksz, source->bytes);
	}

	boot_index = source->hwpart - EMMC_HWPART_BOOT1;
	ret = recovery_appendf(json, json_size, off,
			       "%s{\"id\":\"boot%d\",\"kind\":\"hardware\",",
			       comma ? "," : "", boot_index);
	if (ret)
		return ret;
	ret = recovery_appendf(json, json_size, off,
			       "\"path\":\"/backup/boot%d.bin\",\"name\":\"%s\",",
			       boot_index, source->name);
	if (ret)
		return ret;
	ret = recovery_appendf(json, json_size, off,
			       "\"start\":0,\"blocks\":%llu,\"block_size\":%lu,",
			       (unsigned long long)source->blocks,
			       source->blksz);
	if (ret)
		return ret;
	return recovery_appendf(json, json_size, off, "\"size\":%llu}",
				 source->bytes);
}

static int recovery_open_partitions(struct fs_file *file)
{
	static char page[RECOVERY_PARTITIONS_JSON_MAX + 256];
	static char json[RECOVERY_PARTITIONS_JSON_MAX];
	struct recovery_backup_source source;
	struct blk_desc *desc;
	unsigned int cursor = 0;
	size_t off = 0;
	int count = 0;
	int header_len;
	int ret;

	if (recovery_backup_active)
		return recovery_http_error(file, "409 Conflict",
					   "A backup stream is already active.\n");
	if ((prog_phase > 0 && prog_phase < 3) || recovery_stream.active ||
	    recovery_restore.receiving || restore_prepare_request || flash_request ||
	    prog_reboot || reboot_request)
		return recovery_http_error(file, "409 Conflict",
					   "A destructive storage operation is active.\n");

	ret = blk_get_device_by_str("mmc", recovery_mmcdev(), &desc);
	if (ret < 0)
		return recovery_http_error(file, "503 Service Unavailable",
					   "eMMC device is unavailable.\n");

	off += snprintf(json + off, sizeof(json) - off, "{\"partitions\":[");
	while ((ret = recovery_backup_next_source(desc, &cursor, &source)) > 0) {
		ret = recovery_backup_append_source_json(json, sizeof(json), &off,
							 count, &source);
		if (ret)
			break;
		count++;
	}
	if (blk_dselect_hwpart(desc, EMMC_HWPART_DEFAULT) && ret >= 0)
		ret = -EIO;
	if (ret < 0)
		return recovery_http_error(file, "503 Service Unavailable",
					   "Cannot enumerate all eMMC partitions.\n");

	memcpy(json + off, "]}\n", sizeof("]}\n"));
	off += sizeof("]}\n") - 1;
	header_len = snprintf(page, sizeof(page),
			      "HTTP/1.0 200 OK\r\n"
			      "Content-Type: application/json\r\n"
			      "Cache-Control: no-store\r\n"
			      "Content-Length: %lu\r\n"
			      "Connection: close\r\n\r\n",
			      (ulong)off);
	if (header_len < 0 || header_len + off > sizeof(page))
		return 0;

	memcpy(page + header_len, json, off);
	recovery_http_static_file(file, page, header_len + off);
	return 1;
}

static int recovery_backup_partition_number(const char *path)
{
	const char *prefix = "backup/partition-";
	unsigned int number = 0;

	if (strncmp(path, prefix, strlen(prefix)))
		return -ENOENT;

	path += strlen(prefix);
	if (*path < '0' || *path > '9')
		return -EINVAL;
	while (*path >= '0' && *path <= '9') {
		number = number * 10 + (*path++ - '0');
		if (number > MAX_SEARCH_PARTITIONS)
			return -EINVAL;
	}
	if (strcmp(path, ".bin") || !number)
		return -EINVAL;

	return number;
}

enum recovery_backup_request {
	RECOVERY_BACKUP_REQUEST_NONE = 0,
	RECOVERY_BACKUP_REQUEST_PARTITION,
	RECOVERY_BACKUP_REQUEST_BOOT0,
	RECOVERY_BACKUP_REQUEST_BOOT1,
	RECOVERY_BACKUP_REQUEST_ALL,
	RECOVERY_BACKUP_REQUEST_INVALID,
};

static enum recovery_backup_request
recovery_backup_parse_request(const char *path, int *number)
{
	int ret;

	if (!strcmp(path, "backup/boot0.bin"))
		return RECOVERY_BACKUP_REQUEST_BOOT0;
	if (!strcmp(path, "backup/boot1.bin"))
		return RECOVERY_BACKUP_REQUEST_BOOT1;
	if (!strcmp(path, "backup/all.tar"))
		return RECOVERY_BACKUP_REQUEST_ALL;

	ret = recovery_backup_partition_number(path);
	if (ret == -ENOENT)
		return RECOVERY_BACKUP_REQUEST_NONE;
	if (ret < 0)
		return RECOVERY_BACKUP_REQUEST_INVALID;
	*number = ret;
	return RECOVERY_BACKUP_REQUEST_PARTITION;
}

static int recovery_open_backup(struct fs_file *file, const char *path)
{
	struct recovery_backup_file *backup;
	struct recovery_backup_source source;
	struct blk_desc *desc;
	unsigned long long archive_size = 0;
	unsigned int source_count = 0;
	lbaint_t cache_blocks;
	size_t cache_capacity = 0;
	enum recovery_backup_request request;
	const char *error_message = "Cannot prepare eMMC backup.\n";
	const char *error_status = "503 Service Unavailable";
	int header_len;
	int number = 0;
	int ret;

	request = recovery_backup_parse_request(path, &number);
	if (request == RECOVERY_BACKUP_REQUEST_NONE)
		return 0;
	if (request == RECOVERY_BACKUP_REQUEST_INVALID)
		return recovery_http_error(file, "400 Bad Request",
					   "Invalid partition backup path.\n");
	if (recovery_backup_active)
		return recovery_http_error(file, "409 Conflict",
					   "A backup stream is already active.\n");
	if ((prog_phase > 0 && prog_phase < 3) || recovery_stream.active ||
	    recovery_restore.active || restore_prepare_request || flash_request ||
	    prog_reboot || reboot_request)
		return recovery_http_error(file, "409 Conflict",
					   "A destructive storage operation is active.\n");

	ret = blk_get_device_by_str("mmc", recovery_mmcdev(), &desc);
	if (ret < 0)
		return recovery_http_error(file, "503 Service Unavailable",
					   "eMMC device is unavailable.\n");

	backup = calloc(1, sizeof(*backup));
	if (!backup)
		return recovery_http_error(file, "503 Service Unavailable",
					   "Cannot allocate backup state.\n");

	backup->desc = desc;
	if (request == RECOVERY_BACKUP_REQUEST_ALL) {
		ret = recovery_backup_scan_all(desc, &archive_size, &cache_capacity,
					       &source_count);
		if (ret) {
			if (ret == -EFBIG) {
				error_status = "500 Internal Server Error";
				error_message =
					"A partition is too large for a ustar member.\n";
			}
			goto error;
		}
		backup->mode = RECOVERY_BACKUP_ALL_TAR;
	} else {
		if (request == RECOVERY_BACKUP_REQUEST_PARTITION) {
			ret = blk_dselect_hwpart(desc, EMMC_HWPART_DEFAULT);
			if (!ret)
				ret = recovery_backup_source_from_partition(desc,
									    number, &source);
		} else {
			ret = recovery_backup_source_from_boot(desc,
							       request ==
							       RECOVERY_BACKUP_REQUEST_BOOT1,
							       &source);
		}
		if (ret) {
			if (ret == -ENOENT) {
				error_status = "404 Not Found";
				error_message = "Partition does not exist.\n";
			}
			goto error;
		}

		backup->mode = RECOVERY_BACKUP_SINGLE;
		backup->source = source;
		backup->next_lba = source.start;
		backup->blocks_left = source.blocks;
		backup->blksz = source.blksz;
		cache_blocks = RECOVERY_MMC_BACKUP_CHUNK / source.blksz;
		if (!cache_blocks) {
			error_status = "500 Internal Server Error";
			error_message = "eMMC block size exceeds backup buffer.\n";
			goto error;
		}
		if (cache_blocks > source.blocks)
			cache_blocks = source.blocks;
		cache_capacity = cache_blocks * source.blksz;
	}

	backup->cache = memalign(ARCH_DMA_MINALIGN, cache_capacity);
	if (!backup->cache) {
		error_message = "Cannot allocate backup buffer.\n";
		goto error;
	}
	backup->cache_capacity = cache_capacity;

	if (backup->mode == RECOVERY_BACKUP_ALL_TAR) {
		header_len = snprintf(backup->http_header,
				      sizeof(backup->http_header),
				      "HTTP/1.0 200 OK\r\n"
				      "Content-Type: application/x-tar\r\n"
				      "Content-Disposition: attachment; "
				      "filename=\"sbe1v1k-emmc-backup.tar\"\r\n"
				      "Cache-Control: no-store\r\n"
				      "Content-Length: %llu\r\n"
				      "Connection: close\r\n\r\n",
				      archive_size);
		ret = recovery_backup_start_next_tar_source(backup);
		if (ret <= 0)
			goto error;
	} else {
		header_len = snprintf(backup->http_header,
				      sizeof(backup->http_header),
				      "HTTP/1.0 200 OK\r\n"
				      "Content-Type: application/octet-stream\r\n"
				      "Content-Disposition: attachment; "
				      "filename=\"sbe1v1k-%s\"\r\n"
				      "Cache-Control: no-store\r\n"
				      "Content-Length: %llu\r\n"
				      "Connection: close\r\n\r\n",
				      source.filename, source.bytes);
	}
	if (header_len < 0 || header_len >= sizeof(backup->http_header))
		goto error;
	backup->http_header_len = header_len;

	backup->magic = RECOVERY_BACKUP_MAGIC;
	backup->active_counted = true;
	recovery_backup_active++;

	file->data = NULL;
	file->len = INT_MAX;
	file->index = 0;
	file->pextension = (fs_file_extension *)backup;
	file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
	if (backup->mode == RECOVERY_BACKUP_ALL_TAR)
		recovery_debug_printf("httpd: streaming full eMMC backup (%u sources, %llu-byte tar)\n",
				      source_count, archive_size);
	else
		recovery_debug_printf("httpd: streaming backup of %s (%llu bytes)\n",
				      source.name, source.bytes);
	return 1;

error:
	blk_dselect_hwpart(desc, EMMC_HWPART_DEFAULT);
	free(backup->cache);
	free(backup);
	return recovery_http_error(file, error_status, error_message);
}

static int recovery_http_json(struct fs_file *file, char *page,
			      size_t page_size, const char *json, int json_len)
{
	int header_len;
	size_t body_len;

	if (json_len < 0)
		return 0;
	if ((size_t)json_len >= page_size)
		return 0;

	header_len = snprintf(page, page_size,
			      "HTTP/1.0 200 OK\r\n"
			      "Content-Type: application/json\r\n"
			      "Cache-Control: no-store\r\n"
			      "Content-Length: %d\r\n"
			      "Connection: close\r\n\r\n",
			      json_len);
	if (header_len < 0 || (size_t)header_len >= page_size)
		return 0;
	if ((size_t)json_len > page_size - header_len)
		return 0;

	body_len = json_len;
	memcpy(page + header_len, json, body_len);
	recovery_http_static_file(file, page, header_len + body_len);
	return 1;
}

static int recovery_open_status(struct fs_file *file)
{
	static char page[512];
	char json[256];
	int json_len;

	json_len = snprintf(json, sizeof(json),
			    "{\"in_progress\":%d,\"done\":%u,\"total\":%u,"
			    "\"erase_done\":%u,\"erase_total\":%u,"
			    "\"write_done\":%u,\"write_total\":%u,"
			    "\"prepared\":%d,\"ok\":%d,\"error\":%d,\"phase\":%d,"
			    "\"reboot\":%d}\n",
			    prog_phase > 0 && prog_phase < 3,
			    (unsigned int)prog_done, (unsigned int)prog_total,
			    (unsigned int)prog_erase_done,
			    (unsigned int)prog_erase_total,
			    (unsigned int)prog_write_done,
			    (unsigned int)prog_write_total,
			    (recovery_stream.active && recovery_stream.prepared) ||
			    (recovery_restore.active && recovery_restore.prepared),
			    prog_phase == 3, prog_phase == -1, prog_phase,
			    prog_reboot);
	if (json_len < 0)
		return 0;
	if ((size_t)json_len >= sizeof(json))
		json_len = sizeof(json) - 1;

	return recovery_http_json(file, page, sizeof(page), json, json_len);
}

static int recovery_open_about(struct fs_file *file)
{
	const struct recovery_sbe1v1k_layout_desc *layout =
		recovery_sbe1v1k_layout_desc(active_sbe1v1k_layout);
	static char page[896];
	unsigned long long firmware_max;
	unsigned long env_max;
	const char *layout_name;
	const char *rootarg;
	const char *uboot_part;
	ulong kernel_pad;
	bool firmware_locked;
	bool qwrt_unlocked;
	char json[768];
	int json_len;
	qwrt_unlocked = recovery_board_is_sbe1v1k() && qwrt_button_unlocked;

	if (!layout && recovery_board_is_sbe1v1k()) {
		layout_name = sbe1v1k_factory_pre_migration ? "factory" :
			"unknown";
		firmware_max = 0;
		kernel_pad = 0;
		rootarg = "";
		uboot_part = env_get("recovery_part_uboot") ?: "";
	} else {
		if (!layout)
			layout = &sbe1v1k_layout_large;
		layout_name = layout->name;
		firmware_max = (unsigned long long)(layout->kernel_size +
								 layout->rootfs_size) * 512;
		/* Keep the one-shot QWRT profile undiscoverable until the button unlock. */
		if (active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_QWRT &&
		    !qwrt_unlocked)
			layout_name = "mainline";
		env_max = env_get_hex("recovery_max", 0);
		if (env_max && firmware_max > env_max)
			firmware_max = env_max;
		kernel_pad = layout->kernel_pad;
		rootarg = layout->rootarg;
		uboot_part = layout->uboot_part;
	}
	firmware_locked = recovery_board_is_sbe1v1k() &&
		(active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_UNKNOWN ||
		 (active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_QWRT &&
		  !qwrt_unlocked));

#ifdef U_BOOT_DATE
	json_len = snprintf(json, sizeof(json),
			    "{\"u_boot\":\"%s (%s - %s %s)\","
			    "\"layout\":\"%s\",\"kernel_pad\":%lu,"
			    "\"firmware_max\":%llu,\"rootarg\":\"%s\","
				    "\"uboot_part\":\"%s\",\"firmware_locked\":%d,"
				    "\"qwrt_available\":%d,\"qwrt_unlocked\":%d,"
				    "\"qwrt_button_presses\":%u}\n",
				    U_BOOT_VERSION, U_BOOT_DATE, U_BOOT_TIME,
				    U_BOOT_TZ, layout_name, kernel_pad,
				    firmware_max, rootarg, uboot_part, firmware_locked,
				    qwrt_button_available, qwrt_unlocked,
				    qwrt_button_presses);
#else
	json_len = snprintf(json, sizeof(json),
			    "{\"u_boot\":\"%s\",\"layout\":\"%s\","
			    "\"kernel_pad\":%lu,\"firmware_max\":%llu,"
			    "\"rootarg\":\"%s\",\"uboot_part\":\"%s\","
				    "\"firmware_locked\":%d,\"qwrt_available\":%d,"
				    "\"qwrt_unlocked\":%d,\"qwrt_button_presses\":%u}\n",
				    U_BOOT_VERSION, layout_name,
				    kernel_pad, firmware_max, rootarg, uboot_part,
				    firmware_locked, qwrt_button_available, qwrt_unlocked,
				    qwrt_button_presses);
#endif
	if (json_len < 0)
		return 0;
	if ((size_t)json_len >= sizeof(json))
		json_len = sizeof(json) - 1;

	return recovery_http_json(file, page, sizeof(page), json, json_len);
}

/* lwIP httpd custom file hooks; static files continue to use fsdata. */
int fs_open_custom(struct fs_file *file, const char *name)
{
	static const char page_ok[] =
		"HTTP/1.0 200 OK\r\n"
		"Content-Type: text/plain\r\n"
		"Cache-Control: no-store\r\n"
		"Content-Length: 2\r\n"
		"Connection: close\r\n\r\n"
		"OK";
	const char *p;

	if (!file || !name)
		return 0;

	memset(file, 0, sizeof(*file));
	p = *name == '/' ? name + 1 : name;

	if (!strcmp(p, "partitions"))
		return recovery_open_partitions(file);
	if (!strncmp(p, "backup/", 7))
		return recovery_open_backup(file, p);
	if (!strcmp(p, "status"))
		return recovery_open_status(file);
	if (!strcmp(p, "about"))
		return recovery_open_about(file);
	if (!strcmp(p, "ok")) {
		recovery_http_static_file(file, page_ok, sizeof(page_ok) - 1);
		return 1;
	}

	return 0;
}

void fs_close_custom(struct fs_file *file)
{
	struct recovery_backup_file *backup;

	if (!file || !file->pextension)
		return;

	backup = (struct recovery_backup_file *)file->pextension;
	if (backup->magic != RECOVERY_BACKUP_MAGIC)
		return;

	recovery_backup_restore_user(backup);
	backup->magic = 0;
	if (backup->active_counted && recovery_backup_active)
		recovery_backup_active--;
	free(backup->cache);
	free(backup);
	file->pextension = NULL;
}

static int recovery_backup_fill_cache(struct recovery_backup_file *backup)
{
	lbaint_t blocks;
	int ret;

	if (!backup->blocks_left)
		return 0;
	if (backup->desc->hwpart != backup->source.hwpart) {
		ret = blk_dselect_hwpart(backup->desc, backup->source.hwpart);
		if (ret) {
			printf("httpd: cannot select eMMC hwpart %d for backup: %d\n",
			       backup->source.hwpart, ret);
			goto read_error;
		}
	}

	blocks = backup->cache_capacity / backup->blksz;
	if (!blocks)
		goto read_error;
	if (blocks > backup->blocks_left)
		blocks = backup->blocks_left;
	if (blk_dread(backup->desc, backup->next_lba, blocks,
		      backup->cache) != blocks) {
		printf("httpd: eMMC backup read failed on hwpart %d at block "
		       LBAF "\n", backup->source.hwpart, backup->next_lba);
		goto read_error;
	}

	backup->next_lba += blocks;
	backup->blocks_left -= blocks;
	backup->cache_len = blocks * backup->blksz;
	backup->cache_off = 0;
	recovery_watchdog_poll();
	return 1;

read_error:
	backup->failed = true;
	backup->blocks_left = 0;
	backup->cache_len = 0;
	backup->cache_off = 0;
	return -EIO;
}

static int recovery_read_backup_single(struct recovery_backup_file *backup,
				       char *buffer, int count)
{
	int copied = 0;

	while (copied < count) {
		size_t available;
		size_t todo;
		int ret;

		if (backup->cache_off >= backup->cache_len) {
			ret = recovery_backup_fill_cache(backup);
			if (ret <= 0)
				break;
		}

		available = backup->cache_len - backup->cache_off;
		todo = min_t(size_t, available, count - copied);
		memcpy(buffer + copied, backup->cache + backup->cache_off, todo);
		backup->cache_off += todo;
		copied += todo;
	}

	if (!backup->blocks_left && backup->cache_off >= backup->cache_len)
		recovery_backup_restore_user(backup);
	return copied ? copied : FS_READ_EOF;
}

static int recovery_read_backup_tar(struct recovery_backup_file *backup,
				    char *buffer, int count)
{
	int copied = 0;

	while (copied < count) {
		size_t available;
		size_t todo;
		int ret;

		switch (backup->tar_phase) {
		case RECOVERY_BACKUP_TAR_MEMBER_HEADER:
			available = sizeof(backup->tar_header) -
				    backup->tar_header_off;
			todo = min_t(size_t, available, count - copied);
			memcpy(buffer + copied,
			       (const u8 *)&backup->tar_header +
			       backup->tar_header_off, todo);
			backup->tar_header_off += todo;
			copied += todo;
			if (backup->tar_header_off == sizeof(backup->tar_header))
				backup->tar_phase = RECOVERY_BACKUP_TAR_MEMBER_DATA;
			break;

		case RECOVERY_BACKUP_TAR_MEMBER_DATA:
			if (backup->cache_off >= backup->cache_len) {
				ret = recovery_backup_fill_cache(backup);
				if (ret < 0)
					goto failed;
				if (!ret) {
					backup->tar_phase =
						RECOVERY_BACKUP_TAR_MEMBER_PADDING;
					break;
				}
			}
			available = backup->cache_len - backup->cache_off;
			todo = min_t(size_t, available, count - copied);
			memcpy(buffer + copied, backup->cache + backup->cache_off,
			       todo);
			backup->cache_off += todo;
			copied += todo;
			break;

		case RECOVERY_BACKUP_TAR_MEMBER_PADDING:
			if (backup->tar_padding_left) {
				todo = min_t(unsigned long long,
					     backup->tar_padding_left, count - copied);
				memset(buffer + copied, 0, todo);
				backup->tar_padding_left -= todo;
				copied += todo;
				break;
			}
			ret = recovery_backup_start_next_tar_source(backup);
			if (ret < 0)
				goto failed;
			break;

		case RECOVERY_BACKUP_TAR_END:
			if (backup->tar_end_left) {
				todo = min_t(unsigned long long, backup->tar_end_left,
					     count - copied);
				memset(buffer + copied, 0, todo);
				backup->tar_end_left -= todo;
				copied += todo;
				break;
			}
			backup->tar_phase = RECOVERY_BACKUP_TAR_DONE;
			recovery_backup_restore_user(backup);
			break;

		case RECOVERY_BACKUP_TAR_DONE:
			return copied ? copied : FS_READ_EOF;
		}
	}

	return copied ? copied : FS_READ_EOF;

failed:
	backup->failed = true;
	backup->tar_phase = RECOVERY_BACKUP_TAR_DONE;
	recovery_backup_restore_user(backup);
	return copied ? copied : FS_READ_EOF;
}

static int recovery_read_backup(struct recovery_backup_file *backup,
				char *buffer, int count)
{
	int copied = 0;
	int read;

	if (backup->http_header_off < backup->http_header_len) {
		size_t available = backup->http_header_len -
				   backup->http_header_off;
		size_t todo = min_t(size_t, available, count);

		memcpy(buffer, backup->http_header + backup->http_header_off, todo);
		backup->http_header_off += todo;
		copied += todo;
	}
	if (copied == count)
		return copied;

	if (backup->mode == RECOVERY_BACKUP_ALL_TAR)
		read = recovery_read_backup_tar(backup, buffer + copied,
						count - copied);
	else
		read = recovery_read_backup_single(backup, buffer + copied,
						   count - copied);
	if (read > 0)
		copied += read;
	return copied ? copied : read;
}

int fs_read_custom(struct fs_file *file, char *buffer, int count)
{
	struct recovery_backup_file *backup;
	u32_t left;
	int read;
	bool finished;

	if (!file || !buffer || count <= 0)
		return FS_READ_EOF;

	backup = (struct recovery_backup_file *)file->pextension;
	if (backup && backup->magic == RECOVERY_BACKUP_MAGIC) {
		read = recovery_read_backup(backup, buffer, count);
		if (backup->mode == RECOVERY_BACKUP_ALL_TAR)
			finished = backup->tar_phase == RECOVERY_BACKUP_TAR_DONE;
		else
			finished = !backup->blocks_left &&
				   backup->cache_off >= backup->cache_len;
		finished = finished &&
			   backup->http_header_off >= backup->http_header_len;
		if (read < 0 || finished) {
			recovery_backup_restore_user(backup);
			file->index = file->len;
		} else if (file->index > file->len - read - 1) {
			/* Keep lwIP's int-sized cursor live for partitions over 2 GiB. */
			file->index = 0;
		} else {
			file->index += read;
		}
		return read;
	}

	left = file->len - file->index;
	if (left <= 0)
		return FS_READ_EOF;
	if ((u32_t)count > left)
		count = left;
	memcpy(buffer, file->data + file->index, count);
	file->index += count;
	return count;
}

/* bytes_left for custom files is handled by fs_read_custom + file->index; */

/* HTTP POST handlers */
static int recovery_stream_format_from_uri(
	const char *uri, enum recovery_stream_format *format)
{
	const char *value = strstr(uri, "format=");

	*format = RECOVERY_STREAM_RAW;
	if (!value)
		return 0;
	value += strlen("format=");
	if (!strncmp(value, "raw", 3) && (value[3] == '\0' || value[3] == '&'))
		return 0;
	if (!strncmp(value, "tar", 3) && (value[3] == '\0' || value[3] == '&')) {
		*format = RECOVERY_STREAM_TAR;
		return 0;
	}

	return -EINVAL;
}

static int recovery_layout_from_uri(const char *uri,
				    enum recovery_sbe1v1k_layout *layout)
{
	const char *value = strstr(uri, "layout=");

	*layout = RECOVERY_SBE1V1K_LAYOUT_LARGE;
	if (!value)
		return 0;
	value += strlen("layout=");
	if (!strncmp(value, "large", 5) &&
	    (value[5] == '\0' || value[5] == '&'))
		return 0;
	if (!strncmp(value, "mainline", 8) &&
	    (value[8] == '\0' || value[8] == '&')) {
		*layout = RECOVERY_SBE1V1K_LAYOUT_MAINLINE;
		return 0;
	}
	if (!strncmp(value, "qwrt", 4) &&
	    (value[4] == '\0' || value[4] == '&')) {
		*layout = RECOVERY_SBE1V1K_LAYOUT_QWRT;
		return 0;
	}

	return -EINVAL;
}

static int recovery_parse_decimal_ull(const char *value,
				      unsigned long long *result)
{
	unsigned long long parsed = 0;

	if (!value || !*value || !result)
		return -EINVAL;
	while (*value) {
		unsigned int digit;

		if (*value < '0' || *value > '9')
			return -EINVAL;
		digit = *value++ - '0';
		if (parsed > (~0ULL - digit) / 10)
			return -EOVERFLOW;
		parsed = parsed * 10 + digit;
	}

	*result = parsed;
	return 0;
}

static bool recovery_restore_id_valid(const char *id)
{
	static const char prefix[] = "partition-";
	const char *p;
	unsigned int number = 0;

	if (!id)
		return false;
	if (!strcmp(id, "boot0") || !strcmp(id, "boot1"))
		return true;
	if (strncmp(id, prefix, strlen(prefix)))
		return false;

	p = id + strlen(prefix);
	if (*p < '0' || *p > '9')
		return false;
	while (*p >= '0' && *p <= '9') {
		number = number * 10 + (*p++ - '0');
		if (number > MAX_SEARCH_PARTITIONS)
			return false;
	}

	return !*p && number;
}

static int recovery_restore_request_from_uri(
	const char *uri, const char *path, bool chunk, char *id, size_t id_size,
	unsigned long long *offset, unsigned long long *total)
{
	const char *query;
	bool have_id = false;
	bool have_offset = false;
	bool have_total = false;

	if (strncmp(uri, path, strlen(path)) || uri[strlen(path)] != '?')
		return -EINVAL;
	query = uri + strlen(path) + 1;
	if (!*query)
		return -EINVAL;

	while (*query) {
		const char *end = strchr(query, '&');
		const char *equals;
		size_t item_len = end ? (size_t)(end - query) : strlen(query);
		size_t key_len;
		size_t value_len;
		char number[32];
		int ret;

		if (!item_len)
			return -EINVAL;
		equals = memchr(query, '=', item_len);
		if (!equals || equals == query)
			return -EINVAL;
		key_len = equals - query;
		value_len = item_len - key_len - 1;
		if (!value_len)
			return -EINVAL;

		if (key_len == 2 && !memcmp(query, "id", 2)) {
			if (have_id || value_len >= id_size)
				return -EINVAL;
			memcpy(id, equals + 1, value_len);
			id[value_len] = '\0';
			if (!recovery_restore_id_valid(id))
				return -EINVAL;
			have_id = true;
		} else if (chunk && key_len == 6 &&
			   !memcmp(query, "offset", 6)) {
			if (have_offset || value_len >= sizeof(number))
				return -EINVAL;
			memcpy(number, equals + 1, value_len);
			number[value_len] = '\0';
			ret = recovery_parse_decimal_ull(number, offset);
			if (ret)
				return ret;
			have_offset = true;
		} else if (((chunk && key_len == 5 &&
			     !memcmp(query, "total", 5)) ||
			    (!chunk && key_len == 4 &&
			     !memcmp(query, "size", 4)))) {
			if (have_total || value_len >= sizeof(number))
				return -EINVAL;
			memcpy(number, equals + 1, value_len);
			number[value_len] = '\0';
			ret = recovery_parse_decimal_ull(number, total);
			if (ret)
				return ret;
			have_total = true;
		} else {
			return -EINVAL;
		}

		if (!end)
			break;
		query = end + 1;
	}

	return have_id && have_total && (!chunk || have_offset) ? 0 : -EINVAL;
}

err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                       u16_t http_request_len, int content_len, char *response_uri,
                       u16_t response_uri_len, u8_t *post_auto_wnd)
{
	bool stream_enabled;
	const char *tname;

	(void)http_request;
	(void)http_request_len;
	/*
	 * Throttle large uploads explicitly: recovery accepts firmware images
	 * that are much larger than a typical lwIP POST body and manual window
	 * updates avoid over-optimistic receive windows.
	 */
	if (post_auto_wnd)
		*post_auto_wnd = 0;
	if (post_connection) {
		printf("httpd: rejecting concurrent POST connection\n");
		strlcpy(response_uri, "/fail.html", response_uri_len);
		return ERR_USE;
	}
	post_ok = 0;
	current_restore_prepare = false;
	current_restore_chunk = false;
	current_restore_id[0] = '\0';
	current_restore_size = 0;
	current_restore_offset = 0;

	if (!strncmp(uri, "/action/prepare/restore", 23) &&
	    (uri[23] == '\0' || uri[23] == '?')) {
		current_restore_prepare = true;
		if (recovery_restore_request_from_uri(
			    uri, "/action/prepare/restore", false,
			    current_restore_id, sizeof(current_restore_id), NULL,
			    &current_restore_size)) {
			prog_phase = -1;
			printf("httpd: invalid partition restore prepare request\n");
			strlcpy(response_uri, "/fail.html", response_uri_len);
			return ERR_ARG;
		}
	} else if (!strncmp(uri, "/upload/restore", 15) &&
		   (uri[15] == '\0' || uri[15] == '?')) {
		current_restore_chunk = true;
		if (recovery_restore_request_from_uri(
			    uri, "/upload/restore", true, current_restore_id,
			    sizeof(current_restore_id), &current_restore_offset,
			    &current_restore_size)) {
			prog_phase = -1;
			printf("httpd: invalid partition restore upload request\n");
			strlcpy(response_uri, "/fail.html", response_uri_len);
			return ERR_ARG;
		}
	}
	if (recovery_backup_active) {
		printf("httpd: rejecting destructive operation while %u backup stream(s) are active\n",
		       recovery_backup_active);
		strlcpy(response_uri, "/fail.html", response_uri_len);
		return ERR_USE;
	}
	if (prepare_request || restore_prepare_request || flash_request ||
	    (prog_phase > 0 && prog_phase < 3) || prog_reboot || reboot_request ||
	    (recovery_restore.active && !current_restore_chunk &&
	     !current_restore_prepare) ||
	    (recovery_stream.active &&
	     (current_restore_prepare || current_restore_chunk))) {
		printf("httpd: rejecting concurrent destructive operation\n");
		strlcpy(response_uri, "/fail.html", response_uri_len);
		return ERR_USE;
	}

	upload_done = 0;
	recv_off = 0;
	recv_total = 0;
	current_force_recreate = false;
	current_prepare_only = false;

	if (current_restore_prepare) {
		if (!recovery_backend_is_mmc() || content_len <= 0 ||
		    content_len > RECOVERY_RESTORE_BODY_MAX ||
		    !current_restore_size) {
			prog_phase = -1;
			printf("httpd: invalid partition restore confirmation\n");
			strlcpy(response_uri, "/fail.html", response_uri_len);
			return ERR_ARG;
		}
		recv_total = content_len;
		recv_base = current_restore_body;
		memset(current_restore_body, 0, sizeof(current_restore_body));
		post_connection = connection;
		post_ok = 1;
		return ERR_OK;
	}

	if (current_restore_chunk) {
		unsigned long long remaining;
		ulong blksz;

		if (!recovery_backend_is_mmc() || !recovery_restore.active ||
		    !recovery_restore.prepared || recovery_restore.receiving ||
		    strcmp(current_restore_id, recovery_restore.id) ||
		    current_restore_size != recovery_restore.total ||
		    current_restore_offset != recovery_restore.written ||
		    content_len <= 0 || content_len > RECOVERY_MMC_RESTORE_CHUNK) {
			printf("httpd: partition restore chunk does not match the prepared session\n");
			strlcpy(response_uri, "/fail.html", response_uri_len);
			return ERR_USE;
		}

		blksz = recovery_restore.source.blksz;
		remaining = recovery_restore.total - recovery_restore.written;
		if (!blksz || current_restore_offset % blksz ||
		    (ulong)content_len % blksz ||
		    (unsigned long long)content_len > remaining) {
			printf("httpd: unaligned or oversized partition restore chunk\n");
			strlcpy(response_uri, "/fail.html", response_uri_len);
			return ERR_ARG;
		}

		recv_total = content_len;
		recovery_restore.chunk_offset = current_restore_offset;
		recovery_restore.chunk_received = 0;
		recovery_restore.buf_used = 0;
		recovery_restore.prepared = false;
		recovery_restore.receiving = true;
		prog_phase = 2;
		post_connection = connection;
		post_ok = 1;
		return ERR_OK;
	}

	/* Accept optional query parameters after the target path. */
	if (!strncmp(uri, "/action/prepare/firmware", 24) &&
	    (uri[24] == '\0' || uri[24] == '?')) {
		current_target = TARGET_FIRMWARE;
		current_prepare_only = true;
	} else if (!strncmp(uri, "/action/prepare/uboot", 21) &&
		   (uri[21] == '\0' || uri[21] == '?')) {
		current_target = TARGET_UBOOT;
		current_prepare_only = true;
	} else if (!strncmp(uri, "/upload/firmware", 16) &&
		   (uri[16] == '\0' || uri[16] == '?')) {
		current_target = TARGET_FIRMWARE;
	} else if (!strncmp(uri, "/upload/uboot", 13) &&
		   (uri[13] == '\0' || uri[13] == '?')) {
		current_target = TARGET_UBOOT;
	} else if (!strncmp(uri, "/action/repartition", 19) &&
		   (uri[19] == '\0' || uri[19] == '?')) {
		current_target = TARGET_REPARTITION;
	} else if (!strncmp(uri, "/upload", 7) &&
		   (uri[7] == '\0' || uri[7] == '?')) {
		current_target = TARGET_FIRMWARE;
	} else {
		prog_phase = -1;
		strlcpy(response_uri, "/fail.html", response_uri_len);
		return ERR_ARG;
	}
	if (recovery_stream_format_from_uri(uri, &current_stream_format)) {
		prog_phase = -1;
		printf("httpd: invalid stream format\n");
		strlcpy(response_uri, "/fail.html", response_uri_len);
		return ERR_ARG;
	}
	if (current_target == TARGET_REPARTITION &&
	    recovery_layout_from_uri(uri, &current_repartition_layout)) {
		prog_phase = -1;
		printf("httpd: invalid SBE1V1K partition profile\n");
		strlcpy(response_uri, "/fail.html", response_uri_len);
		return ERR_ARG;
	}
	if (current_target == TARGET_REPARTITION &&
	    current_repartition_layout == RECOVERY_SBE1V1K_LAYOUT_QWRT &&
	    !qwrt_button_unlocked) {
		printf("httpd: QWRT profile is unavailable in this recovery session\n");
		strlcpy(response_uri, "/fail.html", response_uri_len);
		return ERR_USE;
	}
	if (current_target == TARGET_UBOOT &&
	    current_stream_format != RECOVERY_STREAM_RAW) {
		prog_phase = -1;
		printf("httpd: chainloader uploads require raw format\n");
		strlcpy(response_uri, "/fail.html", response_uri_len);
		return ERR_ARG;
	}

	if (current_target == TARGET_FIRMWARE &&
	    (strstr(uri, "?recreate=1") || strstr(uri, "&recreate=1") ||
	     strstr(uri, "?force_recreate=1") ||
	     strstr(uri, "&force_recreate=1")))
		current_force_recreate = true;

	stream_enabled = recovery_backend_is_mmc() &&
			 env_get_yesno("recovery_stream") == 1 &&
			 (current_target == TARGET_FIRMWARE ||
			  current_target == TARGET_UBOOT);

	if (current_prepare_only) {
		if (!stream_enabled || content_len <= 0 ||
		    content_len > RECOVERY_PREPARE_BODY_MAX) {
			prog_phase = -1;
			printf("httpd: invalid destructive prepare request\n");
			strlcpy(response_uri, "/fail.html", response_uri_len);
			return ERR_ARG;
		}
	} else if (current_target == TARGET_REPARTITION) {
		if (content_len <= 0 ||
		    (ulong)content_len > RECOVERY_REPARTITION_MAX) {
			prog_phase = -1;
			printf("httpd: invalid repartition request length %d\n",
			       content_len);
			strlcpy(response_uri, "/fail.html", response_uri_len);
			return ERR_ARG;
		}
	} else if (content_len <= 0 ||
		   recovery_validate_upload_length(current_target,
						   current_force_recreate,
						   content_len,
						   current_stream_format)) {
		prog_phase = -1;
		strlcpy(response_uri, "/fail.html", response_uri_len);
		return ERR_ARG;
	}

	recv_total = content_len;
	if (stream_enabled && !current_prepare_only) {
		if (!recovery_stream.active || !recovery_stream.prepared ||
		    recovery_stream.target != current_target ||
		    recovery_stream.format != current_stream_format ||
		    recovery_stream.total_expected != recv_total) {
			recovery_mmc_stream_reset();
			recovery_stream_completed = false;
			prog_phase = -1;
			printf("httpd: destructive upload rejected; prepare target and exact size first\n");
			strlcpy(response_uri, "/fail.html", response_uri_len);
			return ERR_USE;
		}

		recovery_stream.prepared = false;
		recovery_stream_completed = false;
		prog_phase = 2;
		prog_write_done = 0;
		prog_write_total = recv_total;
		prog_total = prog_erase_total + prog_write_total;
		prog_done = prog_erase_total;
		prog_reboot = 0;
		post_ok = 1;
		tname = current_target == TARGET_FIRMWARE ? "firmware" : "uboot";
		recovery_debug_printf("httpd: accepting prepared destructive stream of %u bytes for %s\n",
				      recv_total, tname);
		post_connection = connection;
		return ERR_OK;
	}

	/* Preparation bodies and non-streaming uploads use the bounded RAM path. */
	recovery_mmc_stream_reset();
	recovery_stream_completed = false;
	prog_phase = 0;
	prog_done = 0;
	prog_total = 0;
	prog_erase_done = 0;
	prog_erase_total = 0;
	prog_write_done = 0;
	prog_write_total = 0;
	prog_reboot = 0;

	/*
	 * Pick a stable upload buffer:
	 * recovery_addr -> loadaddr -> CONFIG_SYS_LOAD_ADDR -> RAM fallback.
	 */
	{
		ulong ram_start = (ulong)gd->ram_base;
		ulong ram_end = (ulong)gd->ram_base + (ulong)gd->ram_size;
		ulong base = env_get_hex("recovery_addr", 0);

		if (!base)
			base = env_get_hex("loadaddr", 0);
		if (!base)
			base = CONFIG_SYS_LOAD_ADDR;

		/* Keep the buffer inside usable RAM. */
		if (base < ram_start || (base + recv_total) > ram_end) {
			ulong fallback = ram_start + 0x01000000UL;

			if (fallback >= ram_start &&
			    fallback + recv_total <= ram_end) {
				base = fallback;
			} else if (ram_end - recv_total > ram_start) {
				base = ram_end - recv_total;
			} else {
				prog_phase = -1;
				printf("httpd: no sufficient RAM for upload (%u bytes)\n",
				       recv_total);
				strlcpy(response_uri, "/fail.html",
					response_uri_len);
				return ERR_MEM;
			}
		}
		recv_base = (u8 *)base;
	}

	post_ok = 1;
	tname = current_target == TARGET_FIRMWARE ? "firmware" :
		current_target == TARGET_UBOOT ? "uboot" : "repartition";
	if (current_prepare_only)
		recovery_debug_printf("httpd: accepting erase preparation for %s\n", tname);
	else
		recovery_debug_printf("httpd: accepting upload of %u bytes for %s to 0x%08lx%s\n",
				      recv_total, tname, (ulong)recv_base,
				      current_force_recreate ? " (force recreate)" : "");
	post_connection = connection;
	return ERR_OK;
}

err_t httpd_post_receive_data(void *connection, struct pbuf *p)
{
	struct pbuf *q;
	u16_t recved = 0;

	if (!post_ok) {
		pbuf_free(p);
		return ERR_ARG;
	}

	/* Stream directly to eMMC when enabled; otherwise retain the RAM path. */
	for (q = p; q != NULL; q = q->next) {
		size_t avail = recv_total - recv_off;
		size_t clen = q->len;

		if (clen > avail)
			clen = avail;
		if (current_restore_chunk) {
			if (recovery_restore_append(q->payload, clen)) {
				post_ok = 0;
				prog_phase = -1;
				recovery_restore_reset();
				pbuf_free(p);
				return ERR_IF;
			}
		} else if (recovery_stream.active) {
			if (recovery_mmc_stream_append(q->payload, clen)) {
				post_ok = 0;
				prog_phase = -1;
				recovery_mmc_stream_reset();
				pbuf_free(p);
				return ERR_IF;
			}
		} else {
			memcpy(recv_base + recv_off, q->payload, clen);
		}
		recv_off += clen;
	}
    recved = p->tot_len;
    pbuf_free(p);

    if (recv_off >= recv_total) {
        upload_done = 1;
    }

#if LWIP_HTTPD_POST_MANUAL_WND
    if (recved)
        httpd_post_data_recved(connection, recved);
#endif

    return ERR_OK;
}

void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len)
{
	unsigned long requested_size;
	int ret;

	if (connection != post_connection) {
		strlcpy(response_uri, "/fail.html", response_uri_len);
		return;
	}
	post_connection = NULL;
	if (current_restore_prepare) {
		char expected[RECOVERY_RESTORE_BODY_MAX + 1];
		int expected_len;

		expected_len = snprintf(expected, sizeof(expected), "%s:%s",
					RECOVERY_SBE1V1K_RESTORE_TOKEN,
					current_restore_id);
		if (!post_ok || !recv_total || recv_off != recv_total ||
		    expected_len < 0 || expected_len >= sizeof(expected) ||
		    recv_total != expected_len ||
		    memcmp(current_restore_body, expected, expected_len)) {
			post_ok = 0;
			prog_phase = -1;
			strlcpy(response_uri, "/fail.html", response_uri_len);
			return;
		}

		restore_prepare_request = -1;
		strlcpy(restore_prepare_id, current_restore_id,
			sizeof(restore_prepare_id));
		restore_prepare_size = current_restore_size;
		prog_phase = 0;
		prog_reboot = 0;
		strlcpy(response_uri, "/ok", response_uri_len);
		sys_timeout(RECOVERY_PREPARE_START_DELAY_MS,
			    restore_prepare_delay_cb, NULL);
		return;
	}

	if (current_restore_chunk) {
		if (!post_ok || !recv_total || recv_off != recv_total ||
		    recovery_restore_finish_chunk()) {
			post_ok = 0;
			prog_phase = -1;
			recovery_restore_reset();
			strlcpy(response_uri, "/fail.html", response_uri_len);
		} else {
			strlcpy(response_uri, "/ok", response_uri_len);
		}
		return;
	}

	if (current_prepare_only) {
		if (!post_ok || !recv_total || recv_off != recv_total) {
			post_ok = 0;
			prog_phase = -1;
		} else {
			recv_base[recv_off] = '\0';
			ret = strict_strtoul((const char *)recv_base, 10,
					     &requested_size);
			if (ret || recovery_validate_upload_length(current_target,
								   false,
								   requested_size,
								   current_stream_format)) {
				post_ok = 0;
				prog_phase = -1;
			} else {
				prepare_target = current_target;
				prepare_size = requested_size;
				prepare_stream_format = current_stream_format;
				/* -1 reserves the operation until the delayed callback fires. */
				prepare_request = -1;
				prog_phase = 0;
			}
		}

		recovery_debug_printf("httpd: prepare request finished, %u/%u bytes received\n",
				      recv_off, recv_total);
		if (post_ok) {
			strlcpy(response_uri, "/ok", response_uri_len);
			sys_timeout(RECOVERY_PREPARE_START_DELAY_MS,
				    prepare_delay_cb, NULL);
		} else {
			strlcpy(response_uri, "/fail.html", response_uri_len);
		}
		return;
	}

	if (post_ok && recovery_stream.active && recv_off == recv_total) {
		if (recovery_mmc_stream_finish()) {
			post_ok = 0;
			prog_phase = -1;
		} else {
			recovery_stream_completed = true;
			prog_phase = 3;
			prog_write_done = prog_write_total;
			prog_done = prog_total;
			prog_reboot = 1;
		}
	}
	recovery_debug_printf("httpd: post finished, %u/%u bytes received\n",
			      recv_off, recv_total);
    /* Tell httpd which page to return after POST (keep user on main page) */
    if (post_ok && recv_total && (recv_off >= recv_total))
        strlcpy(response_uri, "/ok", response_uri_len);
    else
        strlcpy(response_uri, "/fail.html", response_uri_len);

    /*
     * Delay flashing slightly so the browser can finish receiving the POST
     * response before erase/write work blocks the network loop.
     */
	if (post_ok && recv_total && recv_off >= recv_total) {
		if (recovery_stream_completed)
			sys_timeout(REBOOT_DELAY_MS, reboot_delay_cb, NULL);
		else
			sys_timeout(FLASH_START_DELAY_MS, post_delay_cb, NULL);
	} else if (recovery_stream.active) {
		post_ok = 0;
		prog_phase = -1;
		recovery_mmc_stream_reset();
	}
}

static int flash_image(struct recovery_status_led_ctrl *status_leds)
{
	struct recovery_target target;
	size_t retlen;
	int ret;

	if (!recv_off) {
		printf("No data received to flash\n");
		prog_phase = -1;
		return -EINVAL;
	}

	if (current_target == TARGET_REPARTITION) {
		prog_reboot = 0;
		if (current_repartition_layout == RECOVERY_SBE1V1K_LAYOUT_QWRT &&
		    (active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_MAINLINE ||
		     active_sbe1v1k_layout == RECOVERY_SBE1V1K_LAYOUT_QWRT))
			ret = recovery_select_sbe1v1k_qwrt_profile(status_leds);
		else
			ret = recovery_repartition_factory(status_leds,
							   current_repartition_layout);
		if (ret) {
			printf("SBE1V1K profile operation failed: %d\n", ret);
			prog_phase = -1;
			return ret;
		}

		prog_phase = 3;
		recovery_debug_printf("SBE1V1K profile operation complete.\n");
		return 0;
	}

	ret = recovery_resolve_target(current_target, &target);
	if (ret) {
		printf("No flash target found for upload type %d\n", current_target);
		prog_phase = -1;
		return ret;
	}

	if (current_force_recreate && current_target == TARGET_FIRMWARE &&
	    target.backend == RECOVERY_BACKEND_UBI)
		target.limit = recovery_calc_forced_ubi_limit(&target);

	if (recv_off > target.limit) {
		printf("Image size %u exceeds target size %llu\n",
		       recv_off, target.limit);
		recovery_release_target(&target);
		prog_phase = -1;
		return -EFBIG;
	}

	if (target.backend == RECOVERY_BACKEND_MMC) {
		ret = recovery_flash_mmc_target(current_target, status_leds);
		recovery_release_target(&target);
		if (ret) {
			printf("eMMC flash failed: %d\n", ret);
			prog_phase = -1;
			return ret;
		}

		prog_phase = 3;
		prog_reboot = 1;
		recovery_debug_printf("Flashing complete.\n");
		return 0;
	}

	if (target.backend == RECOVERY_BACKEND_UBI) {
		bool reformatted = false;

		if (current_force_recreate && current_target == TARGET_FIRMWARE)
			recovery_debug_printf("Force recreate requested: removing non-preserved UBI volumes before flashing.\n");

		ret = recovery_prepare_ubi_target(&target, status_leds, recv_off,
						      &reformatted);
		if (ret) {
			printf("Failed to prepare UBI target '%s' on '%s': %d\n",
			       target.name, target.ubi_part, ret);
			recovery_release_target(&target);
			prog_phase = -1;
			return ret;
		}

		if (!reformatted) {
			ret = recovery_cleanup_ubi_firmware(&target, status_leds,
							    recv_off);
			if (ret) {
				printf("Failed to clean UBI firmware volumes for '%s': %d\n",
				       target.name, ret);
				recovery_release_target(&target);
				prog_phase = -1;
				return ret;
			}
		}

		if (!target.cur_size) {
			recovery_debug_printf("Creating missing UBI volume '%s' for %u bytes...\n",
					      target.name, recv_off);
			ret = recovery_create_ubi_target(&target, recv_off);
			if (ret) {
				printf("ubi_create_volume failed for '%s': %d\n",
				       target.name, ret);
				prog_phase = -1;
				return ret;
			}
		} else if (recv_off > target.cur_size) {
			recovery_debug_printf("Resizing UBI volume '%s' from %llu to fit %u bytes...\n",
					      target.name, target.cur_size, recv_off);
			ret = recovery_resize_ubi_target(&target, recv_off);
			if (ret) {
				printf("ubi_resize_volume failed for '%s': %d\n",
				       target.name, ret);
				prog_phase = -1;
				return ret;
			}
		}

		ret = recovery_ensure_preserved_ubi_volumes(&target);
		if (ret) {
			recovery_release_target(&target);
			prog_phase = -1;
			return ret;
		}

		ret = recovery_ensure_rootfs_data(&target);
		if (ret) {
			recovery_release_target(&target);
			prog_phase = -1;
			return ret;
		}

			recovery_debug_printf("Writing %u bytes to UBI volume '%s' on '%s'...\n",
					      recv_off, target.name, target.ubi_part);
			prog_phase = 2;
			prog_done = prog_erase_total;
			prog_write_done = 0;
			recovery_service_runtime(status_leds);
			ret = recovery_write_ubi_target(&target, status_leds,
							recv_base, recv_off);
			if (ret) {
				printf("UBI write failed for '%s': %d\n",
				       target.name, ret);
				recovery_release_target(&target);
				prog_phase = -1;
			return ret;
		}

		prog_write_done = recv_off;
		prog_done = prog_erase_total + recv_off;
		recovery_release_target(&target);
		prog_phase = 3;
		prog_reboot = 1;
		if (current_target == TARGET_FIRMWARE)
			xr1710g_sync_factory();
		recovery_debug_printf("Flashing complete.\n");
		return 0;
	}

	{
		struct mtd_info *mtd = target.mtd;
		loff_t ofs = target.ofs;
		loff_t erase_len = ALIGN(target.limit, mtd->erasesize);

		prog_phase = 1;
		prog_done = 0;
		prog_erase_done = 0;
		prog_erase_total = erase_len;
		prog_write_done = 0;
		prog_write_total = recv_off;
		prog_total = prog_erase_total + prog_write_total;

		recovery_debug_printf("Erasing entire target '%s' (%llu bytes) and writing %u bytes...\n",
				      target.name, (unsigned long long)target.limit,
				      recv_off);
		ret = recovery_erase_mtd_region(mtd, ofs, target.limit,
						status_leds);
		if (ret) {
			printf("mtd_erase failed: %d\n", ret);
			recovery_release_target(&target);
			prog_phase = -1;
			return ret;
		}

		prog_phase = 2;
		for (u32 written = 0; written < recv_off; ) {
			u32 remain = recv_off - written;
			u32 chunk = remain > (64 * 1024) ? (64 * 1024) : remain;

			ret = mtd_write(mtd, ofs + written, chunk, &retlen,
					recv_base + written);
			if (ret) {
				printf("mtd_write failed: ret=%d at 0x%llx\n",
				       ret, (unsigned long long)(ofs + written));
				recovery_release_target(&target);
				prog_phase = -1;
				return ret;
			}
			if (!retlen) {
				printf("mtd_write made no progress at 0x%llx\n",
				       (unsigned long long)(ofs + written));
				recovery_release_target(&target);
				prog_phase = -1;
				return -EIO;
			}

			written += retlen;
			prog_write_done = written;
			prog_done = prog_erase_done + prog_write_done;
			recovery_service_runtime(status_leds);
		}
	}

	recovery_release_target(&target);
	prog_phase = 3;
	prog_reboot = 1;
	if (current_target == TARGET_FIRMWARE)
		xr1710g_sync_factory();
	recovery_debug_printf("Flashing complete.\n");
	return 0;
}

int run_http_recovery(void)
{
	struct udevice *udev = NULL;
	struct netif *netif = NULL;
	struct recovery_led_ctrl leds;
	struct recovery_status_led_ctrl status_leds;
	struct recovery_dhcp_server dhcp;
	bool use_status_leds = false;
	bool use_link_leds = false;
	bool eth_started = false;
	const char *old_allow_no_link;
	char *saved_allow_no_link = NULL;
	ulong timeout_ms;
	ulong start_ms;
	int rc;

	old_allow_no_link = env_get("eth_allow_no_link");
	if (old_allow_no_link) {
		saved_allow_no_link = strdup(old_allow_no_link);
		if (!saved_allow_no_link)
			return -ENOMEM;
	}
	env_set("eth_allow_no_link", "1");
	recovery_board_http_acl(true);

	recv_off = recv_total = 0;
	recovery_mmc_stream_reset();
	recovery_restore_reset();
	recovery_stream_completed = false;
	recovery_runtime_status_leds = &status_leds;
	post_ok = 0;
	post_connection = NULL;
	upload_done = 0;
	flash_request = 0;
	prepare_request = 0;
	restore_prepare_request = 0;
	restore_prepare_id[0] = '\0';
	restore_prepare_size = 0;
	prepare_size = 0;
	current_prepare_only = false;
	current_restore_prepare = false;
	current_restore_chunk = false;
	current_restore_id[0] = '\0';
	current_restore_size = 0;
	current_restore_offset = 0;
	reboot_request = 0;
	memset(&leds, 0, sizeof(leds));
	memset(&status_leds, 0, sizeof(status_leds));
	memset(&dhcp, 0, sizeof(dhcp));
	recovery_qwrt_button_init();
	if (recovery_board_is_sbe1v1k() && recovery_backend_is_mmc())
		recovery_detect_sbe1v1k_layout();

	recovery_debug_printf("HTTP recovery: preparing board runtime\n");
	recovery_watchdog_poll();
	rc = recovery_status_led_init(&status_leds);
	if (!rc) {
		use_status_leds = true;
	} else {
		printf("Recovery status LEDs unavailable (%d)\n", rc);
		if (!recovery_board_is_sbe1v1k()) {
			printf("Fallback to link LEDs\n");
			recovery_led_init(&leds);
			use_link_leds = true;
		}
	}
	recovery_prepare_static_network();

	recovery_debug_printf("HTTP recovery: starting Ethernet\n");
	recovery_watchdog_poll();
	rc = net_lwip_eth_start();
	if (rc < 0) {
		printf("Failed to start Ethernet: %d\n", rc);
		goto out;
	}
	eth_started = true;
	recovery_watchdog_poll();

	/*
	 * A previous boot stage can leave the Ethernet device enumerated but
	 * without a current-device pointer (or in the passive state).  The
	 * normal eth_init() path usually repairs this, but the stock bootm /
	 * chainloader handoff does not guarantee it.  Resolve the default device
	 * and start it once more before rejecting recovery networking.
	 */
	udev = eth_get_current();
	if (!udev)
		udev = eth_get_dev();
	if (udev && !eth_is_active(udev)) {
		rc = eth_start_udev(udev);
		if (rc < 0)
			recovery_debug_printf("HTTP recovery: unable to restart Ethernet (%d)\n",
					      rc);
	}
	if (!udev || !eth_is_active(udev)) {
		printf("No active net device\n");
		rc = -ENODEV;
		goto out;
	}

	recovery_debug_printf("HTTP recovery: creating lwIP netif\n");
	netif = net_lwip_new_netif(udev);
	if (!netif) {
		rc = -ENODEV;
		goto out;
	}
	recovery_watchdog_poll();

	recovery_debug_printf("HTTP recovery: starting DHCP helper\n");
	rc = recovery_dhcp_server_init(&dhcp, netif);
	if (rc)
		printf("Failed to start recovery DHCP server: %d\n", rc);
	else
		net_lwip_set_recovery_dhcp_hook(recovery_dhcp_recv, &dhcp);

	recovery_debug_printf("HTTP recovery: starting HTTP server\n");
	httpd_init();
	recovery_debug_printf("HTTP recovery server listening on http://%s/\n",
			      ip4addr_ntoa(netif_ip4_addr(netif)));

	timeout_ms = env_get_ulong("recovery_timeout", 10, 0) * 1000;
	start_ms = get_timer(0);

	while (1) {
		if (tstc()) {
			int c = getchar();

			if (c == 0x03) { /* Ctrl-C */
				recovery_debug_printf("Abort by user\n");
				break;
			}
		}
		/* net_lwip_rx() already runs sys_check_timeouts(). */
		net_lwip_rx(udev, netif);
		recovery_qwrt_button_poll();
		if (use_status_leds)
			recovery_status_led_poll(&status_leds);
		else if (use_link_leds)
			recovery_led_poll(&leds);
		if (prepare_request > 0) {
			prepare_request = 0;
			recovery_debug_printf("Preparing destructive eMMC stream for %lu bytes...\n",
					      (ulong)prepare_size);
			rc = recovery_mmc_stream_prepare(prepare_target,
							 prepare_size,
							 prepare_stream_format);
			if (rc) {
				prog_phase = -1;
				printf("Destructive stream preparation failed: %d\n",
				       rc);
			} else {
				recovery_debug_printf("Target erase complete; waiting for image stream.\n");
			}
		}
		if (restore_prepare_request > 0) {
			char restore_id[RECOVERY_RESTORE_ID_MAX];
			unsigned long long restore_size = restore_prepare_size;

			strlcpy(restore_id, restore_prepare_id, sizeof(restore_id));
			restore_prepare_request = 0;
			restore_prepare_id[0] = '\0';
			restore_prepare_size = 0;
			rc = recovery_restore_prepare_partition(restore_id, restore_size,
							 &status_leds);
			if (rc) {
				prog_phase = -1;
				printf("Partition restore preparation failed: %d\n",
				       rc);
			}
		}
		if (flash_request) {
			flash_request = 0;
			recovery_debug_printf("Upload done, flashing...\n");
			rc = flash_image(&status_leds);
			if (!rc) {
				if (prog_reboot) {
					recovery_debug_printf("Flashing complete. Rebooting in %dms...\n",
							      REBOOT_DELAY_MS);
					reboot_request = 0;
					sys_timeout(REBOOT_DELAY_MS, reboot_delay_cb, NULL);
				} else {
					recovery_debug_printf("Action complete. Keeping recovery server running.\n");
				}
			} else {
				printf("Flashing failed: %d. Keeping server running.\n",
				       rc);
			}
		}
		if (reboot_request)
			do_reset(NULL, 0, 0, NULL);
		if (timeout_ms && !post_ok && !upload_done && !flash_request &&
		    !reboot_request && prog_phase == 0 &&
			get_timer(start_ms) >= timeout_ms) {
			recovery_debug_printf("HTTP recovery timeout after %lums\n", timeout_ms);
			break;
		}
		recovery_watchdog_poll();
	}

	rc = 0;

out:
	qwrt_button_unlocked = false;
	qwrt_button_available = false;
	qwrt_button = NULL;
	recovery_qwrt_button_reset();
	recovery_runtime_status_leds = NULL;
	recovery_mmc_stream_reset();
	recovery_restore_reset();
	recovery_dhcp_server_stop(&dhcp);
	net_lwip_set_recovery_dhcp_hook(NULL, NULL);
	if (netif)
		net_lwip_remove_netif(netif);
	if (eth_started)
		eth_halt();
	recovery_status_led_stop(&status_leds);
	recovery_status_led_release(&status_leds);
	recovery_led_ctrl_free(&leds);
	recovery_board_http_acl(false);
	env_set("eth_allow_no_link", saved_allow_no_link);
	free(saved_allow_no_link);

	return rc;
}
