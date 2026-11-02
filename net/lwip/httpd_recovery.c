// SPDX-License-Identifier: GPL-2.0+
/*
 * HTTP recovery server using lwIP httpd, with read-only SPI-NAND backups.
 * Serves recovery pages and handles POST to /upload/{firmware|uboot}.
 */

#include <dm.h>
#include <dm/ofnode.h>
#include <env.h>
#include <log.h>
#include <malloc.h>
#include <memalign.h>
#include <command.h>
#include <mtd.h>
#include <net-lwip.h>
#include <net.h>
#include <initcall.h>
#include <ubi_uboot.h>
#include <watchdog.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <console.h>
#include <asm/gpio.h>
#include <asm/global_data.h>
#include <dt-bindings/gpio/gpio.h>

#include <lwip/netif.h>
#include <lwip/pbuf.h>
#include <lwip/ip.h>
#include <lwip/timeouts.h>
#include <lwip/udp.h>
#include <lwip/tcp.h>
#include <lwip/etharp.h>
#include <lwip/priv/tcp_priv.h>
#include <lwip/apps/httpd.h>
#include <lwip/apps/fs.h>
#include <lwip/prot/dhcp.h>
#include <lwip/prot/iana.h>
#include <version.h>
#include <timestamp.h>
#include <limits.h>
#include <miiphy.h>
#include <linux/mii.h>
#include <mtd/ubi-user.h>
#include <timer.h>
#include <asm/io.h>
#include "../../drivers/mtd/ubi/ubi.h"
#include "httpd_recovery.h"

DECLARE_GLOBAL_DATA_PTR;

int xr1710g_sync_factory(void);
int xr1710g_sync_factory_part(const char *part);
const char *xr1710g_detect_ubi_version(void);
ulong airoha_recovery_get_lan_activity_ms(void);

/*
 * Upload buffer
 * Use env 'recovery_addr' if set, otherwise fall back to U-Boot 'loadaddr'.
 * Avoid hard-coding a RAM address which may overlap U-Boot/lwIP memory.
 */
/* Default maximum upload size in bytes (override with env 'recovery_max') */
#define RECOVERY_UPLOAD_MAX    (32 * 1024 * 1024UL)
#define RECOVERY_MIN_FIRMWARE_SIZE (1 * 1024 * 1024UL)
#define RECOVERY_MAX_UBOOT_SIZE    (1 * 1024 * 1024UL)

/* Delay before reboot after flashing completes, to let browser finish reads */
#define REBOOT_DELAY_MS        3000
#define RECOVERY_STATIC_IPADDR           "192.168.255.1"
#define RECOVERY_STATIC_NETMASK          "255.255.255.0"
#define RECOVERY_STATIC_GATEWAY          "0.0.0.0"
#define RECOVERY_DHCP_CLIENT_IPADDR      "192.168.255.2"
#define RECOVERY_DHCP_BROADCAST_IPADDR   "192.168.255.255"
#define RECOVERY_DHCP_LEASE_SECS         86400U
#define RECOVERY_DHCP_MAX_MSG_LEN        1500
#define RECOVERY_LED_PORTS     2
#define RECOVERY_LED_POLL_MS   100
#define RECOVERY_LED_PHY_POLL_MS 500
#define RECOVERY_LED_MDIO_BACKOFF_MS 5000
#define RECOVERY_LED_ACTIVITY_MS 350
#define RECOVERY_LED_BLINK_MS  100
#define RECOVERY_STATUS_LED_MAX           8
#define RECOVERY_STATUS_SW_PWM_PERIOD_MS  20
#define RECOVERY_STATUS_HW_PWM_UPDATE_MS  40
#define RECOVERY_STATUS_HW_PWM_PERIOD_TICKS 1
#define RECOVERY_STATUS_BREATHE_PERIOD_MS 1800
#define RECOVERY_STATUS_BREATHE_HALF_MS   (RECOVERY_STATUS_BREATHE_PERIOD_MS / 2)
#define RECOVERY_STATUS_SWEEP_STEP_MS     700
#define RECOVERY_STATUS_OVERLAP_FP        (2 * 256)
#define RECOVERY_STATUS_BRIGHTNESS_FP     256
#define RECOVERY_STATUS_BREATHE_MIN_FP    64

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
#define RECOVERY_REG_GPIO_FLASH_PRD_SET0 0x003c
#define RECOVERY_REG_GPIO_FLASH_MAP0     0x004c
#define RECOVERY_REG_GPIO_FLASH_MAP1     0x0050
#define RECOVERY_REG_CYCLE_CFG_VALUE0    0x0098

#define RECOVERY_GPIO_FLASH_MAP_BUCKET0  0x88888888
#define RECOVERY_GPIO_FLASH_DUTY_FULL    255

#define RECOVERY_GPIO_LAN0_LED0_MODE_MASK BIT(3)
#define RECOVERY_GPIO_LAN0_LED1_MODE_MASK BIT(4)
#define RECOVERY_GPIO_LAN1_LED0_MODE_MASK BIT(5)
#define RECOVERY_GPIO_LAN1_LED1_MODE_MASK BIT(6)
#define RECOVERY_GPIO43_FLASH_MODE_CFG BIT(23)
#define RECOVERY_GPIO44_FLASH_MODE_CFG BIT(24)
#define RECOVERY_UBOOTENV_SIZE (1 * 1024 * 1024UL)
#define RECOVERY_FACTORY_SIZE  (1 * 1024 * 1024UL)
#define RECOVERY_UBI_WRITE_CHUNK (1024 * 1024U)
#define RECOVERY_UBOOT_SLOT_FIT_OFFSET 0x2100U
#define RECOVERY_UBOOT_SLOT_DEFAULT_OFS 0x600000UL
#define RECOVERY_UBOOT_SLOT_DEFAULT_DEV "spi-nand0"
#define RECOVERY_IH_MAGIC 0x27051956U
#define RECOVERY_FDT_MAGIC 0xd00dfeedU

static u8 *recv_base;
static u32 recv_off;
static u32 recv_total;
static int post_ok;
static int flash_request;
static volatile int reboot_request;
/* Progress for /status polling */
static volatile u32 prog_total; /* combined total for backward compat */
static volatile u32 prog_done;  /* combined done for backward compat */
static volatile u32 prog_erase_total;
static volatile u32 prog_erase_done;
static volatile u32 prog_write_total;
static volatile u32 prog_write_done;
static volatile int prog_phase; /* 0 idle, 1 erase, 2 write, 3 done, -1 error */
static unsigned long long prog_erase_volume_base;
static unsigned long long prog_erase_volume_bytes;
static struct recovery_status_led_ctrl *prog_status_leds;
static bool recovery_httpd_started;
static int recovery_ubi_attach_error;

static void recovery_abort_tcp_list(struct tcp_pcb **list)
{
	while (*list)
		tcp_abort(*list);
}

static void recovery_close_tcp_listeners(void)
{
	while (tcp_listen_pcbs.pcbs)
		tcp_close(tcp_listen_pcbs.pcbs);
}

static void recovery_lwip_cleanup(struct netif *netif)
{
	recovery_close_tcp_listeners();
	recovery_abort_tcp_list(&tcp_active_pcbs);
	recovery_abort_tcp_list(&tcp_tw_pcbs);
	recovery_abort_tcp_list(&tcp_bound_pcbs);
	if (netif)
		etharp_cleanup_netif(netif);
	recovery_httpd_started = false;
}

static void reboot_delay_cb(void *arg)
{
    (void)arg;
    reboot_request = 1;
}

static void recovery_cancel_timeouts(void)
{
	sys_untimeout(reboot_delay_cb, NULL);
	flash_request = 0;
	reboot_request = 0;
}

static void recovery_prepare_static_network(void)
{
	env_set("ipaddr", RECOVERY_STATIC_IPADDR);
	env_set("netmask", RECOVERY_STATIC_NETMASK);
	env_set("gatewayip", RECOVERY_STATIC_GATEWAY);
}

static u32 recovery_be32_to_cpu(const void *p)
{
	const u8 *b = p;

	return ((u32)b[0] << 24) | ((u32)b[1] << 16) |
	       ((u32)b[2] << 8) | b[3];
}

enum upload_target {
	TARGET_FIRMWARE = 0,
	TARGET_UBOOT,
};
static enum upload_target current_target = TARGET_FIRMWARE;

struct recovery_ubi_layout {
	const char *version;
	const char *part;
};

static const struct recovery_ubi_layout recovery_ubi_layouts[] = {
	{ "2.0", "ubi" },
	{ "1.5", "ubi1.5" },
	{ "1.0", "ubi1.0" },
};

static const struct recovery_ubi_layout *current_ubi_layout =
	&recovery_ubi_layouts[0];

enum recovery_backend {
	RECOVERY_BACKEND_MTD = 0,
	RECOVERY_BACKEND_UBI,
};

struct recovery_target {
	enum recovery_backend backend;
	const char *name;
	const char *ubi_part;
	struct mtd_info *mtd;
	loff_t ofs;
	unsigned long long cur_size;
	unsigned long long limit;
	bool ubi_needs_format;
};

struct recovery_led_ctrl {
	struct udevice *mdio_dev;
	ulong last_poll;
	ulong last_phy_poll;
	ulong last_mdio_error;
	bool mdio_fault;
	int speed[RECOVERY_LED_PORTS];
};

struct recovery_gpio_pin {
	struct gpio_desc desc;
	ofnode node;
	u8 gpio;
	s8 last_on;
	bool active_low;
	bool valid;
};

struct recovery_status_led_ctrl {
	struct recovery_gpio_pin leds[RECOVERY_STATUS_LED_MAX];
	int led_count;
	ulong start_ms;
	ulong last_pwm_update;
	u32 pwm_mux_mask;
	u32 pwm_mux_mask_ext;
	u32 saved_pwm_duty;
	u32 saved_pwm_map[2];
	u32 saved_pwm_cycle;
	bool pwm_active_low;
	bool hw_pwm;
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

static void recovery_led_stop(struct recovery_led_ctrl *ctrl)
{
	int i;

	if (!ctrl)
		return;

	for (i = 0; i < RECOVERY_LED_PORTS; i++) {
		recovery_led_set_pin(recovery_green_led_gpios[i], 0);
		recovery_led_set_pin(recovery_yellow_led_gpios[i], 0);
	}
}

static void recovery_status_led_set(struct recovery_gpio_pin *pin, int on)
{
	if (!pin->valid)
		return;

	on = !!on;
	if (pin->last_on == on)
		return;

	recovery_gpio_set_value(pin->gpio, pin->active_low, on);
	pin->last_on = on;
}

static void recovery_gpio_pin_release(struct recovery_gpio_pin *pin)
{
	if (!pin->valid)
		return;

	memset(pin, 0, sizeof(*pin));
}

static ofnode recovery_led_alias_node(const char *alias)
{
	ofnode aliases;
	const char *path;

	aliases = ofnode_path("/aliases");
	if (!ofnode_valid(aliases))
		return ofnode_null();

	path = ofnode_read_string(aliases, alias);
	if (!path)
		return ofnode_null();

	return ofnode_path(path);
}

static int recovery_led_node_to_gpio(ofnode node, struct recovery_gpio_pin *pin)
{
	struct ofnode_phandle_args args;
	int ret;

	memset(pin, 0, sizeof(*pin));

	if (!ofnode_valid(node))
		return -ENOENT;

	ret = ofnode_parse_phandle_with_args(node, "gpios", "#gpio-cells", 0, 0,
					     &args);
	if (ret)
		return ret;

	if (args.args_count < 1)
		return -EINVAL;

	pin->gpio = args.args[0];
	pin->active_low = args.args_count > 1 &&
			  (args.args[1] & GPIO_ACTIVE_LOW);
	pin->last_on = -1;
	pin->node = node;
	pin->valid = true;

	return 0;
}

static void recovery_status_led_add(struct recovery_status_led_ctrl *ctrl,
				    const struct recovery_gpio_pin *pin)
{
	int i;

	if (!pin->valid)
		return;

	for (i = 0; i < ctrl->led_count; i++) {
		if (ctrl->leds[i].gpio == pin->gpio)
			return;
	}

	if (ctrl->led_count >= ARRAY_SIZE(ctrl->leds))
		return;

	ctrl->leds[ctrl->led_count++] = *pin;
}

static void recovery_status_led_collect_alias(struct recovery_status_led_ctrl *ctrl,
					      const char *alias)
{
	struct recovery_gpio_pin pin;

	if (!recovery_led_node_to_gpio(recovery_led_alias_node(alias), &pin))
		recovery_status_led_add(ctrl, &pin);
}

static int recovery_status_led_request(struct recovery_gpio_pin *pin)
{
	recovery_gpio_prepare_output(pin->gpio);
	recovery_gpio_set_value(pin->gpio, pin->active_low, 0);
	pin->last_on = 0;

	return 0;
}

static void recovery_status_led_hw_pwm_set(struct recovery_status_led_ctrl *ctrl,
					   u32 brightness_fp)
{
	u32 high_ticks, value;

	high_ticks = (brightness_fp * RECOVERY_GPIO_FLASH_DUTY_FULL +
		      RECOVERY_STATUS_BRIGHTNESS_FP / 2) /
		     RECOVERY_STATUS_BRIGHTNESS_FP;
	if (ctrl->pwm_active_low)
		high_ticks = RECOVERY_GPIO_FLASH_DUTY_FULL - high_ticks;

	value = ((RECOVERY_GPIO_FLASH_DUTY_FULL - high_ticks) << 8) |
		high_ticks;
	recovery_clrsetbits_le32(RECOVERY_GPIO_SYSCTL_BASE +
				 RECOVERY_REG_GPIO_FLASH_PRD_SET0,
				 GENMASK(15, 0), value);
}

static int recovery_status_led_hw_pwm_init(struct recovery_status_led_ctrl *ctrl)
{
	u32 flash_mode, flash_mode_ext;
	int i;

	ctrl->pwm_active_low = ctrl->leds[0].active_low;
	for (i = 0; i < ctrl->led_count; i++) {
		uintptr_t reg;
		u32 mask;

		if (ctrl->leds[i].active_low != ctrl->pwm_active_low ||
		    !recovery_gpio_flash_mode_bit(ctrl->leds[i].gpio, &reg,
						  &mask))
			return -EOPNOTSUPP;

		if (reg == RECOVERY_GPIO_SYSCTL_BASE +
			   RECOVERY_REG_GPIO_FLASH_MODE_CFG)
			ctrl->pwm_mux_mask |= mask;
		else
			ctrl->pwm_mux_mask_ext |= mask;
	}

	flash_mode = readl((void __iomem *)(RECOVERY_GPIO_SYSCTL_BASE +
					      RECOVERY_REG_GPIO_FLASH_MODE_CFG));
	flash_mode_ext = readl((void __iomem *)(RECOVERY_GPIO_SYSCTL_BASE +
						  RECOVERY_REG_GPIO_FLASH_MODE_CFG_EXT));
	if ((flash_mode & ~ctrl->pwm_mux_mask) ||
	    (flash_mode_ext & ~ctrl->pwm_mux_mask_ext))
		return -EBUSY;

	ctrl->saved_pwm_duty = readl((void __iomem *)(RECOVERY_GPIO_SYSCTL_BASE +
						 RECOVERY_REG_GPIO_FLASH_PRD_SET0));
	ctrl->saved_pwm_map[0] = readl((void __iomem *)(RECOVERY_GPIO_SYSCTL_BASE +
						  RECOVERY_REG_GPIO_FLASH_MAP0));
	ctrl->saved_pwm_map[1] = readl((void __iomem *)(RECOVERY_GPIO_SYSCTL_BASE +
						  RECOVERY_REG_GPIO_FLASH_MAP1));
	ctrl->saved_pwm_cycle = readl((void __iomem *)(RECOVERY_GPIO_SYSCTL_BASE +
						  RECOVERY_REG_CYCLE_CFG_VALUE0));

	recovery_clrsetbits_le32(RECOVERY_GPIO_SYSCTL_BASE +
				 RECOVERY_REG_CYCLE_CFG_VALUE0,
				 GENMASK(7, 0),
				 RECOVERY_STATUS_HW_PWM_PERIOD_TICKS);
	recovery_status_led_hw_pwm_set(ctrl, 0);
	/* Extended PWM muxes use fixed GPIO0..15 channels; share one bucket. */
	writel(RECOVERY_GPIO_FLASH_MAP_BUCKET0,
	       (void __iomem *)(RECOVERY_GPIO_SYSCTL_BASE +
				RECOVERY_REG_GPIO_FLASH_MAP0));
	writel(RECOVERY_GPIO_FLASH_MAP_BUCKET0,
	       (void __iomem *)(RECOVERY_GPIO_SYSCTL_BASE +
				RECOVERY_REG_GPIO_FLASH_MAP1));
	recovery_clrsetbits_le32(RECOVERY_GPIO_SYSCTL_BASE +
				 RECOVERY_REG_GPIO_FLASH_MODE_CFG,
				 0, ctrl->pwm_mux_mask);
	recovery_clrsetbits_le32(RECOVERY_GPIO_SYSCTL_BASE +
				 RECOVERY_REG_GPIO_FLASH_MODE_CFG_EXT,
				 0, ctrl->pwm_mux_mask_ext);

	ctrl->hw_pwm = true;

	return 0;
}

static void recovery_status_led_hw_pwm_stop(struct recovery_status_led_ctrl *ctrl)
{
	int i;

	recovery_status_led_hw_pwm_set(ctrl, 0);
	recovery_clrsetbits_le32(RECOVERY_GPIO_SYSCTL_BASE +
				 RECOVERY_REG_GPIO_FLASH_MODE_CFG,
				 ctrl->pwm_mux_mask, 0);
	recovery_clrsetbits_le32(RECOVERY_GPIO_SYSCTL_BASE +
				 RECOVERY_REG_GPIO_FLASH_MODE_CFG_EXT,
				 ctrl->pwm_mux_mask_ext, 0);

	writel(ctrl->saved_pwm_map[0],
	       (void __iomem *)(RECOVERY_GPIO_SYSCTL_BASE +
				RECOVERY_REG_GPIO_FLASH_MAP0));
	writel(ctrl->saved_pwm_map[1],
	       (void __iomem *)(RECOVERY_GPIO_SYSCTL_BASE +
				RECOVERY_REG_GPIO_FLASH_MAP1));
	writel(ctrl->saved_pwm_duty,
	       (void __iomem *)(RECOVERY_GPIO_SYSCTL_BASE +
				RECOVERY_REG_GPIO_FLASH_PRD_SET0));
	writel(ctrl->saved_pwm_cycle,
	       (void __iomem *)(RECOVERY_GPIO_SYSCTL_BASE +
				RECOVERY_REG_CYCLE_CFG_VALUE0));

	ctrl->hw_pwm = false;
	for (i = 0; i < ctrl->led_count; i++) {
		ctrl->leds[i].last_on = -1;
		recovery_status_led_set(&ctrl->leds[i], 0);
	}
}

static void recovery_status_led_release(struct recovery_status_led_ctrl *ctrl)
{
	int i;

	for (i = 0; i < ctrl->led_count; i++)
		recovery_gpio_pin_release(&ctrl->leds[i]);

	memset(ctrl, 0, sizeof(*ctrl));
}

static int recovery_status_led_init(struct recovery_status_led_ctrl *ctrl)
{
	static const char *const status_aliases[] = {
		"led-boot",
		"led-running",
		"led-failsafe",
		"led-upgrade",
	};
	ofnode leds, node;
	int i, keep = 0;

	memset(ctrl, 0, sizeof(*ctrl));

	leds = ofnode_path("/leds");
	if (ofnode_valid(leds)) {
		ofnode_for_each_subnode(node, leds) {
			const char *function;
			struct recovery_gpio_pin pin;

			function = ofnode_read_string(node, "function");
			if (!function || strcmp(function, "status"))
				continue;

			if (!recovery_led_node_to_gpio(node, &pin))
				recovery_status_led_add(ctrl, &pin);
		}
	}

	for (i = 0; i < ARRAY_SIZE(status_aliases); i++)
		recovery_status_led_collect_alias(ctrl, status_aliases[i]);

	if (!ctrl->led_count)
		return -ENOENT;

	for (i = 0; i < ctrl->led_count; i++) {
		int ret;

		ret = recovery_status_led_request(&ctrl->leds[i]);
		if (ret) {
			printf("Recovery status LED gpio%u request failed: %d\n",
			       ctrl->leds[i].gpio, ret);
			recovery_gpio_pin_release(&ctrl->leds[i]);
			continue;
		}

		if (keep != i) {
			ctrl->leds[keep] = ctrl->leds[i];
			memset(&ctrl->leds[i], 0, sizeof(ctrl->leds[i]));
		}
		keep++;
	}

	ctrl->led_count = keep;

	if (!ctrl->led_count) {
		recovery_status_led_release(ctrl);
		return -ENODEV;
	}

	ctrl->start_ms = get_timer(0);
	if (ctrl->led_count == 1) {
		int pwm_ret = recovery_status_led_hw_pwm_init(ctrl);

		if (pwm_ret)
			printf("Recovery status LED hardware PWM unavailable (%d), using GPIO PWM\n",
			       pwm_ret);
	}

	printf("Recovery status LEDs:");
	for (i = 0; i < ctrl->led_count; i++)
		printf(" gpio%u%s", ctrl->leds[i].gpio,
		       ctrl->leds[i].active_low ? "(L)" : "");
	printf(" (%s PWM)\n", ctrl->hw_pwm ? "hardware" : "software");

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

static ulong recovery_status_led_position_fp(struct recovery_status_led_ctrl *ctrl,
					     ulong elapsed)
{
	ulong range_fp, phase;
	int span;

	if (ctrl->led_count <= 1)
		return 0;

	span = ctrl->led_count - 1;
	range_fp = span * RECOVERY_STATUS_BRIGHTNESS_FP;
	phase = elapsed % (2 * span * RECOVERY_STATUS_SWEEP_STEP_MS);

	if (phase <= span * RECOVERY_STATUS_SWEEP_STEP_MS)
		return phase * RECOVERY_STATUS_BRIGHTNESS_FP /
		       RECOVERY_STATUS_SWEEP_STEP_MS;

	phase -= span * RECOVERY_STATUS_SWEEP_STEP_MS;

	return range_fp - (phase * RECOVERY_STATUS_BRIGHTNESS_FP /
			   RECOVERY_STATUS_SWEEP_STEP_MS);
}

static u32 recovery_status_led_brightness_fp(struct recovery_status_led_ctrl *ctrl,
					     int idx, ulong elapsed)
{
	ulong center_fp, pwm_phase;
	u32 breath_fp, weight_fp, dist_fp;
	ulong led_pos_fp;

	center_fp = recovery_status_led_position_fp(ctrl, elapsed);
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

	return pwm_phase;
}

static void recovery_status_led_poll(struct recovery_status_led_ctrl *ctrl)
{
	ulong elapsed, now, pwm_phase;
	u32 brightness_fp;
	int duty_ms;
	int i;

	if (!ctrl->led_count)
		return;

	now = get_timer(0);
	elapsed = now - ctrl->start_ms;
	if (ctrl->hw_pwm) {
		if (ctrl->last_pwm_update &&
		    now - ctrl->last_pwm_update < RECOVERY_STATUS_HW_PWM_UPDATE_MS)
			return;

		ctrl->last_pwm_update = now;
		recovery_status_led_hw_pwm_set(ctrl,
					       recovery_status_led_breath_fp(elapsed));
		return;
	}

	pwm_phase = elapsed % RECOVERY_STATUS_SW_PWM_PERIOD_MS;

	for (i = 0; i < ctrl->led_count; i++) {
		brightness_fp =
			recovery_status_led_brightness_fp(ctrl, i, elapsed);
		duty_ms = (brightness_fp * RECOVERY_STATUS_SW_PWM_PERIOD_MS +
			   RECOVERY_STATUS_BRIGHTNESS_FP / 2) /
			  RECOVERY_STATUS_BRIGHTNESS_FP;
		recovery_status_led_set(&ctrl->leds[i],
					duty_ms && pwm_phase < duty_ms);
	}
}

static void recovery_status_led_service(void *arg)
{
	recovery_status_led_poll(arg);
}

static void recovery_status_led_stop(struct recovery_status_led_ctrl *ctrl)
{
	int i;

	if (!ctrl->led_count)
		return;

	if (ctrl->hw_pwm) {
		recovery_status_led_hw_pwm_stop(ctrl);
		return;
	}

	for (i = 0; i < ctrl->led_count; i++)
		recovery_status_led_set(&ctrl->leds[i], 0);
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

	(void)pcb;
	(void)addr;

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

	printf("DHCP recovery server: %s/67 -> offer %s mask %s gw %s bcast %s\n",
	       ip4addr_ntoa_r(&srv->server_ip, server_ip, sizeof(server_ip)),
	       ip4addr_ntoa_r(&srv->client_ip, client_ip, sizeof(client_ip)),
	       ip4addr_ntoa_r(&srv->netmask, netmask, sizeof(netmask)),
	       ip4addr_ntoa_r(&srv->router, router, sizeof(router)),
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
		return bmsr;

	bmsr = dm_mdio_read(ctrl->mdio_dev, recovery_led_phy_addrs[idx],
			    MDIO_DEVAD_NONE, MII_BMSR);
	if (bmsr < 0)
		return bmsr;

	if (!(bmsr & BMSR_LSTATUS))
		return 0;

	bmcr = dm_mdio_read(ctrl->mdio_dev, recovery_led_phy_addrs[idx],
			    MDIO_DEVAD_NONE, MII_BMCR);
	if (bmcr < 0)
		return bmcr;

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
	if (stat1000 < 0 || ctrl1000 < 0)
		return stat1000 < 0 ? stat1000 : ctrl1000;

	if (stat1000 >= 0 && ctrl1000 >= 0) {
		stat1000 &= ctrl1000 << 2;
		if (stat1000 & (PHY_1000BTSR_1000FD | PHY_1000BTSR_1000HD))
			return SPEED_1000;
	}

	lpa = dm_mdio_read(ctrl->mdio_dev, recovery_led_phy_addrs[idx],
			   MDIO_DEVAD_NONE, MII_ADVERTISE);
	if (lpa < 0)
		return lpa;

	bmsr = dm_mdio_read(ctrl->mdio_dev, recovery_led_phy_addrs[idx],
			    MDIO_DEVAD_NONE, MII_LPA);
	if (bmsr < 0)
		return bmsr;

	lpa &= bmsr;
	if (lpa & (LPA_100FULL | LPA_100HALF))
		return SPEED_100;

	return SPEED_10;
}

static void recovery_led_poll(struct recovery_led_ctrl *ctrl)
{
	ulong activity_ms;
	ulong now;
	bool activity;
	bool blink_on;
	bool poll_phys;
	int i;

	now = get_timer(0);
	if (ctrl->last_poll && now - ctrl->last_poll < RECOVERY_LED_POLL_MS)
		return;

	ctrl->last_poll = now;
	poll_phys = !ctrl->last_phy_poll ||
		    now - ctrl->last_phy_poll >= RECOVERY_LED_PHY_POLL_MS;
	if (ctrl->mdio_fault &&
	    get_timer(ctrl->last_mdio_error) < RECOVERY_LED_MDIO_BACKOFF_MS)
		poll_phys = false;

	if (poll_phys) {
		ctrl->last_phy_poll = now;
		for (i = 0; i < RECOVERY_LED_PORTS; i++) {
			int speed = recovery_led_phy_speed(ctrl, i);

			if (speed < 0) {
				ctrl->last_mdio_error = now;
				ctrl->mdio_fault = true;
				break;
			}

			ctrl->speed[i] = speed;
		}

		if (i == RECOVERY_LED_PORTS)
			ctrl->mdio_fault = false;
	}

	activity_ms = airoha_recovery_get_lan_activity_ms();
	activity = activity_ms && now - activity_ms <= RECOVERY_LED_ACTIVITY_MS;
	blink_on = !activity ||
		   ((now / RECOVERY_LED_BLINK_MS) & 1);

	for (i = 0; i < RECOVERY_LED_PORTS; i++) {
		recovery_led_set_pin(recovery_green_led_gpios[i],
				     ctrl->speed[i] == SPEED_1000 && blink_on);
		recovery_led_set_pin(recovery_yellow_led_gpios[i],
				     (ctrl->speed[i] == SPEED_10 ||
				      ctrl->speed[i] == SPEED_100) &&
					     blink_on);
	}
}

static const char *recovery_default_target(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return "fit";
	case TARGET_UBOOT:
		return "uboot";
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
	}

	return "recovery_size";
}

static ulong recovery_raw_offset(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_FIRMWARE:
		return env_get_hex("recovery_ofs", 0x050000);
	case TARGET_UBOOT:
		return env_get_hex("uboot_ofs",
				   RECOVERY_UBOOT_SLOT_DEFAULT_OFS);
	}

	return 0;
}

static const char *recovery_default_raw(enum upload_target tgt)
{
	switch (tgt) {
	case TARGET_UBOOT:
		return RECOVERY_UBOOT_SLOT_DEFAULT_DEV;
	case TARGET_FIRMWARE:
	default:
		return NULL;
	}
}

static unsigned long recovery_target_size_cap(enum upload_target tgt)
{
	ulong size_cap = env_get_hex(recovery_size_env(tgt), 0);

	if (tgt == TARGET_UBOOT &&
	    (!size_cap || size_cap > RECOVERY_MAX_UBOOT_SIZE))
		size_cap = RECOVERY_MAX_UBOOT_SIZE;

	return size_cap;
}

static const char *recovery_ubi_part(enum upload_target tgt)
{
	const char *part;

	if (tgt == TARGET_FIRMWARE && current_ubi_layout)
		return current_ubi_layout->part;

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

static int recovery_parse_ubi_layout(const char *uri)
{
	const char *query;
	int i;

	current_ubi_layout = &recovery_ubi_layouts[0];
	query = strchr(uri, '?');
	if (!query)
		return 0;

	if (strncmp(query, "?layout=", 8))
		return -EINVAL;
	query += 8;

	for (i = 0; i < ARRAY_SIZE(recovery_ubi_layouts); i++) {
		if (!strcmp(query, recovery_ubi_layouts[i].version)) {
			current_ubi_layout = &recovery_ubi_layouts[i];
			return 0;
		}
	}

	return -EINVAL;
}

static int recovery_select_ubi(const char *part)
{
	struct ubi_device *ubi;
	bool selected = false;
	int ret;

	if (!IS_ENABLED(CONFIG_CMD_UBI) || !IS_ENABLED(CONFIG_MTD_UBI))
		return -ENODEV;

	ubi = ubi_get_device(0);
	if (ubi) {
		selected = ubi->mtd && !strcmp(ubi->mtd->name, part);
		ubi_put_device(ubi);
	}
	if (selected)
		return 0;

	if (recovery_ubi_attach_error)
		return recovery_ubi_attach_error;

	ret = ubi_part((char *)part, NULL);
	if (ret)
		recovery_ubi_attach_error = ret;

	return ret;
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

	if (recovery_select_ubi(part)) {
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

	if (!name)
		name = recovery_default_target(tgt);

	mtd_probe_devices();

	raw = env_get(recovery_raw_env(tgt));
	if (!raw || !*raw)
		raw = recovery_default_raw(tgt);
	if (raw && *raw) {
		mtd = get_mtd_device_nm(raw);
		if (!IS_ERR_OR_NULL(mtd)) {
			ulong size_cap = recovery_target_size_cap(tgt);

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

	/*
	 * The XR1710G U-Boot upload target is the raw chainloader slot. Do
	 * not silently fall back to an MTD/UBI volume named "uboot"; old saved
	 * environments may lack recovery_dev_uboot, and writing there makes the
	 * browser report success while the first-stage loader still reads stale
	 * or invalid bytes from 0x600000/0x602100.
	 */
	if (tgt == TARGET_UBOOT)
		return -ENODEV;

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
		ulong size_cap = recovery_target_size_cap(tgt);

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

static int recovery_force_ubi_rebuild(struct recovery_target *target)
{
	struct mtd_info *mtd;

	if (target->backend != RECOVERY_BACKEND_UBI || !target->ubi_part)
		return -EINVAL;

	ubi_detach();
	recovery_ubi_attach_error = 0;
	if (target->mtd) {
		put_mtd_device(target->mtd);
		target->mtd = NULL;
	}

	mtd_probe_devices();
	mtd = get_mtd_device_nm(target->ubi_part);
	if (IS_ERR_OR_NULL(mtd))
		return IS_ERR(mtd) ? PTR_ERR(mtd) : -ENODEV;

	target->mtd = mtd;
	target->cur_size = 0;
	target->limit = mtd->size;
	target->ubi_needs_format = true;

	return 0;
}

static void recovery_service_runtime(struct recovery_status_led_ctrl *status_leds)
{
	struct udevice *udev = eth_get_dev();
	struct netif *netif = net_lwip_get_netif();

	if (udev && netif)
		net_lwip_rx(udev, netif);
	recovery_status_led_poll(status_leds);
	WATCHDOG_RESET();
}

static int recovery_validate_uboot_slot_image(const void *image, size_t size)
{
	const u8 *p = image;
	u32 prefix_magic;
	u32 fit_magic;

	if (size <= RECOVERY_UBOOT_SLOT_FIT_OFFSET + sizeof(u32)) {
		printf("U-Boot slot image is too small: %lu bytes\n",
		       (unsigned long)size);
		return -EINVAL;
	}

	prefix_magic = recovery_be32_to_cpu(p);
	fit_magic = recovery_be32_to_cpu(p + RECOVERY_UBOOT_SLOT_FIT_OFFSET);

	if (prefix_magic != RECOVERY_IH_MAGIC ||
	    fit_magic != RECOVERY_FDT_MAGIC) {
		printf("Invalid U-Boot slot image: magic[0]=0x%08x magic[0x%04x]=0x%08x\n",
		       prefix_magic, RECOVERY_UBOOT_SLOT_FIT_OFFSET, fit_magic);
		printf("Expected xr1710g-chainloader-slot.bin, not u-boot.bin or bare .itb\n");
		return -EINVAL;
	}

	return 0;
}

static bool recovery_mtd_is_block_aligned(struct mtd_info *mtd, u64 value)
{
	return mtd->erasesize && !(value % mtd->erasesize);
}

static int recovery_mtd_validate_region(struct mtd_info *mtd, loff_t ofs,
					size_t len)
{
	if (!mtd || !mtd->erasesize)
		return -EINVAL;

	if (!len)
		return 0;

	if (ofs < 0 || ofs >= mtd->size ||
	    (u64)len > (u64)(mtd->size - ofs)) {
		printf("MTD region 0x%llx..+0x%lx is outside '%s' size 0x%llx\n",
		       (unsigned long long)ofs, (unsigned long)len,
		       mtd->name, (unsigned long long)mtd->size);
		return -EINVAL;
	}

	if (!recovery_mtd_is_block_aligned(mtd, ofs) ||
	    !recovery_mtd_is_block_aligned(mtd, len)) {
		printf("MTD region 0x%llx..+0x%lx is not eraseblock aligned (0x%x)\n",
		       (unsigned long long)ofs, (unsigned long)len,
		       mtd->erasesize);
		return -EINVAL;
	}

	return 0;
}

static void recovery_update_erase_progress(loff_t done)
{
	prog_erase_done = done > prog_erase_total ? prog_erase_total : done;
	prog_done = prog_erase_done + prog_write_done;
}

static int recovery_mtd_mark_bad(struct mtd_info *mtd, loff_t addr)
{
	int ret;

	ret = mtd_block_markbad(mtd, addr);
	if (ret)
		printf("Warning: failed to mark bad block at 0x%llx on '%s': %d\n",
		       (unsigned long long)addr, mtd->name, ret);
	else
		printf("Marked bad block at 0x%llx on '%s'\n",
		       (unsigned long long)addr, mtd->name);

	return ret;
}

static int recovery_mtd_verify_write(struct mtd_info *mtd, loff_t addr,
				     const u8 *src, size_t len, u8 *buf)
{
	size_t retlen = 0;
	int ret;

	ret = mtd_read(mtd, addr, len, &retlen, buf);
	if ((ret && ret != -EUCLEAN) || retlen != len) {
		printf("mtd_read verify failed: ret=%d retlen=%lu at 0x%llx\n",
		       ret, (unsigned long)retlen, (unsigned long long)addr);
		return ret ? ret : -EIO;
	}

	if (memcmp(buf, src, len)) {
		printf("mtd_read verify mismatch at 0x%llx\n",
		       (unsigned long long)addr);
		return -EIO;
	}

	return 0;
}

static void recovery_ubi_progress(struct ubi_volume *vol, int done, int total)
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
	int ret;

	if (!mtd || !len)
		return 0;

	ret = recovery_mtd_validate_region(mtd, ofs, len);
	if (ret)
		return ret;

	ei.addr = ofs;
	ei.len = len;

	ret = mtd_unlock(mtd, ei.addr, ei.len);
	if (ret && ret != -EOPNOTSUPP) {
		printf("Warning: initial mtd_unlock 0x%llx..+0x%llx failed: %d\n",
		       (unsigned long long)ei.addr,
		       (unsigned long long)ei.len, ret);
	}

	for (loff_t addr = 0; addr < len; addr += mtd->erasesize) {
		struct erase_info e = {
			.addr = ofs + addr,
			.len = mtd->erasesize,
		};
		int tries = 0;

		ret = mtd_block_isbad(mtd, e.addr);
		if (ret < 0) {
			printf("Failed to query bad block at 0x%llx: %d\n",
			       (unsigned long long)e.addr, ret);
			return ret;
		}
		if (ret > 0) {
			printf("Skipping bad block at 0x%llx\n",
			       (unsigned long long)e.addr);
			recovery_update_erase_progress(addr + mtd->erasesize);
			recovery_service_runtime(status_leds);
			continue;
		}

		do {
			ret = mtd_erase(mtd, &e);
			if (!ret)
				break;
			if (ret == -EROFS || ret == -EACCES)
				mtd_unlock(mtd, e.addr, e.len);
			else
				break;
		} while (++tries < 2);

		if (ret) {
			printf("mtd_erase failed at 0x%llx: %d\n",
			       (unsigned long long)e.addr, ret);
			if (recovery_mtd_mark_bad(mtd, e.addr))
				return ret;
		}

		recovery_update_erase_progress(addr + mtd->erasesize);
		recovery_service_runtime(status_leds);
	}

	return 0;
}

static int recovery_write_mtd_region(struct mtd_info *mtd, loff_t ofs,
				     size_t region_len, const void *image,
				     size_t image_size, bool verify,
				     struct recovery_status_led_ctrl *status_leds)
{
	const u8 *src = image;
	u8 *verify_buf = NULL;
	u32 written = 0;
	int ret;

	if (!image_size)
		return 0;

	ret = recovery_mtd_validate_region(mtd, ofs, region_len);
	if (ret)
		return ret;

	if (verify) {
		verify_buf = malloc_cache_aligned(mtd->erasesize);
		if (!verify_buf)
			return -ENOMEM;
	}

	for (loff_t addr = ofs; addr < ofs + region_len && written < image_size;
	     addr += mtd->erasesize) {
		size_t chunk = min((size_t)mtd->erasesize,
				   image_size - (size_t)written);
		size_t retlen = 0;

		ret = mtd_block_isbad(mtd, addr);
		if (ret < 0) {
			printf("Failed to query bad block at 0x%llx: %d\n",
			       (unsigned long long)addr, ret);
			goto out;
		}
		if (ret > 0) {
			printf("Skipping bad block at 0x%llx\n",
			       (unsigned long long)addr);
			recovery_service_runtime(status_leds);
			continue;
		}

		ret = mtd_write(mtd, addr, chunk, &retlen, src + written);
		if (ret || retlen != chunk) {
			printf("mtd_write failed: ret=%d retlen=%lu at 0x%llx\n",
			       ret, (unsigned long)retlen,
			       (unsigned long long)addr);
			if (recovery_mtd_mark_bad(mtd, addr)) {
				if (!ret)
					ret = -EIO;
				goto out;
			}
			recovery_service_runtime(status_leds);
			continue;
		}

		if (verify_buf) {
			ret = recovery_mtd_verify_write(mtd, addr,
							src + written, chunk,
							verify_buf);
			if (ret) {
				if (recovery_mtd_mark_bad(mtd, addr))
					goto out;
				recovery_service_runtime(status_leds);
				continue;
			}
		}

		written += chunk;
		prog_write_done = written;
		prog_done = prog_erase_done + prog_write_done;
		recovery_service_runtime(status_leds);
	}

	if (written < image_size) {
		printf("Not enough good eraseblocks in MTD target: wrote %u/%lu bytes\n",
		       written, (unsigned long)image_size);
		ret = -ENOSPC;
		goto out;
	}

	ret = 0;

out:
	free(verify_buf);
	return ret;
}

static bool recovery_preserve_ubi_volume(const char *name)
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
recovery_calc_rebuild_ubi_limit(struct recovery_target *target)
{
#if IS_ENABLED(CONFIG_CMD_UBI) && IS_ENABLED(CONFIG_MTD_UBI)
	struct ubi_device *ubi;
	unsigned long long limit;
	int i;

	if (target->backend != RECOVERY_BACKEND_UBI)
		return target->limit;

	if (target->ubi_needs_format)
		return target->limit;

	if (recovery_select_ubi(target->ubi_part))
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

static int recovery_create_ubi_volume(const char *name, size_t size, int vol_type)
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

static int recovery_remove_ubi_volume(const char *name)
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

	if (recovery_select_ubi(target->ubi_part))
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
		return recovery_select_ubi(target->ubi_part);

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

	printf("UBI partition '%s' is invalid, erasing it to recreate layout...\n",
	       target->ubi_part);
	ret = recovery_erase_mtd_region(target->mtd, 0, target->mtd->size,
					status_leds);
	if (ret)
		return ret;

	recovery_ubi_attach_error = 0;
	ret = recovery_select_ubi(target->ubi_part);
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

	if (recovery_select_ubi(target->ubi_part))
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

	if (recovery_select_ubi(target->ubi_part))
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

		printf("Removing UBI volume '%s' before flashing '%s'...\n",
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

static void recovery_abort_ubi_update(struct ubi_volume *vol)
{
	if (!vol || !vol->updating)
		return;

	if (vol->upd_buf) {
		vfree(vol->upd_buf);
		vol->upd_buf = NULL;
	}
	vol->updating = 0;
	vol->upd_bytes = 0;
	vol->upd_received = 0;
	vol->upd_ebs = 0;
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

	if (recovery_select_ubi(target->ubi_part))
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
	if (ret)
		recovery_abort_ubi_update(vol);
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

	if (recovery_select_ubi(target->ubi_part))
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

	if (recovery_select_ubi(target->ubi_part))
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
					      loff_t *p_ofs)
{
	struct recovery_target target;
	unsigned long limit = 0;
	unsigned long long effective_limit;

	if (recovery_resolve_target(tgt, &target))
		return 0;

	if (p_ofs)
		*p_ofs = target.ofs;

	effective_limit = target.limit;
	if (tgt == TARGET_FIRMWARE)
		effective_limit = recovery_calc_rebuild_ubi_limit(&target);

	limit = (effective_limit > ULONG_MAX) ? ULONG_MAX :
		(unsigned long)effective_limit;
	recovery_release_target(&target);

	return limit;
}

/* Only dynamic endpoints here; static files come from fsdata */

static const char recovery_page_ok[] =
	"HTTP/1.0 200 OK\r\n"
	"Content-Type: text/plain\r\n"
	"Cache-Control: no-store\r\n"
	"Content-Length: 2\r\n"
	"Connection: close\r\n"
	"\r\n"
	"OK";

int recovery_open_custom_response(struct fs_file *file, const char *status,
					 const char *content_type,
					 const char *body, int body_len)
{
	char header[160];
	char *page;
	int header_len;
	int total_len;

	if (body_len < 0)
		return 0;

	header_len = snprintf(header, sizeof(header),
			      "HTTP/1.0 %s\r\n"
			      "Content-Type: %s\r\n"
			      "Cache-Control: no-store\r\n"
			      "Content-Length: %d\r\n"
			      "Connection: close\r\n"
			      "\r\n",
			      status, content_type, body_len);
	if (header_len < 0 || header_len >= (int)sizeof(header))
		return 0;

	total_len = header_len + body_len;
	page = malloc(total_len + 1);
	if (!page)
		return 0;

	memcpy(page, header, header_len);
	memcpy(page + header_len, body, body_len);
	page[total_len] = '\0';

	file->data = page;
	file->len = total_len;
	file->index = file->len;
	file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
	return 1;
}

/* lwIP httpd custom file hooks: serve only dynamic endpoints; static files via fsdata */
int fs_open_custom(struct fs_file *file, const char *name)
{
	const char *p;

	if (!file || !name)
		return 0;
	file->pextension = NULL;
	/* Normalize the slash used by httpd's default filenames. */
	p = *name == '/' ? name + 1 : name;

	if (!strncmp(p, "backup/", 7))
		return recovery_backup_open(file, p, post_ok || flash_request ||
					    reboot_request || prog_phase > 0);
	if (!strcmp(p, "status")) {
		char json[256];
		int json_len = snprintf(json, sizeof(json),
			"{\"in_progress\":%d,\"done\":%u,\"total\":%u,"
			"\"erase_done\":%u,\"erase_total\":%u,"
			"\"write_done\":%u,\"write_total\":%u,"
			"\"ok\":%d,\"error\":%d,\"phase\":%d}\n",
			prog_phase > 0 && prog_phase < 3,
			(unsigned int)prog_done, (unsigned int)prog_total,
			(unsigned int)prog_erase_done, (unsigned int)prog_erase_total,
			(unsigned int)prog_write_done, (unsigned int)prog_write_total,
			prog_phase == 3, prog_phase == -1, prog_phase);

		if (json_len < 0 || json_len >= (int)sizeof(json))
			return 0;
		return recovery_open_custom_response(file, "200 OK",
					     "application/json", json, json_len);
	}
	if (!strcmp(p, "ok")) {
		file->data = recovery_page_ok;
		file->len = sizeof(recovery_page_ok) - 1;
		file->index = file->len;
		file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
		return 1;
	}
	if (!strcmp(p, "about")) {
		char json[256];
		int json_len;

#ifdef U_BOOT_DATE
		json_len = snprintf(json, sizeof(json),
			"{\"u_boot\":\"%s (%s - %s %s)\",\"detected_layout\":\"%s\"}\n",
			U_BOOT_VERSION, U_BOOT_DATE, U_BOOT_TIME, U_BOOT_TZ,
			xr1710g_detect_ubi_version());
#else
		json_len = snprintf(json, sizeof(json),
			"{\"u_boot\":\"%s\",\"detected_layout\":\"%s\"}\n",
			U_BOOT_VERSION, xr1710g_detect_ubi_version());
#endif
		if (json_len < 0 || json_len >= (int)sizeof(json))
			return 0;
		return recovery_open_custom_response(file, "200 OK",
					     "application/json", json, json_len);
	}
	/* Static pages, including backup.html, come from fsdata. */
	return 0;
}

void fs_close_custom(struct fs_file *file)
{
	if (!file)
		return;
	if (file->pextension) {
		recovery_backup_close(file);
		return;
	}
	if (file->data && file->data != recovery_page_ok) {
		free((void *)file->data);
		file->data = NULL;
	}
}

int fs_read_custom(struct fs_file *file, char *buffer, int count)
{
	int left;

	if (!file || !buffer || count <= 0)
		return FS_READ_EOF;
	if (file->pextension)
		return recovery_backup_read(file, buffer, count);
	left = file->len - file->index;
	if (left <= 0)
		return FS_READ_EOF;
	if (count > left)
		count = left;
	memcpy(buffer, file->data + file->index, count);
	file->index += count;
	return count;
}

/* Backup streams use fs_read_custom(); small responses use file->data. */

/* HTTP POST handlers */
err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                       u16_t http_request_len, int content_len, char *response_uri,
                       u16_t response_uri_len, u8_t *post_auto_wnd)
{
    (void)http_request; (void)http_request_len; (void)connection;
    /*
     * Throttle large uploads explicitly: XR1710G recovery accepts firmware
     * images that are much larger than a typical lwIP POST body and manual
     * window updates avoid over-optimistic receive windows.
     */
	if (post_auto_wnd)
		*post_auto_wnd = 0;

	if (recovery_backup_active() || post_ok || flash_request ||
	    reboot_request || prog_phase > 0) {
        printf("httpd: rejecting upload while recovery flash is busy (phase=%d)\n",
               prog_phase);
        strlcpy(response_uri, "/fail.html", response_uri_len);
        return ERR_ARG;
    }

    post_ok = 0;
    recovery_ubi_attach_error = 0;
    recv_off = 0;
    recv_total = 0;
    /* Reset progress state. */
    prog_phase = 0;
    prog_done = 0;
    prog_total = 0;
    prog_erase_done = 0;
    prog_erase_total = 0;
    prog_write_done = 0;
    prog_write_total = 0;
    /* Accept optional query parameters after the target path. */
    if (!strncmp(uri, "/upload/firmware", 16) &&
        (uri[16] == '\0' || uri[16] == '?')) {
        current_target = TARGET_FIRMWARE;
        if (recovery_parse_ubi_layout(uri)) {
            printf("httpd: rejecting unknown UBI layout in '%s'\n", uri);
            prog_phase = -1;
            strlcpy(response_uri, "/fail.html", response_uri_len);
            return ERR_ARG;
        }
    } else if (!strncmp(uri, "/upload/uboot", 13) &&
               (uri[13] == '\0' || uri[13] == '?')) {
        current_target = TARGET_UBOOT;
    } else if (!strncmp(uri, "/upload", 7) &&
               (uri[7] == '\0' || uri[7] == '?')) {
        current_target = TARGET_FIRMWARE;
        if (recovery_parse_ubi_layout(uri)) {
            prog_phase = -1;
            strlcpy(response_uri, "/fail.html", response_uri_len);
            return ERR_ARG;
        }
    } else {
        prog_phase = -1;
        strlcpy(response_uri, "/fail.html", response_uri_len);
        return ERR_ARG;
    }

    {
        ulong min = 0;
        ulong env_max = env_get_hex("recovery_max", 0);
        loff_t tmpofs = 0;
        ulong dts_max = recovery_calc_target_max(current_target, &tmpofs);
        ulong max = dts_max ? dts_max : RECOVERY_UPLOAD_MAX;

        if (current_target == TARGET_FIRMWARE)
            min = RECOVERY_MIN_FIRMWARE_SIZE;
        else if (current_target == TARGET_UBOOT &&
                 max > RECOVERY_MAX_UBOOT_SIZE)
            max = RECOVERY_MAX_UBOOT_SIZE;

        if (env_max && env_max < max)
            max = env_max; /* allow env to further cap */
        if (content_len <= 0 || (ulong)content_len > max ||
            (min && (ulong)content_len < min)) {
            prog_phase = -1;
            if (min && (ulong)content_len < min) {
                printf("httpd: content_len %d below allowed min %lu for target %d\n",
                       content_len, min, current_target);
            } else {
                printf("httpd: content_len %d exceeds allowed max %lu (target limit %lu, ofs 0x%llx)\n",
                       content_len, max, dts_max, (unsigned long long)tmpofs);
            }
            strlcpy(response_uri, "/fail.html", response_uri_len);
            return ERR_ARG;
        }
    }

    recv_total = content_len;

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
        if (base < ram_start || base >= ram_end ||
            recv_total > (ram_end - base)) {
            ulong fallback = ram_start + 0x01000000UL;
            if (fallback >= ram_start && fallback < ram_end &&
                recv_total <= (ram_end - fallback)) {
                base = fallback;
            } else {
                prog_phase = -1;
                printf("httpd: no sufficient RAM for upload (%u bytes)\n", recv_total);
                strlcpy(response_uri, "/fail.html", response_uri_len);
                return ERR_MEM;
            }
        }
        recv_base = (u8 *)base;
    }

    post_ok = 1;
    /* Leave response_uri untouched here so the POST can complete normally. */
    if (current_target == TARGET_FIRMWARE)
        printf("httpd: accepting %u-byte firmware for UBI %s\n",
               recv_total, current_ubi_layout->version);
    else
        printf("httpd: accepting %u-byte U-Boot image\n", recv_total);
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

	/* Copy request payload into RAM and defer flash work to the main loop. */
	for (q = p; q != NULL; q = q->next) {
        size_t avail = recv_total - recv_off;
        size_t clen = q->len;
	        if (clen > avail)
	            clen = avail;
	        memcpy(recv_base + recv_off, q->payload, clen);
	        recv_off += clen;
    }
    recved = p->tot_len;
    pbuf_free(p);

#if LWIP_HTTPD_POST_MANUAL_WND
    if (recved)
        httpd_post_data_recved(connection, recved);
#endif

    return ERR_OK;
}

void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len)
{
	bool upload_complete;

	(void)connection;
	printf("httpd: post finished, %u/%u bytes received\n", recv_off, recv_total);
	upload_complete = post_ok && recv_total && (recv_off >= recv_total);
	if (upload_complete) {
		strlcpy(response_uri, "/ok", response_uri_len);
		printf("httpd: upload received; waiting for POST response ACK\n");
	} else {
		strlcpy(response_uri, "/fail.html", response_uri_len);
		post_ok = 0;
		prog_phase = -1;
	}
}

#if LWIP_HTTPD_POST_RESPONSE_ACK
void httpd_post_response_complete(void *connection, err_t result)
{
	(void)connection;

	if (!post_ok || !recv_total || recv_off < recv_total)
		return;

	if (result != ERR_OK) {
		printf("httpd: POST response not acknowledged (%d); flash cancelled\n",
		       result);
		post_ok = 0;
		prog_phase = -1;
		return;
	}

	printf("httpd: POST response acknowledged; flashing may start\n");
	flash_request = 1;
}
#endif

static int flash_image(struct recovery_status_led_ctrl *status_leds)
{
	struct recovery_target target;
	const u8 *image = recv_base;
	u32 image_size = recv_off;
	int ret;

	post_ok = 0;

	if (!image_size) {
		printf("No data received to flash\n");
		prog_phase = -1;
		return -EINVAL;
	}

	if (current_target == TARGET_UBOOT) {
		ret = recovery_validate_uboot_slot_image(image, image_size);
		if (ret) {
			prog_phase = -1;
			return ret;
		}
	}

	ret = recovery_resolve_target(current_target, &target);
	if (ret) {
		printf("No flash target found for upload type %d\n", current_target);
		prog_phase = -1;
		return ret;
	}

	if (current_target == TARGET_FIRMWARE &&
	    target.backend == RECOVERY_BACKEND_UBI) {
		ret = recovery_force_ubi_rebuild(&target);
		if (ret) {
			printf("Failed to select UBI %s rebuild target: %d\n",
			       current_ubi_layout->version, ret);
			recovery_release_target(&target);
			prog_phase = -1;
			return ret;
		}
	}

	if (current_target == TARGET_FIRMWARE &&
	    target.backend == RECOVERY_BACKEND_UBI)
		target.limit = recovery_calc_rebuild_ubi_limit(&target);

	if (image_size > target.limit) {
		printf("Image size %u exceeds target size %llu\n",
		       image_size, target.limit);
		recovery_release_target(&target);
		prog_phase = -1;
		return -EFBIG;
	}

	if (target.backend == RECOVERY_BACKEND_UBI) {
		bool reformatted = false;

		if (current_target == TARGET_FIRMWARE)
			printf("Recovery mode: recreating UBI %s on '%s'.\n",
			       current_ubi_layout->version, target.ubi_part);

		ret = recovery_prepare_ubi_target(&target, status_leds, image_size,
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
							    image_size);
			if (ret) {
				printf("Failed to clean UBI firmware volumes for '%s': %d\n",
				       target.name, ret);
				recovery_release_target(&target);
				prog_phase = -1;
				return ret;
			}
		}

		if (!target.cur_size) {
			printf("Creating missing UBI volume '%s' for %u bytes...\n",
			       target.name, image_size);
			ret = recovery_create_ubi_target(&target, image_size);
			if (ret) {
				printf("ubi_create_volume failed for '%s': %d\n",
				       target.name, ret);
				recovery_release_target(&target);
				prog_phase = -1;
				return ret;
			}
		} else if (image_size > target.cur_size) {
			printf("Resizing UBI volume '%s' from %llu to fit %u bytes...\n",
			       target.name, target.cur_size, image_size);
			ret = recovery_resize_ubi_target(&target, image_size);
			if (ret) {
				printf("ubi_resize_volume failed for '%s': %d\n",
				       target.name, ret);
				recovery_release_target(&target);
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

		printf("Writing %u bytes to UBI volume '%s' on '%s'...\n",
		       image_size, target.name, target.ubi_part);
		prog_phase = 2;
		prog_done = prog_erase_total;
		prog_write_done = 0;
		recovery_service_runtime(status_leds);
		ret = recovery_write_ubi_target(&target, status_leds,
						image, image_size);
		if (ret) {
			printf("UBI write failed for '%s': %d\n",
			       target.name, ret);
			recovery_release_target(&target);
			prog_phase = -1;
			return ret;
		}

		prog_write_done = image_size;
		prog_done = prog_erase_total + image_size;
		recovery_release_target(&target);
		prog_phase = 3;
		if (current_target == TARGET_FIRMWARE)
			xr1710g_sync_factory_part(target.ubi_part);
		return 0;
	}

	{
		struct mtd_info *mtd = target.mtd;
		loff_t ofs = target.ofs;
		loff_t erase_len = target.limit;

		prog_phase = 1;
		prog_done = 0;
		prog_erase_done = 0;
		prog_erase_total = erase_len;
		prog_write_done = 0;
		prog_write_total = image_size;
		prog_total = prog_erase_total + prog_write_total;

		printf("Erasing entire target '%s' (%llu bytes) and writing %u bytes...\n",
		       target.name, (unsigned long long)target.limit, image_size);
		ret = recovery_erase_mtd_region(mtd, ofs, target.limit,
						status_leds);
		if (ret) {
			printf("mtd_erase failed: %d\n", ret);
			recovery_release_target(&target);
			prog_phase = -1;
			return ret;
		}

		prog_phase = 2;
		ret = recovery_write_mtd_region(mtd, ofs, target.limit, image,
						image_size,
						current_target == TARGET_UBOOT,
						status_leds);
		if (ret) {
			printf("mtd_write failed: %d\n", ret);
			recovery_release_target(&target);
			prog_phase = -1;
			return ret;
		}
	}

	recovery_release_target(&target);
	prog_phase = 3;
	if (current_target == TARGET_FIRMWARE)
		xr1710g_sync_factory();
	return 0;
}

int run_http_recovery(void)
{
	struct udevice *udev;
	struct netif *netif;
	struct recovery_led_ctrl leds;
	struct recovery_status_led_ctrl status_leds;
	struct recovery_dhcp_server dhcp;
	bool use_status_leds = false;
	int rc;

	recovery_cancel_timeouts();
	recv_off = recv_total = 0;
	post_ok = 0;
	flash_request = 0;
	reboot_request = 0;
	prog_phase = 0;
	prog_done = 0;
	prog_total = 0;
	prog_erase_done = 0;
	prog_erase_total = 0;
	prog_write_done = 0;
	prog_write_total = 0;
	recovery_ubi_attach_error = 0;
	memset(&leds, 0, sizeof(leds));
	rc = recovery_status_led_init(&status_leds);
	if (!rc) {
		use_status_leds = true;
	} else {
		printf("Recovery status LEDs unavailable (%d), fallback to link LEDs\n",
		       rc);
	}
	recovery_led_init(&leds);
	recovery_prepare_static_network();

	rc = net_lwip_eth_start();
	if (rc < 0) {
		printf("Failed to start Ethernet: %d\n", rc);
		recovery_lwip_cleanup(NULL);
		recovery_status_led_stop(&status_leds);
		recovery_status_led_release(&status_leds);
		recovery_led_stop(&leds);
		recovery_led_ctrl_free(&leds);
		return rc;
	}

	udev = eth_get_dev();
	if (!udev) {
		printf("No active net device\n");
		recovery_lwip_cleanup(NULL);
		recovery_status_led_stop(&status_leds);
		recovery_status_led_release(&status_leds);
		recovery_led_stop(&leds);
		recovery_led_ctrl_free(&leds);
		net_lwip_eth_stop();
		return -ENODEV;
	}

	netif = net_lwip_new_netif(udev);
	if (!netif) {
		recovery_lwip_cleanup(NULL);
		recovery_status_led_stop(&status_leds);
		recovery_status_led_release(&status_leds);
		recovery_led_stop(&leds);
		recovery_led_ctrl_free(&leds);
		net_lwip_eth_stop();
		return -ENODEV;
	}

	rc = recovery_dhcp_server_init(&dhcp, netif);
	if (rc)
		printf("Failed to start recovery DHCP server: %d\n", rc);
	else
		net_lwip_set_recovery_dhcp_hook(recovery_dhcp_recv, &dhcp);

	if (!recovery_httpd_started) {
		httpd_init();
		recovery_httpd_started = true;
	}
	printf("HTTP recovery server listening on http://%s/\n",
	       ip4addr_ntoa(netif_ip4_addr(netif)));
	if (use_status_leds)
		net_lwip_set_recovery_poll_hook(recovery_status_led_service, &status_leds);

	while (1) {
		if (tstc()) {
			int c = getchar();

			if (c == 0x03) { /* Ctrl-C */
				printf("Abort by user\n");
				break;
			}
		}
		/* net_lwip_rx() already runs sys_check_timeouts(). */
		net_lwip_rx(udev, netif);
		if (use_status_leds)
			recovery_status_led_poll(&status_leds);
		recovery_led_poll(&leds);
		if (flash_request) {
			flash_request = 0;
			printf("Upload done, flashing...\n");
			rc = flash_image(&status_leds);
			if (!rc) {
				printf("Flashing complete. Rebooting in %dms...\n",
				       REBOOT_DELAY_MS);
				reboot_request = 0;
				sys_timeout(REBOOT_DELAY_MS, reboot_delay_cb, NULL);
			} else {
				printf("Flashing failed: %d. Keeping server running.\n",
				       rc);
			}
		}
		if (reboot_request)
			do_reset(NULL, 0, 0, NULL);
		WATCHDOG_RESET();
	}

	net_lwip_set_recovery_poll_hook(NULL, NULL);
	recovery_cancel_timeouts();
	recovery_dhcp_server_stop(&dhcp);
	net_lwip_set_recovery_dhcp_hook(NULL, NULL);
	recovery_lwip_cleanup(netif);
	net_lwip_remove_netif(netif);
	net_lwip_eth_stop();
	recovery_status_led_stop(&status_leds);
	recovery_status_led_release(&status_leds);
	recovery_led_stop(&leds);
	recovery_led_ctrl_free(&leds);
	return 0;
}
