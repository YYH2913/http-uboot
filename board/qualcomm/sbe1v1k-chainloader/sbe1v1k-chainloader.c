// SPDX-License-Identifier: GPL-2.0+
/*
 * Askey SBE1V1K chainloader board support.
 */

#include <asm/io.h>
#include <asm/unaligned.h>
#include <dm.h>
#include <env.h>
#include <fdt_support.h>
#include <linux/err.h>
#include <malloc.h>
#include <smem.h>

#define SBE1V1K_APSS_WDT_BASE		0x0b017000
#define SBE1V1K_APSS_WDT_RST		0x04
#define SBE1V1K_APSS_WDT_EN		0x08
#define SBE1V1K_APSS_WDT_BARK_TIME	0x10
#define SBE1V1K_APSS_WDT_BITE_TIME	0x14

/* Qualcomm's SMEM_HW_SW_BUILD_ID item, as used by the QSDK bootloader. */
#define SBE1V1K_SMEM_HW_SW_BUILD_ID	137U
#define SBE1V1K_SMEM_PLATFORM_ID_OFF	4U
#define SBE1V1K_SMEM_PLATFORM_VERSION_OFF	8U

#define SBE1V1K_CPU_IPQ9570		513U
#define SBE1V1K_SOC_VERSION_MAJOR	1U
#define SBE1V1K_SOC_VERSION_MINOR	1U

#define SBE1V1K_QSDK_COMPATIBLE	"spectrum,sbe1v1k"
#define SBE1V1K_QWRT_R260616_MODEL	"Spectrum SBE1V1K ( WIFI 7 )"
#define SBE1V1K_FSTOOLS_IGNORE_PARTNAME	"fstools_ignore_partname=1"
#define SBE1V1K_LARGE_ROOTPART	"/dev/mmcblk0p29"
#define SBE1V1K_BOOTARGS_MAX		2048U

static void sbe1v1k_watchdog_kick(void)
{
	void __iomem *wdt = (void __iomem *)SBE1V1K_APSS_WDT_BASE;

	writel(1, wdt + SBE1V1K_APSS_WDT_RST);
	readl(wdt + SBE1V1K_APSS_WDT_RST);
}

static void sbe1v1k_disable_watchdog(void)
{
	void __iomem *wdt = (void __iomem *)SBE1V1K_APSS_WDT_BASE;

	writel(0, wdt + SBE1V1K_APSS_WDT_EN);
	sbe1v1k_watchdog_kick();
	writel(0, wdt + SBE1V1K_APSS_WDT_BARK_TIME);
	writel(0, wdt + SBE1V1K_APSS_WDT_BITE_TIME);
	readl(wdt + SBE1V1K_APSS_WDT_EN);
}

void recovery_board_watchdog_kick(void)
{
	sbe1v1k_watchdog_kick();
}

void board_net_phy_watchdog_kick(void)
{
	sbe1v1k_watchdog_kick();
}

/*
 * QSDK's IPQ bootloader adds cpu_type and SoC version properties to the
 * Linux DTB immediately before booting. Legacy AP-AL02-C4 and Spectrum
 * SBE1V1K QSDK FITs do not carry them themselves, and their CPR drivers can
 * BUG when soc_version_major is absent. The QSDK kernel reads these
 * properties as little-endian words, rather than normal DT cells, so they
 * must be written with fdt_setprop() using little-endian storage instead of
 * fdt_setprop_u32().
 */
static void sbe1v1k_get_qsdk_socinfo(u32 *cpu_type, u32 *major, u32 *minor)
{
	const u8 *platform;
	struct udevice *smem;
	u32 version;
	size_t size;
	int ret;

	/* This board is IPQ9570 v1.1; SMEM overrides this when it is available. */
	*cpu_type = SBE1V1K_CPU_IPQ9570;
	*major = SBE1V1K_SOC_VERSION_MAJOR;
	*minor = SBE1V1K_SOC_VERSION_MINOR;

	ret = uclass_get_device(UCLASS_SMEM, 0, &smem);
	if (ret)
		return;

	platform = smem_get(smem, -1, SBE1V1K_SMEM_HW_SW_BUILD_ID, &size);
	if (IS_ERR_OR_NULL(platform) ||
	    size < SBE1V1K_SMEM_PLATFORM_VERSION_OFF + sizeof(version))
		return;

	version = get_unaligned_le32(platform +
				   SBE1V1K_SMEM_PLATFORM_VERSION_OFF);
	if (get_unaligned_le32(platform + SBE1V1K_SMEM_PLATFORM_ID_OFF))
		*cpu_type = get_unaligned_le32(platform +
					   SBE1V1K_SMEM_PLATFORM_ID_OFF);
	if (version >> 16) {
		*major = version >> 16;
		*minor = version & 0xffff;
	}
}

static int sbe1v1k_fdt_set_le32_if_missing(void *blob, int root,
					   const char *name, u32 value)
{
	const void *existing;
	u8 bytes[sizeof(value)];
	int len;

	existing = fdt_getprop(blob, root, name, &len);
	if (existing)
		return len == sizeof(value) ? 0 : -FDT_ERR_BADVALUE;
	if (len != -FDT_ERR_NOTFOUND)
		return len;

	put_unaligned_le32(value, bytes);
	return fdt_setprop(blob, root, name, bytes, sizeof(bytes));
}

static int sbe1v1k_fdt_get_string(const void *blob, int node,
				  const char *name, const char **value,
				  size_t *value_len)
{
	const char *prop;
	int len;

	prop = fdt_getprop(blob, node, name, &len);
	if (!prop) {
		if (len != -FDT_ERR_NOTFOUND)
			return len;
		*value = NULL;
		*value_len = 0;
		return 0;
	}
	if (!len || strnlen(prop, len) != len - 1)
		return -FDT_ERR_BADVALUE;

	*value = prop;
	*value_len = len - 1;
	return 0;
}

static bool sbe1v1k_uses_large_layout(void)
{
	const char *rootpart = env_get("rootpart");

	return rootpart && !strcmp(rootpart, SBE1V1K_LARGE_ROOTPART);
}

static int sbe1v1k_apply_qsdk_bootargs(void *blob, int root)
{
	const char *bootargs;
	const char *append;
	const char *model;
	char *combined;
	size_t bootargs_len, append_len, model_len;
	size_t combined_len;
	bool legacy_qwrt;
	bool use_rootdisk_overlay;
	int chosen, ret;

	/* QSDK's bootloader carries this non-standard property into bootargs. */
	if (fdt_node_check_compatible(blob, root, SBE1V1K_QSDK_COMPATIBLE))
		return 0;

	chosen = fdt_path_offset(blob, "/chosen");
	if (chosen < 0)
		return chosen;

	ret = sbe1v1k_fdt_get_string(blob, chosen, "bootargs", &bootargs,
				     &bootargs_len);
	if (ret)
		return ret;
	ret = sbe1v1k_fdt_get_string(blob, chosen, "bootargs-append", &append,
				     &append_len);
	if (ret)
		return ret;
	ret = sbe1v1k_fdt_get_string(blob, root, "model", &model, &model_len);
	if (ret)
		return ret;

	legacy_qwrt = model &&
		model_len == strlen(SBE1V1K_QWRT_R260616_MODEL) &&
		!memcmp(model, SBE1V1K_QWRT_R260616_MODEL, model_len);
	use_rootdisk_overlay = legacy_qwrt && !sbe1v1k_uses_large_layout();
	if (!append_len && !legacy_qwrt)
		return 0;

	combined_len = bootargs_len + append_len +
		(bootargs_len && append_len ? 1 : 0) +
		(use_rootdisk_overlay ?
		 sizeof(" " SBE1V1K_FSTOOLS_IGNORE_PARTNAME) - 1 : 0) + 1;
	if (combined_len > SBE1V1K_BOOTARGS_MAX)
		return -E2BIG;

	combined = malloc(combined_len);
	if (!combined)
		return -ENOMEM;

	combined[0] = '\0';
	if (bootargs_len)
		strlcpy(combined, bootargs, combined_len);
	if (append_len) {
		if (bootargs_len && append[0] != ' ')
			strlcat(combined, " ", combined_len);
		strlcat(combined, append, combined_len);
	}

	/* Mainline has only a small rootfs tail; large uses its p30 data volume. */
	if (use_rootdisk_overlay &&
	    !strstr(combined, SBE1V1K_FSTOOLS_IGNORE_PARTNAME)) {
		if (combined[0] && combined[strlen(combined) - 1] != ' ')
			strlcat(combined, " ", combined_len);
		strlcat(combined, SBE1V1K_FSTOOLS_IGNORE_PARTNAME, combined_len);
	}

	ret = fdt_setprop_string(blob, chosen, "bootargs", combined);
	free(combined);

	return ret;
}

int ft_board_setup(void *blob, struct bd_info __maybe_unused *bd)
{
	u32 cpu_type, major, minor;
	int root, ret;

	root = fdt_path_offset(blob, "/");
	if (root < 0)
		return root;

	sbe1v1k_get_qsdk_socinfo(&cpu_type, &major, &minor);

	ret = sbe1v1k_fdt_set_le32_if_missing(blob, root, "cpu_type", cpu_type);
	if (ret)
		return ret;
	ret = sbe1v1k_fdt_set_le32_if_missing(blob, root,
					      "soc_version_major", major);
	if (ret)
		return ret;
	ret = sbe1v1k_apply_qsdk_bootargs(blob, root);
	if (ret)
		return ret;

	return sbe1v1k_fdt_set_le32_if_missing(blob, root,
					       "soc_version_minor", minor);
}

int board_early_init_f(void)
{
	sbe1v1k_disable_watchdog();

	return 0;
}

void qcom_board_init(void)
{
	sbe1v1k_disable_watchdog();
}
