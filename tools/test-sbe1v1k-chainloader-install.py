#!/usr/bin/env python3
"""Exercise production profile installers with an in-memory eMMC fixture.

Uses real libfdt trees and SHA-256 image hashes. Block I/O, GPT operations and
environment commits are fault-injection doubles; no target is accessed.
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "net/lwip/httpd_recovery.c").read_text()


def function(name):
    match = re.search(r"^static[^;{}]*\b" + name + r"\([^;{}]*\)\n\{.*?^\}",
                      source, re.M | re.S)
    assert match, name
    return match.group(0)


defines = "\n".join(re.findall(r"^#define RECOVERY_SBE1V1K_.*$", source, re.M))
descriptors = source[source.index("enum recovery_sbe1v1k_layout {"):
                     source.index("static enum upload_target current_target")]
assert "ret = recovery_change_sbe1v1k_profile(status_leds," in function("flash_image")
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <libfdt.h>
#include <openssl/sha.h>
typedef unsigned char u8;
typedef unsigned int u32;
typedef unsigned long ulong;
typedef unsigned long long lbaint_t;
#define SHA256_SUM_LEN 32
#define RECOVERY_QWRT_AUTH_CODE_LEN 64
#define RECOVERY_MMC_WRITE_CHUNK (1024 * 1024UL)
#define RECOVERY_MAX_UBOOT_SIZE_MMC (4 * 1024 * 1024UL)
#define RECOVERY_MMC_ERASE_PROGRESS_STEPS 1000U
#define RECOVERY_APPSBLENV_SIZE (256 * 1024UL)
#define ARCH_DMA_MINALIGN 64
#define ALIGN(value, alignment) (((value) + (alignment) - 1) & ~((alignment) - 1))
#define LBAF "%llu"
#define FIT_IMAGES_PATH "/images"
#define FIT_DESC_PROP "description"
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define recovery_debug_printf(...) ((void)0)
''' + defines + "\n" + descriptors + r'''
struct disk_partition { lbaint_t start, size; ulong blksz; };
struct blk_desc { ulong blksz; };
struct recovery_status_led_ctrl { int unused; };
static struct blk_desc device = { .blksz = 512 };
static enum recovery_sbe1v1k_layout active_sbe1v1k_layout;
static u32 prog_phase, prog_erase_done, prog_erase_total;
static u32 prog_write_done, prog_write_total, prog_done, prog_total;
static const u8 *recv_base;
static size_t recv_off;
static const struct recovery_sbe1v1k_layout_desc *target;
static unsigned char *original, *preserved, *storage;
static size_t original_size, preserved_size, storage_size;
static unsigned int erases, writes, reads, env_commits, applies, gpts, clones;
static bool hlos_1, prefix_valid, readback_verified;
static enum recovery_sbe1v1k_layout geometry;
static char events[512];
static size_t events_len;
enum fault {
    NONE, PRESERVE, ENV_CHECK, ERASE, WRITE, READ, CORRUPT_FIRST,
    CORRUPT_MIDDLE, CORRUPT_LAST, READ_HASH, ALLOC, WRONG_START,
    WRONG_SIZE, WRONG_SECTOR, IDENTITY, POST_GPT, ENV_COMMIT
};
static enum fault fault;
static void event(char c) { assert(events_len + 1 < sizeof(events)); events[events_len++] = c; events[events_len] = 0; }
static ulong recovery_uboot_limit(void) { return 4 * 1024 * 1024UL; }
static void recovery_service_runtime(struct recovery_status_led_ctrl *leds) { (void)leds; }
static void *memalign(size_t alignment, size_t size)
{
    void *buffer = NULL;
    if (fault == ALLOC) return NULL;
    return posix_memalign(&buffer, alignment, size) ? NULL : buffer;
}
static int fit_check_format(const void *fit, size_t size)
{ return fdt_check_full(fit, size); }
static size_t fit_get_size(const void *fit) { return fdt_totalsize(fit); }
static bool recovery_data_range_ok(const void *base, size_t total, const void *data, size_t size)
{
    const u8 *b = base, *d = data;
    return d >= b && size <= total && (size_t)(d - b) <= total - size;
}
static int fit_image_get_data(const void *fit, int node, const void **data, size_t *size)
{
    int len;
    *data = fdt_getprop(fit, node, "data", &len);
    if (!*data || len < 0) return -EINVAL;
    *size = len;
    return 0;
}
static int fit_all_image_verify(const void *fit)
{
    int images = fdt_path_offset(fit, FIT_IMAGES_PATH), image;
    if (images < 0 || (reads && fault == READ_HASH)) return 0;
    fdt_for_each_subnode(image, fit, images) {
        const void *data;
        size_t size;
        int hash = fdt_subnode_offset(fit, image, "hash-1"), len;
        u8 actual[SHA256_DIGEST_LENGTH];
        const u8 *expected = fdt_getprop(fit, hash, "value", &len);
        if (fit_image_get_data(fit, image, &data, &size) ||
            !expected || len != sizeof(actual)) return 0;
        SHA256(data, size, actual);
        if (memcmp(actual, expected, sizeof(actual))) return 0;
    }
    if (reads) readback_verified = true;
    event('H');
    return 1;
}
static unsigned long long recovery_mmc_part_bytes(const struct disk_partition *part)
{ return part->size * part->blksz; }
static int recovery_get_mmc_part(const char *spec, struct blk_desc **desc, struct disk_partition *part)
{
    assert(!strcmp(spec, target->uboot_part));
    *desc = &device;
    *part = (struct disk_partition){ target->uboot_start, target->uboot_size, 512 };
    if (fault == WRONG_START) part->start++;
    if (fault == WRONG_SIZE) part->size--;
    if (fault == WRONG_SECTOR) part->blksz = 4096;
    return 0;
}
static lbaint_t blk_dwrite(struct blk_desc *desc, lbaint_t start, lbaint_t count, const void *data)
{
    size_t offset = (start - target->uboot_start) * 512, size = count * 512;
    assert(desc == &device && start >= target->uboot_start && offset + size <= storage_size);
    event('W'); writes++;
    if (fault == WRITE && writes == 2) return count - 1;
    memcpy(storage + 16 + offset, data, size);
    return count;
}
static lbaint_t blk_dread(struct blk_desc *desc, lbaint_t start, lbaint_t count, void *data)
{
    size_t size = count * 512;
    assert(desc == &device && start == target->uboot_start && size <= storage_size);
    assert(size == ALIGN(preserved_size, 512));
    event('R'); reads++;
    if (fault == READ) return count - 1;
    memcpy(data, storage + 16, size);
    if (fault == CORRUPT_FIRST) ((u8 *)data)[0] ^= 1;
    if (fault == CORRUPT_MIDDLE) ((u8 *)data)[preserved_size / 2] ^= 1;
    if (fault == CORRUPT_LAST) ((u8 *)data)[preserved_size - 1] ^= 1;
    return count;
}
static int recovery_mmc_erase_part(const char *spec, struct recovery_status_led_ctrl *leds, u32 base)
{
    (void)leds;
    assert(!strcmp(spec, target->uboot_part));
    event('E'); erases++;
    if (fault == ERASE) return -EIO;
    memset(storage + 16, 0xff, storage_size);
    prog_erase_done = base + RECOVERY_MMC_ERASE_PROGRESS_STEPS;
    return 0;
}
''' + function("recovery_mmc_write_part") + "\n" + function("recovery_verify_fit_payload") + "\n" + \
function("recovery_fit_has_image_node") + "\n" + function("recovery_is_sbe1v1k_chainloader_fit") + "\n" + \
function("recovery_validate_chainloader_fit") + "\n" + function("recovery_install_chainloader_fit") + "\n" + \
function("recovery_sbe1v1k_layout_desc") + r'''
static int normalized(enum recovery_sbe1v1k_layout id)
{
    if (id == RECOVERY_SBE1V1K_LAYOUT_QWRT) return RECOVERY_SBE1V1K_LAYOUT_MAINLINE;
    return id;
}
static int recovery_verify_sbe1v1k_gpt(const struct recovery_sbe1v1k_layout_desc *layout, bool require_hlos)
{
    assert(require_hlos);
    if (fault == POST_GPT && gpts) return -EINVAL;
    return prefix_valid && hlos_1 && normalized(geometry) == normalized(layout->id) ? 0 : -EINVAL;
}
static bool recovery_backend_is_mmc(void) { return true; }
static int recovery_qwrt_auth_code(char *code)
{ memset(code, 'a', 64); code[64] = 0; return fault == IDENTITY ? -ENOENT : 0; }
static void recovery_qwrt_secure_zero(void *data, size_t size) { memset(data, 0, size); }
static int recovery_preserve_chainloader_fit(void **fit, size_t *size)
{
    event('P');
    if (fault == PRESERVE) return -ENOENT;
    *size = preserved_size;
    *fit = malloc(*size);
    assert(*fit);
    memcpy(*fit, preserved, *size);
    return 0;
}
static int recovery_check_appsblenv(void) { return fault == ENV_CHECK ? -EINVAL : 0; }
static int recovery_write_appsblenv(struct recovery_status_led_ctrl *leds, size_t progress,
                                   const struct recovery_sbe1v1k_layout_desc *layout)
{
    (void)leds; (void)progress;
    assert(layout == target && readback_verified && reads == 1);
    assert(!memcmp(storage + 16, preserved, preserved_size));
    event('B'); env_commits++;
    return fault == ENV_COMMIT ? -EIO : 0;
}
static int recovery_apply_sbe1v1k_layout(const struct recovery_sbe1v1k_layout_desc *layout)
{ event('A'); applies++; active_sbe1v1k_layout = layout->id; return 0; }
static int recovery_verify_factory_gpt(struct recovery_status_led_ctrl *leds, bool *has_hlos)
{
    (void)leds;
    *has_hlos = hlos_1;
    prog_erase_total = 25;
    return prefix_valid ? 0 : -EINVAL;
}
static int recovery_clone_sbe1v1k_hlos_1(struct recovery_status_led_ctrl *leds, size_t progress)
{ (void)leds; (void)progress; event('C'); clones++; hlos_1 = true; return 0; }
static int recovery_build_sbe1v1k_gpt(const struct recovery_sbe1v1k_layout_desc *layout, char **gpt)
{ assert(layout == target); *gpt = strdup("fixture"); return *gpt ? 0 : -ENOMEM; }
static int env_set(const char *key, const char *value) { (void)key; (void)value; return 0; }
static const char *recovery_mmcdev(void) { return "0"; }
static int run_commandf(const char *format, ...)
{
    if (!strncmp(format, "gpt write", 9)) { event('G'); gpts++; geometry = target->id; }
    else assert(!strncmp(format, "mmc dev", 7));
    return 0;
}
''' + "\n".join(function(name) for name in (
    "recovery_select_sbe1v1k_profile", "recovery_repartition_factory",
    "recovery_change_sbe1v1k_profile")) + r'''
static void make_fit(void)
{
    const char *names[] = { "kernel-1", "uboot-1" };
    size_t payload_size = 1024 * 1024 + 17, capacity = payload_size + 4096;
    u8 *data = malloc(payload_size);
    assert(data);
    memset(data, 0x5a, payload_size);
    original = malloc(capacity);
    assert(original && !fdt_create_empty_tree(original, capacity));
    assert(!fdt_setprop_string(original, 0, "description", "SBE1V1K chainloader test"));
    assert(fdt_add_subnode(original, 0, "images") >= 0);
    for (unsigned int i = 0; i < ARRAY_SIZE(names); i++) {
        u8 hash[32];
        size_t length = i ? payload_size : 17;
        int images = fdt_path_offset(original, "/images");
        int node = fdt_add_subnode(original, images, names[i]);
        assert(node >= 0 && !fdt_setprop(original, node, "data", data, length));
        SHA256(data, length, hash);
        int hash_node = fdt_add_subnode(original, node, "hash-1");
        assert(hash_node >= 0);
        assert(!fdt_setprop_string(original, hash_node, "algo", "sha256"));
        assert(!fdt_setprop(original, hash_node, "value", hash, sizeof(hash)));
    }
    assert(!fdt_pack(original));
    original_size = fdt_totalsize(original);
    free(data);
}
static void reset(int id, int old_geometry, bool has_hlos)
{
    free(storage); free(preserved);
    target = recovery_sbe1v1k_layout_desc(id);
    assert(target);
    storage_size = target->uboot_size * 512;
    storage = malloc(storage_size + 32);
    assert(storage);
    memset(storage, 0xa5, storage_size + 32);
    preserved_size = original_size;
    preserved = malloc(preserved_size);
    assert(preserved);
    memcpy(preserved, original, preserved_size);
    fault = NONE;
    geometry = old_geometry;
    active_sbe1v1k_layout = old_geometry;
    hlos_1 = has_hlos; prefix_valid = true; readback_verified = false;
    erases = writes = reads = env_commits = applies = gpts = clones = 0;
    prog_phase = prog_erase_done = prog_erase_total = prog_write_done = 0;
    prog_write_total = prog_done = prog_total = 0;
    events_len = 0; events[0] = 0;
    recv_base = (const u8 *)RECOVERY_SBE1V1K_REPARTITION_TOKEN;
    recv_off = strlen((const char *)recv_base);
}
static void guards(void)
{
    for (unsigned int i = 0; i < 16; i++) {
        assert(storage[i] == 0xa5);
        assert(storage[16 + storage_size + i] == 0xa5);
    }
}
static void success(bool migration, bool clone)
{
    assert(!recovery_change_sbe1v1k_profile(NULL, target->id));
    assert(erases == 1 && writes >= 2 && reads == 1 && env_commits == 1 && applies == 1);
    assert(gpts == migration && clones == clone);
    assert(!memcmp(storage + 16, preserved, preserved_size));
    for (size_t i = preserved_size; i < ALIGN(preserved_size, 512); i++) assert(storage[16 + i] == 0);
    assert(strchr(events, 'P') < strchr(events, 'E'));
    assert(strchr(events, 'E') < strchr(events, 'W'));
    assert(strchr(events, 'R') < strchr(events, 'B'));
    assert(strchr(events, 'B') < strchr(events, 'A'));
    assert(prog_done == prog_total && prog_write_done == prog_write_total);
    guards();
}
int main(void)
{
    make_fit();
    const enum fault faults[] = { PRESERVE, ENV_CHECK, ERASE, WRITE, READ,
        CORRUPT_FIRST, CORRUPT_MIDDLE, CORRUPT_LAST, READ_HASH, ALLOC,
        WRONG_START, WRONG_SIZE, WRONG_SECTOR };
    for (int id = RECOVERY_SBE1V1K_LAYOUT_MAINLINE; id <= RECOVERY_SBE1V1K_LAYOUT_QWRT; id++) {
        /* Every profile, even an already matching GPT, reinstalls the FIT. */
        reset(id, id, true); success(false, false);
        reset(id, normalized(id) == RECOVERY_SBE1V1K_LAYOUT_LARGE ?
              RECOVERY_SBE1V1K_LAYOUT_MAINLINE : RECOVERY_SBE1V1K_LAYOUT_LARGE, true);
        success(true, false);
        reset(id, RECOVERY_SBE1V1K_LAYOUT_UNKNOWN, false); success(true, true);
        for (int migration = 0; migration < 2; migration++) {
            for (unsigned int n = 0; n < ARRAY_SIZE(faults); n++) {
                reset(id, migration ? RECOVERY_SBE1V1K_LAYOUT_UNKNOWN : id, true);
                fault = faults[n];
                assert(recovery_change_sbe1v1k_profile(NULL, id));
                assert(!env_commits && !applies);
                if (fault == PRESERVE || fault == ENV_CHECK || fault == ALLOC ||
                    fault == WRONG_START || fault == WRONG_SIZE || fault == WRONG_SECTOR)
                    assert(!erases && !writes);
                guards();
            }
        }
        reset(id, id, true);
        int node = fdt_path_offset(preserved, "/images/uboot-1"), len;
        u8 *data = (u8 *)fdt_getprop(preserved, node, "data", &len);
        assert(data && len > 0); data[len - 1] ^= 1;
        assert(recovery_change_sbe1v1k_profile(NULL, id));
        assert(!erases && !writes && !env_commits && !gpts);
        reset(id, RECOVERY_SBE1V1K_LAYOUT_UNKNOWN, true); prefix_valid = false;
        assert(recovery_change_sbe1v1k_profile(NULL, id));
        assert(!erases && !writes && !env_commits && !gpts);
        reset(id, id, true); recv_off--;
        assert(recovery_change_sbe1v1k_profile(NULL, id));
        assert(!erases && !env_commits);
        reset(id, RECOVERY_SBE1V1K_LAYOUT_UNKNOWN, true); fault = POST_GPT;
        assert(recovery_change_sbe1v1k_profile(NULL, id));
        assert(gpts == 1 && !erases && !env_commits);
        reset(id, id, true); fault = ENV_COMMIT;
        assert(recovery_change_sbe1v1k_profile(NULL, id));
        assert(readback_verified && env_commits == 1 && !applies);
        printf("PASS: profile %s, matching/different/unknown GPT, readback/hash failures and environment ordering\n", target->name);
    }
    reset(RECOVERY_SBE1V1K_LAYOUT_QWRT, RECOVERY_SBE1V1K_LAYOUT_QWRT, true);
    fault = IDENTITY;
    assert(recovery_change_sbe1v1k_profile(NULL, target->id));
    assert(!erases && !writes && !env_commits);

    reset(RECOVERY_SBE1V1K_LAYOUT_LARGE, RECOVERY_SBE1V1K_LAYOUT_LARGE, true);
    assert(recovery_install_chainloader_fit(target, NULL, preserved, 4 * 1024 * 1024UL + 1, 0, 0));
    assert(recovery_install_chainloader_fit(target, NULL, preserved, original_size - 1, 0, 0));
    assert(!erases && !writes);
    u8 *padded = calloc(1, 4 * 1024 * 1024UL);
    assert(padded); memcpy(padded, preserved, preserved_size);
    free(preserved); preserved = padded; preserved_size = 4 * 1024 * 1024UL;
    assert(!recovery_install_chainloader_fit(target, NULL, preserved, preserved_size, 0, 0));
    assert(reads == 1 && writes == 4 && !memcmp(storage + 16, preserved, preserved_size));
    guards();
    puts("PASS: complete 4 MiB FIT boundary, short/oversized rejection and no out-of-partition writes");
    free(storage); free(preserved); free(original);
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="sbe1v1k-chainloader-install-") as tmp:
    binary = Path(tmp) / "installer-test"
    subprocess.run([os.environ.get("HOSTCC", "cc"), "-x", "c", "-std=gnu11", "-g",
                    "-Wall", "-Wextra", "-Werror", "-Wno-sign-compare",
                    "-o", str(binary), "-", "-lfdt", "-lcrypto"],
                   input=harness, text=True, check=True)
    result = subprocess.run([str(binary)], capture_output=True, text=True)
    if result.returncode:
        print(result.stdout)
        print(result.stderr)
        raise SystemExit(result.returncode)
    for line in result.stdout.splitlines():
        if line.startswith("PASS:"):
            print(line)
