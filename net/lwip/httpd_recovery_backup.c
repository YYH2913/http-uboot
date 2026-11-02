// SPDX-License-Identifier: GPL-2.0+
/* Read-only, offset-preserving SPI-NAND downloads for XR1710G recovery. */

#include <limits.h>
#include <malloc.h>
#include <memalign.h>
#include <mtd.h>
#include <u-boot/schedule.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <lwip/apps/fs.h>

#include "httpd_recovery.h"

#define RECOVERY_BACKUP_MTD	"spi-nand0"
#define RECOVERY_BACKUP_CHUNK	(64 * 1024U)
#define RECOVERY_BACKUP_INFO_MAX	8192
#define RECOVERY_BACKUP_NAME_MAX	64

struct recovery_backup_file {
	struct mtd_info *mtd;
	u8 *cache;
	size_t cache_capacity;
	size_t cache_len;
	size_t cache_off;
	loff_t next_offset;
	u64 bytes_left;
	char header[384];
	size_t header_len;
	size_t header_off;
};

static bool backup_active;

bool recovery_backup_active(void)
{
	return backup_active;
}

static int recovery_backup_error(struct fs_file *file, const char *status,
				 const char *message)
{
	return recovery_open_custom_response(file, status, "text/plain", message,
					     strlen(message));
}

/* MTD names can come from mtdparts; never insert them verbatim into JSON. */
static bool recovery_backup_escape_name(const char *name, char *escaped)
{
	const u8 *p = (const u8 *)name;
	char *out = escaped;

	if (!name || strlen(name) > RECOVERY_BACKUP_NAME_MAX)
		return false;
	for (; *p; p++) {
		if (*p < 0x20 || *p == '"' || *p == '\\') {
			snprintf(out, 7, "\\u%04x", *p);
			out += 6;
		} else {
			*out++ = *p;
		}
	}
	*out = '\0';
	return true;
}

static bool recovery_backup_valid_partition(struct mtd_info *mtd,
					    struct mtd_info *master)
{
	return mtd->parent == master && mtd->size && mtd->writesize &&
	       mtd->offset <= master->size &&
	       mtd->size <= master->size - mtd->offset;
}

static int recovery_backup_info(struct fs_file *file, struct mtd_info *master,
				bool busy)
{
	char name[6 * RECOVERY_BACKUP_NAME_MAX + 1];
	struct mtd_info *part;
	char *json;
	int len, n, ret;
	bool first = true;

	json = malloc(RECOVERY_BACKUP_INFO_MAX);
	if (!json)
		return recovery_backup_error(file, "503 Service Unavailable",
					     "Not enough memory for backup info.\n");
	len = snprintf(json, RECOVERY_BACKUP_INFO_MAX,
		       "{\"device\":\"%s\",\"size\":%llu,\"erase_size\":%u,"
		       "\"page_size\":%u,\"busy\":%s,\"active\":%s,\"partitions\":[",
		       RECOVERY_BACKUP_MTD, (unsigned long long)master->size,
		       master->erasesize, master->writesize,
		       busy ? "true" : "false", backup_active ? "true" : "false");
	if (len < 0 || len >= RECOVERY_BACKUP_INFO_MAX)
		goto overflow;

	list_for_each_entry(part, &master->partitions, node) {
		if (!recovery_backup_valid_partition(part, master))
			continue;
		if (!recovery_backup_escape_name(part->name, name))
			goto overflow;
		n = snprintf(json + len, RECOVERY_BACKUP_INFO_MAX - len,
			     "%s{\"index\":%d,\"name\":\"%s\",\"offset\":%llu,"
			     "\"size\":%llu}", first ? "" : ",", part->index, name,
			     (unsigned long long)part->offset,
			     (unsigned long long)part->size);
		if (n < 0 || n >= RECOVERY_BACKUP_INFO_MAX - len)
			goto overflow;
		len += n;
		first = false;
	}
	if (len + 4 >= RECOVERY_BACKUP_INFO_MAX)
		goto overflow;
	memcpy(json + len, "]}\n", 3);
	ret = recovery_open_custom_response(file, "200 OK", "application/json",
					    json, len + 3);
	free(json);
	return ret;

overflow:
	free(json);
	return recovery_backup_error(file, "500 Internal Server Error",
				     "MTD partition list is too large.\n");
}

/* Only accept canonical decimal MTD indexes, not names, offsets or paths. */
static int recovery_backup_partition_index(const char *path)
{
	const char *p;
	int index = 0;

	if (strncmp(path, "backup/mtd", 10))
		return -EINVAL;
	p = path + 10;
	if (*p < '0' || *p > '9')
		return -EINVAL;
	if (*p == '0' && p[1] != '.')
		return -EINVAL;
	while (*p >= '0' && *p <= '9') {
		if (index > (INT_MAX - (*p - '0')) / 10)
			return -EINVAL;
		index = index * 10 + *p++ - '0';
	}
	return strcmp(p, ".bin") ? -EINVAL : index;
}

int recovery_backup_open(struct fs_file *file, const char *path, bool busy)
{
	struct recovery_backup_file *backup;
	struct mtd_info *master, *mtd;
	char filename[64];
	bool info = !strcmp(path, "backup/info");
	bool full = !strcmp(path, "backup/full.bin");
	int index = -1, ret, header_len;

	if (!info && !full) {
		index = recovery_backup_partition_index(path);
		if (index < 0)
			return recovery_backup_error(file, "404 Not Found",
						     "Unknown backup target.\n");
	}
	busy |= backup_active;
	if (!info && busy)
		return recovery_backup_error(file, "409 Conflict",
					     "Recovery is busy. Try again when idle.\n");

	/* Never reparse MTD partitions while an upload or download owns them. */
	if (!busy)
		mtd_probe_devices();
	master = get_mtd_device_nm(RECOVERY_BACKUP_MTD);
	if (IS_ERR_OR_NULL(master))
		return recovery_backup_error(file, "503 Service Unavailable",
					     "SPI-NAND is unavailable.\n");
	if (mtd_is_partition(master) || !master->size || !master->writesize) {
		put_mtd_device(master);
		return recovery_backup_error(file, "503 Service Unavailable",
					     "Backup source is not a complete MTD.\n");
	}
	if (info) {
		ret = recovery_backup_info(file, master, busy);
		put_mtd_device(master);
		return ret;
	}

	mtd = master;
	if (!full) {
		mtd = get_mtd_device(NULL, index);
		if (IS_ERR_OR_NULL(mtd)) {
			put_mtd_device(master);
			return recovery_backup_error(file, "404 Not Found",
						     "MTD partition is unavailable.\n");
		}
		ret = recovery_backup_valid_partition(mtd, master);
		put_mtd_device(master);
		if (!ret) {
			put_mtd_device(mtd);
			return recovery_backup_error(file, "404 Not Found",
						     "Not a SPI-NAND partition.\n");
		}
	}

	backup = calloc(1, sizeof(*backup));
	if (!backup) {
		put_mtd_device(mtd);
		return recovery_backup_error(file, "503 Service Unavailable",
					     "Not enough memory for backup.\n");
	}
	backup->mtd = mtd;
	backup->bytes_left = mtd->size;
	backup->cache_capacity = max_t(size_t, RECOVERY_BACKUP_CHUNK,
				       mtd->writesize);
	backup->cache_capacity -= backup->cache_capacity % mtd->writesize;
	backup->cache = malloc_cache_aligned(backup->cache_capacity);
	if (!backup->cache)
		goto fail;
	if (full)
		strcpy(filename, "xr1710g-spi-nand-full.bin");
	else
		snprintf(filename, sizeof(filename), "xr1710g-mtd%d.bin", index);
	header_len = snprintf(backup->header, sizeof(backup->header),
			      "HTTP/1.0 200 OK\r\n"
			      "Content-Type: application/octet-stream\r\n"
			      "Content-Disposition: attachment; filename=\"%s\"\r\n"
			      "Cache-Control: no-store\r\n"
			      "Accept-Ranges: none\r\n"
			      "Content-Length: %llu\r\n"
			      "Connection: close\r\n\r\n",
			      filename, (unsigned long long)mtd->size);
	if (header_len < 0 || header_len >= (int)sizeof(backup->header) ||
	    mtd->size > (u64)(INT_MAX - header_len))
		goto fail;
	backup->header_len = header_len;
	backup_active = true;
	file->data = NULL;
	file->len = header_len + (int)mtd->size;
	file->index = 0;
	file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
	file->pextension = backup;
	printf("httpd: streaming %llu bytes from '%s' (read only, no OOB)\n",
	       (unsigned long long)mtd->size, mtd->name);
	return 1;

fail:
	put_mtd_device(mtd);
	free(backup->cache);
	free(backup);
	return recovery_backup_error(file, "503 Service Unavailable",
				     "Cannot allocate or represent this backup.\n");
}

void recovery_backup_close(struct fs_file *file)
{
	struct recovery_backup_file *backup = file->pextension;

	printf("httpd: backup stream closed (%d/%d bytes queued)\n",
	       file->index, file->len);
	put_mtd_device(backup->mtd);
	free(backup->cache);
	free(backup);
	file->pextension = NULL;
	backup_active = false;
}

static int recovery_backup_fill_cache(struct recovery_backup_file *backup)
{
	size_t chunk, retlen = 0;
	int ret;

	if (!backup->bytes_left)
		return 0;
	chunk = min_t(u64, backup->bytes_left, backup->cache_capacity);
	/* Do not skip bad blocks: doing so would shift every subsequent offset. */
	ret = mtd_read(backup->mtd, backup->next_offset, chunk, &retlen,
		       backup->cache);
	if ((ret && ret != -EUCLEAN) || retlen != chunk) {
		printf("httpd: backup read failed at 0x%llx: ret=%d len=%lu/%lu\n",
		       (unsigned long long)backup->next_offset, ret,
		       (unsigned long)retlen, (unsigned long)chunk);
		backup->bytes_left = 0;
		return -EIO;
	}
	backup->next_offset += chunk;
	backup->bytes_left -= chunk;
	backup->cache_len = chunk;
	backup->cache_off = 0;
	schedule();
	return 1;
}

int recovery_backup_read(struct fs_file *file, char *buffer, int count)
{
	struct recovery_backup_file *backup = file->pextension;
	int copied = 0;

	count = min(count, file->len - file->index);
	while (copied < count) {
		size_t available, todo;

		if (backup->header_off < backup->header_len) {
			available = backup->header_len - backup->header_off;
			todo = min_t(size_t, available, count - copied);
			memcpy(buffer + copied, backup->header + backup->header_off,
			       todo);
			backup->header_off += todo;
			copied += todo;
			continue;
		}
		if (backup->cache_off >= backup->cache_len &&
		    recovery_backup_fill_cache(backup) <= 0)
			break;
		available = backup->cache_len - backup->cache_off;
		todo = min_t(size_t, available, count - copied);
		memcpy(buffer + copied, backup->cache + backup->cache_off, todo);
		backup->cache_off += todo;
		copied += todo;
	}
	file->index += copied;
	/* A short HTTP body is intentional on read failure; never pad bad data. */
	return copied ? copied : FS_READ_EOF;
}
