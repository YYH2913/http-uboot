/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __HTTPD_RECOVERY_H
#define __HTTPD_RECOVERY_H

#include <linux/types.h>

struct fs_file;

int recovery_open_custom_response(struct fs_file *file, const char *status,
				 const char *content_type, const char *body,
				 int body_len);
bool recovery_backup_active(void);
int recovery_backup_open(struct fs_file *file, const char *path, bool busy);
void recovery_backup_close(struct fs_file *file);
int recovery_backup_read(struct fs_file *file, char *buffer, int count);

#endif
