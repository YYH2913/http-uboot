# SBE1V1K Factory eMMC Layout

This document describes the supported factory eMMC user-area layout.
The eMMC hardware boot areas, `boot0` and `boot1`, are separate from the GPT
partitions and must be backed up separately or included in a complete backup archive.

Factory p43 `ASKEYMFC` can reach the final sector and overlap the secondary GPT.
The migration layout below leaves room for a valid secondary GPT.

The recovery migration check intentionally does not require `ASKEYMFC` to be
resolvable by U-Boot. Some factory images expose the same corrupt-secondary-GPT
tail condition through U-Boot's partition parser, while the preserved p1-p26
boot-chain boundary is still valid.

Sector size is 512 bytes. The user area has 15,269,888 sectors.

## Factory Partitions

| No. | Start | End | Sectors | Size | Name |
| ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 34 | 2081 | 2048 | 1 MiB | `0:SBL1` |
| 2 | 2082 | 4129 | 2048 | 1 MiB | `0:SBL1_1` |
| 3 | 4130 | 5153 | 1024 | 512 KiB | `0:BOOTCONFIG` |
| 4 | 5154 | 6177 | 1024 | 512 KiB | `0:BOOTCONFIG1` |
| 5 | 6178 | 12321 | 6144 | 3 MiB | `0:QSEE` |
| 6 | 12322 | 18465 | 6144 | 3 MiB | `0:QSEE_1` |
| 7 | 18466 | 19489 | 1024 | 512 KiB | `0:DEVCFG` |
| 8 | 19490 | 20513 | 1024 | 512 KiB | `0:DEVCFG_1` |
| 9 | 20514 | 21537 | 1024 | 512 KiB | `0:APDP` |
| 10 | 21538 | 22561 | 1024 | 512 KiB | `0:APDP_1` |
| 11 | 22562 | 23585 | 1024 | 512 KiB | `0:TME` |
| 12 | 23586 | 24609 | 1024 | 512 KiB | `0:TME_1` |
| 13 | 24610 | 25633 | 1024 | 512 KiB | `0:RPM` |
| 14 | 25634 | 26657 | 1024 | 512 KiB | `0:RPM_1` |
| 15 | 26658 | 27681 | 1024 | 512 KiB | `0:CDT` |
| 16 | 27682 | 28705 | 1024 | 512 KiB | `0:CDT_1` |
| 17 | 28706 | 29217 | 512 | 256 KiB | `0:APPSBLENV` |
| 18 | 29218 | 33313 | 4096 | 2 MiB | `0:APPSBL` |
| 19 | 33314 | 37409 | 4096 | 2 MiB | `0:APPSBL_1` |
| 20 | 37410 | 39457 | 2048 | 1 MiB | `0:ART` |
| 21 | 39458 | 40481 | 1024 | 512 KiB | `0:ETHPHYFW` |
| 22 | 40482 | 40993 | 512 | 256 KiB | `0:LICENSE` |
| 23 | 40994 | 61473 | 20480 | 10 MiB | `0:WIFIFW` |
| 24 | 61474 | 81953 | 20480 | 10 MiB | `0:WIFIFW_1` |
| 25 | 81954 | 96289 | 14336 | 7 MiB | `0:HLOS` |
| 26 | 96290 | 110625 | 14336 | 7 MiB | `0:HLOS_1` |
| 27 | 110626 | 360481 | 249856 | 122 MiB | `rootfs` |
| 28 | 360482 | 610337 | 249856 | 122 MiB | `rootfs_1` |
| 29 | 610338 | 1658913 | 1048576 | 512 MiB | `rootfs_data` |
| 30 | 1658914 | 2707489 | 1048576 | 512 MiB | `rootfs_data_1` |
| 31 | 2707490 | 2723873 | 16384 | 8 MiB | `econfig` |
| 32 | 2723874 | 2756641 | 32768 | 16 MiB | `edata` |
| 33 | 2756642 | 3018785 | 262144 | 128 MiB | `log` |
| 34 | 3018786 | 3051553 | 32768 | 16 MiB | `persist` |
| 35 | 3051554 | 5148705 | 2097152 | 1 GiB | `usr_app` |
| 36 | 5148706 | 5156897 | 8192 | 4 MiB | `tls` |
| 37 | 5156898 | 5165089 | 8192 | 4 MiB | `backup_tls` |
| 38 | 5165090 | 5169185 | 4096 | 2 MiB | `bypass_cert` |
| 39 | 5169186 | 5201953 | 32768 | 16 MiB | `rsvd_1` |
| 40 | 5201954 | 5267489 | 65536 | 32 MiB | `rsvd_2` |
| 41 | 5267490 | 5398561 | 131072 | 64 MiB | `rsvd_3` |
| 42 | 5398562 | 15249407 | 9850846 | 4.7 GiB | `user_data` |
| 43 | 15249408 | 15269887 | 20480 | 10 MiB | `ASKEYMFC` |

The factory layout is dual-image:

```text
slot 0: 0:HLOS,   rootfs,   rootfs_data
slot 1: 0:HLOS_1, rootfs_1, rootfs_data_1
```

Do not install the chainloader into `0:HLOS` or `0:HLOS_1`. Those 7 MiB
partitions are part of the Askey dual-image verification and backup flow.

Keep `rsvd_1` available for the factory repair flow.
The `mainline` recovery profile stores the 4 MiB chainloader
image at the start of `rsvd_2` (LBA `5201954`, `0x4f6022`) without using either
HLOS slot.

QWRT compatibility uses the factory-compatible partition map, with the
chainloader preserved in `rsvd_2`.

## Large Chainloader Migration Layout

The table below is the layout produced from the standard factory GPT above.
Recovery requires the exact P1-P25 prefix. If the source omits P26 `0:HLOS_1`,
it copies P25 `0:HLOS` into the unallocated 7 MiB slot and verifies the copy
before writing the GPT. This keeps the large-layout partition numbers stable.

| Start | End | Sectors | Size | Name |
| ---: | ---: | ---: | ---: | --- |
| before 96290 (`0x17822`) | unchanged | unchanged | unchanged | P1-P25 |
| 96290 (`0x17822`) | 110625 | 14336 (`0x3800`) | 7 MiB | P26 `0:HLOS_1` |
| 110626 (`0x1b022`) | 118817 | 8192 (`0x2000`) | 4 MiB | P27 `chainloader` |
| 118818 (`0x1d022`) | 184353 | 65536 (`0x10000`) | 32 MiB | P28 `kernel` |
| 184354 (`0x2d022`) | 2281505 | 2097152 (`0x200000`) | 1 GiB | P29 `rootfs` |
| 2281506 (`0x22d022`) | 15269854 | 12988349 | about 6.2 GiB | P30 `rootfs_data` |

This is a raw eMMC/GPT layout. `rootfs_data` is a GPT partition label, not a UBI
volume, and the SBE1V1K recovery path must not use UBI management for firmware
or data writes.

Byte offsets used by U-Boot `gpt write`:

```text
chainloader start 0x3604400,  size 0x400000
kernel      start 0x3a04400,  size 0x2000000
rootfs      start 0x5a04400,  size 0x40000000
rootfs_data start 0x45a04400, size=-
```

The last usable sector is `15269854`, leaving the final 33 sectors for the
secondary GPT.

## Installation Flow

From stock U-Boot, boot the raw chainloader FIT from `0x80000000`:

```sh
setenv ipaddr 192.168.1.1
setenv serverip 192.168.1.2
tftpboot 0x80000000 sbe1v1k-chainloader.itb
bootm 0x80000000
```

Do not run `saveenv` from stock U-Boot.

In the HTTP page, run `Repartition factory eMMC` with confirmation token:

```text
SBE1V1K_REPARTITION
```

The action verifies the exact P1-P25 factory prefix, preserves the running FIT from
`0x44000000` or `0x80000000` (falling back to the current eMMC chainloader
partition), clones and verifies `0:HLOS` into P26 when `0:HLOS_1` is absent,
writes the migration GPT and selected chainloader target, and
updates `0:APPSBLENV` with `fw_setenv`-equivalent variable changes while
preserving other entries.
The APPSBLENV partition is read back after writing so the migration fails if
the expected variables did not persist.

The resulting stock U-Boot environment is:

```text
bootargs=console=ttyMSM0,115200n8 rootwait root=PARTLABEL=rootfs
boot_chainloader=mmc dev 0 0; mmc read 0x44000000 0x0001b022 0x2000; bootm 0x44000000
do_boot=run boot_chainloader
do_nothing=true
bootcmd=echo "Hit ctrl+c for shell..."; if sleep 3; then setenv bootargs console=ttyMSM0,115200n8 rootwait root=PARTLABEL=rootfs; run do_boot; else run do_nothing; fi;
```

After migration, upload the OpenWrt/QSDK firmware from the same HTTP page.
Firmware uploads erase `kernel`, `rootfs`, and `rootfs_data` before receiving
the request body, then stream the first 32 MiB to `kernel` and the remainder to
`rootfs`. Chainloader uploads erase `chainloader` first and stream the raw FIT
directly. Interrupted or invalid uploads are destructive by design. OpenWrt/QSDK
boots with `root=PARTLABEL=rootfs` and does not depend on `0:HLOS`.
