#!/bin/sh
# SPDX-License-Identifier: GPL-2.0+
# Regenerate the embedded recovery pages using lwIP's own host generator.
set -eu

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT HUP INT TERM
touch "$tmp/lwipopts.h"
lwip="$root/lib/lwip/lwip"
"${HOSTCC:-cc}" -O2 -I"$tmp" -I"$lwip/src/include" \
	-I"$lwip/contrib/ports/unix/port/include" \
	"$lwip/src/apps/http/makefsdata/makefsdata.c" -o "$tmp/makefsdata"
"$tmp/makefsdata" "$root/lib/lwip/httpd/htdocs" -nossi \
	-f:"$tmp/fsdata.c"
tr -d '\r' < "$tmp/fsdata.c" > "$root/lib/lwip/httpd/fsdata_xr1710g.c"
