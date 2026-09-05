#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
#
# Copyright (c) 2011 The Chromium OS Authors.
#

"""Stub for the former in-tree patman tool

patman is now maintained as a standalone 'patch-manager' package, rather
than living in the U-Boot tree. This stub just tells the user how to get
it.
"""

import sys


def main():
    """Print instructions for installing the patch-manager package"""
    print(
        'patman is no longer part of U-Boot. It is now maintained as a\n'
        "separate package called 'patch-manager'.\n"
        '\n'
        'Install it with:\n'
        '\n'
        '    pip install patch-manager\n'
        '\n'
        'Documentation: https://deinde.dev/patman\n',
        file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
