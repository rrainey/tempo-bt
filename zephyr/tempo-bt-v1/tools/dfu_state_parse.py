#!/usr/bin/env python3
"""Parse smpmgr `image state-read` output (stdin) into one line per image state:

    slot image HASH active confirmed pending

Used by tools/dfu-upgrade.sh. smpmgr prints each image as an ImageState(...) block
containing a nested HashBytes('...') whose parentheses defeat naive outer-paren
matching, so we split on the "ImageState(" marker and pull fields by regex.
"""
import sys
import re

txt = sys.stdin.read()
for blk in txt.split("ImageState(")[1:]:
    def g(key):
        m = re.search(key + r"=(True|False|\d+)", blk)
        return m.group(1) if m else "?"
    hm = re.search(r"HashBytes\([^0-9A-Fa-f]*([0-9A-Fa-f]+)", blk)
    h = hm.group(1).upper() if hm else "?"
    print(g("slot"), g("image"), h, g("active"), g("confirmed"), g("pending"))
