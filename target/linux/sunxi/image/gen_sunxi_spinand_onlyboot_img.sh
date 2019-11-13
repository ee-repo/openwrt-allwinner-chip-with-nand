#!/usr/bin/env bash
#
# Copyright (C) 2013 OpenWrt.org
#               2019 Benedikt-Alexander Mokroß (iCOGNIZE GmbH)
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

set -ex
[ $# -eq 3 ] || {
    echo "SYNTAX: $0 <outputfile> <u-boot image> <nand page size in kb>"
    echo "Given: $@"
    exit 1
}

OUTPUT="$1"
UBOOT="$2"
PAGESIZE="$3"
# SPL-Size is an uint32 at 16 bytes offset contained in the SPL header
SPLSIZE=$(od -An -t u4 -j16 -N4 "$UBOOT" | xargs)

ALIGNCHECK=$(($PAGESIZE%1024))
if [ "$ALIGNCHECK" -ne "0" ]; then
	echo "Page-size is not 1k alignable and thus not supported by EGON"
	exit -1
fi

KPAGESIZE=$(($PAGESIZE/1024))
splblocks=$(($SPLSIZE/1024))
loopsplblocks=$(($splblocks-1))
ubootblock=$(($splblocks*$KPAGESIZE))

# The BROM of the SUNXI is only able to load 1k per page from SPI-NAND
# Thus, even if we have an 2k or 4k page-size, we have to chunk the SPL in 1k pieces

echo "Generating 0-image for boot part of size $SPLSIZE ($splblocks blocks)"
dd if="/dev/zero" of="$OUTPUT" bs=1024 count=$splblocks

echo "Copying block 0 to 0"
dd if="$UBOOT" of="$OUTPUT" bs=1024 count=2 seek=0 skip=0 conv=notrunc

for from in `seq 1 $loopsplblocks`;
do
	to=$(($from*$KPAGESIZE))
	echo "Copying block $from to $to" 
	dd if="$UBOOT" of="$OUTPUT" bs=1024 count=1 seek=$to skip=$from conv=notrunc
done

echo "Appending u-boot to chunked SPL at block $ubootblock (origin: $splblocks)"
dd if="$UBOOT" of="$OUTPUT" bs=1024 seek=$ubootblock skip=$splblocks conv=notrunc

# ToDo: Embedd SPL multiple times
# Entry-Pages:
#          32,      64,      96,     128,     160,     192,     224
# 2k: 0x10000, 0x20000, 0x30000, 0x40000, 0x50000, 0x60000, 0x70000
# 4K: 0x20000, 0x40000, 0x60000, 0x80000, 0xA0000, 0xC0000, 0xE0000
# assuming the SPL is smaller then 24576 (0x6000) ->
# using the first 4 should be sufficient and u-boot can be placed at 0x50000 or 0xA0000
# for a 2KiB page-size and 128KiB PEB, the copy at 64 and 96 are in another PEB, neat!
# for a 4KiB page-size and 128KiB PEB, every copy is in its own PEB, neat^2! 
# maybe place an second copy of u-boot, too?
