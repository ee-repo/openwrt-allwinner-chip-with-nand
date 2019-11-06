#!/usr/bin/env bash
#
# Copyright (C) 2013 OpenWrt.org
#               2019 Benedikt-Alexander Mokroß (iCOGNIZE GmbH)
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

set -ex
[ $# -eq 9 ] || {
    echo "SYNTAX: $0 <outputfile> <kernel> <kernel offset> <dtb> <dtb offset> <rootfs image> <rootfs offset> <u-boot image> <nand page size in kb>"
    echo "Given: $@"
    exit 1
}

OUTPUT="$1"
KERNEL="$2"
KERNELOFFSET="$3"
DTB="$4"
DTBOFFSET="$5"
ROOTFS="$6"
ROOTFSOFFSET="$7"
UBOOT="$8"
PAGESIZE="$9"
# SPL-Size is an uint32 at 16 bytes offset contained in the SPL header
SPLSIZE=$(od -An -t u4 -j16 -N4 "$UBOOT" | xargs)

let totalblocks=$ROOTFSOFFSET/1024
let splblocks=$SPLSIZE/1024
let loopsplblocks=$splblocks-1
let ubootblock=$splblocks*$PAGESIZE
let kernblock=$KERNELOFFSET/1024
let dtbblock=$DTBOFFSET/1024
let rootfsblock=$ROOTFSOFFSET/1024

# The BROM of the SUNXI is only able to load 1k per page from SPI-NAND
# Thus, even if we have an 2k or 4k page-size, we have to chunk the SPL in 1k pieces

echo "Generating 0-image for boot part of size $ROOTFSOFFSET ($totalblocks blocks)"
dd if="/dev/zero" of="$OUTPUT" bs=1024 count=$totalblocks

echo "Copying block 0 to 0"
dd if="$UBOOT" of="$OUTPUT" bs=1024 count=2 seek=0 skip=0 conv=notrunc

for from in `seq 1 $loopsplblocks`;
do
	let to=$from*$PAGESIZE
	echo "Copying block $from to $to" 
	dd if="$UBOOT" of="$OUTPUT" bs=1024 count=1 seek=$to skip=$from conv=notrunc
done

echo "Appending u-boot to chunked SPL at block $ubootblock (origin: $splblocks)"
dd if="$UBOOT" of="$OUTPUT" bs=1024 seek=$ubootblock skip=$splblocks conv=notrunc

echo "Appending kernel at block $kernblock"
dd if="$KERNEL" of="$OUTPUT" bs=1024 seek=$kernblock conv=notrunc

echo "Appending dtb at block $dtbblock"
dd if="$DTB" of="$OUTPUT" bs=1024 seek=$dtbblock conv=notrunc

echo "Appending rootfs at block $rootfsblock"
dd if="$ROOTFS" of="$OUTPUT" bs=1024 seek=$rootfsblock conv=notrunc
