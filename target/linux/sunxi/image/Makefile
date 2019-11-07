#
# Copyright (C) 2013-2016 OpenWrt.org
# Copyright (C) 2016 Yousong Zhou
# Copyright (C) 2019 Benedikt-Alexander Mokroß (iCOGNIZE GmbH)
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

FAT32_BLOCK_SIZE=1024
FAT32_BLOCKS=$(shell echo $$(($(CONFIG_SUNXI_SD_BOOT_PARTSIZE)*1024*1024/$(FAT32_BLOCK_SIZE))))

KERNEL_LOADADDR:=0x40008000

define Build/sunxi-sdcard
	rm -f $@.boot
	mkfs.fat $@.boot -C $(FAT32_BLOCKS)

	mcopy -i $@.boot $(STAGING_DIR_IMAGE)/$(DEVICE_NAME)-boot.scr ::boot.scr
	mcopy -i $@.boot $(DTS_DIR)/$(SUNXI_DTS).dtb ::dtb
	mcopy -i $@.boot $(IMAGE_KERNEL) ::uImage
	./gen_sunxi_sdcard_img.sh $@ \
		$@.boot \
		$(IMAGE_ROOTFS) \
		$(CONFIG_SUNXI_SD_BOOT_PARTSIZE) \
		$(CONFIG_TARGET_ROOTFS_PARTSIZE) \
		$(STAGING_DIR_IMAGE)/$(DEVICE_NAME)-u-boot-with-spl.bin
	rm -f $@.boot
endef

# SYNTAX: $0 <outputfile> <kernel> <kernel offset> <dtb> <dtb offset> <rootfs image> <rootfs offset> <u-boot image> <nand page size in kb>
define Build/sunxi-spinand
	./gen_sunxi_spinand_img.sh $@ \
		$(IMAGE_KERNEL) \
		$(CONFIG_SUNXI_SPINAND_OFFSET_KERNEL) \
		$(DTS_DIR)/$(SUNXI_DTS).dtb \
		$(CONFIG_SUNXI_SPINAND_OFFSET_DTB) \
		$(IMAGE_ROOTFS) \
		$(CONFIG_SUNXI_SPINAND_OFFSET_ROOTFS) \
		$(STAGING_DIR_IMAGE)/$(DEVICE_NAME)-u-boot-with-spl.bin \
		$(CONFIG_SUNXI_SPINAND_PAGESIZE)
endef

define Build/sunxi-imageszip
	zip -D $@ $(IMAGE_KERNEL)
	zip -uD $@ $(DTS_DIR)/$(SUNXI_DTS).dtb
	zip -uD $@ $(IMAGE_ROOTFS)
	zip -uD $@ $(STAGING_DIR_IMAGE)/$(DEVICE_NAME)-u-boot-with-spl.bin
endef

# why \x00\x00\x00\x00 for zImage-initramfs
define Device/Default
  PROFILES := Default
  DEVICE_VARS := SUNXI_DTS SUNXI_UBOOT
  KERNEL_NAME := zImage
  KERNEL := kernel-bin | uImage none
  IMAGES := sdcard.img.gz spinand.img.gz images.zip
  IMAGE/images.zip := sunxi-imageszip
  IMAGE/sdcard.img.gz := sunxi-sdcard | append-metadata | gzip
  IMAGE/spinand.img.gz := sunxi-spinand | append-metadata | gzip
endef

include cortex-a7.mk
include cortex-a8.mk
include cortex-a53.mk

$(eval $(call BuildImage))
