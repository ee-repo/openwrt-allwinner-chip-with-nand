ifeq ($(SUBTARGET),cortexa7v3sspinand)

define Device/sun8i-v3s-dolphinpi
  DEVICE_VENDOR := Petit-Miner
  DEVICE_MODEL := Dolphin Pi
  DEVICE_PACKAGES := kmod-rtc-sunxi
  SUPPORTED_DEVICES := dolphinpi
  SUNXI_DTS := sun8i-v3s-dolphinpi

  KERNEL := kernel-bin | lzma | uImage lzma 
  IMAGES := ubispinand.img.gz

  MKUBIFS_OPTS := -F -m $(CONFIG_SUNXI_SPINAND_PAGESIZE) -e $(shell echo $$(($(CONFIG_SUNXI_SPINAND_BLOCKIZE) - (($(CONFIG_SUNXI_SPINAND_PAGESIZE)/1024)*2))))KiB -c 880 -U
  UBINIZE_OPTS := -vv

  BLOCKSIZE := $(CONFIG_SUNXI_SPINAND_BLOCKIZE)KiB
  PAGESIZE := $(CONFIG_SUNXI_SPINAND_PAGESIZE)
  SUBPAGESIZE := $(CONFIG_SUNXI_SPINAND_PAGESIZE)
  VID_HDR_OFFSET := $(CONFIG_SUNXI_SPINAND_PAGESIZE)
  IMAGE_SIZE := 110m
  KERNEL_SIZE := 4m
  KERNEL_IN_UBI := 1
  UBOOTENV_IN_UBI := 1
  UBINIZE_PARTS := dtb=$(DTS_DIR)/$$(SUNXI_DTS).dtb=1

  IMAGE/ubispinand.img.gz := \
      sunxi-spinandboot | \
      pad-to $$(CONFIG_SUN8I_V3S_OFFSET_UBI) | \
      append-ubi | \
      gzip
endef

TARGET_DEVICES += sun8i-v3s-dolphinpi

define Device/sun8i-v3s-rawspinandaccessuart2
  DEVICE_VENDOR := Allwinner
  DEVICE_MODEL := Generic V3s Board
  DEVICE_PACKAGES:=kmod-rtc-sunxi
  SUPPORTED_DEVICES:=allwinner,sun8i-v3s
  SUNXI_DTS:=sun8i-v3s-rawspinandaccessuart2
  IMAGES := sdcard.img.gz
endef

TARGET_DEVICES += sun8i-v3s-rawspinandaccessuart2

endif