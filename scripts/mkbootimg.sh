#!/bin/bash

# common setting
OUTDIR=$1
TARGET_ARCH=$2
BUILD_TYPE=$3
TARGET_PLATFORM=$4

if [ $TARGET_ARCH = "arm64" ]; then
	KERNEL=$OUTDIR/arch/$TARGET_ARCH/boot/Image.gz-dtb
else
	KERNEL=$OUTDIR/arch/$TARGET_ARCH/boot/zImage-dtb
fi

# mkbootimg args, check device/qcom/<product>/BoardConfig.mk  BOARD_KERNEL_CMDLINE
if [ CHIPSET_$TARGET_PLATFORM = "CHIPSET_msm8953" ]; then
CMDLINES="console=ttyMSM0,115200,n8 androidboot.console=ttyMSM0 androidboot.hardware=qcom msm_rtb.filter=0x237 ehci-hcd.park=3 lpm_levels.sleep_disabled=1 androidboot.bootdevice=7824900.sdhci earlycon=msm_serial_dm,0x78af000 androidboot.usbconfigfs=true loop.max_part=7 buildvariant="    
else
CMDLINES="console=ttyMSM0,115200,n8 androidboot.console=ttyMSM0 androidboot.hardware=qcom msm_rtb.filter=0x237 ehci-hcd.park=3 lpm_levels.sleep_disabled=1 androidboot.bootdevice=7824900.sdhci earlycon=msm_serial_dm,0x78B0000 androidboot.usbconfigfs=true loop.max_part=7 buildvariant="
fi
PAGE_SIZE=2048
BASECONFIG="--base 0x80000000 --pagesize $PAGE_SIZE"

# avb signing args, see BOARD_BOOTIMAGE_PARTITION_SIZE / BOARD_DTBOIMG_PARTITION_SIZE in BoardConfig.mk
BOOT_PSIZE=0x04000000
DTBO_PSIZE=0x0800000

# Check build/make/core/version_defaults.mk
ANDROID_INFO=" --os_version 10 --os_patch_level 2020-04-05 "
HEADER_VER="--header_version 1"
AVB_OS_INFO="com.android.build.boot.os_version:10"
AVB_SECURE_INFO="com.android.build.boot.security_patch:2019-12-05"
# Get product name
BOARD_CFG=`cat $OUTDIR/.config | grep CONFIG_HISENSE_PRODUCT_NAME | sed "s/.*=\"\([A-Za-z0-9\._\-]*\)\"/\1/"`

# Tool path configure, need compile from Android
MKBOOTIMG=bootimg/bin/mkbootimg
AVBTOOL=bootimg/bin/avbtool
MKDTIMG=bootimg/bin/mkdtimg

BOOTIMG=bootimg/boot.img
DTBOIMG=bootimg/dtbo.img

if [ -f $KERNEL ]; then
	rm -f $BOOTIMG

	echo "Build boot.img"
	$MKBOOTIMG --kernel $KERNEL --cmdline "${CMDLINES}${BUILD_TYPE}" $BASECONFIG $ANDROID_INFO  $HEADER_VER --output $BOOTIMG
	$AVBTOOL add_hash_footer --image $BOOTIMG --partition_size $BOOT_PSIZE --partition_name boot  --prop  $AVB_OS_INFO --prop $AVB_SECURE_INFO

	echo "Build dtbo.img"
	DTBO_LIST=`find $OUTDIR/arch/$TARGET_ARCH/boot/dts -name *.dtbo`
	$MKDTIMG create bootimg/prebuilt_dtbo.img --page_size=$PAGE_SIZE $DTBO_LIST
	chmod a+r bootimg/prebuilt_dtbo.img
	cp bootimg/prebuilt_dtbo.img $DTBOIMG
	$AVBTOOL add_hash_footer --image $DTBOIMG --partition_size $DTBO_PSIZE --partition_name dtbo
else
	echo "Kernel image \"$KERNEL\" not found."
fi
