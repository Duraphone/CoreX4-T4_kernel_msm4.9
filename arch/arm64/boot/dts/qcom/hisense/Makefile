
ifeq ($(subst \",,$(CONFIG_HISENSE_PRODUCT_NAME)),)
#$(error "Please set CONFIG_HISENSE_PRODUCT_NAME value.")
endif

ifeq ($(subst \",,$(CONFIG_HISENSE_PRODUCT_PLATFORM)),)
#$(error "Please set CONFIG_HISENSE_PRODUCT_PLATFORM value.")
endif

PRODUCT_DTS := $(subst $\",,$(CONFIG_HISENSE_PRODUCT_NAME))

PRODUCT_DTS_BASE := $(subst $\",,$(CONFIG_HISENSE_PRODUCT_PLATFORM))

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
$(warning "CONFIG_BUILD_ARM64_DT_OVERLAY is set")
dtbo-y += $(PRODUCT_DTS).dtbo

$(PRODUCT_DTS).dtbo-base := $(PRODUCT_DTS_BASE).dtb
else
dtb-y += $(PRODUCT_DTS).dtb
endif

ifeq ($(CONFIG_ARM64),y)
always       := $(dtb-y)
subdir-y     := $(dts-dirs)
else
targets      += dtbs
targets      += $(addprefix ../../, $(dtb-y))

$(obj)/../../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../../,$(dtb-y))
endif

clean-files := *.dtb
