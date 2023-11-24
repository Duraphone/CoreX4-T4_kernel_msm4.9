ccflags-y += -Idrivers/soc/hisense/include

obj-$(CONFIG_RS_RECORDER_SUPPORT)  += \
		rs_recorder_main.o rs_savelog.o rs_compress.o \
		rs_allps_info.o rs_blocked_task.o rs_dump_dmesg.o \
		rs_dump_slabinfo.o rs_dump_meminfo.o \
		rs_dump_irqs.o rs_save_dump.o rs_last_kmsg.o

ifeq ($(CONFIG_RS_RECORDER_SUPPORT),y)
obj-$(CONFIG_EXCEPTION_MONITOR)  += exm_core.o
endif

