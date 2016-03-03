obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter.o battery_common.o

ifneq ($(MTK_FAN5405_SUPPORT)_$(MTK_BQ24158_SUPPORT), no_no)
    obj-$(CONFIG_MTK_SMART_BATTERY) += switch_charging.o 
else
    obj-$(CONFIG_MTK_SMART_BATTERY) += linear_charging.o
endif
