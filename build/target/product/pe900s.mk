
PRODUCT_PACKAGES := \
#    FMRadio
#    MyTube \
#    VideoPlayer


$(call inherit-product, $(SRC_TARGET_DIR)/product/common.mk)
$(call inherit-product, $(SRC_TARGET_DIR)/product/telephony.mk)

# Overrides
ifneq ($(TARGET_BUILD_BRAND),)
	PRODUCT_BRAND := $(TARGET_BUILD_BRAND)
else
	PRODUCT_BRAND  := YUDI
endif
PRODUCT_MANUFACTURER := HYIPC
PRODUCT_NAME   := $(TARGET_PRODUCT)
PRODUCT_DEVICE := $(TARGET_PRODUCT)

# This is for custom project language configuration.
PRODUCT_LOCALES := $(MTK_PRODUCT_LOCALES)

