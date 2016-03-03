include $(MTK_PATH_BUILD)/common.mk

ifeq ($(CUSTOM_KERNEL_PS),capPXS)
ccflags-y += -DTPD_PROXIMITY
endif

$(call all-modules-src-or-makefile,$(obj),CUSTOM_KERNEL_)
obj-n := dummy.o

obj-y += gsl_point_id.o
$(obj)/gsl_point_id.o: $(srctree)/../$(obj)/touchpanel/gsl_point_id
	cp $(srctree)/../$(obj)/touchpanel/gsl_point_id $(obj)/gsl_point_id.o
