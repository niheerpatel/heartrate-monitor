ccflags-y += ${PROJECTINCLUDE} \
                -I$(srctree)/include/drivers \
                -I$(srctree)/drivers

ifeq ($(ARCH), arc)
	obj-y = main_arc.o heartrate.o
else
	obj-y = main_x86.o
endif
