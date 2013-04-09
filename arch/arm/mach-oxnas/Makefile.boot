ifeq ($(CONFIG_ARCH_OXNAS_FPGA),y)
	ifeq ($(CONFIG_ARCH_OXNAS_FPGA_VERSION), 0)
		initrd_phys-$(CONFIG_ARCH_OXNAS)	:= 0x44200000
		params_phys-$(CONFIG_ARCH_OXNAS)	:= 0x44000100
		zreladdr-$(CONFIG_ARCH_OXNAS)		:= 0x44008000
	else
		initrd_phys-$(CONFIG_ARCH_OXNAS)	:= 0x48200000
		params_phys-$(CONFIG_ARCH_OXNAS)	:= 0x48000100
		zreladdr-$(CONFIG_ARCH_OXNAS)		:= 0x48008000
	endif
else
	initrd_phys-$(CONFIG_ARCH_OXNAS)	:= 0x48200000
	params_phys-$(CONFIG_ARCH_OXNAS)	:= 0x48000100
	zreladdr-$(CONFIG_ARCH_OXNAS)		:= 0x48008000
endif
