#This file is generated by VisualGDB.
#It contains GCC settings automatically derived from the board support package (BSP).
#DO NOT EDIT MANUALLY. THE FILE WILL BE OVERWRITTEN. 
#Use VisualGDB Project Properties dialog or modify Makefile or per-configuration .mak files instead.

#VisualGDB provides BSP_ROOT and TOOLCHAIN_ROOT via environment when running Make. The line below will only be active if GNU Make is started manually.
BSP_ROOT ?= $(LOCALAPPDATA)/VisualGDB/EmbeddedBSPs/arm-eabi/com.sysprogs.arm.nordic.nrf5x
TOOLCHAIN_ROOT ?= C:/SysGCC/arm-eabi

#Embedded toolchain
CC := $(TOOLCHAIN_ROOT)/bin/arm-eabi-gcc.exe
CXX := $(TOOLCHAIN_ROOT)/bin/arm-eabi-g++.exe
LD := $(CXX)
AR := $(TOOLCHAIN_ROOT)/bin/arm-eabi-ar.exe
OBJCOPY := $(TOOLCHAIN_ROOT)/bin/arm-eabi-objcopy.exe

#Additional flags
PREPROCESSOR_MACROS += NRF52 S132 ARM_MATH_CM4 SOFTDEVICE_PRESENT SWI_DISABLE0
INCLUDE_DIRS += . $(BSP_ROOT)/nRF5x/components/softdevice/S132/headers $(BSP_ROOT)/nRF5x/components/toolchain $(BSP_ROOT)/nRF5x/components/toolchain/CMSIS/Include $(BSP_ROOT)/nRF5x/components/device $(BSP_ROOT)/nRF5x/components/softdevice/common/softdevice_handler $(BSP_ROOT)/nRF5x/components/drivers_nrf/adc $(BSP_ROOT)/nRF5x/components/drivers_nrf/ble_flash $(BSP_ROOT)/nRF5x/components/drivers_nrf/clock $(BSP_ROOT)/nRF5x/components/drivers_nrf/common $(BSP_ROOT)/nRF5x/components/drivers_nrf/comp $(BSP_ROOT)/nRF5x/components/drivers_nrf/config $(BSP_ROOT)/nRF5x/components/drivers_nrf/delay $(BSP_ROOT)/nRF5x/components/drivers_nrf/gpiote $(BSP_ROOT)/nRF5x/components/drivers_nrf/hal $(BSP_ROOT)/nRF5x/components/drivers_nrf/i2s $(BSP_ROOT)/nRF5x/components/drivers_nrf/lpcomp $(BSP_ROOT)/nRF5x/components/drivers_nrf/nrf_soc_nosd $(BSP_ROOT)/nRF5x/components/drivers_nrf/pdm $(BSP_ROOT)/nRF5x/components/drivers_nrf/ppi $(BSP_ROOT)/nRF5x/components/drivers_nrf/pstorage $(BSP_ROOT)/nRF5x/components/drivers_nrf/pstorage/config $(BSP_ROOT)/nRF5x/components/drivers_nrf/pwm $(BSP_ROOT)/nRF5x/components/drivers_nrf/qdec $(BSP_ROOT)/nRF5x/components/drivers_nrf/radio_config $(BSP_ROOT)/nRF5x/components/drivers_nrf/rng $(BSP_ROOT)/nRF5x/components/drivers_nrf/rtc $(BSP_ROOT)/nRF5x/components/drivers_nrf/saadc $(BSP_ROOT)/nRF5x/components/drivers_nrf/sdio $(BSP_ROOT)/nRF5x/components/drivers_nrf/sdio/config $(BSP_ROOT)/nRF5x/components/drivers_nrf/spi_master $(BSP_ROOT)/nRF5x/components/drivers_nrf/spi_slave $(BSP_ROOT)/nRF5x/components/drivers_nrf/swi $(BSP_ROOT)/nRF5x/components/drivers_nrf/timer $(BSP_ROOT)/nRF5x/components/drivers_nrf/twis_slave $(BSP_ROOT)/nRF5x/components/drivers_nrf/twi_master $(BSP_ROOT)/nRF5x/components/drivers_nrf/twi_master/deprecated $(BSP_ROOT)/nRF5x/components/drivers_nrf/twi_master/deprecated/config $(BSP_ROOT)/nRF5x/components/drivers_nrf/uart $(BSP_ROOT)/nRF5x/components/drivers_nrf/wdt $(BSP_ROOT)/nRF5x/components/libraries/util
LIBRARY_DIRS += $(BSP_ROOT)/nRF5x/SoftdeviceLibraries
LIBRARY_NAMES += compactcpp
ADDITIONAL_LINKER_INPUTS += 
MACOS_FRAMEWORKS += 
LINUX_PACKAGES += 

CFLAGS += -std=gnu99
CXXFLAGS += 
ASFLAGS += -mfpu=fpv4-sp-d16
LDFLAGS +=  
COMMONFLAGS += -mabi=aapcs  -mcpu=cortex-m4 -mthumb -mfloat-abi=hard
LINKER_SCRIPT := $(BSP_ROOT)/nRF5x/LinkerScripts/nRF52832_XXAA_S132_32k.lds
