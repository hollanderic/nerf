
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) )


### THESE will likely need to be modified based on your local installation
###		Best practice would be to mirror what is here in your local filesystem

NRFSDK_PATH =../../vendor/nRF51_SDK_9.0.0
TEMPLATE_PATH = $(NRFSDK_PATH)/components/toolchain/gcc
SOFTDEVICE :=$(NRFSDK_PATH)/components/softdevice/s110/hex/s110_nrf51_softdevice.hex
OPENOCD_PATH := $(HOME)/src/tools/openocd
OPENOCD := $(OPENOCD_PATH)/src/openocd -s $(OPENOCD_PATH)/tcl -f interface/jlink.cfg -c "transport select swd" -c "jlink pid 0x1015" -c "set WORKAREASIZE 0" -f target/nrf51.cfg


UNAME := $(shell uname -s)
ifeq ($(UNAME),Linux)
    GNU_INSTALL_ROOT := ../../prebuilt/arm-none-eabi
else
    GNU_INSTALL_ROOT := ../prebuilt/osx-gcc-arm-none-eabi-4_9-2014q4
endif

GNU_VERSION := 4.9
GNU_PREFIX := arm-none-eabi
MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO :=
else
NO_ECHO := @
endif

# Toolchain commands
CC		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP	:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY	:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE	:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

C_SOURCE_FILES += \
$(NRFSDK_PATH)/components/libraries/button/app_button.c \
$(NRFSDK_PATH)/components/libraries/util/app_error.c \
$(NRFSDK_PATH)/components/libraries/timer/app_timer.c \
$(NRFSDK_PATH)/components/libraries/trace/app_trace.c \
$(NRFSDK_PATH)/components/libraries/util/nrf_assert.c \
$(NRFSDK_PATH)/components/libraries/uart/retarget.c \
$(NRFSDK_PATH)/components/drivers_nrf/uart/app_uart.c \
$(NRFSDK_PATH)/components/drivers_nrf/hal/nrf_delay.c \
$(NRFSDK_PATH)/components/drivers_nrf/common/nrf_drv_common.c \
$(NRFSDK_PATH)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
$(NRFSDK_PATH)/components/drivers_nrf/pstorage/pstorage.c \
$(NRFSDK_PATH)/examples/bsp/bsp.c \
$(NRFSDK_PATH)/examples/bsp/bsp_btn_ble.c \
./main.c \
$(NRFSDK_PATH)/components/ble/common/ble_advdata.c \
$(NRFSDK_PATH)/components/ble/ble_advertising/ble_advertising.c \
$(NRFSDK_PATH)/components/ble/common/ble_conn_params.c \
$(NRFSDK_PATH)/components/ble/common/ble_srv_common.c \
$(NRFSDK_PATH)/components/ble/device_manager/device_manager_peripheral.c \
$(NRFSDK_PATH)/components/toolchain/system_nrf51.c \
$(NRFSDK_PATH)/components/softdevice/common/softdevice_handler/softdevice_handler.c \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(NRFSDK_PATH)/components/toolchain/gcc/gcc_startup_nrf51.s

#includes common to all targets
INC_PATHS += -I$(NRFSDK_PATH)/components/toolchain/gcc
INC_PATHS += -I$(NRFSDK_PATH)/components/toolchain
INC_PATHS += -I$(NRFSDK_PATH)/components/ble/common
INC_PATHS += -I$(NRFSDK_PATH)/components/ble/ble_radio_notification
INC_PATHS += -I$(NRFSDK_PATH)/components/libraries/timer
INC_PATHS += -I$(NRFSDK_PATH)/components/drivers_nrf/hal

INC_PATHS += -I$(NRFSDK_PATH)/components/drivers_nrf/common

INC_PATHS += -I$(NRFSDK_PATH)/components/drivers_nrf/gpiote
INC_PATHS += -I$(NRFSDK_PATH)/components/drivers_nrf/pstorage
INC_PATHS += -I$(NRFSDK_PATH)/components/drivers_nrf/uart
INC_PATHS += -I$(NRFSDK_PATH)/components/softdevice/common/softdevice_handler
INC_PATHS += -I$(NRFSDK_PATH)/components/libraries/scheduler
INC_PATHS += -I$(NRFSDK_PATH)/components/libraries/util

INC_PATHS += -I$(NRFSDK_PATH)/components/libraries/button
INC_PATHS += -I$(NRFSDK_PATH)/components/libraries/trace
INC_PATHS += -I$(NRFSDK_PATH)/components/ble/device_manager
INC_PATHS += -I$(NRFSDK_PATH)/components/ble/ble_advertising
INC_PATHS += -I$(NRFSDK_PATH)/examples/bsp
INC_PATHS += -I$(NRFSDK_PATH)/components/softdevice/s110/headers
INC_PATHS += -I$(NRFSDK_PATH)/components/device
INC_PATHS += -I./

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets

CFLAGS  += -DNRF51
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DS110
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O3
CFLAGS += -mfloat-abi=soft
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -flto -fno-builtin
CFLAGS := $(OPTIONS) $(CFLAGS)

LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF51
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DS110
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DBOARD_PCA10000
#default target - first one defined
default: pca10028 

#target for printing all targets
help:
	@echo following targets are available:
	@echo	nrf51822_xxaa_s110


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)
OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

pca10028: TARGET=xxac
pca10028: BOARD = pca10028
pca10028: CFLAGS += -DBOARD_PCA10028
pca10028: CFLAGS += -DUART_ENABLE
pca10028: CFLAGS += -DLEDS_ENABLE
pca10028: nrf51822
	@echo Built xxac for target $(BOARD)
	@echo $(CFLAGS)

pca10000: TARGET=xxaa
pca10000: BOARD = pca10000
pca10000: CFLAGS += -DPCA10000
pca10000: nrf51822
	@echo Built xxaa for target $(BOARD)
	@echo $(CFLAGS)


nrf51822: LINKER_SCRIPT=./ble_app_template_gcc_nrf51.ld
nrf51822: export OUTPUT_FILENAME = $(BOARD)-$(TARGET)
nrf51822: clean $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize
	@echo "$(BOARD)-$(TARGET)" > $(OUTPUT_BINARY_DIRECTORY)/target.lst


## Create build directories
$(BUILD_DIRECTORIES):
	@$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<

# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out

finalize: echosize

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: OUTPUT_FILENAME= $(shell cat $(OUTPUT_BINARY_DIRECTORY)/target.lst)
flash: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
	$(OPENOCD) -c init -c halt -c "flash write_image erase $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex" -c reset -c shutdown

$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(BUILD_DIRECTORIES)
	$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

flash-softdevice: erase-all
ifndef SOFTDEVICE
	$(error "You need to set the SOFTDEVICE command-line parameter to a path (without spaces) to the softdevice hex-file")
endif
	$(OPENOCD) -c init -c halt -c "flash write_image $(SOFTDEVICE)" -c shutdown

erase-all:
	$(OPENOCD) -c init -c halt -c "nrf51 mass_erase" -c "reset"  -c shutdown
