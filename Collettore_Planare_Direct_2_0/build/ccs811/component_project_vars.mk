# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/ccs811
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/ccs811 -lccs811
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += ccs811
COMPONENT_LDFRAGMENTS += 
component-ccs811-build: component-i2cdev-build component-log-build
