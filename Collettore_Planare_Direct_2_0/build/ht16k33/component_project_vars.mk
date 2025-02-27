# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/ht16k33
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/ht16k33 -lht16k33
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += ht16k33
COMPONENT_LDFRAGMENTS += 
component-ht16k33-build: component-i2cdev-build component-log-build
