# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/tca9548
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/tca9548 -ltca9548
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += tca9548
COMPONENT_LDFRAGMENTS += 
component-tca9548-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
