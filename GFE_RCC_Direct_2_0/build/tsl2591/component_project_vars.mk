# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/tsl2591
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/tsl2591 -ltsl2591
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += tsl2591
COMPONENT_LDFRAGMENTS += 
component-tsl2591-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
