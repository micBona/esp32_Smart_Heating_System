# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/tsl4531
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/tsl4531 -ltsl4531
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += tsl4531
COMPONENT_LDFRAGMENTS += 
component-tsl4531-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
