# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/scd4x
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/scd4x -lscd4x
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += scd4x
COMPONENT_LDFRAGMENTS += 
component-scd4x-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
