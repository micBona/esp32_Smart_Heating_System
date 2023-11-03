# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/sht4x
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/sht4x -lsht4x
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += sht4x
COMPONENT_LDFRAGMENTS += 
component-sht4x-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
