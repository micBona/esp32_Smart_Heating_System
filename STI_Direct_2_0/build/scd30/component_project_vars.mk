# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/scd30
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/scd30 -lscd30
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += scd30
COMPONENT_LDFRAGMENTS += 
component-scd30-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
