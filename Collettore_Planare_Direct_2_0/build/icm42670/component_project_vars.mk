# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/icm42670
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/icm42670 -licm42670
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += icm42670
COMPONENT_LDFRAGMENTS += 
component-icm42670-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
