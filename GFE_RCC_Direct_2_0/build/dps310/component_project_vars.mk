# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/dps310/include
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/dps310 -ldps310
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += dps310
COMPONENT_LDFRAGMENTS += 
component-dps310-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
