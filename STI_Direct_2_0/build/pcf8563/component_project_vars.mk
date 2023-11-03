# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/pcf8563
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/pcf8563 -lpcf8563
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += pcf8563
COMPONENT_LDFRAGMENTS += 
component-pcf8563-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
