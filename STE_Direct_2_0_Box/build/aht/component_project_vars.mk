# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/aht
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/aht -laht
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += aht
COMPONENT_LDFRAGMENTS += 
component-aht-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
