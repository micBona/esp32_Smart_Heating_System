# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/sts21
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/sts21 -lsts21
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += sts21
COMPONENT_LDFRAGMENTS += 
component-sts21-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
