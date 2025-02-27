# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/ds3502
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/ds3502 -lds3502
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += ds3502
COMPONENT_LDFRAGMENTS += 
component-ds3502-build: component-i2cdev-build component-esp_idf_lib_helpers-build
