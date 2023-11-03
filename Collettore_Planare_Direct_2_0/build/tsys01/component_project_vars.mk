# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/tsys01
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/tsys01 -ltsys01
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += tsys01
COMPONENT_LDFRAGMENTS += 
component-tsys01-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
