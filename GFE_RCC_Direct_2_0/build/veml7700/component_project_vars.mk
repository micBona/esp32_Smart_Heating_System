# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/veml7700
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/veml7700 -lveml7700
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += veml7700
COMPONENT_LDFRAGMENTS += 
component-veml7700-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
