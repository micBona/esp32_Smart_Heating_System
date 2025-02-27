# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/mcp9808
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/mcp9808 -lmcp9808
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += mcp9808
COMPONENT_LDFRAGMENTS += 
component-mcp9808-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
