# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/mcp342x
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/mcp342x -lmcp342x
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += mcp342x
COMPONENT_LDFRAGMENTS += 
component-mcp342x-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
