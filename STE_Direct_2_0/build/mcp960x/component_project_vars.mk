# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/mcp960x
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/mcp960x -lmcp960x
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += mcp960x
COMPONENT_LDFRAGMENTS += 
component-mcp960x-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
