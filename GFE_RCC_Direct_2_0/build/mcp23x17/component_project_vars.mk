# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/mcp23x17
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/mcp23x17 -lmcp23x17
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += mcp23x17
COMPONENT_LDFRAGMENTS += 
component-mcp23x17-build: component-driver-build component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
