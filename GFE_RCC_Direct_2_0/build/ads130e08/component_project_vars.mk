# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/ads130e08
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/ads130e08 -lads130e08
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += ads130e08
COMPONENT_LDFRAGMENTS += 
component-ads130e08-build: component-driver-build component-log-build
