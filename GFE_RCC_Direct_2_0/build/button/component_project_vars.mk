# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/button
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/button -lbutton
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += button
COMPONENT_LDFRAGMENTS += 
component-button-build: component-driver-build
