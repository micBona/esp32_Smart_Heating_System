# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/max31865
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/max31865 -lmax31865
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += max31865
COMPONENT_LDFRAGMENTS += 
component-max31865-build: component-driver-build component-log-build
