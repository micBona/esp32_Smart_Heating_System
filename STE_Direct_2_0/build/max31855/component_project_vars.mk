# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/max31855
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/max31855 -lmax31855
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += max31855
COMPONENT_LDFRAGMENTS += 
component-max31855-build: component-driver-build component-log-build
