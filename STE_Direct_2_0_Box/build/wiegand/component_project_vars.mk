# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/wiegand
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/wiegand -lwiegand
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += wiegand
COMPONENT_LDFRAGMENTS += 
component-wiegand-build: component-driver-build component-log-build component-esp_idf_lib_helpers-build
