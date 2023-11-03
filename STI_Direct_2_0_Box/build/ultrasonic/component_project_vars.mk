# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/ultrasonic
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/ultrasonic -lultrasonic
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += ultrasonic
COMPONENT_LDFRAGMENTS += 
component-ultrasonic-build: component-driver-build component-freertos-build component-esp_idf_lib_helpers-build
