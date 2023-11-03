# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/dht
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/dht -ldht
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += dht
COMPONENT_LDFRAGMENTS += 
component-dht-build: component-driver-build component-freertos-build component-log-build component-esp_idf_lib_helpers-build
