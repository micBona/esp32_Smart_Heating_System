# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/sgp40
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/sgp40 -lsgp40
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += sgp40
COMPONENT_LDFRAGMENTS += 
component-sgp40-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
