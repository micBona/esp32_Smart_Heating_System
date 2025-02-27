# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/rda5807m
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/rda5807m -lrda5807m
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += rda5807m
COMPONENT_LDFRAGMENTS += 
component-rda5807m-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
