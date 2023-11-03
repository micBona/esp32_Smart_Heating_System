# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/hts221
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/hts221 -lhts221
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += hts221
COMPONENT_LDFRAGMENTS += 
component-hts221-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
