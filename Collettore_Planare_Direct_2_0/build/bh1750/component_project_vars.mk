# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/bh1750
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/bh1750 -lbh1750
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += bh1750
COMPONENT_LDFRAGMENTS += 
component-bh1750-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
