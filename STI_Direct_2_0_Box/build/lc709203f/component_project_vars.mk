# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/lc709203f
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/lc709203f -llc709203f
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += lc709203f
COMPONENT_LDFRAGMENTS += 
component-lc709203f-build: component-i2cdev-build component-esp_idf_lib_helpers-build
