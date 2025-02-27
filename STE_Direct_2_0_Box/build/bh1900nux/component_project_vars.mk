# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/bh1900nux
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/bh1900nux -lbh1900nux
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += bh1900nux
COMPONENT_LDFRAGMENTS += 
component-bh1900nux-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
