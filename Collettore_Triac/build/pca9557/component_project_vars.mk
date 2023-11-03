# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/pca9557
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/pca9557 -lpca9557
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += pca9557
COMPONENT_LDFRAGMENTS += 
component-pca9557-build: component-i2cdev-build component-log-build component-esp_idf_lib_helpers-build
