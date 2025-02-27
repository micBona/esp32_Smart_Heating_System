# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/led_strip_spi
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/led_strip_spi -lled_strip_spi
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += led_strip_spi
COMPONENT_LDFRAGMENTS += 
component-led_strip_spi-build: component-log-build component-color-build component-esp_idf_lib_helpers-build
