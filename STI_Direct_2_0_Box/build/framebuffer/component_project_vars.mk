# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)-lib/components/framebuffer
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/framebuffer -lframebuffer
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += framebuffer
COMPONENT_LDFRAGMENTS += 
component-framebuffer-build: component-log-build component-color-build component-lib8tion-build component-noise-build
