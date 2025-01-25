# Define Spectron SRC dirs
COMMON_PATH = $(SOURCE_PATH)/../common

# Get Spectron sources
COMMON_CPPSRC = $(patsubst $(SOURCE_PATH)/%,%,$(wildcard $(COMMON_PATH)/*.cpp))

# add user sources to include path
INCLUDE_DIRS += $(SOURCE_PATH)
INCLUDE_DIRS += $(COMMON_PATH)

# add CPP files
CPPSRC += $(call target_files,$(USRSRC_SLASH),*.cpp)
CPPSRC += $(COMMON_CPPSRC)

APPSOURCES = $(call target_files,$(USRSRC_SLASH),*.cpp)
APPSOURCES += $(COMMON_CPPSRC)

