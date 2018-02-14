
# YARP needs CMake 3.0 or greater
cmake_minimum_required(VERSION 3.0)

# produce the cmake var PROJECT_NAME
project(EEForcesReader)

# find YARP
find_package(YARP REQUIRED)
#find_package(ICUBcontrib REQUIRED)

# extend the current search path used by cmake to load helpers
#list(APPEND CMAKE_MODULE_PATH $(ICUBCONTRIB_MODULE_PATH))

# helpers defining certain macros (e.g. "yarp install"))
#include(ICUBcontribHelpers)

# specify to install in $ICUBcontrib_DIR/bin
#icubcontrib_set_default_prefix()

# extend the current search path used by cmake to load helpers
#list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

# building
#include_directories(${YARP_INCLUDE_DIRS})
add_executable(EEForcesReader ${CMAKE_SOURCE_DIR}/EEForcesReader.cpp)
target_link_libraries(EEForcesReader ${YARP_LIBRARIES})
install(TARGETS EEForcesReader DESTINATION bin)

#icubcontrib_add_uninstall_target()
