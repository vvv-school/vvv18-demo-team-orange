# requires minimum cmake version
cmake_minimum_required(VERSION 2.8)

# produce the cmake var PROJECT_NAME
project(objectRecognizer)

# mandatory use of these packages
find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(Caffe REQUIRED)
find_package(OpenCV REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# helpers defining certain macros (e.g. "yarp_install")
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# verify Caffe version
get_target_property(CAFFE_SONAME caffe IMPORTED_SONAME_RELEASE)
if("${CAFFE_SONAME}" MATCHES "libcaffe.so.([0-9]+).([0-9]+).([0-9]+)(-(.+))?")
    set(CAFFE_MAJOR "${CMAKE_MATCH_1}")
    set(CAFFE_MINOR "${CMAKE_MATCH_2}")
    set(CAFFE_PATCH "${CMAKE_MATCH_3}")
    set(CAFFE_PATCH2 "${CMAKE_MATCH_5}")
    set(CAFFE_VERSION_STRING "${CAFFE_MAJOR}.${CAFFE_MINOR}.${CAFFE_PATCH}")
    if(NOT "${CAFFE_PATCH2}" STREQUAL "")
        set(CAFFE_VERSION_STRING "${CAFFE_VERSION_STRING}-${CAFFE_PATCH2}")
    endif()
else()
    set(CAFFE_MAJOR 0)
    set(CAFFE_MINOR 0)
    set(CAFFE_PATCH 0)
    set(CAFFE_PATCH2 0)
    set(CAFFE_VERSION_STRING unknown)
endif()
math(EXPR CAFFE_VERSION "${CAFFE_MAJOR}*10000 + ${CAFFE_MINOR}*100 + ${CAFFE_PATCH}")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/caffe-version.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/caffe-version.h
               @ONLY)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
add_definitions(${Caffe_DEFINITIONS})    # ex. -DCPU_ONLY

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Caffe_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/include)

set(folder_source ${PROJECT_NAME}/src/main.cpp)
set(folder_header ${PROJECT_NAME}/include/CaffeWrapper.hpp
                  ${PROJECT_NAME}/include/definitions.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Caffe_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/app/conf/*.ini ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/app/conf/*.prototxt)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/app/conf/*.ini.template ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/app/conf/*.prototxt.template)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/app/scripts/*.xml.template ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/app/scripts/*.xml)

yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()
