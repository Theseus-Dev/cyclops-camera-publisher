cmake_minimum_required(VERSION 3.16)

project(cyclops_external_camera_ecal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(eCAL REQUIRED)

if(NOT DEFINED CapnProto_DIR AND EXISTS "/usr/local/lib/cmake/CapnProto/CapnProtoConfig.cmake")
    set(CapnProto_DIR "/usr/local/lib/cmake/CapnProto" CACHE PATH "Path to CapnProto CMake config")
endif()
find_package(CapnProto REQUIRED)
find_package(OpenCV REQUIRED)

find_program(CYCLOPS_CAPNP_EXECUTABLE NAMES capnp PATHS /usr/local/bin /usr/bin /bin NO_DEFAULT_PATH)
find_program(CYCLOPS_CAPNPC_CXX_EXECUTABLE NAMES capnpc-c++ PATHS /usr/local/bin /usr/bin /bin NO_DEFAULT_PATH)
if(NOT CYCLOPS_CAPNP_EXECUTABLE)
    find_program(CYCLOPS_CAPNP_EXECUTABLE capnp)
endif()
if(NOT CYCLOPS_CAPNPC_CXX_EXECUTABLE)
    find_program(CYCLOPS_CAPNPC_CXX_EXECUTABLE capnpc-c++)
endif()
if(NOT CYCLOPS_CAPNP_EXECUTABLE)
    message(FATAL_ERROR "capnp executable not found")
endif()
if(NOT CYCLOPS_CAPNPC_CXX_EXECUTABLE)
    message(FATAL_ERROR "capnpc-c++ executable not found")
endif()

set(CYCLOPS_CAPNP_SCHEMA_DIR "${CMAKE_CURRENT_LIST_DIR}/capnp" CACHE PATH
    "Directory containing Cyclops camera .capnp schemas such as image.capnp")

set(CYCLOPS_CAPNP_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/capnp")
file(MAKE_DIRECTORY "${CYCLOPS_CAPNP_GENERATED_DIR}")

set(CYCLOPS_CAMERA_CAPNP_SCHEMAS
    image.capnp
    header.capnp
    cameraintrinsic.capnp
    sensorextrinsic.capnp
    se3.capnp
    so3.capnp
    vector3.capnp
)

set(CYCLOPS_CAMERA_CAPNP_FILES "")
set(CYCLOPS_CAMERA_CAPNP_SRCS "")
set(CYCLOPS_CAMERA_CAPNP_HDRS "")

foreach(schema_name IN LISTS CYCLOPS_CAMERA_CAPNP_SCHEMAS)
    list(APPEND CYCLOPS_CAMERA_CAPNP_FILES "${CYCLOPS_CAPNP_SCHEMA_DIR}/${schema_name}")
    string(REPLACE ".capnp" "" schema_base "${schema_name}")
    list(APPEND CYCLOPS_CAMERA_CAPNP_SRCS "${CYCLOPS_CAPNP_GENERATED_DIR}/${schema_base}.capnp.c++")
    list(APPEND CYCLOPS_CAMERA_CAPNP_HDRS "${CYCLOPS_CAPNP_GENERATED_DIR}/${schema_base}.capnp.h")
endforeach()

add_custom_command(
    OUTPUT ${CYCLOPS_CAMERA_CAPNP_SRCS} ${CYCLOPS_CAMERA_CAPNP_HDRS}
    COMMAND ${CYCLOPS_CAPNP_EXECUTABLE}
        compile
        -o${CYCLOPS_CAPNPC_CXX_EXECUTABLE}:${CYCLOPS_CAPNP_GENERATED_DIR}
        --src-prefix=${CYCLOPS_CAPNP_SCHEMA_DIR}
        ${CYCLOPS_CAMERA_CAPNP_FILES}
    DEPENDS ${CYCLOPS_CAMERA_CAPNP_FILES}
    VERBATIM
)

add_library(cyclops_camera_ecal_cpp STATIC
    cyclops_camera_ecal.cpp
    ${CYCLOPS_CAMERA_CAPNP_SRCS}
)

target_include_directories(cyclops_camera_ecal_cpp
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CYCLOPS_CAPNP_GENERATED_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(cyclops_camera_ecal_cpp
    PUBLIC
        eCAL::core
        CapnProto::capnp
        CapnProto::kj
        Threads::Threads
        ${OpenCV_LIBS}
)

# ─────────────────────────────────────────────────────────────────────────────
# Runnable example: USB camera publisher
# Mirrors `example_usb_camera.py`. Builds a thin wrapper around the helper
# that opens cv::VideoCapture and publishes on S1/cama.
# ─────────────────────────────────────────────────────────────────────────────
find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
    add_executable(example_usb_camera example_usb_camera.cpp)
    target_link_libraries(example_usb_camera
        PRIVATE
            cyclops_camera_ecal_cpp
            yaml-cpp
    )
else()
    message(STATUS "yaml-cpp not found; skipping example_usb_camera target. "
                   "Install libyaml-cpp-dev to enable it.")
endif()
