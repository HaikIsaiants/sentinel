cmake_minimum_required(VERSION 3.25)

project(Sentinel VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

option(BUILD_TESTING "Build tests" ON)

find_package(Protobuf REQUIRED)

set(SENTINEL_PROTO ${CMAKE_SOURCE_DIR}/proto/sentinel/v1/sentinel.proto)
set(SENTINEL_GENERATED ${CMAKE_BINARY_DIR}/generated)
set(SENTINEL_PROTO_SOURCE ${SENTINEL_GENERATED}/sentinel/v1/sentinel.pb.cc)
set(SENTINEL_PROTO_HEADER ${SENTINEL_GENERATED}/sentinel/v1/sentinel.pb.h)

file(MAKE_DIRECTORY ${SENTINEL_GENERATED})

add_custom_command(
    OUTPUT ${SENTINEL_PROTO_SOURCE} ${SENTINEL_PROTO_HEADER}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        --proto_path=${CMAKE_SOURCE_DIR}/proto
        --cpp_out=${SENTINEL_GENERATED}
        ${SENTINEL_PROTO}
    DEPENDS ${SENTINEL_PROTO}
    VERBATIM
)

add_library(sentinel_proto
    ${SENTINEL_PROTO_SOURCE}
    proto/src/framing.cpp
)
target_include_directories(sentinel_proto PUBLIC ${SENTINEL_GENERATED} proto/include)
target_link_libraries(sentinel_proto PUBLIC protobuf::libprotobuf)

include(CTest)
if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    include(GoogleTest)
    add_executable(sentinel_tests tests/framing_test.cpp)
    target_link_libraries(sentinel_tests PRIVATE sentinel_proto GTest::gtest_main)
    gtest_discover_tests(sentinel_tests)
endif()
