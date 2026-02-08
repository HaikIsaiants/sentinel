cmake_minimum_required(VERSION 3.25)

project(Sentinel VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
option(BUILD_TESTING "Build tests" ON)

find_package(Protobuf REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

include(FetchContent)

set(SENTINEL_BUILD_TESTING ${BUILD_TESTING})
set(BUILD_TESTING OFF)
set(BTCPP_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BTCPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BTCPP_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BTCPP_GROOT_INTERFACE OFF CACHE BOOL "" FORCE)
set(BTCPP_SQLITE_LOGGING OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    behaviortree_cpp
    URL https://github.com/BehaviorTree/BehaviorTree.CPP/archive/3ff6a32ba0497a08519c77a1436e3b81eff1bcd6.tar.gz
    URL_HASH SHA256=84486d12ce7249a611726dd4fbf4a43a9b2f3abf0b00448b6d910a3a54d9a281
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(behaviortree_cpp)
set(BUILD_TESTING ${SENTINEL_BUILD_TESTING})

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

add_library(sentinel_proto ${SENTINEL_PROTO_SOURCE} proto/src/framing.cpp)
target_include_directories(sentinel_proto PUBLIC ${SENTINEL_GENERATED} proto/include)
target_link_libraries(sentinel_proto PUBLIC protobuf::libprotobuf)

add_library(sentinel_planning planning/src/planner.cpp)
target_include_directories(sentinel_planning PUBLIC planning/include)
target_link_libraries(sentinel_planning PUBLIC sentinel_proto Eigen3::Eigen)

add_library(sentinel_core
    core/src/event_log.cpp
    core/src/hash.cpp
    core/src/network.cpp
    core/src/rng.cpp
    core/src/scenario.cpp
    core/src/simulator.cpp
)
target_include_directories(sentinel_core PUBLIC core/include)
target_link_libraries(sentinel_core PUBLIC sentinel_proto sentinel_planning)

add_executable(sentinel_sim sim/src/main.cpp)
target_link_libraries(sentinel_sim PRIVATE sentinel_core)

add_library(sentinel_agent_logic
    agent/src/allocation.cpp
    agent/src/autonomy.cpp
)
target_include_directories(sentinel_agent_logic PUBLIC agent/include)
target_link_libraries(
    sentinel_agent_logic
    PUBLIC sentinel_proto sentinel_planning BT::behaviortree_cpp
)

add_executable(sentinel_agent agent/src/main.cpp)
target_link_libraries(sentinel_agent PRIVATE sentinel_agent_logic)

if(UNIX)
    add_executable(sentinel_supervisor runner/src/main.cpp)
    target_link_libraries(sentinel_supervisor PRIVATE sentinel_proto)
endif()

include(CTest)
if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    include(GoogleTest)
    add_executable(sentinel_tests
        tests/allocation_test.cpp
        tests/autonomy_test.cpp
        tests/distributed_allocation_test.cpp
        tests/event_log_test.cpp
        tests/framing_test.cpp
        tests/mission_execution_test.cpp
        tests/network_test.cpp
        tests/planner_test.cpp
        tests/reservation_test.cpp
        tests/rng_test.cpp
        tests/route_coordination_test.cpp
        tests/scenario_test.cpp
        tests/simulator_test.cpp
        tests/trace_export_test.cpp
    )
    target_include_directories(sentinel_tests PRIVATE tests)
    target_link_libraries(sentinel_tests PRIVATE sentinel_core sentinel_agent_logic GTest::gtest_main)
    gtest_discover_tests(sentinel_tests)
    add_test(
        NAME agent_smoke
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/test_agent.py
            --agent $<TARGET_FILE:sentinel_agent>
    )
    if(UNIX)
        add_test(
            NAME runner_tests
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/test_runner.py
                --runner $<TARGET_FILE:sentinel_supervisor>
        )
    endif()
endif()
