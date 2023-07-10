cmake_minimum_required(VERSION 3.12)
project(IntentionModel LANGUAGES CXX)

# Set the C++ standard to use
set(CMAKE_CXX_STANDARD 20)

# Add your source files
set(SOURCES
    src/insert_ais.cpp
    src/generate_intention_model.cpp
)

include_directories(inc external/Eigen external/smile)

# Find packages
find_package(Eigen3 REQUIRED)

# Executable for insert_ais
add_executable(insert_ais src/insert_ais.cpp)
target_link_libraries(insert_ais PRIVATE Eigen3::Eigen ${CMAKE_SOURCE_DIR}/external/smile/libsmile.a)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -O3")

# Executable for generate_intention_model
add_executable(generate_intention_model src/generate_intention_model.cpp)
target_link_libraries(generate_intention_model PRIVATE Eigen3::Eigen ${CMAKE_SOURCE_DIR}/external/smile/libsmile.a)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -O3")

# Set the appropriate extension for the module files based on the platform
if(WIN32)
    set_target_properties(insert_ais PROPERTIES SUFFIX ".pyd")
    set_target_properties(generate_intention_model PROPERTIES SUFFIX ".pyd")
else()
    set_target_properties(insert_ais PROPERTIES PREFIX "")
    set_target_properties(generate_intention_model PROPERTIES PREFIX "")
endif()
