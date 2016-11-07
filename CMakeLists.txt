cmake_minimum_required(VERSION 3.2)
project(hunky-dory LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 11)

configure_file(src/hunky_dory.hpp.in ${PROJECT_BINARY_DIR}/hunky_dory.hpp)

find_package(Cpd REQUIRED)
find_package(Entwine REQUIRED)
find_package(PCL REQUIRED)
find_package(PDAL REQUIRED)

add_executable(hunky-dory
    src/bounds.cpp
    src/chip.cpp
    src/cpd.cpp
    src/main.cpp
    src/icp.cpp
    src/utils.cpp
    vendor/docopt.cpp/docopt.cpp
    )
target_include_directories(hunky-dory
    PRIVATE
    ${PROJECT_BINARY_DIR}
    vendor/docopt.cpp
    ${PDAL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
target_link_libraries(hunky-dory
    PRIVATE
    Cpd::Library-C++
    Entwine::entwine
    pdalcpp
    ${PCL_LIBRARIES}
    )
install(TARGETS hunky-dory
    RUNTIME DESTINATION bin
    )
