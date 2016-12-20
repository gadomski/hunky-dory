project(hunky-dory CXX)
cmake_minimum_required(VERSION 3.2)

set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

find_package(Cpd REQUIRED)
find_package(PDAL REQUIRED)
set(DOCOPT_DIR vendor/docopt.cpp-af03fa0)

add_executable(hunky-dory
    src/bounds.cpp
    src/cpd.cpp
    src/create_reader.cpp
    src/main.cpp
    src/read.cpp
    src/write.cpp
    ${DOCOPT_DIR}/docopt.cpp
    )
target_link_libraries(hunky-dory PUBLIC Cpd::Library-C++ ${PDAL_LIBRARIES})
target_include_directories(hunky-dory PRIVATE ${DOCOPT_DIR})

install(TARGETS hunky-dory RUNTIME DESTINATION bin)
