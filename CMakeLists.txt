cmake_minimum_required(VERSION 3.21)
project(intan-headstages VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

#Create Library
set(Sources
        src/IntanCommon.cpp
        src/rhd2000registers.cpp
        src/rhs2000registers.cpp
)

set(Headers
        include/IntanCommon.h
        include/rhd2000registers.h
        include/rhs2000registers.h
)

set(Public_Headers
    include/rhd2000registers.h
    include/rhs2000registers.h
)

add_library(intan-headstages SHARED ${Sources} ${Headers})

target_include_directories(intan-headstages
                        PUBLIC
                        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

set_target_properties(intan-headstages PROPERTIES PUBLIC_HEADER "${Public_Headers}")

install(TARGETS intan-headstages
        EXPORT intan-headstagesTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        PUBLIC_HEADER DESTINATION include/intan-headstages # include/SomeLibrary
)

include(CMakePackageConfigHelpers)

#Create config File
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/intan-headstagesConfig.cmake"
  INSTALL_DESTINATION cmake
)

#install config file
install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/intan-headstagesConfig.cmake"
        DESTINATION cmake
)

# generate and install export file
install(EXPORT intan-headstagesTargets
        FILE intan-headstagesTargets.cmake
        NAMESPACE intan-headstages::
        DESTINATION cmake
)

#Generate the export targets for the build tree
export(EXPORT "intan-headstagesTargets"
    FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/intan-headstagesTargets.cmake"
    NAMESPACE intan-headstages::
)
