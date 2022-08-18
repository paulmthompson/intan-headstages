cmake_minimum_required(VERSION 3.21)
project(intan-headstages VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
    include/IntanCommon.h
    include/rhd2000registers.h
    include/rhs2000registers.h
)

add_library(intan-headstages SHARED ${Sources} ${Headers})

target_include_directories(intan-headstages
                        PUBLIC
                        PRIVATE
                        include)

