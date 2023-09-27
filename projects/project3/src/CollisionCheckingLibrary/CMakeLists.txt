project(CollisionChecking)
cmake_minimum_required(VERSION 3.2.2)

add_definitions(-D UNIX)
add_definitions(-D LINUX)
add_definitions(-D _GNU_SOURCE)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ompl)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_AUTOMOC NO)
set(CMAKE_INCLUDE_CURRENT_DIR YES)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(${PROJECT_SOURCE_DIR})

set(SOURCES
    CollisionChecking.cpp
)

set(HEADERS
    CollisionChecking.h
    #include/RTP.h
)

set(srcs
    ${HEADERS}
    ${SOURCES}
)

add_library(collisionchecking ${srcs})
target_link_libraries(collisionchecking Eigen3::Eigen ${CMAKE_CURRENT_SOURCE_DIR})

#add_executable(ex2project3 ${srcs} src/Project3Exercise2.cpp)
#target_link_libraries(ex2project3 Eigen3::Eigen ${OMPL_LIBRARIES})

#add_executable(ex3project3 ${srcs} src/Project3Exercise3.cpp)
#target_link_libraries(ex3project3 Eigen3::Eigen ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES})
