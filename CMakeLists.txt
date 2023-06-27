project(stereolib)
cmake_minimum_required(VERSION 3.4)

SET(CMAKE_CXX_STANDARD 14)
#SET(WITH_ORB_SLAM3 TRUE)

set(ENV{stereoLib_DIR} ${CMAKE_CURRENT_SOURCE_DIR})


find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (jsoncpp REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(Gflags REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io filters visualization)



add_definitions(${PCL_DEFINITIONS})


#--- Hack for Mac brew
link_directories(/opt/dep/local/lib)

#---- 
message(STATUS "utLib_DIR=$ENV{utLib_DIR}")
message(STATUS "vsnLib_DIR=$ENV{vsnLib_DIR}")

include_directories(
    inc 
    inc_local
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    $ENV{utLib_DIR}/inc
    $ENV{vsnLib_DIR}/inc
    ${JsonCpp_INCLUDE_DIR}
    /opt/dep/local/include
    /usr/include/jsoncpp
    )


#------ ORB-SLAM3 (option) -----------
if(WITH_ORB_SLAM3)
    set(ORB_SLAM3_DIR dep/ORB_SLAM3t)
    find_package(Pangolin REQUIRED)

    include_directories(
        ${ORB_SLAM3_DIR}
        ${ORB_SLAM3_DIR}/include
        ${ORB_SLAM3_DIR}/include/CameraModels
        ${ORB_SLAM3_DIR}/Thirdparty/Sophus
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        )

    add_definitions(-DWITH_ORB_SLAM3)
    link_directories(${ORB_SLAM3_DIR}/lib)
    set(EXT_LINK "ORB_SLAM3")
    add_subdirectory(${ORB_SLAM3_DIR})
    message(STATUS "ORB_SLAM3_DIR = ${ORB_SLAM3_DIR}")
endif()


#----- stereoLib    
FILE(GLOB  SRC_CORE src/*.cpp)
add_library(stereoLib ${SRC_CORE} )
target_link_libraries(stereoLib   
    ${EXT_LINK}
    vsnLib 
    -fPIC
    pthread
#    -lGL
#    ${OpenCV_LIBRARIES}
#    ${Eigen3_LIBRARY}    
    )

    
FILE(GLOB  SRC_CMD app/stereocmd/*.cpp)
add_executable(stereocmd ${SRC_CMD})
target_link_libraries(
    stereocmd 
    stereoLib
    
    )

