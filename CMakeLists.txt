cmake_minimum_required(VERSION 3.10)

project(XRobot)

set(CMAKE_CXX_STANDARD 17)

include_directories(
    include
    /usr/include/opencascade
)

find_package(tinyxml2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(
    xrobot
    src/main.cpp
    src/parser.cpp
    src/step_loader.cpp
    src/model.cpp
    src/gz_spawner.cpp
)

target_link_libraries(
    xrobot

    tinyxml2::tinyxml2

    TKSTEP
    TKSTEPBase
    TKSTEPAttr
    TKXSBase

    TKernel

    TKCAF
    TKLCAF
    TKXCAF
    TKXDESTEP

    TKMath
    TKBRep
    TKG3d
    TKG2d

    TKMesh
    TKSTL

    TKTopAlgo

    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
)