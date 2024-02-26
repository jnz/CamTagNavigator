cmake_minimum_required(VERSION 2.6)
project(camtagnavigator)

if(WIN32)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_definitions(-D_WINSOCK_DEPRECATED_NO_WARNINGS)
else(WIN32)
    add_definitions(-O2 -std=c++14 -Wno-deprecated-declarations)
endif(WIN32)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(./)
include_directories(./src/)
include_directories(./src/apriltags)
set (REQUIRED_LIBRARIES ${OpenCV_LIBS})

set(APRILTAGS_SRC src/apriltags/Edge.cc
                  src/apriltags/FloatImage.cc
                  src/apriltags/Gaussian.cc
                  src/apriltags/GLine2D.cc
                  src/apriltags/GLineSegment2D.cc
                  src/apriltags/GrayModel.cc
                  src/apriltags/Homography33.cc
                  src/apriltags/MathUtil.cc
                  src/apriltags/Quad.cc
                  src/apriltags/Segment.cc
                  src/apriltags/TagDetection.cc
                  src/apriltags/TagDetector.cc
                  src/apriltags/TagFamily.cc
                  src/apriltags/UnionFindSimple.cc)

SOURCE_GROUP(apriltags FILES ${APRILTAGS_SRC})

add_executable(camtagnav src/main.cpp src/camtagnav.cpp ${APRILTAGS_SRC})

target_link_libraries(camtagnav ${OpenCV_LIBS})

