cmake_minimum_required(VERSION 3.9)
project(libpcc)

set(CMAKE_CXX_STANDARD 11)

find_package(PkgConfig REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

pkg_check_modules(ZMQ REQUIRED libzmq)
pkg_check_modules(ZLIB REQUIRED zlib)

set(ALL_LIBS
        ${ZMQ_LIBRARIES}
        ${ZLIB_LIBRARIES})

include_directories(${ZMQ_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS})

add_executable(libpcc
        examples/test_cmdp.cpp
        include/CMDParser.hpp
        include/Encoder.hpp
        include/PointCloud.hpp
        src/CMDParser.cpp
        src/Encoder.cpp
        include/Measure.hpp
        src/Measure.cpp
        include/BitVec.hpp
        include/PointCloudGrid.hpp
        include/PointCloudGridEncoder.hpp
        src/PointCloudGridEncoder.cpp
        include/BitValue.hpp
        include/BitVecArray.hpp
        src/BitVecArray.cpp
        src/BitVec.cpp
        include/Vec.hpp
        include/BoundingBox.hpp
        src/BinaryFile.cpp
        include/BinaryFile.hpp
        include/UncompressedVoxel.hpp)

target_link_libraries(libpcc ${ALL_LIBS})
