cmake_minimum_required(VERSION 3.20)
project(PCloudExtract)

enable_language(CUDA)

set(CMAKE_CXX_STANDARD 17)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
include_directories(/usr/include/python3.8)
link_directories(${PCL_LIBRARY_DIRS})

add_subdirectory(extern/pybind11)

MESSAGE( [MAIN] "Found pybind11 v${pybind11_VERSION}: ${pybind11_INCLUDE_DIRS}")
MESSAGE( [Main] " pybind11_INCLUDE_DIRS = ${pybind11_INCLUDE_DIRS}")
MESSAGE( [Main] " pybind11_LIBRARIES = ${pybind11_LIBRARIES}")

add_definitions(${PCL_DEFINITIONS})

file(GLOB project_headers "${CMAKE_SOURCE_DIR}/include/*.h")
file(GLOB project_src "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB project_cu "${CMAKE_SOURCE_DIR}/cuda/*.cu")

add_library(cudaHelper STATIC ${project_cu} )

set_target_properties(cudaHelper PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

set_target_properties(cudaHelper PROPERTIES CUDA_ARCHITECTURES 35 50 61)

add_executable(PCloudExtract src/main.cpp src/BagFile.cpp include/BagFile.h src/helper.cpp include/helper.h include/pclhelper.h)
target_link_libraries(PCloudExtract PRIVATE
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        cudaHelper
        realsense2
        pybind11::embed
        )
