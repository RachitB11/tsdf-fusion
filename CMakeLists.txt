CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(tsdf-fusion)

FIND_PACKAGE(PCL REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(CUDA REQUIRED)

set (EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/")

############# RESOLVE VTK ISSUES ON UBUNTU 16.04 WITH CUDA 9 ###################

if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
set(vtk_flags)
foreach(it ${dir_defs})
    if(it MATCHES "vtk*")
    list(APPEND vtk_flags ${it})
    endif()
endforeach()

foreach(d ${vtk_flags})
    remove_definitions(-D${d})
endforeach()

################################################################################


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_61,code=sm_61;)

# add -Wextra compiler flag for gcc compilations
if (UNIX)
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-Xcompiler -Wextra")
endif (UNIX)

ADD_LIBRARY(pcd_utils SHARED pcd_utils/pcd_utils.cpp)
TARGET_INCLUDE_DIRECTORIES(pcd_utils PUBLIC ${PCL_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(pcd_utils ${PCL_LIBRARIES})

CUDA_ADD_LIBRARY(utils SHARED utils/cuda_utils.cpp utils/utils.cpp)
TARGET_INCLUDE_DIRECTORIES(utils PUBLIC ${OpenCV_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(utils ${OpenCV_LIBS})

CUDA_ADD_LIBRARY(tsdf SHARED tsdf/tsdf.cu)
TARGET_LINK_LIBRARIES(tsdf utils)

ADD_EXECUTABLE(main main.cpp)
TARGET_LINK_LIBRARIES(main pcd_utils tsdf)
