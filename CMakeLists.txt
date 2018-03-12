cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to Release")
#set(CMAKE_BUILD_TYPE Debug CACHE STRING "set build type to debug")
Project(rs_tools)
SET(CMAKE_C_COMPILER clang++) #g++
add_compile_options(-std=c++11)
add_definitions(-w)
#set(CMAKE_BUILD_TYPE debug  GCACHE STRING "set build type to debug")
# PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV 2 REQUIRED)

include_directories(include)
include_directories("/usr/include/eigen3")
# Head files
#set(HEAD_FILES
#    include/map_feature.h
#    include/plane_extract.h
#  )

#set(SOURCE_FILES
#    src/keypoints.cpp
#   )

set( CMAKE_INSTALL_PREFIX "/usr/local/" )
set(VERSION "1.0.1")
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_PACKAGE_NAME "rs_tools")
set(CPACK_GENERATOR "ZIP;TGZ")
set(CPACK_PACKAGE_RELEASE 1)
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CPACK_OUTPUT_FILE_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/installer")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_PACKAGE_RELEASE}.${CMAKE_SYSTEM_PROCESSOR}")
include(CPack)

# shared lib
#add_library(rs_feature SHARED ${HEAD_FILES}  ${SOURCE_FILES})
#target_link_libraries(rs_feature ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES} )


# Executable


#------- group file--------
add_executable(rs_pcd2xyz  tools/pcd2xyz.cpp )
target_link_libraries(rs_pcd2xyz  ${PCL_LIBRARIES})

add_executable(rs_pcd_split  tools/pcd_split.cpp )
target_link_libraries(rs_pcd_split  ${PCL_LIBRARIES})

add_executable(rs_pcd_merge  tools/pcd_merge.cpp )
target_link_libraries(rs_pcd_merge  ${PCL_LIBRARIES})

#------- group feature--------
add_executable(rs_normal_estimation  tools/normal_estimation.cpp )
target_link_libraries(rs_normal_estimation  ${PCL_LIBRARIES})

add_executable(qua_euler_demo  tools/qua_euler.cpp )
target_link_libraries(qua_euler_demo  ${PCL_LIBRARIES})


add_executable(rs_pca_estimation  tools/pca_estimation.cpp )
target_link_libraries(rs_pca_estimation  ${PCL_LIBRARIES})

#---------------gtest-----------------
set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} /usr /usr/local)
enable_testing()
find_path(GTEST_INCLUDE_DIR gtest/gtest.h)
add_executable(rs_test test/test.cpp ${GTEST_INCLUDE_DIR} )
target_link_libraries(rs_test rs_proprogress /usr/local/lib/libgtest.a -pthread )
#------------install shell-------------

install (TARGETS
				 rs_pcd2xyz
				 rs_pcd_split
				 rs_pcd_merge
				 rs_normal_estimation
				 rs_pca_estimation
	RUNTIME DESTINATION bin )

#install(FILES ${HEAD_FILES} DESTINATION include/pcl_tools)
















