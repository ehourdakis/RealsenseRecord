project(BasicCMake)
cmake_minimum_required(VERSION 2.8)

SET (CMAKE_BUILD_TYPE RELEASE CACHE STRING "Debug configuration" FORCE)
set (CMAKE_CXX_STANDARD 14) #This is the std=c++11 standard for easy logging
set (CMAKE_CXX_FLAGS "-g")  #-Wall #Dont include all warnings ..
# set	(EASYLOGGING /files/Projects/UnderDev/roboslam/libraries/easyloggingpp )
# include_directories	(${EASYLOGGING}/include)
# link_directories	(${EASYLOGGING}/lib)
# set 	(EXTRA_LIBS ${EXTRA_LIBS} easyloggingpp)
find_path(EIGEN3_INCLUDE_DIR    NAMES signature_of_eigen3_matrix_library
                                PATHS
                                ${CMAKE_INSTALL_PREFIX}/include
                                ${KDE4_INCLUDE_DIR}
                                PATH_SUFFIXES eigen3 eigen)
    
include_directories		        (${EIGEN3_INCLUDE_DIR})
include_directories		        (/usr/local/include)
link_directories		        (/usr/local/lib/)

set					            (PCL_DIR "/files/Projects/Libraries/pcls/pcl/install/share/pcl-1.9/")
find_package		            (PCL REQUIRED COMPONENTS common io registration visualization filters features keypoints segmentation gpu gpu_features gpu_kinfu gpu_kinfu_large_scale gpu_octree)
include_directories	            (${PCL_INCLUDE_DIRS})
link_directories	            (${PCL_LIBRARY_DIRS})
add_definitions		            (${PCL_DEFINITIONS})
add_definitions 	            (-DWITH_PCL)
set                             (EXTRA_LIBS ${EXTRA_LIBS} ${PCL_LIBRARIES})

find_package        		    (Boost COMPONENTS system program_options iostreams filesystem python thread coroutine context REQUIRED)  #last three for ros interrface
include_directories		        (${Boost_INCLUDE_DIRS})
link_directories		        (${Boost_LIBRARY_DIRS})
set 				            (EXTRA_LIBS ${EXTRA_LIBS} ${Boost_LIBRARIES})
message				            (${Boost_LIBRARY_DIRS} ${Boost_INCLUDE_DIRS}  ${Boost_LIBRARIES})

include                 		(/files/Projects/Libraries/opencv/install/lib/cmake/opencv4/OpenCVConfig.cmake )
include_directories     		(${OpenCV_INCLUDE_DIRS})
link_directories		        (${OpenCV_INSTALL_PATH}/lib)
set                     		(EXTRA_LIBS ${EXTRA_LIBS} ${OpenCV_LIBS}) 

set                             (realsense2_DIR /files/Projects/Libraries/intelrealsense/librealsense/install/lib/cmake/realsense2)
find_package                    (realsense2 REQUIRED)
include_directories             (${realsense2_INCLUDE_DIRS})
include_directories             (/files/Projects/Libraries/intelrealsense/librealsense/install/include)
link_directories                (/files/Projects/Libraries/intelrealsense/librealsense/install/lib)
set 				            (EXTRA_LIBS ${EXTRA_LIBS} ${realsense2_LIBRARY})

include_directories             (/files/Projects/Libraries/intelrealsense/librealsense/third-party/realsense-file/lz4)

find_library(GLFW_LIBRARIES     NAMES glfw glfw3
                                PATHS /usr/lib64
                                /usr/lib
                                /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                                /usr/local/lib64
                                /usr/local/lib
                                /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                                /usr/X11R6/lib
)

find_package(OpenGL)
set (EXTRA_LIBS ${EXTRA_LIBS} ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} )
add_executable  (realsense_advanced_config realsense_advanced_config.cpp CV_Helpers.cpp roboslam_realsense.cpp)
target_link_libraries(realsense_advanced_config ${EXTRA_LIBS} )


add_executable                  (rs_async_RGBDAccRot_synced     rs_async_RGBDAccRot_synced.cpp  CV_Helpers.cpp roboslam_realsense.cpp)
add_executable                  (my_rs_motion                   my_rs_motion.cpp  CV_Helpers.cpp roboslam_realsense.cpp)
add_executable                  (rs_async_drop_RGBDIMU          rs_async_drop_RGBDIMU.cpp       CV_Helpers.cpp roboslam_realsense.cpp)
add_executable                  (record_IMU_as_R                record_IMU_as_R.cpp             CV_Helpers.cpp roboslam_realsense.cpp)
add_executable                  (record_IMU_async               rs_async.cpp                    CV_Helpers.cpp roboslam_realsense.cpp)

target_link_libraries           (record_IMU_as_R            ${EXTRA_LIBS})
target_link_libraries           (my_rs_motion               ${EXTRA_LIBS})
target_link_libraries           (record_IMU_async           ${EXTRA_LIBS})
target_link_libraries           (rs_async_RGBDAccRot_synced ${EXTRA_LIBS})
target_link_libraries           (rs_async_drop_RGBDIMU      ${EXTRA_LIBS})