project(RealseRecorder)
cmake_minimum_required(VERSION 2.8)

# A minimal build configuration, with the 14 standard
SET (CMAKE_BUILD_TYPE RELEASE CACHE STRING "Build configuration" FORCE)
set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_FLAGS "-g") 

# Include Eigen library
find_path(EIGEN3_INCLUDE_DIR    NAMES signature_of_eigen3_matrix_library
                                PATHS
                                ${CMAKE_INSTALL_PREFIX}/include
                                ${KDE4_INCLUDE_DIR}
                                PATH_SUFFIXES eigen3 eigen)
include_directories		        (${EIGEN3_INCLUDE_DIR})
include_directories		        (/usr/local/include)
link_directories		        (/usr/local/lib/)

# Include BOOST with the required components
find_package        		    (Boost COMPONENTS system program_options iostreams filesystem python thread coroutine context REQUIRED)  #last three for ros interrface
include_directories		        (${Boost_INCLUDE_DIRS})
link_directories		        (${Boost_LIBRARY_DIRS})
set 				            (EXTRA_LIBS ${EXTRA_LIBS} ${Boost_LIBRARIES})
message				            (${Boost_LIBRARY_DIRS} ${Boost_INCLUDE_DIRS}  ${Boost_LIBRARIES})

# # Link with the local PCL copy
# set					            (PCL_DIR "/files/Libraries/pcl/install/share/pcl-1.12/")
# find_package		            (PCL REQUIRED COMPONENTS common io registration visualization filters features keypoints segmentation gpu gpu_features gpu_kinfu gpu_kinfu_large_scale gpu_octree)
# include_directories	            (${PCL_INCLUDE_DIRS})
# link_directories	            (${PCL_LIBRARY_DIRS})
# add_definitions		            (${PCL_DEFINITIONS})
# add_definitions 	            (-DWITH_PCL)
# set                             (EXTRA_LIBS ${EXTRA_LIBS} ${PCL_LIBRARIES})

# Link with OpenCV4
include                 		(/files/Libraries/opencv4/install/lib/cmake/opencv4/OpenCVConfig.cmake )
include_directories     		(${OpenCV_INCLUDE_DIRS})
link_directories		        (${OpenCV_INSTALL_PATH}/lib)
set                     		(EXTRA_LIBS ${EXTRA_LIBS} ${OpenCV_LIBS}) 

# Link with the realsense SDK
set                             (realsense2_DIR /files/Libraries/realsense/install/lib/cmake/realsense2)
find_package                    (realsense2 REQUIRED)
include_directories             (${realsense2_INCLUDE_DIRS})
include_directories             (/files/Projects/Libraries/intelrealsense/librealsense/install/include)
link_directories                (/files/Projects/Libraries/intelrealsense/librealsense/install/lib)
set 				            (EXTRA_LIBS ${EXTRA_LIBS} ${realsense2_LIBRARY})
# This is included in order to decode the camera image for rendering
include_directories             (/files/Libraries/realsense/librealsense/third-party/realsense-file/lz4)

# Link with GLFW for creating OpenGL windows/contexts
find_library(GLFW_LIBRARIES     NAMES glfw glfw3
                                PATHS /usr/lib64
                                /usr/lib
                                /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                                /usr/local/lib64
                                /usr/local/lib
                                /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
                                /usr/X11R6/lib
)
set (EXTRA_LIBS ${EXTRA_LIBS} ${GLFW_LIBRARIES})

find_package(OpenGL)
set (EXTRA_LIBS ${EXTRA_LIBS} ${OPENGL_LIBRARIES} )

# Create the executables
add_executable                  (rs_pipeline_group_frame        rs_pipeline_group_frame.cpp  helpers.cpp)
target_link_libraries           (rs_pipeline_group_frame        ${EXTRA_LIBS})

add_executable                  (rs_async_drop_RGBDIMU          rs_async_drop_RGBDIMU.cpp    helpers.cpp)
target_link_libraries           (rs_async_drop_RGBDIMU          ${EXTRA_LIBS})