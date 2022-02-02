project(RealseRecorder)
cmake_minimum_required(VERSION 3.1)

# A minimal build configuration, with the 14 standard
SET (CMAKE_BUILD_TYPE RELEASE CACHE STRING "Build configuration" FORCE)
SET (CMAKE_CXX_STANDARD 14)
SET (CMAKE_CXX_FLAGS "-g") 

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
find_package        		    (Boost COMPONENTS system program_options iostreams filesystem python thread coroutine context REQUIRED) 
include_directories		        (${Boost_INCLUDE_DIRS})
link_directories		        (${Boost_LIBRARY_DIRS})
set 				            (EXTRA_LIBS ${EXTRA_LIBS} ${Boost_LIBRARIES})
message				            (${Boost_LIBRARY_DIRS} ${Boost_INCLUDE_DIRS}  ${Boost_LIBRARIES})

# Link with OpenCV4
SET                             (OpenCV_DIR /files/Libraries/opencv4/install/lib/cmake/opencv4/ CACHE STRING "OpenCV installation directory")
find_package                    (OpenCV REQUIRED)
include_directories     		(${OpenCV_INCLUDE_DIRS})
link_directories		        (${OpenCV_INSTALL_PATH}/lib)
set                     		(EXTRA_LIBS ${EXTRA_LIBS} ${OpenCV_LIBS}) 

# Link with the realsense SDK
SET                             (realsense2_DIR /files/Libraries/realsense/install/lib/cmake/realsense2 CACHE STRING "Intel realsense SDK directory")
find_package                    (realsense2 REQUIRED)
include_directories             (${realsense2_INCLUDE_DIRS})
set 				            (EXTRA_LIBS ${EXTRA_LIBS} ${realsense2_LIBRARY})

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
set                             (EXTRA_LIBS ${EXTRA_LIBS} ${GLFW_LIBRARIES})

set                             (OpenGL_GL_PREFERENCE LEGACY)
find_package                    (OpenGL)
set                             (EXTRA_LIBS ${EXTRA_LIBS} ${OPENGL_LIBRARIES} )

# Create the executables
add_executable                  (rs_pipeline_group_frame        rs_pipeline_group_frame.cpp  helpers.cpp)
target_link_libraries           (rs_pipeline_group_frame        ${EXTRA_LIBS})

add_executable                  (rs_async_drop_RGBDIMU          rs_async_drop_RGBDIMU.cpp    helpers.cpp)
target_link_libraries           (rs_async_drop_RGBDIMU          ${EXTRA_LIBS})