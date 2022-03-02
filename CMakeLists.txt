project(frame_generator)
cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0028 NEW)

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfp16-format=ieee ${OpenMP_CXX_FLAGS} ")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfp16-format=ieee ${OpenMP_C_FLAGS}") 
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS} ")
endif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")

set(CMAKE_CXX_STANDARD 14)

include_directories(
                    ${PROJECT_SOURCE_DIR}/src/tracker 
                    ${PROJECT_SOURCE_DIR}/src/tracker/predictor
                    ${PROJECT_SOURCE_DIR}/src/tracker/predictor/kalman
                    ${PROJECT_SOURCE_DIR}/src/util
                    ${PROJECT_SOURCE_DIR}/src/event
                    ${opencv4_INCLUDE_DIRS}
                    )

add_definitions(-DWITH_OPENMP=true)
FIND_PACKAGE(OpenMP REQUIRED)
find_package(prophesee_driver REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(Threads REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (dlib REQUIRED)

#find_package(OpenCV COMPONENTS core highgui REQUIRED)
find_package(OpenCV REQUIRED)#/usr/local/share/opencv4 /home/abednego/intel/openvino_2019.3.376/opencv  PATHS /usr/local/share/opencv4 NO_DEFAULT_PATH
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(vehicle_tracker  src/vehicle_tracker.cpp  
                                src/frame_generator.cpp
                                src/util/BoundingBox.cpp 
                                src/util/Detection.cpp
                                src/util/Tracking.cpp
                                src/util/FilterState.cpp
                                src/tracker/Affinity.cpp
                                src/tracker/TrackingEngine.cpp
                                src/tracker/Tracker.cpp
                                src/tracker/predictor/Predictor.cpp
                                src/tracker/predictor/kalman/KalmanPredictor.cpp
                               )#${OpenCV_LIBS} src/tracker.cpp

target_link_libraries(vehicle_tracker prophesee_driver Boost::program_options Threads::Threads ${EIGEN3_DIR} ${OpenCV_LIBS}

)

