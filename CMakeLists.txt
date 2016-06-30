cmake_minimum_required(VERSION 2.6)
project(cv_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

set(SOURCE_FILES
	src/find_bottle.cpp)

add_executable(cv_test ${SOURCE_FILES})
target_link_libraries(cv_test ${OpenCV_LIBS})

set(SOURCE_FILES_2
    src/kal.cpp)

add_executable(kal ${SOURCE_FILES_2})
target_link_libraries(kal ${OpenCV_LIBS})

set(SOURCE_FILES_3
    src/track_rect.cpp)

add_executable(track_rect ${SOURCE_FILES_3})
target_link_libraries(track_rect ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
