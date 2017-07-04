cmake_minimum_required(VERSION 2.8)

add_subdirectory(piotr_fhog)

set(trackers_LIB_SRC tasms.cpp tcbt.cpp tkcf.cpp tmosse.cpp)

add_library(trackers STATIC ${trackers_LIB_SRC})
target_link_libraries(trackers ${OpenCV_LIBS} kcf color )
set_target_properties(trackers PROPERTIES VERSION 1.0.0 SOVERSION 1)
