project(ure_controller)
cmake_minimum_required(VERSION 3.10)

find_package(Protobuf REQUIRED)


find_package(Boost REQUIRED system thread )
SET(Boost_USE_MULTITHREAD ON)

if (DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
    set(WEBOTS_HOME /usr/local/webots)
#TODO ここは恐らくwinと分けるべき
endif()

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ure.proto)
link_directories(${WEBOTS_HOME}/lib/controller)

# add_executable(URe_controller ure_controller.cpp ure_controller_constants.h sUDPSocket.hpp ${PROTO_SRCS} ${PROTO_HDRS})
add_executable(URe_controller ure_controllers.hpp ure_controller_constants.h main.cpp)

list(APPEND target_list URe_controller )

foreach(itm ${target_list})
    target_compile_features(${itm}  PUBLIC cxx_std_17)
    target_compile_options(${itm}  PUBLIC -Wall -pedantic-errors) 
    target_include_directories(${itm} PRIVATE
        ${WEBOTS_HOME}/include/controller/c 
        ${WEBOTS_HOME}/include/controller/cpp 
        ${Protobuf_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${Boost_INCLUDE_DIRS}
    )   
endforeach()


set (WEBOTS_LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
target_link_libraries(URe_controller 
    ${Boost_LIBRARIES} 
    ${WEBOTS_LIBRARIES}
    -lrt 
    protobuf::libprotobuf
)

install(TARGETS URe_controller DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})