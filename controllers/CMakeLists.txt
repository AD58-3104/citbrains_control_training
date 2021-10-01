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
add_executable(URe_controller ure_controller.cpp sUDPSocket.hpp ${PROTO_SRCS} ${PROTO_HDRS})
target_compile_features(URe_controller  PUBLIC cxx_std_17)
target_compile_options(URe_controller  PUBLIC -Wall -pedantic-errors) 
target_include_directories(URe_controller PRIVATE
${WEBOTS_HOME}/include/controller/c 
${WEBOTS_HOME}/include/controller/cpp 
${Protobuf_INCLUDE_DIRS}
${CMAKE_CURRENT_BINARY_DIR}
${Boost_INCLUDE_DIRS}
)
link_directories(${WEBOTS_HOME}/lib/controller)
# set (WEBOTS_LIBRARIES Controller CppController) 良く分からんけどこの2行はいらないみたい
# set (WEBOTS_LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
target_link_libraries(URe_controller 
    ${Boost_LIBRARIES} 
    # ${WEBOTS_LIBRARIES}
    -lrt 
    protobuf::libprotobuf
    )

install(TARGETS URe_controller DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})