project(URe_sample)
cmake_minimum_required(VERSION 3.10)

find_package(Protobuf REQUIRED)

set(CONT_DIR ${CMAKE_CURRENT_SOURCE_DIR}../../controllers)

find_package(Boost REQUIRED system thread )
SET(Boost_USE_MULTITHREAD ON)


protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CONT_DIR}/ure.proto)
set(URE_SRC ure_sample.cpp ${CONT_DIR}/sUDPSocket.hpp ${PROTO_SRCS} ${PROTO_HDRS})
add_executable(URe_sample ${URE_SRC})
target_compile_features(URe_sample  PUBLIC cxx_std_17)
target_compile_options(URe_sample  PUBLIC -Wall -pedantic-errors) 
target_include_directories(URe_sample PRIVATE
${WEBOTS_HOME}/include/controller/c 
${WEBOTS_HOME}/include/controller/cpp 
${Protobuf_INCLUDE_DIRS}
${CMAKE_CURRENT_BINARY_DIR}
${CONT_DIR}
${Boost_INCLUDE_DIRS}
)

target_link_libraries(URe_sample 
    ${Boost_LIBRARIES} 
    -lrt 
    protobuf::libprotobuf
    )

install(TARGETS URe_sample DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})