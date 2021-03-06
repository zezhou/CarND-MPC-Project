project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/MPC.cpp src/main.cpp)
set(debug_sources src/MPC.cpp src/csv.h src/debug.cpp)

include_directories(/usr/local/include)
include_directories(/usr/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})
add_executable(mpc_debug ${debug_sources})

#target_link_libraries(mpc ipopt z ssl uv uWS gflags glog)
target_link_libraries(mpc ipopt z ssl uv uWS)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
#target_link_libraries(mpc_debug ipopt python2.7 pthread glog gflags)
target_link_libraries(mpc_debug ipopt python2.7 pthread)
else()
#target_link_libraries(mpc_debug ipopt python pthread glog gflags)
target_link_libraries(mpc_debug ipopt python pthread)
endif()
