cmake_minimum_required(VERSION 3.21)
project(BB)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -std=c++11 -g -D_GLIBCXX_USE_CXX11_ABI=0")

# declara header gurobi directory
set(GUROBI_INCLUDE_DIRECTORY "/Library/gurobi951/macos_universal2/include")

# adicionar header gurobi
include_directories(${GUROBI_INCLUDE_DIRECTORY})

# declara library gurobi directory
set(GUROBI_LIB_DIRECTORY "/Library/gurobi951/macos_universal2/lib")

# adicionar library gurobi
link_directories(${GUROBI_LIB_DIRECTORY})

set(CMAKE_CXX_STANDARD 14)

add_executable(BB main.cpp node.h tree.h lpmodel.h branchandbound.h test.h utils.h)

# adicionar libraries
target_link_libraries(BB gurobi_c++)
target_link_libraries(BB gurobi95)