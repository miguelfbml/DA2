cmake_minimum_required(VERSION 3.23)
project(DA2)

set(CMAKE_CXX_STANDARD 14)

include_directories(DataStructures)
include_directories(src)

add_executable(DA2
        #DataStructures/Graph.cpp
        #DataStructures/Graph.h
        DataStructures/Heap.cpp
        DataStructures/Heap.h
        DataStructures/MutablePriorityQueue.h
        DataStructures/UFDS.cpp
        DataStructures/UFDS.h
        DataStructures/VertexEdge.cpp
        DataStructures/VertexEdge.h
        src/main.cpp
        src/Menu.cpp
        src/Menu.h
        src/Readfiles.cpp
        src/Readfiles.h
        src/tsp.cpp
        src/tsp.h
        DataStructures/Graf.cpp
        DataStructures/Graf.h
        DataStructures/TSP.cpp
        DataStructures/TSP.h
        )
