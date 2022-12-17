add_library(ACOSteiner STATIC ACOSteiner.cpp ACOTable.cpp QuadTree.cpp Ant.cpp)
add_executable(main main.cpp)
target_link_libraries(main ACOSteiner)