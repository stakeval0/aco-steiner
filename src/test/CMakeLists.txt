add_executable(test test.cpp)
target_link_libraries(test ACOSteiner)
target_compile_options(test PUBLIC -g)