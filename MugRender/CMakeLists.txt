cmake_minimum_required(VERSION 3.10)
project(RenderTest)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)

add_executable(RenderTest Render.cpp RenderEngine.hpp RenderEngine.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(RenderTest ${OpenCV_LIBRARIES})
