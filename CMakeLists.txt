# Specify the minimum required CMake version
cmake_minimum_required(VERSION 3.12)

# Define the project name and enable C++ support
project(HelloWorld LANGUAGES CXX)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)

# eigen
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(eigen)

# LiePlusPlus
FetchContent_Declare(
  LiePlusPlus
  GIT_REPOSITORY https://github.com/aau-cns/Lie-plusplus.git
  GIT_TAG main
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(LiePlusPlus)

# vnproglib (VectorNav C++ SDK)
FetchContent_Declare(
  vnproglib
  GIT_REPOSITORY https://github.com/UniStuttgart-INS/vnproglib.git
  GIT_TAG 6f7243a178d0fcbb9792a23683c707ab5e6dbfac
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(vnproglib)
add_subdirectory(${vnproglib_SOURCE_DIR}/cpp ${vnproglib_BINARY_DIR}/cpp)


# Collect all source files
file(GLOB SOURCES CONFIGURE_DEPENDS Src/*.cpp)

# Add an executable target
add_executable(eqf main.cpp ${SOURCES})
target_include_directories(
  eqf
  PRIVATE
    Inc/
    ${vnproglib_SOURCE_DIR}/cpp/include
)
target_link_libraries(eqf PRIVATE LiePlusPlus Eigen3::Eigen libvncxx)
