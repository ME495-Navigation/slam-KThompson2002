# Lines that begin with a # are comments
# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.22)

# project_name sets the name of the project and causes cmake to
# find the c and c++ compilers
project(turtlelib)

# Find dependencies.
# Many libraries ship with files that allow CMake to find them
# Then general behavior is to call "find_package" but the options
# provided are package specific.  Usually there is then a CMAKE variable
# That is defined to reference the library
# here: we find the eigen library as per the instruction
# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target" (in this case the target is called "libname")
# add_library(libname src/turtlelib.cpp)
# add_library(libname )
add_library(turtlelib src/geometry2d.cpp src/se2d.cpp src/svg.cpp src/diff_drive.cpp)


# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# using the library from the build directory or after installation the header
# files are actually in different locations.
# During the build, the headers are read from the source code directory
# When used from the installed location, headers are in the
# system include/ directory
target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# specify additional compilation flags for the library
# Public causes the flags to propagate to anything
# that links against this library
target_compile_options(turtlelib PUBLIC -Wall -Wextra -pedantic)

# Enable c++23 support.
# PUBLIC causes the features to propagate to anything
# that links against this library
# whereas PRIVATE would make it apply only to the library itself
target_compile_features(turtlelib PUBLIC cxx_std_23)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"
include(CTest)
add_executable(converter src/converter.cpp)
add_executable(frame src/frame_main.cpp)

if(BUILD_TESTING)
    find_package(Catch2 3 REQUIRED)
    add_executable(test_geometry2d tests/test_geometry2d.cpp)
    add_executable(test_se2d tests/test_se2d.cpp)
    add_executable(test_svg tests/test_svg.cpp)
    add_executable(test_diff_drive tests/test_diff_drive.cpp)   
    add_test(NAME test_geometry2d COMMAND test_geometry2d)
    add_test(NAME test_se2d COMMAND test_se2d)
    add_test(NAME test_svg COMMAND test_svg)
    add_test(NAME test_diff_drive COMMAND test_diff_drive)
    target_link_libraries(test_geometry2d Catch2::Catch2WithMain turtlelib)
    target_link_libraries(test_se2d Catch2::Catch2WithMain turtlelib)
    target_link_libraries(test_svg Catch2::Catch2WithMain turtlelib)
    target_link_libraries(test_diff_drive Catch2::Catch2WithMain turtlelib)
endif()

find_package(Doxygen)
# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build the documentation
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()
# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to th locations of header files.
# Because libname has PUBLIC cxx_std_23 as a compile feature
# Name will also be compiled with c++23 as the standard.
target_link_libraries(converter Eigen3::Eigen turtlelib)
target_link_libraries(frame Eigen3::Eigen turtlelib)


# install the include files by copying the whole include directory
install(DIRECTORY include/turtlelib DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
# Also create CMake Export called project_name-targets
# The CMake Export contains files that allow other CMake projects
# to find this project. It must be installed separately.
# install(TARGETS Name libname EXPORT project_name-targets)

# The project_name-targets created by install(TARGETS) needs to be installed.
# install(EXPORT ...) will generate a file called project_name-config.cmake
# that contains the exported targets.
# After installation this file will then be found when calling
# find_package(project_name) from another cmake project
# A user can then target_link_libraries(target project_name::library)
# to use the libraries installed here
# install(EXPORT project_name-targets
#         FILE project_name-config.cmake
#         NAMESPACE project_name::
#         DESTINATION lib/cmake/turtlelib)