cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(TetGeo-V2 VERSION 0.1.0)

option(BUILD_LIB "build lib" OFF)

add_definitions(-DASSETS_DIR=\"${CMAKE_SOURCE_DIR}/asset/\")


set(CMAKE_BUILD_FILES_DIRECTORY "${CMAKE_HOME_DIRECTORY}/build")
set(CMAKE_BUILD_DIRECTORY "${CMAKE_HOME_DIRECTORY}/build")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin")
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(BUILD_SHARED_LIBS OFF)
#set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX -D_USE_MATH_DEFINES ")

include_directories(src)
include_directories(src/pipeline)
include_directories(third)



set(VTK_SMP_ENABLE_STDTHREAD OFF)
set(VTK_Group_Rendering OFF)
set(VTK_RENDERING_BACKEND OFF)
set(VTK_Group_StandAlone OFF)
set(VTK_GROUP_ENABLE_Web DONT_WANT)
set(VTK_GROUP_ENABLE_Imaging DONT_WANT)
set(VTK_GROUP_ENABLE_Qt DONT_WANT)
set(VTK_GROUP_ENABLE_Rendering DONT_WANT)

add_subdirectory(third/VTK)


option(GEOGRAM_WITH_GRAPHICS "Viewers and geogram_gfx library" OFF)
add_subdirectory(third/geogram)

add_subdirectory(third/spdlog)

include_directories(third/CLI11)

include_directories(third/gmp/include)
if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    link_directories(third/gmp/lib/windows-x64)
endif ()

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    link_directories(third/gmp/lib/linux-x64)
endif ()

include_directories(third/nlohmann)

option(TBB_BUILD_TESTS "Build TBB tests and enable testing infrastructure" OFF)
add_subdirectory(third/tbb)


file(GLOB_RECURSE SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.c
)
if (BUILD_LIB)

    add_definitions(-DBUILD_LIB)
    add_library(tetgeo SHARED ${SOURCE_FILES})
else ()

    add_executable(tetgeo ${SOURCE_FILES})

endif ()

target_link_libraries(tetgeo
        PUBLIC
        VTK::IOXML
        tbb_static
        spdlog
        geogram
        gmp

)


