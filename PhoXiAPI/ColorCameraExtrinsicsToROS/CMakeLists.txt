cmake_minimum_required (VERSION 3.10)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

project(ColorCameraExtrinsicsToROS)

set(CMAKE_RELEASE_POSTFIX "_Release")
set(CMAKE_DEBUG_POSTFIX "_Debug")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

if (UNIX AND NOT APPLE)
    add_compile_options(-std=c++1y)
    add_compile_options(-pthread)
endif(UNIX AND NOT APPLE)

set(Files
    ${ColorCameraExtrinsicsToROS_SOURCE_DIR}/ColorCameraExtrinsicsToROS.cpp
    ${ColorCameraExtrinsicsToROS_SOURCE_DIR}/ReadMe.txt
)

add_executable(ColorCameraExtrinsicsToROS
    ${Files}
)

if (NOT PHO_API_CMAKE_CONFIG_PATH)
    set(PHO_API_CMAKE_CONFIG_PATH "$ENV{PHOXI_CONTROL_PATH}")
endif()

find_package(PhoXi REQUIRED CONFIG PATHS "${PHO_API_CMAKE_CONFIG_PATH}")

target_link_libraries(ColorCameraExtrinsicsToROS
    ${PHOXI_LIBRARY}
    $<$<PLATFORM_ID:Linux>:rt>
    Eigen3::Eigen
)

if(MSVC)
    add_custom_command(TARGET ColorCameraExtrinsicsToROS POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<$<CONFIG:Release>:${PHOXI_DLL_RELEASE}>
            $<$<CONFIG:Debug>:${PHOXI_DLL_DEBUG}>
            $<TARGET_FILE_DIR:ColorCameraExtrinsicsToROS>
    )
endif()

target_include_directories(ColorCameraExtrinsicsToROS PUBLIC 
    ${PHOXI_INCLUDE_DIRS}
)
