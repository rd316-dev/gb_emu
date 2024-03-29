cmake_minimum_required(VERSION 3.27)

project(gb_emu LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB IMGUI_SOURCES ${IMGUI_PATH}/**.cpp)

set(CMAKE_CXX_FLAGS "-static-libgcc -static-libstdc++")

add_executable(gb_emu
    src/main.cpp
    src/gameboyemulator.cpp
    src/gbtest.cpp

    src/utils.cpp

    src/imgui_impl_sdlrenderer2.cpp
    src/imgui_impl_sdl2.cpp

    src/memorymapper.h src/memorymapper.cpp
    src/mbc1_mapper.h src/mbc1_mapper.cpp
    
    src/tinyfiledialogs.c
)
target_include_directories(gb_emu
    PRIVATE src/
    PUBLIC include/
    PUBLIC "${SDL2_PATH}/x86_64-w64-mingw32/include"
    PUBLIC "${IMGUI_PATH}"
)
target_link_directories(gb_emu
    PUBLIC "${SDL2_PATH}/x86_64-w64-mingw32/lib"
)
target_link_libraries(gb_emu
    PUBLIC SDL2
    PUBLIC ${IMGUI_SOURCES}
)

install(TARGETS gb_emu
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
