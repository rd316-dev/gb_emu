#define SDL_MAIN_HANDLED
#include "gameboyemulator.h"
#include "imgui.h"
#include "backends/imgui_impl_sdl.h"
#include "backends/imgui_impl_sdlrenderer.h"

#include "SDL2/SDL.h"

#include <iostream>
#include <fstream>

std::shared_ptr<uint8_t*> load_file(std::string path, size_t *size)
{
    std::ifstream input(path, std::ios::binary);

    std::vector<uint8_t> buffer(std::istreambuf_iterator<char>(input), {});
    input.close();

    *size = buffer.size();
    uint8_t *data = new uint8_t[buffer.size()];

    std::copy(buffer.begin(), buffer.end(), data);

    return std::make_shared<uint8_t*>(data);
}

int main()
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0){
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow("Game Boy Emulator", 100, 100, 640, 480, SDL_WINDOW_SHOWN);
    if (window == nullptr){
        std::cout << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (renderer == nullptr){
        SDL_DestroyWindow(window);
        std::cout << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer_Init(renderer);

    GbE emu;

    size_t boot_rom_size = 0;
    auto boot_rom = load_file("C:\\Projects\\gb_emu\\resource\\dmg_rom.bin", &boot_rom_size);
    emu.load_boot_rom(boot_rom_size, boot_rom);

    size_t rom_size = 0;
    auto rom = load_file("C:\\Projects\\gb_emu\\resource\\test_rom.gb", &rom_size);
    emu.load_rom(rom_size, rom);

    //uint16_t breakpoint = 0x9f;
    bool breakpoint_reached = false;

    SDL_Event e;
    bool quit = false;
    while (!quit){
        while (SDL_PollEvent(&e)){
            ImGui_ImplSDL2_ProcessEvent(&e);

            if (e.type == SDL_QUIT)
                quit = true;

            if (e.type == SDL_KEYDOWN){
                switch (e.key.keysym.sym) {
                case SDLK_ESCAPE:
                    quit = true;
                }
            }
        }

        ImGui_ImplSDLRenderer_NewFrame();
        ImGui_ImplSDL2_NewFrame();

        ImGui::NewFrame();
        ImGui::Begin("Hello, world!");

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                    1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        if (!breakpoint_reached) {
            emu.execute();

            if (emu.get_PC() >= rom_size) {
                breakpoint_reached = true;
                std::cout << "Breakpoint at " << std::hex << (unsigned int) emu.get_PC() << std::dec << " reached" << std::endl;
            }
        }

        if (ImGui::Button("Cycle")) {
            emu.execute();
        }

        ImGui::End();
        ImGui::Render();

        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer_RenderDrawData(ImGui::GetDrawData());
        SDL_RenderPresent(renderer);
    }

    ImGui_ImplSDLRenderer_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
