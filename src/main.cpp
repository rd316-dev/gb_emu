#define SDL_MAIN_HANDLED
#include "gameboyemulator.h"
#include "imgui.h"
#include "backends/imgui_impl_sdl.h"
#include "backends/imgui_impl_sdlrenderer.h"

#include "SDL2/SDL.h"

#include <iostream>
#include <fstream>
#include <iomanip>

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

void debuggerRegisters(GbE *emu)
{
    if (ImGui::BeginTable("Registers", 7)) {
        ImGui::TableNextRow();
        char names[] = {'A', 'B', 'C', 'D', 'E', 'H', 'L'};
        for (char c : names) {
            ImGui::TableNextColumn();
            ImGui::Text("%c", c);
        }
        Reg8 regs[] = {Reg8::A, Reg8::B, Reg8::C, Reg8::D, Reg8::E, Reg8::H, Reg8::L};

        ImGui::TableNextRow();
        for (Reg8 r : regs) {
            ImGui::TableNextColumn();
            ImGui::Text("%02X", emu->read_register8(r));
        }

        ImGui::EndTable();
    }
}

void debuggerFlags(const GbE *emu)
{
    if (ImGui::BeginTable("Flags", 4)) {
        ImGui::TableNextRow();
        char names[] = {'Z', 'N', 'H', 'C'};
        for (char c : names) {
            ImGui::TableNextColumn();
            ImGui::Text("%c", c);
        }

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%d", emu->get_Z() ? 1 : 0);
        ImGui::TableNextColumn();
        ImGui::Text("%d", emu->get_N() ? 1 : 0);
        ImGui::TableNextColumn();
        ImGui::Text("%d", emu->get_H() ? 1 : 0);
        ImGui::TableNextColumn();
        ImGui::Text("%d", emu->get_C() ? 1 : 0);

        ImGui::EndTable();
    }
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
    auto rom = load_file("C:\\Projects\\gb_emu\\resource\\01-special.gb", &rom_size);
    emu.load_rom(rom_size, rom);

    uint16_t breakpoints[] = {0x200};
    bool use_breakpoints = true;
    bool breakpoint_reached = false;
    bool cycling_enabled = true;
    bool stop_at_0x0100 = true;

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
        ImGui::Begin("FPS");
        ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
        ImGui::End();

        ImGui::Begin("Registers");
        ImGui::Text("PC: %#04x\n ", emu.get_PC());

        debuggerRegisters(&emu);
        debuggerFlags(&emu);

        if (cycling_enabled && !breakpoint_reached) {
            for (int i = 0; i < 18000; i++) {
                emu.execute();

                if (emu.isHalted()) {
                    std::cout << "Received HALT" << std::endl;
                    breakpoint_reached = true;
                    break;
                } else if (emu.isStopped()) {
                    std::cout << "Received STOP" << std::endl;
                    breakpoint_reached = true;
                    break;
                }

                if (use_breakpoints) {
                    uint16_t pc = emu.get_PC();

                    for (uint16_t addr : breakpoints) {
                        if (addr == pc) {
                            std::cout << "Breakpoint reached at " << std::hex
                                      << (unsigned int) addr << std::dec <<std::endl;
                            breakpoint_reached = true;
                            break;
                        }
                    }
                }

                if (breakpoint_reached) {
                    break;
                }
            }
        }

        if (ImGui::Button("Cycle")) {
            emu.execute();
            std::cout << std::hex << std::setw(4) << std::setfill('0')
                      << (unsigned int) emu.get_last_PC() << "\t"
                      << std::setw(2) << std::setfill('0')
                      << (unsigned int) emu.get_last_opcode();

            if (emu.get_last_opcode() == 0xCB) {
                std::cout << (unsigned int) emu.get_last_CB_opcode();
            }

            std::cout << std::dec << std::endl;

            if (emu.isHalted()) {
                std::cout << "Received HALT" << std::endl;
                break;
            } else if (emu.isStopped()) {
                std::cout << "Received STOP" << std::endl;
                break;
            }
        }

        if (cycling_enabled && !breakpoint_reached) {
            if (ImGui::Button("Pause")) {
                cycling_enabled = false;
            }
        } else {
            if (ImGui::Button("Resume")) {
                cycling_enabled = true;
                breakpoint_reached = false;
            }
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
