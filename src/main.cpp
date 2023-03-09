#define SDL_MAIN_HANDLED
#include "imgui.h"
#include "gameboyemulator.h"
#include "gbtest.h"
#include "backends/imgui_impl_sdl.h"
#include "backends/imgui_impl_sdlrenderer.h"

#include "SDL2/SDL.h"

#include <iostream>
#include <fstream>
#include <iomanip>

typedef struct BREAKPOINT {
    uint16_t address;
    bool enabled;
} Breakpoint;

std::vector<Breakpoint> breakpoints = {
    {0xc7af, true},
    {0xc7b0, true},
    {0xc7b1, true},
    {0xc7b2, true},
    {0x0100, true},
};

bool use_breakpoints = false;
bool cycling_enabled = true;
bool breakpoint_reached = false;

bool output_instr = true;
bool testing_mode = true;

int cycles_per_frame = 100000;

GbE::CPU *emu;

uint8_t* load_file(const std::string &path, size_t *size)
{
    std::ifstream input(path, std::ios::binary);

    if (!input.is_open()) {
        std::cout << "Error while opening file " << path << std::endl;
    }

    std::vector<uint8_t> buffer(std::istreambuf_iterator<char>(input), {});
    input.close();

    *size = buffer.size();
    uint8_t *ptr = (uint8_t*) malloc(*size);
    std::copy(buffer.begin(), buffer.end(), ptr);

    return ptr;
}

void debugger_main()
{
    ImGui::Begin("Registers");
    ImGui::Text("PC: %#04x\n ", emu->get_PC());

    if (ImGui::BeginTable("GbE::Registers", 7)) {
        ImGui::TableNextRow();
        char names[] = {'A', 'B', 'C', 'D', 'E', 'H', 'L'};
        for (char c : names) {
            ImGui::TableNextColumn();
            ImGui::Text("%c", c);
        }
        GbE::Reg8 regs[] = {
            GbE::Reg8::A,
            GbE::Reg8::B,
            GbE::Reg8::C,
            GbE::Reg8::D,
            GbE::Reg8::E,
            GbE::Reg8::H,
            GbE::Reg8::L
        };

        ImGui::TableNextRow();
        for (GbE::Reg8 r : regs) {
            ImGui::TableNextColumn();
            ImGui::Text("%02X", emu->read_register8(r));
        }

        ImGui::EndTable();
    }

    if (ImGui::BeginTable("Flags", 4)) {
        ImGui::TableNextRow();
        char names[] = {'Z', 'N', 'H', 'C'};
        for (char c : names) {
            ImGui::TableNextColumn();
            ImGui::Text("%c", c);
        }

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%d", emu->get_flag(GbE::Flag::Z) ? 1 : 0);
        ImGui::TableNextColumn();
        ImGui::Text("%d", emu->get_flag(GbE::Flag::N) ? 1 : 0);
        ImGui::TableNextColumn();
        ImGui::Text("%d", emu->get_flag(GbE::Flag::H) ? 1 : 0);
        ImGui::TableNextColumn();
        ImGui::Text("%d", emu->get_flag(GbE::Flag::C) ? 1 : 0);

        ImGui::EndTable();
    }

    if (ImGui::Button("Cycle")) {
        uint16_t PC = emu->get_PC();

        emu->execute();

        if (output_instr) {
            std::cout << std::hex << std::setfill('0')
                      << std::setw(4) << PC
                      << std::dec << "\t"
                      << emu->get_inst() << "\t"
                      << emu->get_arg1() << "\t"
                      << emu->get_arg2() << std::endl;
        }

        if (emu->isHalted()) {
            std::cout << "Received HALT" << std::endl;
        } else if (emu->isStopped()) {
            std::cout << "Received STOP" << std::endl;
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

    if (ImGui::Button("Print SPI")) {
        std::cout << "SPI buffer data:\n"
                  << emu->get_spi_buffer_data().data() << std::endl
                  << "SPI buffer data end" << std::endl;
    }

    ImGui::End();
}

void debugger_breakpoints()
{
    ImGui::Begin("Breakpoints");

    ImGui::Checkbox("Breakpoints active", &use_breakpoints);

    if (ImGui::BeginTable("Breakpoints", 2)) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("Address");
        ImGui::TableNextColumn();
        ImGui::Text("Enabled");

        for (auto i = breakpoints.begin(); i != breakpoints.end(); i++) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%#04x", i->address);

            ImGui::TableNextColumn();

            auto label = std::to_string(ImGui::TableGetRowIndex());
            ImGui::Checkbox(label.c_str(), &(i->enabled));
        }

        ImGui::EndTable();
    }
    ImGui::End();
}

void debugger_fps_counter()
{
    ImGui::NewFrame();
    ImGui::Begin("FPS");
    ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
    ImGui::End();
}

void start_unit_tests()
{
    GbTest test(emu);
    test.launch_tests();
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

    emu = new GbE::CPU();
    start_unit_tests();
    emu->reset();

    size_t boot_rom_size = 0;
    auto boot_rom = load_file("C:\\Projects\\gb_emu\\resource\\dmg_rom.bin", &boot_rom_size);
    emu->load_boot_rom(boot_rom_size, boot_rom);

    size_t rom_size = 0;
    auto rom = load_file("C:\\Projects\\gb_emu\\resource\\blargg\\06-ld r,r.gb", &rom_size);
    emu->load_rom(rom_size, rom);

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

        debugger_fps_counter();
        debugger_main();
        debugger_breakpoints();

        if (cycling_enabled && !breakpoint_reached && !emu->isStopped() && !emu->isHalted()) {
            for (int i = 0; i < cycles_per_frame; i++) {
                emu->execute();

                if (emu->isHalted()) {
                    std::cout << "Received HALT" << std::endl;
                    break;
                } else if (emu->isStopped()) {
                    std::cout << "Received STOP" << std::endl;
                    break;
                }

                if (use_breakpoints) {
                    uint16_t pc = emu->get_PC();

                    for (Breakpoint bp : breakpoints) {
                        if (bp.enabled && bp.address == pc) {
                            std::cout << "Breakpoint reached at " << std::hex
                                      << (unsigned int) bp.address << std::dec <<std::endl;
                            breakpoint_reached = true;
                            break;
                        }
                    }
                }
            }
        }

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
