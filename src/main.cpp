#define SDL_MAIN_HANDLED

#include "tinyfiledialogs.h"
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#ifdef _WIN32
#include "windows.h"
#endif

#include "SDL2/SDL.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

#include "gameboyemulator.h"
#include "gbtest.h"

typedef struct DEBUG_INSTRUCTION {
    uint16_t address = 0;
    uint8_t opcode = 0;
    std::string inst = "";
    std::string arg1 = "";
    std::string arg2 = "";
} DebugInstruction;

typedef struct BREAKPOINT {
    uint16_t address;
    bool enabled;
} Breakpoint;

std::vector<Breakpoint> breakpoints = {
    {0x000c, true}
};

std::vector<DebugInstruction> instruction_listing;
int max_instructions_in_listing = 10;

bool use_breakpoints = true;
bool cycling_enabled = true;
bool breakpoint_reached = false;

bool output_instr = true;
bool testing_mode = true;

int cycles_per_frame = 10000;

uint64_t cycles = 0;

GbE::CPU *emu;

bool is_active()
{
    return cycling_enabled && !breakpoint_reached && !emu->isStopped() && !emu->isHalted();
}

std::string to_hex(uint32_t val, int length)
{
    std::stringstream stream;
    stream << std::setfill('0') << std::setw(length)
           << std::hex << val;

    return stream.str();
}

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

bool emu_cycle()
{
    emu->execute();

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

    if (output_instr) {
        auto inst = DebugInstruction {
            emu->get_last_PC(),
            emu->get_last_opcode(),
            emu->get_inst(),
            emu->get_arg1(),
            emu->get_arg2()
        };

        instruction_listing.insert(instruction_listing.begin(), inst);

        if (instruction_listing.size() > max_instructions_in_listing) {
            instruction_listing.pop_back();
        }
    }

    cycles++;

    if (emu->isHalted()) [[unlikely]] {
        std::cout << "Received HALT" << std::endl;
        return false;
    } else if (emu->isStopped()) {
        std::cout << "Received STOP" << std::endl;
        return false;
    }

    return true;
}

void debugger_main()
{
    if (!ImGui::Begin("Registers")) {
        return;
    }

    ImGui::Text("PC: %#04x\n ", emu->get_PC());
    ImGui::Text("SP: %#04x\n ", emu->read_register16(GbE::Reg16_SP::SP));

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

    ImGui::Text("Cycles: %i", cycles);

    if (is_active()) {
        ImGui::Text("Active...");
        if (ImGui::Button("Pause")) {
            cycling_enabled = false;
        }
    } else {
        if (ImGui::Button("Cycle")) {
            emu_cycle();
        }
        if (ImGui::Button("Resume")) {
            emu->resume();
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
    if (!ImGui::Begin("Breakpoints")) {
        return;
    }

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
    if (!ImGui::Begin("FPS")) {
        return;
    }

    ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
    ImGui::End();
}

void debugger_program_listing()
{
    if (!ImGui::Begin("Assembly Listing")) {
        output_instr = false;
        return;
    }

    output_instr = true;

    if (ImGui::BeginTable("Listing", 5)) {
        int row_pad = max_instructions_in_listing - instruction_listing.size();
        for (int i = 0; i < row_pad; i++) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TableNextColumn();
            ImGui::TableNextColumn();
            ImGui::TableNextColumn();
            ImGui::TableNextColumn();
        }

        for (auto i = instruction_listing.rbegin(); i != instruction_listing.rend(); i++) {
            std::string padded_inst = i->inst;
            for (int i = padded_inst.length(); i < 4; i++) {
                padded_inst.push_back(' ');
            }

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text(to_hex(i->address, 4).c_str());
            
            ImGui::TableNextColumn();
            ImGui::Text(to_hex(i->opcode, 2).c_str());
            
            ImGui::TableNextColumn();
            ImGui::Text(padded_inst.c_str());
            
            ImGui::TableNextColumn();
            ImGui::Text(i->arg1.c_str());
            
            ImGui::TableNextColumn();
            ImGui::Text(i->arg2.c_str());
        }

        ImGui::EndTable();
    }
    ImGui::End();
}

void debug_hex_editor()
{
    if (!ImGui::Begin("Hex Viewer"))
        return;

    if (ImGui::BeginTable("HEX", 17)) {
        ImGui::TableNextRow();
        ImU32 index_bg_color = ImGui::GetColorU32(ImVec4(0.3f, 0.3f, 0.7f, 1.0f));
        ImU32 val_bg_color = ImGui::GetColorU32(ImVec4(0.1f, 0.1f, 0.1f, 1.0f));
        ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, index_bg_color);

        ImGui::TableNextColumn();
        for (int i = 0; i < 16; i++) {
            ImGui::TableNextColumn();
            ImGui::Text(to_hex(i, 2).c_str());
        }

        for (int i = 0; i < 256; i++) {
            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, index_bg_color);
            ImGui::TableNextColumn();
            ImGui::Text(to_hex(i << 4, 4).c_str());

            ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, val_bg_color);
            for (int j = 0; j < 16; j++) {
                ImGui::TableNextColumn();

                uint16_t addr = (i << 4) | j;
                uint8_t val = emu->peek_memory(addr);

                ImGui::TextUnformatted(to_hex(val, 2).c_str());
            }
        }

        ImGui::EndTable();
    }

    ImGui::End();
}

void start_unit_tests()
{
    GbTest test(emu);
    test.launch_tests();
}

int main()
{
    #ifdef _WIN32
    SetProcessDPIAware();
    #endif

    if (SDL_Init(SDL_INIT_VIDEO) != 0){
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    float ddpi = -1.0f;
    float hdpi = -1.0f;
    float vdpi = -1.0f;
    if (SDL_GetDisplayDPI(0, &ddpi, &hdpi, &vdpi) != 0) {
        std::cout << "SDL_GetDisplayDPI Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    std::cout << "Diagonal DPI: " << ddpi <<
               "\nHorizontal DPI: " << hdpi <<
               "\nVertical DPI: " << vdpi << std::endl;

    float scaling = ddpi / 96.0f;

    int width = 1280 * scaling;
    int height = 720 * scaling;

    SDL_Window *window = SDL_CreateWindow("Game Boy Emulator", 100, 100, width, height, SDL_WINDOW_SHOWN);
    if (window == nullptr){
        std::cout << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
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
    ImGuiIO& io = ImGui::GetIO();

    float imgui_font_size = 16.0f * scaling;

    io.Fonts->AddFontFromFileTTF("Inter-VariableFont_slnt,wght.ttf", imgui_font_size);

    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    emu = new GbE::CPU();
    start_unit_tests();
    emu->reset();

    char const * boot_rom_patterns[1] = {"*.bin"}; 
    char const * boot_rom_filename = tinyfd_openFileDialog(
        "Open Boot ROM File", 
        NULL, 
        1, 
        boot_rom_patterns, 
        "Boot ROM File (.bin)",
        0
    );
    size_t boot_rom_size = 0;
    auto boot_rom = load_file(boot_rom_filename, &boot_rom_size);
    emu->load_boot_rom(boot_rom_size, boot_rom);

    char const * rom_patterns[1] = {"*.gb"}; 
    char const * rom_filename = tinyfd_openFileDialog(
        "Open Game Boy ROM File", 
        NULL, 
        1, 
        rom_patterns, 
        "Game Boy ROM File (.gb)",
        0
    );
    size_t rom_size = 0;
    auto rom = load_file(rom_filename, &rom_size);
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

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();

        ImGui::NewFrame();
        debugger_fps_counter();
        debugger_main();
        debugger_breakpoints();
        debug_hex_editor();
        debugger_program_listing();

        for (int i = 0; i < cycles_per_frame && is_active(); i++) {
            if (!emu_cycle()) {
                break;
            }
        }

        ImGui::Render();

        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
        SDL_RenderPresent(renderer);
    }

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
