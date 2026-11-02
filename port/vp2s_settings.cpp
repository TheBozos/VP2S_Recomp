// vp2s.ini: the player's settings, in one file next to vp2s.exe instead of environment variables. Each setting is a
// runtime variable under a readable name; [env] passes any other variable as it is (testing, diagnostics). A
// variable already set in the environment wins over the file.

#include "vp2s_settings.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

namespace
{
    struct Setting
    {
        const char *section;
        const char *key;
        const char *variable;
    };

    constexpr Setting kSettings[] = {
        {"display", "window_size", "PS2X_WINDOW_SIZE"},
        {"display", "show_fps", "PS2X_SHOW_FPS"},
        {"display", "interpolate", "PS2X_NATIVE_INTERPOLATE"},
        {"audio", "mute", "PS2X_AUDIO_MUTE"},
        {"input", "start_analog", "PS2X_PAD_START_ANALOG"},
    };

    constexpr const char *kDefault =
        "; Valkyrie Profile 2: Silmeria (native port) settings. An environment variable with the same meaning wins.\n"
        "\n"
        "[game]\n"
        "; The boot ELF extracted from your own disc (tools/pipeline.py): vp2s.exe then starts without an argument.\n"
        "; elf = D:\\VP2S_Recomp\\build_usa\\extract\\iso\\SLUS_214.52\n"
        "\n"
        "[display]\n"
        "; The window's size at start, WIDTHxHEIGHT.\n"
        "; window_size = 1280x960\n"
        "; The frame counter at the top right (F3 shows or hides it).\n"
        "show_fps = 1\n"
        "; Frames in between the game's own, from its last two (smoother motion, one frame later).\n"
        "interpolate = 0\n"
        "\n"
        "[audio]\n"
        "mute = 0\n"
        "\n"
        "[input]\n"
        "; The pad starts in analog mode (the party moves with the left stick in battles).\n"
        "start_analog = 1\n"
        "\n"
        "[env]\n"
        "; Any other runtime variable, NAME = value (testing and diagnostics).\n";

    std::filesystem::path executableDirectory()
    {
#if defined(_WIN32)
        wchar_t buffer[MAX_PATH];
        const DWORD length = GetModuleFileNameW(nullptr, buffer, MAX_PATH);
        if (length > 0 && length < MAX_PATH)
            return std::filesystem::path(buffer).parent_path();
#endif
        return std::filesystem::current_path();
    }

    std::string trim(const std::string &text)
    {
        const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char c) { return std::isspace(c); });
        const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char c) { return std::isspace(c); }).base();
        return first < last ? std::string(first, last) : std::string();
    }

    void setVariable(const std::string &name, const std::string &value)
    {
        if (name.empty() || std::getenv(name.c_str()))
            return;
#if defined(_WIN32)
        _putenv_s(name.c_str(), value.c_str());
#else
        setenv(name.c_str(), value.c_str(), 0);
#endif
    }
}

std::string vp2sLoadSettings()
{
    const std::filesystem::path path = executableDirectory() / "vp2s.ini";
    std::ifstream in(path);
    if (!in)
    {
        std::ofstream out(path);
        if (out)
        {
            out << kDefault;
            std::cout << "[vp2s] settings: wrote the default " << path.string() << std::endl;
        }
        in.open(path);
    }
    std::string elf, section, line;
    for (int number = 1; std::getline(in, line); ++number)
    {
        line = trim(line);
        if (line.empty() || line[0] == ';' || line[0] == '#')
            continue;
        if (line.front() == '[' && line.back() == ']')
        {
            section = trim(line.substr(1, line.size() - 2));
            std::transform(section.begin(), section.end(), section.begin(), [](unsigned char c) { return std::tolower(c); });
            continue;
        }
        const size_t equals = line.find('=');
        if (equals == std::string::npos)
        {
            std::cerr << "[vp2s] settings: line " << number << " has no '=': " << line << std::endl;
            continue;
        }
        std::string key = trim(line.substr(0, equals));
        const std::string value = trim(line.substr(equals + 1));
        if (section == "env")
        {
            setVariable(key, value);
            continue;
        }
        std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) { return std::tolower(c); });
        if (section == "game" && key == "elf")
        {
            elf = value;
            continue;
        }
        const auto known = std::find_if(std::begin(kSettings), std::end(kSettings),
                                        [&](const Setting &s) { return section == s.section && key == s.key; });
        if (known == std::end(kSettings))
        {
            std::cerr << "[vp2s] settings: unknown [" << section << "] " << key << " (line " << number << ")" << std::endl;
            continue;
        }
        setVariable(known->variable, value);
    }
    return elf;
}
