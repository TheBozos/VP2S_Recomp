#pragma once
// vp2s.ini, the player's settings (vp2s_settings.cpp).

#include <string>

// Reads vp2s.ini next to vp2s.exe (a commented default is written when there is none) and sets the runtime's
// variables it names; one already set in the environment wins, so tests and diagnostics still override the file.
// Returns [game] elf, the boot ELF to start when none is given on the command line (empty when not set).
std::string vp2sLoadSettings();
