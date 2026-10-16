// Collie Kart Portable P001 -- display-only Mario Kart 7 plugin.
// No game patches, controller override, learning, or SD writes.
// Derived from CTRPluginFramework-BlankTemplate (Nanquitas / PabloMK7).

#include <3ds.h>
#include <CTRPluginFramework.hpp>
#include <string>

namespace CTRPluginFramework
{
    namespace
    {
        constexpr u64 MK7_USA = 0x0004000000030800ULL;
        constexpr u64 MK7_EUR = 0x0004000000030700ULL;
        constexpr u64 MK7_JPN = 0x0004000000030600ULL;

        bool g_verified = false;
        bool g_ready = false;
        unsigned int g_toggles = 0;
        const char *g_region = "UNKNOWN";

        bool DrawStatus(const Screen &screen)
        {
            if (screen.IsTop || !g_verified)
                return true;

            // Bottom-screen placement is provisional; verify it does not
            // hide the MK7 minimap/HUD during Time Trials.
            screen.DrawRect(2, 224, 156, 14, Color::Black, true);
            screen.Draw(g_ready ? "CK P001 | READY" : "CK P001 | INACTIVE",
                        5, 226,
                        g_ready ? Color::Green : Color::White,
                        Color::Black);
            return true;
        }

        void ToggleReady(MenuEntry *)
        {
            if (!g_verified)
                return;
            g_ready = !g_ready;
            ++g_toggles;
        }

        void ShowDiagnostics(MenuEntry *)
        {
            if (!g_verified)
                return;

            std::string details = "Game: Mario Kart 7 (";
            details += g_region;
            details += ")\nTitle ID: VERIFIED\n";
            details += "Status: ";
            details += (g_ready ? "READY\n" : "INACTIVE\n");
            details += "Toggle count: " + std::to_string(g_toggles) + "\n";
            details += "Race detection: NOT IMPLEMENTED\n";
            details += "Kart controls: DISABLED\n";
            details += "Learning: DISABLED\n";
            details += "SD writes: NONE\n\n";
            details += "P001 only tests the UI and game attachment.";
            MessageBox("Collie Kart P001 Diagnostics", details)();
        }
    }

    // The upstream template patches the game touchscreen/NFC behavior.
    // P001 deliberately does not patch any process memory.
    void PatchProcess(FwkSettings &) {}
    void OnProcessExit(void) {}

    int main(void)
    {
        const u64 id = Process::GetTitleID();

        if (id == MK7_USA) { g_verified = true; g_region = "USA"; }
        else if (id == MK7_EUR) { g_verified = true; g_region = "EUR"; }
        else if (id == MK7_JPN) { g_verified = true; g_region = "JPN"; }

        // Refuse to initialize inside other games.
        if (!g_verified)
            return 0;

        PluginMenu menu("Collie Kart P001", 0, 0, 1,
            "Systems check only: no driving, learning, or game-memory edits.\n"
            "Select an entry with A. Close the menu to view the bottom HUD.");

        menu.SynchronizeWithFrame(true);
        menu.ShowWelcomeMessage(false);

        menu += new MenuEntry("Toggle READY / INACTIVE", nullptr, ToggleReady,
            "Only changes the on-screen status flag; never drives the kart.");
        menu += new MenuEntry("Show diagnostics", nullptr, ShowDiagnostics,
            "Shows attachment verification, toggles and disabled functions.");

        OSD::Run(DrawStatus);
        menu.Run();
        OSD::Stop(DrawStatus);
        return 0;
    }
}
