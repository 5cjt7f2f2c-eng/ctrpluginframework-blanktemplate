// Collie Kart Portable P002 -- experimental read-only MK7 telemetry.
// No game-memory writes, SD writes, controller injection, or learning.
// Pointer layout is a hypothesis from mk7re/MK7-Emu-Scripts (Citra).

#include <3ds.h>
#include <CTRPluginFramework.hpp>
#include <cmath>
#include <cstdio>
#include <string>
#include "CTRPluginFramework/System/Directory.hpp"
#include "CTRPluginFramework/System/File.hpp"

namespace CTRPluginFramework
{
    namespace
    {
        constexpr u64 MK7_USA = 0x0004000000030800ULL;
        constexpr u64 MK7_EUR = 0x0004000000030700ULL;
        constexpr u64 MK7_JPN = 0x0004000000030600ULL;
        // USA rev1/rev2 Citra addresses: NOT validated on hardware.
        constexpr u32 ROOTSYSTEM_USA_REV2 = 0x006789B8;
        constexpr u32 ENGINE_XOR = 0x75F1B26B;

        bool g_verified = false, g_ready = false, g_live = false;
        bool g_recording = false;
        bool g_bufferFull = false;
        unsigned g_toggles = 0, g_samples = 0, g_frames = 0;
        unsigned g_invalidSamples = 0;
        u64 g_startedMs = 0;
        std::string g_lastFile = "(none)";
        const char *g_region = "UNKNOWN";

        struct Observation
        {
            bool candidate = false;
            bool speedValid = false;
            const char *stage = "No snapshot yet";
            int lap = 0;
            unsigned checkpoint = 0;
            float progress = 0, maxProgress = 0;
            float x = 0, y = 0, z = 0, speed = 0;
        };
        Observation g_observation;

        // RAM-only buffer: 2 samples/sec x 15 minutes maximum.
        // No filesystem writes happen from DrawStatus or Probe.
        constexpr unsigned MAX_LOG_SAMPLES = 1800;
        struct Record
        {
            u32 elapsedMs = 0;
            Observation value;
        };
        Record g_records[MAX_LOG_SAMPLES];
        unsigned g_count = 0;

        void CaptureSample()
        {
            if (!g_recording || g_count >= MAX_LOG_SAMPLES)
                return;

            Record &r = g_records[g_count++];
            u64 now = osGetTime();
            r.elapsedMs = now >= g_startedMs
                          ? static_cast<u32>(now - g_startedMs) : 0;
            r.value = g_observation;
            if (!r.value.candidate)
                ++g_invalidSamples;

            if (g_count >= MAX_LOG_SAMPLES)
            {
                g_recording = false; // Preserve buffered data for Stop & Save.
                g_bufferFull = true;
            }
        }

        void StartRecording(MenuEntry *)
        {
            if (!g_verified || Process::GetTitleID() != MK7_USA)
            {
                MessageBox("Recorder unavailable",
                    "P003's memory layout is currently supported only for MK7 USA.")();
                return;
            }
            if (!g_ready)
            {
                MessageBox("Recorder inactive",
                    "First toggle CK P003 to READY, then start recording.")();
                return;
            }
            if (g_recording)
            {
                MessageBox("Already recording",
                    "Use Stop & Save before starting another session.")();
                return;
            }
            if (g_count != 0)
            {
                MessageBox("Unsaved samples",
                    "Use Stop & Save to save the existing session before starting another.")();
                return;
            }

            g_recording = true;
            g_bufferFull = false;
            g_startedMs = osGetTime();
            g_frames = 0;
            g_invalidSamples = 0;
            g_observation = Observation();
            MessageBox("Flight recorder started",
                "Close the plugin menu and drive a short Time Trial.\n"
                "When finished, open this menu and choose Stop & Save.")();
        }

        void StopAndSave(MenuEntry *)
        {
            // Stop sampling before making any filesystem calls.
            g_recording = false;
            if (g_count == 0)
            {
                MessageBox("Nothing to save",
                    "Start recording, close the menu, and drive first.")();
                return;
            }

            // Scope all writes to one dedicated SD folder.
            const std::string folder = "/CollieKart";
            if (Directory::IsExists(folder) != 1 &&
                Directory::Create(folder) != Directory::SUCCESS)
            {
                MessageBox("Recorder save failed",
                    "Could not create SD:/CollieKart. Samples are still in RAM.")();
                return;
            }

            char basename[48];
            std::string filePath;
            bool freeName = false;
            for (unsigned i = 1; i <= 999; ++i)
            {
                std::snprintf(basename, sizeof(basename),
                    "/CollieKart/session_%03u.csv", i);
                filePath = basename;
                if (File::Exists(filePath) == 0)
                {
                    freeName = true;
                    break;
                }
            }
            if (!freeName)
            {
                MessageBox("Recorder save failed",
                    "All 999 session filenames are occupied. No existing logs were replaced.")();
                return;
            }

            if (File::Create(filePath) != File::SUCCESS)
            {
                MessageBox("Recorder save failed",
                    "Could not create a session file. Samples remain in RAM.")();
                return;
            }

            File csv;
            if (File::Open(csv, filePath, File::WRITE | File::TRUNCATE) != File::SUCCESS)
            {
                MessageBox("Recorder save failed",
                    "Could not open session file. Samples remain in RAM.")();
                return;
            }

            bool saved = (csv.WriteLine(
                "elapsed_ms,valid,lap,checkpoint,completion,max_completion,"
                "pos_x,pos_y,pos_z,speed_valid,speed_game_units,probe_stage") == File::SUCCESS);

            char line[260];
            for (unsigned i = 0; saved && i < g_count; ++i)
            {
                const Record &r = g_records[i];
                const Observation &v = r.value;
                int length = std::snprintf(line, sizeof(line),
                    "%lu,%d,%d,%u,%.6f,%.6f,%.3f,%.3f,%.3f,%d,%.4f,%s",
                    static_cast<unsigned long>(r.elapsedMs),
                    v.candidate ? 1 : 0,
                    v.candidate ? v.lap : 0,
                    v.candidate ? v.checkpoint : 0,
                    v.candidate ? v.progress : 0.0f,
                    v.candidate ? v.maxProgress : 0.0f,
                    v.candidate ? v.x : 0.0f,
                    v.candidate ? v.y : 0.0f,
                    v.candidate ? v.z : 0.0f,
                    v.speedValid ? 1 : 0,
                    v.speedValid ? v.speed : 0.0f,
                    v.stage);
                if (length <= 0 || length >= static_cast<int>(sizeof(line)) ||
                    csv.WriteLine(line) != File::SUCCESS)
                    saved = false;
            }
            if (csv.Flush() != File::SUCCESS)
                saved = false;
            if (csv.Close() != File::SUCCESS)
                saved = false;

            if (!saved)
            {
                MessageBox("Recorder save incomplete",
                    "SD write failed. A PARTIAL CSV may exist.\n"
                    "Samples remain in RAM for another attempt.")();
                return;
            }

            unsigned written = g_count;
            g_lastFile = filePath;
            g_count = 0;
            g_invalidSamples = 0;
            g_bufferFull = false;
            std::string msg = "Saved " + std::to_string(written) +
                " samples to SD:" + g_lastFile +
                "\n\nYou can retrieve it with your iPhone.";
            MessageBox("Flight recorder saved", msg)();
        }


        bool PlausibleAddress(u32 address)
        {
            return address >= 0x00100000 && address <= 0x3FFF0000 &&
                   (address & 3) == 0;
        }
        bool ReadWord(u32 address, u32 &value)
        {
            return PlausibleAddress(address) && Process::Read32(address, value);
        }
        bool ReadPointer(u32 address, u32 &value)
        {
            return ReadWord(address, value) && PlausibleAddress(value);
        }
        bool ReadNumber(u32 address, float &value)
        {
            return PlausibleAddress(address) && Process::ReadFloat(address, value) &&
                   std::isfinite(value);
        }

        void Probe()
        {
            ++g_samples;
            Observation o;

            if (Process::GetTitleID() != MK7_USA)
            {
                o.stage = "Only USA memory layout available";
                g_observation = o;
                return;
            }

            u32 sceneMgr = 0, scene = 0, encoded = 0, characterEngine = 0;
            u32 collection = 0, raceDirector = 0, modeManager = 0, lapChecker = 0;
            u32 count = 0, player = 0, array = 0, kart = 0, lap = 0;
            u8 cp = 0;

            o.stage = "No RootSystem scene manager";
            if (!ReadPointer(ROOTSYSTEM_USA_REV2 + 0x04, sceneMgr)) { g_observation = o; return; }
            o.stage = "No current scene";
            if (!ReadPointer(sceneMgr + 0x04, scene)) { g_observation = o; return; }
            o.stage = "No character engine";
            if (!ReadWord(scene + 0x1E4, encoded)) { g_observation = o; return; }
            characterEngine = encoded ^ ENGINE_XOR;
            if (!PlausibleAddress(characterEngine)) { g_observation = o; return; }
            o.stage = "No engine collection";
            if (!ReadPointer(characterEngine + 0x1C, collection)) { g_observation = o; return; }
            o.stage = "No race director";
            if (!ReadPointer(collection + 0x1C, raceDirector)) { g_observation = o; return; }
            o.stage = "No mode manager";
            if (!ReadPointer(raceDirector + 0x1BC, modeManager)) { g_observation = o; return; }
            o.stage = "No lap checker (possible menu)";
            if (!ReadPointer(modeManager + 0x504, lapChecker)) { g_observation = o; return; }
            o.stage = "Invalid player count";
            if (!ReadWord(lapChecker + 0x28, count) || count < 1 || count > 8)
            { g_observation = o; return; }
            o.stage = "Invalid player index";
            if (!ReadWord(lapChecker + 0x1C, player) || player >= count)
            { g_observation = o; return; }
            o.stage = "No kart info array";
            if (!ReadPointer(lapChecker + 0x2C, array)) { g_observation = o; return; }
            kart = array + player * 0x44;
            if (!PlausibleAddress(kart)) { g_observation = o; return; }
            o.stage = "Invalid kart lap/checkpoint";
            if (!ReadWord(kart + 0x1C, lap) || lap > 9 ||
                !Process::Read8(kart + 0x0D, cp))
            { g_observation = o; return; }
            o.stage = "Invalid completion";
            if (!ReadNumber(kart + 0x14, o.progress) ||
                !ReadNumber(kart + 0x18, o.maxProgress) ||
                o.progress < -0.5f || o.progress > 10.0f ||
                o.maxProgress < -0.5f || o.maxProgress > 10.0f)
            { g_observation = o; return; }
            o.stage = "Invalid position";
            if (!ReadNumber(kart + 0x28, o.x) ||
                !ReadNumber(kart + 0x2C, o.y) ||
                !ReadNumber(kart + 0x30, o.z) ||
                std::fabs(o.x) > 1000000.0f ||
                std::fabs(o.y) > 1000000.0f ||
                std::fabs(o.z) > 1000000.0f)
            { g_observation = o; return; }

            o.candidate = true;
            o.lap = static_cast<int>(lap) + 1;
            o.checkpoint = cp;
            o.stage = "Candidate telemetry (NOT verified)";

            // Optional speed from Vehicle. Pointer failure is not fatal.
            u32 director = 0, units = 0, unit = 0, vehicle = 0;
            if (ReadPointer(collection + 0x10, director) &&
                ReadPointer(director + 0x2C, units) &&
                ReadPointer(units + player * 4, unit) &&
                ReadPointer(unit + 0x2C, vehicle) &&
                ReadNumber(vehicle + 0xF2C, o.speed) &&
                o.speed >= -20.0f && o.speed < 500.0f)
                o.speedValid = true;

            g_observation = o;
        }

        bool DrawStatus(const Screen &screen)
        {
            if (screen.IsTop || !g_verified)
                return true;

            // P001's hardware-approved position remains unchanged.
            screen.DrawRect(2, 224, 156, 14, Color::Black, true);
            screen.Draw(g_ready ? "CK P003 | READY" : "CK P003 | INACTIVE",
                        5, 226, g_ready ? Color::Green : Color::White, Color::Black);

            // Sharing the same probe avoids duplicate reads when showing
            // telemetry and recording simultaneously.
            if (g_ready && (g_live || g_recording))
            {
                if (++g_frames % 30 == 0)
                {
                    Probe();
                    if (g_recording)
                        CaptureSample();
                }
            }
            if (g_live && g_ready)
            {
                char line[96];
                if (g_observation.candidate)
                {
                    if (g_observation.speedValid)
                        std::snprintf(line, sizeof(line),
                                      "OBS? L%d CP%u P%.2f SPD%.1f",
                                      g_observation.lap, g_observation.checkpoint,
                                      g_observation.progress, g_observation.speed);
                    else
                        std::snprintf(line, sizeof(line),
                                      "OBS? L%d CP%u P%.2f SPD?",
                                      g_observation.lap, g_observation.checkpoint,
                                      g_observation.progress);
                }
                else
                    std::snprintf(line, sizeof(line), "OBS? NO DATA -- see diagnostics");
                screen.DrawRect(2, 211, 299, 12, Color::Black, true);
                screen.Draw(line, 5, 212, Color::Yellow, Color::Black);
            }
            if (g_recording && g_ready)
            {
                char rec[44];
                std::snprintf(rec, sizeof(rec), "REC %u/%u", g_count, MAX_LOG_SAMPLES);
                screen.DrawRect(185, 224, 115, 14, Color::Black, true);
                screen.Draw(rec, 187, 226, Color::Green, Color::Black);
            }
            else if (g_bufferFull)
            {
                screen.DrawRect(185, 224, 115, 14, Color::Black, true);
                screen.Draw("BUFFER FULL", 187, 226, Color::Yellow, Color::Black);
            }
            return true;
        }

        void ToggleReady(MenuEntry *)
        {
            g_ready = !g_ready;
            if (!g_ready)
                g_recording = false; // Stop, but never discard pending data.
            ++g_toggles;
        }
        void ToggleLive(MenuEntry *)
        {
            g_live = !g_live;
            g_frames = 0;
            g_observation = Observation();
        }
        void ShowDiagnostics(MenuEntry *)
        {
            std::string msg = std::string("Game: MK7 (") + g_region +
                ")\nTitle ID: VERIFIED\nStatus: " +
                (g_ready ? "READY" : "INACTIVE") +
                "\nToggle count: " + std::to_string(g_toggles) +
                "\nTelemetry overlay: " + (g_live ? "ON" : "OFF") +
                "\nSamples: " + std::to_string(g_samples) +
                "\nProbe: " + g_observation.stage +
                "\nRecorder: " + (g_recording ? "RECORDING" : "STOPPED") +
                "\nBuffered rows: " + std::to_string(g_count) +
                "\nInvalid rows: " + std::to_string(g_invalidSamples) +
                "\nLast CSV: " + g_lastFile +
                "\nRace detection: NOT VERIFIED\n"
                "Controls: DISABLED\nLearning: DISABLED";
            MessageBox("Collie Kart P003 Diagnostics", msg)();
        }
        void Snapshot(MenuEntry *)
        {
            Probe();
            std::string msg = "Source: USA rev1/rev2 emulator\n";
            msg += std::string("Probe: ") + g_observation.stage + "\n";
            if (g_observation.candidate)
            {
                char report[255];
                std::snprintf(report, sizeof(report),
                    "Lap candidate: %d\nCheckpoint: %u\n"
                    "Completion: %.4f\nMax completion: %.4f\n"
                    "Position: %.1f, %.1f, %.1f\n",
                    g_observation.lap, g_observation.checkpoint,
                    g_observation.progress, g_observation.maxProgress,
                    g_observation.x, g_observation.y, g_observation.z);
                msg += report;
                if (g_observation.speedValid)
                {
                    std::snprintf(report, sizeof(report),
                                  "Speed: %.3f (game units)\n",
                                  g_observation.speed);
                    msg += report;
                }
                else
                    msg += "Speed: UNAVAILABLE\n";
            }
            msg += "\nCompare values against real gameplay.";
            MessageBox("P003 Experimental Snapshot", msg)();
        }
    }

    // No patches and no writes to MK7 process memory.
    void PatchProcess(FwkSettings &) {}
    void OnProcessExit(void) {}

    int main(void)
    {
        const u64 title = Process::GetTitleID();
        if (title == MK7_USA) { g_verified = true; g_region = "USA"; }
        else if (title == MK7_EUR) { g_verified = true; g_region = "EUR"; }
        else if (title == MK7_JPN) { g_verified = true; g_region = "JPN"; }
        if (!g_verified)
            return 0;

        PluginMenu menu("Collie Kart P003", 0, 0, 3,
            "Passive buffered SD recorder.\n"
            "No game-memory writes or autonomous controls.");
        menu.SynchronizeWithFrame(true);
        menu.ShowWelcomeMessage(false);
        menu += new MenuEntry("Toggle READY / INACTIVE", nullptr, ToggleReady,
            "Retains P001's display-only status toggle.");
        menu += new MenuEntry("Show diagnostics", nullptr, ShowDiagnostics,
            "Status of the last attempted telemetry probe.");
        menu += new MenuEntry("Read telemetry snapshot", nullptr, Snapshot,
            "Attempt a single read-only game-memory snapshot.");
        menu += new MenuEntry("Toggle live telemetry", nullptr, ToggleLive,
            "Opt-in sampled observations, only while READY.");
        menu += new MenuEntry("Start recording", nullptr, StartRecording,
            "Begin buffering telemetry in RAM. Requires READY.");
        menu += new MenuEntry("Stop & Save CSV", nullptr, StopAndSave,
            "Stop capturing and save a new CSV under SD:/CollieKart.");
        OSD::Run(DrawStatus);
        menu.Run();
        OSD::Stop(DrawStatus);
        return 0;
    }
}
