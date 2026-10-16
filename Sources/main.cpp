// Collie Kart Portable P005 -- passive sensor fusion and reward diagnostics.
// Compiled research prototype. No controller injection, game RAM writes,
// learning or automatic SD writes. Stop & Save CSV is opt-in.
// P001 HUD, P002 telemetry, P003 flight recorder and P004 rewards preserved.

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
        unsigned g_toggles = 0, g_samples = 0;
        unsigned g_invalidSamples = 0;
        // Time-based sampling avoids P004's assumption that OSD ran at 60Hz.
        constexpr u64 TELEMETRY_INTERVAL_MS = 500;
        constexpr u64 ENVIRONMENT_INTERVAL_MS = 100; // 10Hz target
        constexpr u64 MAX_EVENT_GAP_MS = 350;
        constexpr u64 BOOST_EVENT_COOLDOWN_MS = 400;
        u64 g_lastTelemetryAt = 0, g_lastEnvironmentAt = 0;
        u32 g_lastCaptureElapsed = 0;
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

        // Mode classification and candidate MK7 VehicleMove flags.
        enum Mode { UNKNOWN = 0, GROUND = 1, UNDERWATER = 2,
                    AIRBORNE = 3, GLIDING = 4 };
        const char *ModeName(Mode m)
        {
            switch (m)
            {
                case GROUND: return "GROUND";
                case UNDERWATER: return "UNDERWATER";
                case AIRBORNE: return "AIRBORNE";
                case GLIDING: return "GLIDING";
                default: return "UNKNOWN";
            }
        }

        struct Environment
        {
            bool valid = false;
            Mode mode = UNKNOWN;
            u32 statusFlags = 0, boostFlags = 0;
            u32 airFrames = 0, groundFrames = 0;
            u32 surfaceCode = 0xFFFFFFFF;
            u32 dashType = 0xFFFFFFFF;
            bool brakeHeld = false; // Candidate physical B / brake input.
        };
        Environment g_environment;
        u32 g_vehicleAddress = 0;
        bool g_havePreviousFlags = false;
        u32 g_previousStatus = 0, g_previousBoost = 0;
        u32 g_previousDashType = 0xFFFFFFFF;
        u64 g_previousModeAt = 0, g_recentBrakeAt = 0;
        u64 g_previousBoostEventAt = 0;
        Mode g_previousMode = UNKNOWN;
        unsigned g_gliderPadActivations = 0, g_boostPadActivations = 0;
        unsigned g_gliderDeployments = 0, g_forceCloseSignals = 0;
        unsigned g_airClosures = 0, g_landingClosures = 0;
        unsigned g_dashPanelEdges = 0, g_boostFlagEdges = 0;
        unsigned g_brakeWhileGliding = 0, g_brakeLinkedClosures = 0;
        unsigned g_envPolls = 0, g_envReadFails = 0;
        u32 g_envPollGapMs = 0, g_envLastReadMs = 0, g_envMaxReadMs = 0;
        u32 g_modeMs[5] = {0, 0, 0, 0, 0};

        // The reward evaluator is PASSIVE. Only new max progress receives
        // forward reward; repeated traversal of old progress does not.
        bool g_rewardAnchored = false;
        float g_previousRewardMaximum = 0;
        int g_previousRewardLap = 0;
        u32 g_previousRewardAt = 0;
        float g_totalReward = 0;
        float g_latestReward = 0;

        void ResetEnvironmentSession()
        {
            g_environment = Environment();
            g_vehicleAddress = 0;
            g_havePreviousFlags = false;
            g_previousStatus = g_previousBoost = 0;
            g_previousDashType = 0xFFFFFFFF;
            g_previousModeAt = g_recentBrakeAt = 0;
            g_previousBoostEventAt = 0;
            g_dashPanelEdges = g_boostFlagEdges = 0;
            g_brakeWhileGliding = g_brakeLinkedClosures = 0;
            g_envPolls = g_envReadFails = 0;
            g_envPollGapMs = g_envLastReadMs = g_envMaxReadMs = 0;
            g_lastTelemetryAt = g_lastEnvironmentAt = 0;
            g_lastCaptureElapsed = 0;
            g_previousMode = UNKNOWN;
            g_gliderPadActivations = g_boostPadActivations = 0;
            g_gliderDeployments = g_forceCloseSignals = 0;
            g_airClosures = g_landingClosures = 0;
            for (unsigned i = 0; i < 5; ++i)
                g_modeMs[i] = 0;
            g_rewardAnchored = false;
            g_previousRewardMaximum = 0;
            g_previousRewardLap = 0;
            g_previousRewardAt = 0;
            g_totalReward = g_latestReward = 0;
        }

        // RAM-only buffer: 2 samples/sec x 15 minutes maximum.
        // No filesystem writes happen from DrawStatus or Probe.
        constexpr unsigned MAX_LOG_SAMPLES = 1800;
        struct Record
        {
            u32 elapsedMs = 0;
            Observation value;
            Environment environment;
            float rewardProgress = 0, rewardTime = 0, rewardLap = 0;
            float rewardTotal = 0, cumulativeReward = 0;
            unsigned gliderPads = 0, boostPads = 0, deployments = 0;
            unsigned forceClosures = 0, midairClosures = 0, landedClosures = 0;
            u32 durationMs[5] = {0, 0, 0, 0, 0};
            unsigned dashPanelEdges = 0, boostFlagEdges = 0;
            unsigned brakeWhileGliding = 0, brakeLinkedClosures = 0;
            unsigned envPolls = 0, envReadFails = 0;
            u32 pollGapMs = 0, pollReadMs = 0, maxPollReadMs = 0;
            u32 sampleGapMs = 0;
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
            r.sampleGapMs = g_lastCaptureElapsed != 0 &&
                r.elapsedMs >= g_lastCaptureElapsed
                ? r.elapsedMs - g_lastCaptureElapsed : 0;
            g_lastCaptureElapsed = r.elapsedMs;

            r.value = g_observation;
            r.environment = g_environment;
            r.gliderPads = g_gliderPadActivations;
            r.boostPads = g_boostPadActivations;
            r.deployments = g_gliderDeployments;
            r.forceClosures = g_forceCloseSignals;
            r.midairClosures = g_airClosures;
            r.landedClosures = g_landingClosures;
            r.dashPanelEdges = g_dashPanelEdges;
            r.boostFlagEdges = g_boostFlagEdges;
            r.brakeWhileGliding = g_brakeWhileGliding;
            r.brakeLinkedClosures = g_brakeLinkedClosures;
            r.envPolls = g_envPolls;
            r.envReadFails = g_envReadFails;
            r.pollGapMs = g_envPollGapMs;
            r.pollReadMs = g_envLastReadMs;
            r.maxPollReadMs = g_envMaxReadMs;
            for (unsigned i = 0; i < 5; ++i)
                r.durationMs[i] = g_modeMs[i];

            if (r.value.candidate)
            {
                if (!g_rewardAnchored)
                {
                    g_rewardAnchored = true;
                    g_previousRewardMaximum = r.value.maxProgress;
                    g_previousRewardLap = r.value.lap;
                    g_previousRewardAt = r.elapsedMs;
                }
                else
                {
                    // Genuine progress and valid lap transitions will be
                    // checked against real-time gameplay before training.
                    float delta = r.value.maxProgress - g_previousRewardMaximum;
                    float seconds = r.elapsedMs >= g_previousRewardAt
                        ? (r.elapsedMs - g_previousRewardAt) * 0.001f : 0.0f;
                    // Ignore menu pauses/huge intervals for time penalties.
                    if (seconds <= 3.0f)
                        r.rewardTime = -0.1f * seconds;
                    // Reject abrupt progress jumps or backwards max readings.
                    if (delta > 0.0f && delta <= 0.10f)
                        r.rewardProgress = 100.0f * delta;
                    // Count one upward lap transition at most, not a reset.
                    if (r.value.lap == g_previousRewardLap + 1 &&
                        delta >= 0.0f && delta <= 0.10f)
                        r.rewardLap = 20.0f;

                    if (r.value.maxProgress >= g_previousRewardMaximum)
                        g_previousRewardMaximum = r.value.maxProgress;
                    g_previousRewardLap = r.value.lap;
                    g_previousRewardAt = r.elapsedMs;
                }
            }
            // No automatic brake/glider penalties yet: no reliable
            // brake-to-closure causal detector is implemented.
            r.rewardTotal = r.rewardProgress + r.rewardTime + r.rewardLap;
            g_totalReward += r.rewardTotal;
            r.cumulativeReward = g_totalReward;
            g_latestReward = r.rewardTotal;

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
                    "P005's memory layout is currently supported only for MK7 USA.")();
                return;
            }
            if (!g_ready)
            {
                MessageBox("Recorder inactive",
                    "First toggle CK P005 to READY, then start recording.")();
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
            g_lastTelemetryAt = g_lastEnvironmentAt = 0;
            g_invalidSamples = 0;
            g_observation = Observation();
            ResetEnvironmentSession();
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
                "pos_x,pos_y,pos_z,speed_valid,speed_game_units,probe_stage,"
                "mode,mode_valid,status_flags,boost_flags,air_frames,ground_frames,"
                "surface_code,dash_type,glider_pad_activations,boost_pad_activations,"
                "glider_deployments,force_close_signals,midair_glider_closures,"
                "landing_glider_closures,ground_ms,underwater_ms,airborne_ms,gliding_ms,"
                "reward_progress,reward_time,reward_lap,reward_total,"
                "reward_cumulative,"
                "dash_panel_edges,boost_flag_edges,brake_held,"
                "brake_while_gliding_polls,brake_linked_midair_closures,"
                "environment_polls,environment_read_failures,"
                "environment_poll_gap_ms,environment_read_ms,"
                "environment_max_read_ms,sample_gap_ms") == File::SUCCESS);

            char line[900];
            for (unsigned i = 0; saved && i < g_count; ++i)
            {
                const Record &r = g_records[i];
                const Observation &v = r.value;
                const Environment &e = r.environment;
                int length = std::snprintf(line, sizeof(line),
                    "%lu,%d,%d,%u,%.6f,%.6f,%.3f,%.3f,%.3f,%d,%.4f,%s,"
                    "%s,%d,0x%08lX,0x%08lX,%lu,%lu,%lu,%lu,"
                    "%u,%u,%u,%u,%u,%u,%lu,%lu,%lu,%lu,"
                    "%.4f,%.4f,%.4f,%.4f,%.4f,"
                    "%u,%u,%d,%u,%u,%u,%u,%lu,%lu,%lu,%lu",
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
                    v.stage,
                    ModeName(e.mode), e.valid ? 1 : 0,
                    static_cast<unsigned long>(e.statusFlags),
                    static_cast<unsigned long>(e.boostFlags),
                    static_cast<unsigned long>(e.airFrames),
                    static_cast<unsigned long>(e.groundFrames),
                    static_cast<unsigned long>(e.surfaceCode),
                    static_cast<unsigned long>(e.dashType),
                    r.gliderPads, r.boostPads, r.deployments,
                    r.forceClosures, r.midairClosures, r.landedClosures,
                    static_cast<unsigned long>(r.durationMs[GROUND]),
                    static_cast<unsigned long>(r.durationMs[UNDERWATER]),
                    static_cast<unsigned long>(r.durationMs[AIRBORNE]),
                    static_cast<unsigned long>(r.durationMs[GLIDING]),
                    r.rewardProgress, r.rewardTime, r.rewardLap,
                    r.rewardTotal, r.cumulativeReward,
                    r.dashPanelEdges, r.boostFlagEdges,
                    e.brakeHeld ? 1 : 0,
                    r.brakeWhileGliding, r.brakeLinkedClosures,
                    r.envPolls, r.envReadFails,
                    static_cast<unsigned long>(r.pollGapMs),
                    static_cast<unsigned long>(r.pollReadMs),
                    static_cast<unsigned long>(r.maxPollReadMs),
                    static_cast<unsigned long>(r.sampleGapMs));
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
            g_vehicleAddress = 0;

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
                ReadPointer(unit + 0x2C, vehicle))
            {
                g_vehicleAddress = vehicle;
                if (ReadNumber(vehicle + 0xF2C, o.speed) &&
                    o.speed >= -20.0f && o.speed < 500.0f)
                    o.speedValid = true;
            }

            g_observation = o;
        }


        // Poll frequently, in RAM only. Rising edges avoid counting one pad
        // for every game frame that its flag remains active.
        void PollEnvironment()
        {
            Environment next;
            if (g_vehicleAddress == 0 || !g_observation.candidate)
            {
                g_environment = next;
                g_havePreviousFlags = false;
                g_previousModeAt = 0;
                return;
            }

            const u32 v = g_vehicleAddress;
            if (!ReadWord(v + 0xC30, next.statusFlags) ||
                !ReadWord(v + 0xC28, next.boostFlags) ||
                !ReadWord(v + 0xD48, next.airFrames) ||
                !ReadWord(v + 0xD50, next.groundFrames))
            {
                g_environment = next;
                g_havePreviousFlags = false;
                g_previousModeAt = 0;
                return;
            }

            // Optional research fields. Unknown = 0xFFFFFFFF.
            u32 raw = 0;
            if (ReadWord(v + 0xD14, raw) && raw <= 31)
                next.surfaceCode = raw;
            if (ReadWord(v + 0xF98, raw) && raw <= 8)
                next.dashType = raw;

            next.valid = true;
            if ((next.statusFlags & 0x80) != 0)
                next.mode = GLIDING;
            else if ((next.statusFlags & 0x1000) != 0)
                next.mode = UNDERWATER;
            else if (next.airFrames > 1 && next.groundFrames == 0)
                next.mode = AIRBORNE;
            else if (next.groundFrames > 0)
                next.mode = GROUND;
            else
                next.mode = UNKNOWN;

            u64 now = osGetTime();
            // Exclude menu pauses from accumulated state durations; prevent
            // stale flag-edge events after unexpectedly long polling gaps.
            bool continuous = g_previousModeAt != 0 &&
                now >= g_previousModeAt && now - g_previousModeAt <= 200;
            if (g_recording && continuous && g_previousMode > UNKNOWN &&
                g_previousMode <= GLIDING)
                g_modeMs[g_previousMode] += static_cast<u32>(now - g_previousModeAt);

            if (g_recording && g_havePreviousFlags && continuous)
            {
                const u32 risingStatus = next.statusFlags & ~g_previousStatus;
                const u32 risingBoost = next.boostFlags & ~g_previousBoost;

                if (risingStatus & 0x100)
                    ++g_gliderPadActivations;
                if (risingBoost & 0x2)
                    ++g_boostPadActivations;
                if (risingStatus & 0x80)
                    ++g_gliderDeployments;
                if (risingStatus & 0x400)
                    ++g_forceCloseSignals;

                // End-of-glide classification is a hypothesis:
                // landing and midair closures are NOT yet penalties.
                if ((g_previousStatus & 0x80) &&
                    !(next.statusFlags & 0x80))
                {
                    if (next.groundFrames > 0)
                        ++g_landingClosures;
                    else
                        ++g_airClosures;
                }
            }

            g_environment = next;
            g_previousStatus = next.statusFlags;
            g_previousBoost = next.boostFlags;
            g_previousMode = next.mode;
            g_previousModeAt = now;
            g_havePreviousFlags = true;
        }

        bool DrawStatus(const Screen &screen)
        {
            if (screen.IsTop || !g_verified)
                return true;

            // P001's hardware-approved position remains unchanged.
            screen.DrawRect(2, 224, 156, 14, Color::Black, true);
            screen.Draw(g_ready ? "CK P004 | READY" : "CK P004 | INACTIVE",
                        5, 226, g_ready ? Color::Green : Color::White, Color::Black);

            // Sharing the same probe avoids duplicate reads when showing
            // telemetry and recording simultaneously.
            if (g_ready && (g_live || g_recording))
            {
                if (++g_frames % 30 == 0)
                    Probe();
                // Lightweight candidate flag polling once per display frame.
                // Avoid capturing new CSV rows until mode was polled.
                PollEnvironment();
                if (g_recording && g_frames % 30 == 0)
                    CaptureSample();
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
                // State label is drawn only when optional telemetry is ON.
                // It shares the right HUD area when NOT recording.
                if (!g_recording)
                {
                    screen.DrawRect(185, 224, 115, 14, Color::Black, true);
                    screen.Draw(ModeName(g_environment.mode), 187, 226,
                                Color::Yellow, Color::Black);
                }
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
            {
                g_recording = false; // Preserve unsaved data.
                g_havePreviousFlags = false;
                g_previousModeAt = 0;
            }
            ++g_toggles;
        }
        void ToggleLive(MenuEntry *)
        {
            g_live = !g_live;
            g_frames = 0;
            g_observation = Observation();
            g_vehicleAddress = 0;
            g_environment = Environment();
            g_havePreviousFlags = false;
            g_previousModeAt = 0;
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
                "\nMode: " + ModeName(g_environment.mode) +
                "\nPads glide/boost: " + std::to_string(g_gliderPadActivations) +
                    "/" + std::to_string(g_boostPadActivations) +
                "\nReward total (test): " + std::to_string(g_totalReward) +
                "\nLast CSV: " + g_lastFile +
                "\nRace detection: NOT VERIFIED\n"
                "Controls: DISABLED\nLearning: DISABLED";
            MessageBox("Collie Kart P004 Diagnostics", msg)();
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
            MessageBox("P004 Experimental Snapshot", msg)();
        }

        void ShowEnvironment(MenuEntry *)
        {
            // Menu is paused: attempt a fresh, read-only snapshot first.
            Probe();
            PollEnvironment();
            const Environment &e = g_environment;
            char lines[560];
            std::snprintf(lines, sizeof(lines),
                "Mode: %s (%s)\n"
                "Flags: status 0x%08lX boost 0x%08lX\n"
                "Air frames: %lu  Ground frames: %lu\n"
                "KCL surface: %lu  Dash type: %lu\n"
                "Glide pads: %u  Boost pads: %u\n"
                "Glider deploys: %u\n"
                "Force-close flag: %u\n"
                "Midair closes: %u  Land closes: %u\n"
                "Ground: %lus  Water: %lus\n"
                "Air: %lus  Glide: %lus\n"
                "Last reward: %.3f  Total: %.3f\n"
                "Manual brake penalty: NOT ENABLED",
                ModeName(e.mode), e.valid ? "candidate" : "NO DATA",
                static_cast<unsigned long>(e.statusFlags),
                static_cast<unsigned long>(e.boostFlags),
                static_cast<unsigned long>(e.airFrames),
                static_cast<unsigned long>(e.groundFrames),
                static_cast<unsigned long>(e.surfaceCode),
                static_cast<unsigned long>(e.dashType),
                g_gliderPadActivations, g_boostPadActivations,
                g_gliderDeployments, g_forceCloseSignals,
                g_airClosures, g_landingClosures,
                static_cast<unsigned long>(g_modeMs[GROUND] / 1000),
                static_cast<unsigned long>(g_modeMs[UNDERWATER] / 1000),
                static_cast<unsigned long>(g_modeMs[AIRBORNE] / 1000),
                static_cast<unsigned long>(g_modeMs[GLIDING] / 1000),
                g_latestReward, g_totalReward);
            MessageBox("P004 Environment & Rewards", lines)();
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

        PluginMenu menu("Collie Kart P004", 0, 0, 4,
            "Passive environment and reward evaluation.\n"
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
        menu += new MenuEntry("Environment & rewards", nullptr, ShowEnvironment,
            "Read candidate driving mode, pad activations and reward counters.");
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
