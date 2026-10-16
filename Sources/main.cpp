// Collie Kart Portable P006 -- experimental profiling and first-contact HID.
// No writes to MK7 process memory or autonomous steering / machine learning.
// One opt-in HID InjectKey(A) experiment after a full dry-run; release not yet
// hardware-verified, so each experiment is isolated and deliberately short.
// All prior P001-P005 sensor and SD recorder paths remain supported.

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

        // PROFILER: callback cadence is a proxy, NOT verified game FPS.
        bool g_profilerEnabled = false;
        u64 g_prevCallbackAt = 0;
        u32 g_profileCallbackGapMs = 0, g_profileCallbackWorkMs = 0;
        u32 g_profileProbeMs = 0, g_profileMaxGapMs = 0, g_profileMaxWorkMs = 0;
        unsigned g_profileCallbacks = 0, g_profileLargeGaps = 0;
        u64 g_profileWorkTotalMs = 0;

        // SAFETY: the injection experiment is one-shot; never a held controller.
        // FRAMEWORK DETAILS: Controller::InjectKey ORs a key into 8 HID buffers.
        // It has no guaranteed release API. We cease writing after one burst,
        // relying on normal HID refresh. Physical B or START requests abort.
        enum ControlPhase { DISARMED = 0, WAIT_CLOSE = 1,
            COUNTDOWN = 2, PULSE = 3, FINISHED = 4, ABORTED = 5 };
        const char *ControlName(ControlPhase phase)
        {
            switch (phase)
            {
                case WAIT_CLOSE: return "WAIT_CLOSE";
                case COUNTDOWN: return "COUNTDOWN";
                case PULSE: return "PULSE";
                case FINISHED: return "FINISHED";
                case ABORTED: return "ABORTED";
                default: return "DISARMED";
            }
        }
        ControlPhase g_control = DISARMED;
        bool g_dryRunComplete = false;
        bool g_physicalAbortVerified = false;
        bool g_controlIsDryRun = true;
        u64 g_controlNextAt = 0, g_controlPulseStart = 0;
        u64 g_controlLastCallbackAt = 0;
        unsigned g_hidInjectionCalls = 0, g_controlAborts = 0;
        const char *g_lastAbortReason = "None";
        // Hard bounds: ~1.5s idle countdown, <=80ms injection and single-use.
        constexpr u64 FIRST_CONTACT_COUNTDOWN_MS = 1500;
        constexpr u64 FIRST_CONTACT_MAX_PULSE_MS = 80;
        constexpr u64 FIRST_CONTACT_MAX_GAP_MS = 250;
        u32 g_vehicleControlsObserved = 0;
        bool g_vehicleControlsValid = false;

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
            u32 callbackGapMs = 0, callbackWorkMs = 0, probeWorkMs = 0;
            unsigned callbackCount = 0, slowCallbacks = 0;
            unsigned injectionCalls = 0, abortedCount = 0;
            bool vehicleControlsValid = false;
            u32 vehicleControls = 0;
            ControlPhase controlPhase = DISARMED;
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
            r.callbackGapMs = g_profileCallbackGapMs;
            r.callbackWorkMs = g_profileCallbackWorkMs;
            r.probeWorkMs = g_profileProbeMs;
            r.callbackCount = g_profileCallbacks;
            r.slowCallbacks = g_profileLargeGaps;
            r.injectionCalls = g_hidInjectionCalls;
            r.abortedCount = g_controlAborts;
            r.vehicleControlsValid = g_vehicleControlsValid;
            r.vehicleControls = g_vehicleControlsObserved;
            r.controlPhase = g_control;

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
                    "P006's memory layout is currently supported only for MK7 USA.")();
                return;
            }
            if (!g_ready)
            {
                MessageBox("Recorder inactive",
                    "First toggle CK P006 to READY, then start recording.")();
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
                "environment_max_read_ms,sample_gap_ms,"
                "callback_gap_ms,callback_work_ms,probe_read_ms,"
                "profiler_callbacks,callback_gaps_over_50ms,"
                "control_phase,hid_injection_calls,control_aborts,"
                "mk7_controls_valid,mk7_controls_raw") == File::SUCCESS);

            char line[1100];
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
                    "%u,%u,%d,%u,%u,%u,%u,%lu,%lu,%lu,%lu,"
                    "%lu,%lu,%lu,%u,%u,%s,%u,%u,%d,0x%08lX",
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
                    static_cast<unsigned long>(r.sampleGapMs),
                    static_cast<unsigned long>(r.callbackGapMs),
                    static_cast<unsigned long>(r.callbackWorkMs),
                    static_cast<unsigned long>(r.probeWorkMs),
                    r.callbackCount, r.slowCallbacks,
                    ControlName(r.controlPhase), r.injectionCalls,
                    r.abortedCount, r.vehicleControlsValid ? 1 : 0,
                    static_cast<unsigned long>(r.vehicleControls));
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


        // At most ~10Hz based on elapsed milliseconds, not frame count.
        // Read-only: no file operations and no changes to the MK7 process.
        // Snapshot from the plugin menu does not count events or time.
        void PollEnvironment(bool menuSnapshot = false)
        {
            const u64 started = osGetTime();
            Environment next;
            if (!menuSnapshot)
            {
                ++g_envPolls;
                if (g_previousModeAt && started >= g_previousModeAt)
                    g_envPollGapMs = static_cast<u32>(started - g_previousModeAt);
                else
                    g_envPollGapMs = 0;
            }

            if (g_vehicleAddress == 0 || !g_observation.candidate)
            {
                g_environment = next;
                g_havePreviousFlags = false;
                g_previousModeAt = 0;
                if (!menuSnapshot) ++g_envReadFails;
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
                if (!menuSnapshot) ++g_envReadFails;
                return;
            }

            // Keep both pad-detection signals, plus raw source fields.
            u32 raw = 0;
            if (ReadWord(v + 0xF98, raw) && raw <= 8)
                next.dashType = raw;
            if (ReadWord(v + 0xD14, raw) && raw <= 31)
                next.surfaceCode = raw;

            // P005: candidate B input telemetry, never injected or modified.
            // This input read may need calibration against physical controls.
            next.brakeHeld = Controller::IsKeyDown(Key::B);
            next.valid = true;

            if (next.statusFlags & 0x80)
                next.mode = GLIDING;
            else if (next.statusFlags & 0x1000)
                next.mode = UNDERWATER;
            else if (next.airFrames > 1 && next.groundFrames == 0)
                next.mode = AIRBORNE;
            else if (next.groundFrames > 0)
                next.mode = GROUND;
            else
                next.mode = UNKNOWN;

            // Reading the menu must not create fake triggers or durations.
            if (menuSnapshot)
            {
                g_environment = next;
                g_havePreviousFlags = false;
                g_previousModeAt = 0;
                return;
            }

            const bool continuous = g_previousModeAt != 0 &&
                started >= g_previousModeAt &&
                started - g_previousModeAt <= MAX_EVENT_GAP_MS;
            if (g_recording && continuous &&
                g_previousMode > UNKNOWN && g_previousMode <= GLIDING)
                g_modeMs[g_previousMode] +=
                    static_cast<u32>(started - g_previousModeAt);

            if (g_recording && next.brakeHeld)
            {
                g_recentBrakeAt = started;
                if (next.mode == GLIDING)
                    ++g_brakeWhileGliding;
            }

            if (g_recording && g_havePreviousFlags && continuous)
            {
                const u32 risingStatus = next.statusFlags & ~g_previousStatus;
                const u32 risingBoost = next.boostFlags & ~g_previousBoost;
                const bool padBoostFlagEdge = (risingBoost & 0x2) != 0;
                const bool dashPanelEdge = (next.dashType == 6 &&
                                            g_previousDashType != 6);

                if (risingStatus & 0x100)
                    ++g_gliderPadActivations;
                if (risingStatus & 0x80)
                    ++g_gliderDeployments;
                if (risingStatus & 0x400)
                    ++g_forceCloseSignals;

                // Two different sources, one deduplicated pad-hit counter.
                // dashType==6 is DASH_PANEL per mk7re/MK7-Memory.
                if (padBoostFlagEdge)
                    ++g_boostFlagEdges;
                if (dashPanelEdge)
                    ++g_dashPanelEdges;
                if (padBoostFlagEdge || dashPanelEdge)
                {
                    if (g_previousBoostEventAt == 0 ||
                        started - g_previousBoostEventAt >= BOOST_EVENT_COOLDOWN_MS)
                    {
                        ++g_boostPadActivations;
                        g_previousBoostEventAt = started;
                    }
                }

                // Do not assign any manual glider penalty yet.
                if ((g_previousStatus & 0x80) &&
                    !(next.statusFlags & 0x80))
                {
                    if (next.groundFrames > 0)
                        ++g_landingClosures;
                    else
                    {
                        ++g_airClosures;
                        if (g_recentBrakeAt &&
                            started >= g_recentBrakeAt &&
                            started - g_recentBrakeAt <= MAX_EVENT_GAP_MS)
                            ++g_brakeLinkedClosures;
                    }
                }
            }

            g_environment = next;
            g_previousStatus = next.statusFlags;
            g_previousBoost = next.boostFlags;
            g_previousDashType = next.dashType;
            g_previousMode = next.mode;
            g_previousModeAt = started;
            g_havePreviousFlags = true;

            const u64 completed = osGetTime();
            g_envLastReadMs = completed >= started
                ? static_cast<u32>(completed - started) : 0;
            if (g_envLastReadMs > g_envMaxReadMs)
                g_envMaxReadMs = g_envLastReadMs;
        }


        void AbortControl(const char *reason)
        {
            if (g_control == WAIT_CLOSE || g_control == COUNTDOWN ||
                g_control == PULSE)
            {
                g_control = ABORTED;
                g_lastAbortReason = reason;
                ++g_controlAborts;
                if (reason && reason[0] == 'B' && reason[1] == '/')
                    g_physicalAbortVerified = true;
                // Do not inject any further artificial input.
                g_controlNextAt = 0;
                g_controlPulseStart = 0;
            }
        }

        bool ControllerExperimentOK()
        {
            // A valid local observation is required throughout the pulse.
            if (!g_ready || Process::GetTitleID() != MK7_USA ||
                !g_observation.candidate || !g_observation.speedValid ||
                !g_environment.valid || g_environment.mode != GROUND ||
                g_vehicleAddress == 0)
                return false;
            // Only stationary start of the pulse; we do not abort merely
            // because the kart starts moving after acceleration succeeds.
            return true;
        }

        bool UserAbortPressed()
        {
            return Controller::IsKeyDown(Key::B) ||
                   Controller::IsKeyDown(Key::Start);
        }

        void ControlStep(u64 now, bool menuOpen)
        {
            if (g_control != WAIT_CLOSE && g_control != COUNTDOWN &&
                g_control != PULSE)
                return;
            if (menuOpen)
            {
                // Selecting the menu during a queued experiment aborts it.
                // The initial wait state is allowed while the user exits.
                if (g_control != WAIT_CLOSE)
                    AbortControl("Plugin menu reopened");
                return;
            }

            if (!ControllerExperimentOK())
            {
                AbortControl("No valid grounded telemetry");
                return;
            }
            if (UserAbortPressed())
            {
                AbortControl("B/START physical override");
                return;
            }
            if (g_control == WAIT_CLOSE)
            {
                if (g_observation.speed > 0.35f ||
                    g_observation.speed < -0.35f)
                {
                    AbortControl("Kart already moving");
                    return;
                }
                g_control = COUNTDOWN;
                g_controlNextAt = now + FIRST_CONTACT_COUNTDOWN_MS;
                g_controlLastCallbackAt = now;
                return;
            }
            if (g_controlLastCallbackAt != 0 &&
                (now < g_controlLastCallbackAt ||
                 now - g_controlLastCallbackAt > FIRST_CONTACT_MAX_GAP_MS))
            {
                AbortControl("Game callback gap too long");
                return;
            }
            g_controlLastCallbackAt = now;
            if (g_control == COUNTDOWN)
            {
                if (g_observation.speed > 0.35f ||
                    g_observation.speed < -0.35f)
                {
                    AbortControl("Moved before pulse");
                    return;
                }
                if (now >= g_controlNextAt)
                {
                    if (g_controlIsDryRun)
                    {
                        g_control = FINISHED;
                        g_dryRunComplete = true;
                    }
                    else
                    {
                        g_control = PULSE;
                        g_controlPulseStart = now;
                    }
                }
            }
            if (g_control == PULSE)
            {
                // Single-use small A-key burst. This is NOT confirmed
                // local MK7 control or confirmed release until hardware test.
                if (now < g_controlPulseStart ||
                    now - g_controlPulseStart >= FIRST_CONTACT_MAX_PULSE_MS)
                {
                    g_control = FINISHED;
                    return;
                }
                Controller::InjectKey(static_cast<u32>(Key::A));
                ++g_hidInjectionCalls;
            }
        }

        void QueueControlExperiment(bool dryRun)
        {
            if (g_control == WAIT_CLOSE || g_control == COUNTDOWN ||
                g_control == PULSE)
            {
                MessageBox("P006 experiment already queued",
                    "Finish or abort the current experiment first.")();
                return;
            }
            if (!dryRun && (!g_dryRunComplete || !g_physicalAbortVerified))
            {
                MessageBox("P006 real pulse LOCKED",
                    "First complete one DRY RUN and separately ABORT another"
                    " dry run using physical B/START. Both checks required.")();
                return;
            }
            if (!g_ready || Process::GetTitleID() != MK7_USA)
            {
                MessageBox("P006 experiment unavailable",
                    "READY and Mario Kart 7 USA are required.")();
                return;
            }
            Probe();
            PollEnvironment(true);
            if (!ControllerExperimentOK() ||
                g_observation.speed > 0.35f ||
                g_observation.speed < -0.35f)
            {
                MessageBox("P006 test cannot arm",
                    "Please be stationary on the ground in a Time Trial with valid telemetry.")();
                return;
            }
            if (Controller::IsKeyDown(Key::A) || UserAbortPressed())
            {
                MessageBox("P006 test cannot arm",
                    "Release the physical A, B and START buttons first.")();
                return;
            }
            g_controlIsDryRun = dryRun;
            g_control = WAIT_CLOSE;
            g_controlNextAt = g_controlPulseStart = 0;
            g_controlLastCallbackAt = 0;
            MessageBox(dryRun ? "P006 DRY RUN queued" :
                "P006 REAL HID TEST queued",
                dryRun
                ? "Close the menu while stationary. After about 1.5 seconds,"
                  " P006 will finish WITHOUT any injected key. Try aborting"
                  " an additional dry run with physical B/START."
                : "CAUTION: this writes A into HID shared buffers briefly."
                  " Game acceptance and key release are UNVERIFIED."
                  " Close menu while stationary. Press physical B/START"
                  " to abort. If controls stick, end MK7 and disable plugin.")();
        }

        void QueueDryRun(MenuEntry *) { QueueControlExperiment(true); }
        void QueueRealPulse(MenuEntry *) { QueueControlExperiment(false); }

        void CancelControl(MenuEntry *)
        {
            AbortControl("Manual menu cancellation");
            MessageBox("P006 control experiment",
                "Control experiment cancelled, no additional keys will be injected.")();
        }

        void ToggleProfiler(MenuEntry *)
        {
            g_profilerEnabled = !g_profilerEnabled;
            g_prevCallbackAt = 0;
            g_profileCallbackGapMs = g_profileCallbackWorkMs = 0;
            g_profileProbeMs = g_profileMaxGapMs = g_profileMaxWorkMs = 0;
            g_profileCallbacks = g_profileLargeGaps = 0;
            g_profileWorkTotalMs = 0;
            MessageBox("P006 profiler", g_profilerEnabled
                ? "ON. Compare READY off, READY+live telemetry, and recording."
                : "OFF. Counters reset.")();
        }

        void ShowProfiler(MenuEntry *)
        {
            char msg[650];
            std::snprintf(msg, sizeof(msg),
                "Profiler: %s\n"
                "Bottom OSD callbacks: %u\n"
                "Gap last/max: %lu/%lu ms\n"
                "Gaps >50ms: %u\n"
                "Callback work last/max: %lu/%lu ms\n"
                "Average callback work: %.2f ms\n"
                "Full telemetry probe: %lu ms\n"
                "Environment polls/fails: %u/%u\n"
                "Control phase: %s\n"
                "Dry-run passed: %s\n"
                "Physical abort verified: %s\n"
                "HID injection calls: %u\n"
                "Abort count: %u\n"
                "Last abort: %s\n"
                "MK7 control bits: %s 0x%08lX",
                g_profilerEnabled ? "ON" : "OFF",
                g_profileCallbacks,
                static_cast<unsigned long>(g_profileCallbackGapMs),
                static_cast<unsigned long>(g_profileMaxGapMs),
                g_profileLargeGaps,
                static_cast<unsigned long>(g_profileCallbackWorkMs),
                static_cast<unsigned long>(g_profileMaxWorkMs),
                g_profileCallbacks ? static_cast<float>(g_profileWorkTotalMs) /
                    g_profileCallbacks : 0.0f,
                static_cast<unsigned long>(g_profileProbeMs),
                g_envPolls, g_envReadFails, ControlName(g_control),
                g_dryRunComplete ? "YES" : "NO",
                g_physicalAbortVerified ? "YES" : "NO",
                g_hidInjectionCalls, g_controlAborts,
                g_lastAbortReason, g_vehicleControlsValid ? "READ" : "N/A",
                static_cast<unsigned long>(g_vehicleControlsObserved));
            MessageBox("P006 profiler and controls", msg)();
        }

        bool DrawStatus(const Screen &screen)
        {
            if (screen.IsTop || !g_verified)
                return true;

            const u64 callbackStarted = osGetTime();
            PluginMenu *runningMenu = PluginMenu::GetRunningInstance();
            const bool menuOpen = runningMenu && runningMenu->IsOpen();
            if (g_profilerEnabled && !menuOpen)
            {
                if (g_prevCallbackAt != 0 && callbackStarted >= g_prevCallbackAt)
                {
                    g_profileCallbackGapMs =
                        static_cast<u32>(callbackStarted - g_prevCallbackAt);
                    if (g_profileCallbackGapMs > g_profileMaxGapMs)
                        g_profileMaxGapMs = g_profileCallbackGapMs;
                    if (g_profileCallbackGapMs > 50)
                        ++g_profileLargeGaps;
                }
                g_prevCallbackAt = callbackStarted;
            }
            else if (menuOpen)
                g_prevCallbackAt = 0;

            // P001's hardware-approved position remains unchanged.
            screen.DrawRect(2, 224, 156, 14, Color::Black, true);
            screen.Draw(g_ready ? "CK P006 | READY" : "CK P006 | INACTIVE",
                        5, 226, g_ready ? Color::Green : Color::White, Color::Black);

            // Elapsed-time schedule: avoids assuming a specific MK7 FPS.
            // Only ~2Hz full pointer walks; <=10Hz lightweight flag reads.
            if (!menuOpen && g_ready &&
                (g_live || g_recording || g_control == WAIT_CLOSE ||
                 g_control == COUNTDOWN || g_control == PULSE))
            {
                const u64 now = osGetTime();
                bool dueTelemetry = g_lastTelemetryAt == 0 ||
                    now < g_lastTelemetryAt ||
                    now - g_lastTelemetryAt >= TELEMETRY_INTERVAL_MS;
                if (dueTelemetry)
                {
                    g_lastTelemetryAt = now;
                    const u64 probeStarted = osGetTime();
                    Probe();
                    const u64 probeEnded = osGetTime();
                    g_profileProbeMs = probeEnded >= probeStarted
                        ? static_cast<u32>(probeEnded - probeStarted) : 0;
                    g_vehicleControlsValid = g_vehicleAddress != 0 &&
                        ReadWord(g_vehicleAddress + 0xFC, g_vehicleControlsObserved);
                }
                if (g_lastEnvironmentAt == 0 ||
                    now < g_lastEnvironmentAt ||
                    now - g_lastEnvironmentAt >= ENVIRONMENT_INTERVAL_MS)
                {
                    g_lastEnvironmentAt = now;
                    PollEnvironment();
                }
                ControlStep(osGetTime(), false);
                if (g_recording && dueTelemetry)
                    CaptureSample();
            }
            // A queued test cannot silently survive losing READY or menu reopen.
            else if (g_control == COUNTDOWN || g_control == PULSE)
                AbortControl(menuOpen ? "Menu reopened" : "READY/sensing disabled");
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
            // New control-authority indicator, without shifting P001's HUD.
            if (g_control == WAIT_CLOSE || g_control == COUNTDOWN ||
                g_control == PULSE || g_control == ABORTED)
            {
                screen.DrawRect(160, 196, 157, 13, Color::Black, true);
                screen.Draw(ControlName(g_control), 163, 197,
                    g_control == ABORTED ? Color::Yellow : Color::Green,
                    Color::Black);
            }
            if (g_profilerEnabled && !menuOpen)
            {
                const u64 callbackDone = osGetTime();
                g_profileCallbackWorkMs = callbackDone >= callbackStarted
                    ? static_cast<u32>(callbackDone - callbackStarted) : 0;
                if (g_profileCallbackWorkMs > g_profileMaxWorkMs)
                    g_profileMaxWorkMs = g_profileCallbackWorkMs;
                g_profileWorkTotalMs += g_profileCallbackWorkMs;
                ++g_profileCallbacks;
            }
            return true;
        }

        void ToggleReady(MenuEntry *)
        {
            g_ready = !g_ready;
            if (!g_ready)
            {
                AbortControl("READY turned off");
                g_recording = false; // Preserve unsaved data.
                g_havePreviousFlags = false;
                g_previousModeAt = 0;
            }
            ++g_toggles;
        }
        void ToggleLive(MenuEntry *)
        {
            g_live = !g_live;
            if (!g_recording)
            {
                g_lastTelemetryAt = g_lastEnvironmentAt = 0;
                g_observation = Observation();
                g_vehicleAddress = 0;
                g_environment = Environment();
                g_havePreviousFlags = false;
                g_previousModeAt = 0;
            }
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
                "\nDash/flag edges: " + std::to_string(g_dashPanelEdges) +
                    "/" + std::to_string(g_boostFlagEdges) +
                "\nEnv polls/errors: " + std::to_string(g_envPolls) +
                    "/" + std::to_string(g_envReadFails) +
                "\nReward total (test): " + std::to_string(g_totalReward) +
                "\nLast CSV: " + g_lastFile +
                "\nRace detection: NOT VERIFIED\n"
                "Controls: DISABLED\nLearning: DISABLED";
            MessageBox("Collie Kart P006 Diagnostics", msg)();
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
            MessageBox("P006 Experimental Snapshot", msg)();
        }

        void ShowEnvironment(MenuEntry *)
        {
            // Menu is paused: attempt a fresh, read-only snapshot first.
            Probe();
            PollEnvironment(true); // No event counters from paused menu.
            const Environment &e = g_environment;
            char lines[920];
            std::snprintf(lines, sizeof(lines),
                "Mode: %s (%s)\n"
                "Flags: status 0x%08lX boost 0x%08lX\n"
                "Air frames: %lu  Ground frames: %lu\n"
                "KCL surface: %lu  Dash type: %lu\n"
                "Glide pads: %u  Boost pads: %u\n"
                "Dash/flag edges: %u/%u\n"
                "Glider deploys: %u\n"
                "Force-close flag: %u\n"
                "Midair closes: %u  Land closes: %u\n"
                "B brake held: %s  Linked closes: %u\n"
                "Env polls/fails: %u/%u\n"
                "Poll gap: %lums read: %lums (max %lu)\n"
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
                g_dashPanelEdges, g_boostFlagEdges,
                g_gliderDeployments, g_forceCloseSignals,
                g_airClosures, g_landingClosures,
                e.brakeHeld ? "YES" : "NO", g_brakeLinkedClosures,
                g_envPolls, g_envReadFails,
                static_cast<unsigned long>(g_envPollGapMs),
                static_cast<unsigned long>(g_envLastReadMs),
                static_cast<unsigned long>(g_envMaxReadMs),
                static_cast<unsigned long>(g_modeMs[GROUND] / 1000),
                static_cast<unsigned long>(g_modeMs[UNDERWATER] / 1000),
                static_cast<unsigned long>(g_modeMs[AIRBORNE] / 1000),
                static_cast<unsigned long>(g_modeMs[GLIDING] / 1000),
                g_latestReward, g_totalReward);
            MessageBox("P006 Environment & Rewards", lines)();
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

        PluginMenu menu("Collie Kart P006", 0, 0, 6,
            "Profiler + one-shot experimental HID A pulse.\n"
            "Inputs unverified; no learning or MK7 RAM writes.");
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
            "Inspect grounded/gliding modes, fused pad signals, brake evidence and rewards.");
        menu += new MenuEntry("Start recording", nullptr, StartRecording,
            "Begin buffering telemetry in RAM. Requires READY.");
        menu += new MenuEntry("Stop & Save CSV", nullptr, StopAndSave,
            "Stop capturing and save a new CSV under SD:/CollieKart.");
        menu += new MenuEntry("Toggle performance profiler", nullptr, ToggleProfiler,
            "Read-only OSD callback, full-probe and environment timings.");
        menu += new MenuEntry("Profiler & control diagnostics", nullptr, ShowProfiler,
            "Frame callback timing, experimental control status and abort reason.");
        menu += new MenuEntry("Queue DRY RUN (no input)", nullptr, QueueDryRun,
            "Countdown test with NO artificial buttons. Test physical B abort.");
        menu += new MenuEntry("Queue REAL A pulse (EXPERIMENT)", nullptr, QueueRealPulse,
            "One-shot 80ms maximum HID-key experiment; first pass dry-run.");
        menu += new MenuEntry("ABORT control experiment", nullptr, CancelControl,
            "Cancel any queued experiment. Physical B/START also abort.");
        OSD::Run(DrawStatus);
        menu.Run();
        OSD::Stop(DrawStatus);
        return 0;
    }
}
