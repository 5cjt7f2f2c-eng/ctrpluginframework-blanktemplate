// Collie Kart Portable P006.3a -- safe PASS-THROUGH ONLY frame-hook preflight.
// MK7-research frame-input interception with CTRPF MITM hook. No AI/autonomy.
// No synthetic buttons can be generated in this version.
// The first on-device native-hook test MUST remain read-only.
// Previous P001-P006.2 passive telemetry, reward/CSV and backups preserved.

#include <3ds.h>
#include <CTRPluginFramework.hpp>
#include <cmath>
#include <cstdio>
#include <string>
#include <atomic>
#include "CTRPluginFramework/System/Directory.hpp"
#include "CTRPluginFramework/System/File.hpp"
#include "CTRPluginFramework/System/Hook.hpp"

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
        u32 g_profileProbeUs = 0, g_profileCallbackWorkUs = 0;
        u32 g_profileMaxWorkUs = 0, g_environmentReadUs = 0;
        u32 ElapsedUs(u64 startTick)
        {
            const u64 endTick = svcGetSystemTick();
            return endTick >= startTick
                ? static_cast<u32>((endTick - startTick) * 1000000ULL /
                                    SYSCLOCK_ARM11)
                : 0;
        }
        unsigned g_profileCallbacks = 0, g_profileLargeGaps = 0;
        u64 g_profileWorkTotalMs = 0;

        // SAFETY: the injection experiment is one-shot; never a held controller.
        // FRAMEWORK DETAILS: Controller::InjectKey ORs a key into 8 HID buffers.
        // It has no guaranteed release API. We cease writing after one burst,
        // relying on normal HID refresh. Physical B or START requests abort.
        enum ControlPhase { DISARMED = 0, WAIT_CLOSE = 1,
            COUNTDOWN = 2, PULSE = 3, FINISHED = 4, ABORTED = 5,
            OBSERVE = 6 };
        const char *ControlName(ControlPhase phase)
        {
            switch (phase)
            {
                case WAIT_CLOSE: return "WAIT_CLOSE";
                case COUNTDOWN: return "COUNTDOWN";
                case PULSE: return "PULSE";
                case OBSERVE: return "OBSERVE";
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
        u64 g_controlLastCallbackAt = 0, g_controlObserveEndAt = 0;
        unsigned g_hidInjectionCalls = 0, g_controlAborts = 0;
        unsigned g_pulseControlReads = 0;
        unsigned g_manualAControlReads = 0, g_manualAMatchedReads = 0;
        bool g_pulseMk7AccelerateSeen = false, g_pulseMovedSeen = false;
        bool g_kdPadPauseKnown = false, g_kdPadPaused = false;
        const char *g_lastAbortReason = "None";
        // Five seconds to return from CTRPF to MK7 and leave game pause.
        // Last 800ms is a neutral-input guard; B aborts at any time.
        constexpr u64 FIRST_CONTACT_COUNTDOWN_MS = 5000;
        constexpr u64 FIRST_CONTACT_FINAL_GUARD_MS = 800;
        constexpr u64 FIRST_CONTACT_NEUTRAL_AT_PULSE_MS = 300;
        constexpr u64 FIRST_CONTACT_OBSERVE_MS = 1300;
        constexpr u64 FIRST_CONTACT_MAX_PULSE_MS = 80;
        constexpr u64 FIRST_CONTACT_MAX_GAP_MS = 250;
        // Selecting a CTRPF menu item uses A. Discard UI-held button state
        // until the menu has closed and keys are released for a short interval.
        constexpr u64 FIRST_CONTACT_EXIT_GRACE_MS = 400;
        constexpr u64 FIRST_CONTACT_NEUTRAL_STABLE_MS = 150;
        constexpr u64 FIRST_CONTACT_RELEASE_TIMEOUT_MS = 3500;
        u64 g_controlWaitStartedAt = 0, g_controlNeutralSince = 0;
        u32 g_vehicleControlsObserved = 0;
        bool g_vehicleControlsValid = false;
        bool g_traceInput = false;
        u64 g_lastInputTraceAt = 0;
        struct PadTrace {
            bool valid = false;
            u32 kdButtonsAndStick = 0, kdRawButtons = 0, kdAddin = 0;
            u32 vehicleControls = 0;
            u8 padType = 255, padPaused = 255;
            float steer = 0;
            bool physicalA = false;
        };
        PadTrace g_padTrace;
        unsigned g_padPolls = 0, g_padFails = 0, g_padPhysicalA = 0;
        unsigned g_padAMatches = 0, g_vehicleAMatches = 0, g_allAMatches = 0;
        bool g_setterTrace = false;
        u64 g_lastSetterPollAt = 0;
        unsigned g_setterPolls = 0, g_setterFailures = 0;
        unsigned g_setterStructuralMatches = 0;
        unsigned g_setterAActivePolls = 0, g_setterAInactivePolls = 0;
        // A chain node is NOT identified as KDPadSetter simply because its
        // memory resembles this layout. It remains a *candidate*.
        struct SetterNode
        {
            bool valid = false;
            u32 address = 0, previous = 0, backPad = 0, onFramePtr = 0;
            u32 candidateButtons = 0;
            float candidateStickX = 0.0f;
            u8 disableFlag = 255, updateFlag = 255;
        };
        struct SetterPreflight
        {
            bool chainValid = false;
            bool playerPadValid = false;
            u32 playerPad = 0, activeAddin = 0, addinMask = 0;
            unsigned chainLength = 0, plausibleSetterNodes = 0;
            SetterNode nodes[4];
        };
        SetterPreflight g_setter;

        // P006.3a: Native game-frame input path, completely DISABLED on boot.
        // Source: MK7-Revolution KDPadInputer::calcInput signature (not yet
        // validated on MK7 USA console) and mk7re simulated-player frame hook.
        Hook g_nativeHook;
        // Cross-callback flags/counters: no unsynchronized volatile accesses.
        std::atomic<bool> g_nativeHookEnabled{false};
        u32 g_nativeHookAddress = 0, g_nativeScanMatches = 0;
        std::atomic<u32> g_nativePlayerPad{0};
        std::atomic<u32> g_nativeSeenFrames{0}, g_nativeManualAFrames{0};
        std::atomic<u32> g_nativeReadFailures{0};
        // P006.3a has NO actuator. Counters retained for CSV compatibility.
        volatile u32 g_nativeReleaseFrames = 0, g_nativeInjectedFrames = 0;
        volatile u32 g_nativeAbortSignal = 0;
        volatile u32 g_nativePulseState = 0; // 0 OFF, 1 ARMED, 2 DONE, 3 ABORT
        volatile u32 g_nativeDeadline = 0; // u32 milliseconds (bounded to < 1s)
        volatile u32 g_nativeLastFrameMs = 0;
        unsigned g_nativePulseTrials = 0;
        constexpr u32 NATIVE_PULSE_MAX_FRAMES = 3;
        constexpr u32 NATIVE_FRAME_TIMEOUT_MS = 350;

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
            u32 callbackWorkUs = 0, probeWorkUs = 0, environmentReadUs = 0;
            unsigned callbackCount = 0, slowCallbacks = 0;
            unsigned injectionCalls = 0, abortedCount = 0;
            bool vehicleControlsValid = false;
            u32 vehicleControls = 0;
            ControlPhase controlPhase = DISARMED;
            unsigned pulseInputReads = 0, manualAConfirmed = 0;
            bool pulseAccelSeen = false, pulseMoved = false;
            bool padPauseKnown = false, padPaused = false;
            PadTrace padTrace;
            unsigned padPolls = 0, padFails = 0, padPhysicalA = 0;
            unsigned padAMatches = 0, vehicleAMatches = 0, allAMatches = 0;
            SetterPreflight setter;
            unsigned setterPolls = 0, setterFailures = 0, setterMatches = 0;
            u32 nativeHookAddress = 0, nativeHookFrames = 0;
            u32 nativeManualAFrames = 0, nativeInjectedFrames = 0;
            u32 nativeReleaseFrames = 0, nativePulseState = 0;
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
            r.callbackWorkUs = g_profileCallbackWorkUs;
            r.probeWorkUs = g_profileProbeUs;
            r.environmentReadUs = g_environmentReadUs;
            r.callbackCount = g_profileCallbacks;
            r.slowCallbacks = g_profileLargeGaps;
            r.injectionCalls = g_hidInjectionCalls;
            r.abortedCount = g_controlAborts;
            r.vehicleControlsValid = g_vehicleControlsValid;
            r.vehicleControls = g_vehicleControlsObserved;
            r.controlPhase = g_control;
            r.pulseInputReads = g_pulseControlReads;
            r.manualAConfirmed = g_manualAMatchedReads;
            r.pulseAccelSeen = g_pulseMk7AccelerateSeen;
            r.pulseMoved = g_pulseMovedSeen;
            r.padPauseKnown = g_kdPadPauseKnown;
            r.padPaused = g_kdPadPaused;
            r.padTrace = g_padTrace;
            r.padPolls = g_padPolls;
            r.padFails = g_padFails;
            r.padPhysicalA = g_padPhysicalA;
            r.padAMatches = g_padAMatches;
            r.vehicleAMatches = g_vehicleAMatches;
            r.allAMatches = g_allAMatches;
            r.setter = g_setter;
            r.setterPolls = g_setterPolls;
            r.setterFailures = g_setterFailures;
            r.setterMatches = g_setterStructuralMatches;
            r.nativeHookAddress = g_nativeHookAddress;
            r.nativeHookFrames = g_nativeSeenFrames;
            r.nativeManualAFrames = g_nativeManualAFrames;
            r.nativeInjectedFrames = g_nativeInjectedFrames;
            r.nativeReleaseFrames = g_nativeReleaseFrames;
            r.nativePulseState = g_nativePulseState;

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
                "mk7_controls_valid,mk7_controls_raw,"
                "callback_work_us,probe_read_us,environment_read_us,"
                "pulse_control_read_count,pulse_accel_bit_seen,pulse_speed_change,"
                "manual_a_control_matches,pad_pause_known,pad_pause_candidate,"
                "trace_valid,physical_a,kd_buttons_stick,kd_raw_buttons,kd_addin,"
                "kd_type,kd_paused,vehicle_controls,vehicle_steer_x,"
                "pad_polls,pad_fails,physical_a_polls,kd_a_matches,"
                "vehicle_a_matches,full_chain_matches,"
                "setter_read_valid,setter_chain_valid,setter_mask,"
                "setter_chain_length,setter_candidates,"
                "setter_node0_addr,setter_node0_buttons,setter_node0_update,"
                "setter_node1_addr,setter_node1_buttons,setter_node1_update,"
                "setter_node2_addr,setter_node2_buttons,setter_node2_update,"
                "setter_node3_addr,setter_node3_buttons,setter_node3_update,"
                "setter_polls,setter_failures,setter_structural_matches,"
                "native_hook_address,native_hook_frames,native_manual_a_frames,"
                "native_injected_frames,native_release_frames,native_pulse_state") == File::SUCCESS);

            char line[2100];
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
                    "%lu,%lu,%lu,%u,%u,%s,%u,%u,%d,0x%08lX,"
                    "%lu,%lu,%lu,%u,%d,%d,%u,%d,%d,"
                    "%d,%d,0x%08lX,0x%08lX,0x%08lX,%u,%u,0x%08lX,%.3f,"
                    "%u,%u,%u,%u,%u,%u,"
                    "%d,%d,0x%08lX,%u,%u,"
                    "0x%08lX,0x%08lX,%u,"
                    "0x%08lX,0x%08lX,%u,"
                    "0x%08lX,0x%08lX,%u,"
                    "0x%08lX,0x%08lX,%u,"
                    "%u,%u,%u,0x%08lX,%u,%u,%u,%u,%u",
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
                    static_cast<unsigned long>(r.vehicleControls),
                    static_cast<unsigned long>(r.callbackWorkUs),
                    static_cast<unsigned long>(r.probeWorkUs),
                    static_cast<unsigned long>(r.environmentReadUs),
                    r.pulseInputReads, r.pulseAccelSeen ? 1 : 0,
                    r.pulseMoved ? 1 : 0, r.manualAConfirmed,
                    r.padPauseKnown ? 1 : 0, r.padPaused ? 1 : 0,
                    r.padTrace.valid ? 1 : 0, r.padTrace.physicalA ? 1 : 0,
                    static_cast<unsigned long>(r.padTrace.kdButtonsAndStick),
                    static_cast<unsigned long>(r.padTrace.kdRawButtons),
                    static_cast<unsigned long>(r.padTrace.kdAddin),
                    static_cast<unsigned>(r.padTrace.padType),
                    static_cast<unsigned>(r.padTrace.padPaused),
                    static_cast<unsigned long>(r.padTrace.vehicleControls),
                    r.padTrace.steer,
                    r.padPolls, r.padFails, r.padPhysicalA,
                    r.padAMatches, r.vehicleAMatches, r.allAMatches,
                    r.setter.playerPadValid ? 1 : 0,
                    r.setter.chainValid ? 1 : 0,
                    static_cast<unsigned long>(r.setter.addinMask),
                    r.setter.chainLength, r.setter.plausibleSetterNodes,
                    static_cast<unsigned long>(r.setter.nodes[0].address),
                    static_cast<unsigned long>(r.setter.nodes[0].candidateButtons),
                    static_cast<unsigned>(r.setter.nodes[0].updateFlag),
                    static_cast<unsigned long>(r.setter.nodes[1].address),
                    static_cast<unsigned long>(r.setter.nodes[1].candidateButtons),
                    static_cast<unsigned>(r.setter.nodes[1].updateFlag),
                    static_cast<unsigned long>(r.setter.nodes[2].address),
                    static_cast<unsigned long>(r.setter.nodes[2].candidateButtons),
                    static_cast<unsigned>(r.setter.nodes[2].updateFlag),
                    static_cast<unsigned long>(r.setter.nodes[3].address),
                    static_cast<unsigned long>(r.setter.nodes[3].candidateButtons),
                    static_cast<unsigned>(r.setter.nodes[3].updateFlag),
                    r.setterPolls, r.setterFailures, r.setterMatches,
                    static_cast<unsigned long>(r.nativeHookAddress),
                    r.nativeHookFrames, r.nativeManualAFrames,
                    r.nativeInjectedFrames, r.nativeReleaseFrames,
                    r.nativePulseState);
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



        void Probe(); // defined below

        // MITM: original first, optionally modify only the local PLAYER pad's
        // current game-frame input. Never intercept UI pad or steering here.
        // If disabled, original input is returned on every frame unmodified.
        // P006.3a PASS-THROUGH ONLY: original MK7 input always runs.
        // Deliberately no writes to button memory or synthetic input states.
        void NativeInputHook(u32 inputer)
        {
            HookContext::GetCurrent().OriginalFunction<void>(inputer);
            if (!g_nativeHookEnabled.load(std::memory_order_acquire) ||
                inputer < 0x00100000 || inputer > 0x3FFF0000 ||
                !g_nativePlayerPad.load(std::memory_order_acquire))
                return;

            // Checked reads instead of dereferencing unchecked raw pointers.
            // Failure is fail-closed: simply leave physical input unchanged.
            u32 pad = 0, frame = 0;
            u8 paused = 1;
            u16 physical = 0;
            if (!Process::Read32(inputer + 0x20, pad) ||
                !Process::Read32(inputer + 0x1C, frame) ||
                pad != g_nativePlayerPad.load(std::memory_order_acquire) ||
                pad > 0x3FFF0000 - 0x3D ||
                frame != pad + 0x1C ||
                !Process::Read8(pad + 0x3D, paused) ||
                !Process::Read16(frame, physical))
            {
                ++g_nativeReadFailures;
                return;
            }
            if (paused)
                return;
            ++g_nativeSeenFrames;
            g_nativeLastFrameMs = static_cast<u32>(osGetTime());
            if (physical & 0x0001)
                ++g_nativeManualAFrames;
        }

        // Signature from MK7-Revolution src/base/pointers.cpp. All four words
        // must match uniquely in the current MK7 USA .text. Never use EUR2
        // emulator addresses on this console.
        u32 FindInputFunction()
        {
            g_nativeScanMatches = 0;
            u32 hit = 0;
            const u32 textStart = 0x00100000;
            const u32 textSize = Process::GetTextSize();
            if (textSize < 32 || textSize > 0x00900000)
                return 0;
            constexpr u32 sig[4] = {
                0xE5901004, 0xE3510001, 0x1A000002, 0xE5901000
            };
            // CTRPF's address validator uses a lock/region lookup.
            // Validate each mapped text PAGE, not four times per word:
            // full Read32 for every instruction would stall installation.
            for (u32 a = textStart; a <= textStart + textSize - 16; a += 4)
            {
                if ((a & 0xFFF) == 0 &&
                    !Process::CheckAddress(a, MEMPERM_READ))
                    return 0;
                // A four-word candidate might straddle the next page.
                if ((a & 0xFFF) > 0xFF0 &&
                    !Process::CheckAddress((a + 12) & ~0xFFFu, MEMPERM_READ))
                    return 0;
                const volatile u32 *insn =
                    reinterpret_cast<const volatile u32 *>(a);
                if (insn[0] == sig[0] && insn[1] == sig[1] &&
                    insn[2] == sig[2] && insn[3] == sig[3])
                {
                    ++g_nativeScanMatches;
                    hit = a;
                    if (g_nativeScanMatches > 1)
                        return 0;
                }
            }
            return g_nativeScanMatches == 1 ? hit : 0;
        }

        void HookPassThrough(MenuEntry *)
        {
            if (Process::GetTitleID() != MK7_USA || !g_ready)
            {
                MessageBox("P006.3a hook locked",
                    "Enter MK7 USA Time Trial and enable READY first.")();
                return;
            }
            if (g_nativeHookEnabled)
            {
                MessageBox("P006.3a hook running",
                    "Pass-through hook already installed. Test physical A,"
                    " then examine native hook diagnostics.")();
                return;
            }
            Probe();
            if (!g_observation.candidate || !g_observation.speedValid ||
                !g_vehicleAddress)
            {
                MessageBox("P006.3a hook locked",
                    "Valid kart position and speed required. Use offline Time Trial.")();
                return;
            }
            u32 pad = 0;
            u32 padFrameWord = 0;
            u8 padPaused = 1;
            if (!ReadPointer(g_vehicleAddress + 0xE0, pad) ||
                !Process::Read32(pad + 0x1C, padFrameWord) ||
                !Process::Read8(pad + 0x3D, padPaused))
            {
                MessageBox("P006.3a hook locked",
                    "Invalid player pad. Do not install hook.")();
                return;
            }
            const u32 found = FindInputFunction();
            if (!found)
            {
                char msg[160];
                std::snprintf(msg, sizeof(msg),
                    "USA calcInput signature matches: %lu (need exactly 1)."
                    " No hook installed.",
                    static_cast<unsigned long>(g_nativeScanMatches));
                MessageBox("P006.3a incompatible signature", msg)();
                return;
            }
            g_nativeHookAddress = found;
            g_nativePlayerPad = pad;
            g_nativePulseState = 0;
            g_nativeHook.InitializeForMitm(found,
                reinterpret_cast<u32>(&NativeInputHook));
            HookResult result = g_nativeHook.Enable();
            if (result != HookResult::Success)
            {
                g_nativeHookAddress = 0;
                g_nativePlayerPad = 0;
                MessageBox("P006.3a hook installation failed",
                    "Hook framework rejected the address. No driving allowed.")();
                return;
            }
            g_nativeHookEnabled = true;
            MessageBox("P006.3a PASS-THROUGH enabled",
                "No artificial input is active. Resume Time Trial and"
                " hold physical A for 3 seconds; check native diagnostics."
                " If the game behaves oddly, quit and disable Plugin Loader.")();
        }

        void NativeHookDiagnostics(MenuEntry *)
        {
            char msg[700];
            std::snprintf(msg, sizeof(msg),
                "Hook installed: %s\n"
                "Signature matches: %lu\n"
                "Hook addr: 0x%08lX\n"
                "Player input frames: %lu\n"
                "Physical-A hook frames: %lu\n"
                "Pulse trials: %u\n"
                "Native A frames: %lu\n"
                "Pass-through release frames: %lu\n"
                "Native state: %lu (0 off/1 arm/2 done/3 aborted)\n"
                "Control phase: %s\n"
                "Dry-run/physical B: %s/%s\n"
                "HID input injection: DISABLED\n"
                "Read failures: %lu\n"
                "Actuator: DISABLED in P006.3a\n"
                "Frame-hook compatibility is still UNVERIFIED",
                g_nativeHookEnabled.load(std::memory_order_acquire) ? "YES" : "NO",
                static_cast<unsigned long>(g_nativeScanMatches),
                static_cast<unsigned long>(g_nativeHookAddress),
                static_cast<unsigned long>(g_nativeSeenFrames),
                static_cast<unsigned long>(g_nativeManualAFrames),
                g_nativePulseTrials,
                static_cast<unsigned long>(g_nativeInjectedFrames),
                static_cast<unsigned long>(g_nativeReleaseFrames),
                static_cast<unsigned long>(g_nativePulseState),
                ControlName(g_control),
                g_dryRunComplete ? "YES" : "NO",
                g_physicalAbortVerified ? "YES" : "NO",
                static_cast<unsigned long>(g_nativeReadFailures));
            MessageBox("P006.3a Native Frame Hook", msg)();
        }

        // mk7re/MK7-Memory: KDPad::m_data_on_frame +0x1C,
        // raw buttons +0x2C, active add-in +0x18, type +0x3C.
        // VehicleControl::m_controls +0xFC, m_stick_x +0x100.
        void SampleKDPadReadOnly()
        {
            ++g_padPolls;
            PadTrace t;
            u32 pad = 0;
            u8 type = 255, paused = 255;
            if (!g_observation.candidate || !g_vehicleAddress ||
                !ReadPointer(g_vehicleAddress + 0xE0, pad) ||
                !ReadWord(pad + 0x1C, t.kdButtonsAndStick) ||
                !ReadWord(pad + 0x2C, t.kdRawButtons) ||
                !ReadWord(pad + 0x18, t.kdAddin) ||
                !Process::Read8(pad + 0x3C, type) ||
                !Process::Read8(pad + 0x3D, paused) ||
                !ReadWord(g_vehicleAddress + 0xFC, t.vehicleControls) ||
                !ReadNumber(g_vehicleAddress + 0x100, t.steer))
            {
                ++g_padFails;
                g_padTrace = PadTrace();
                return;
            }
            t.valid = true;
            t.padType = type;
            t.padPaused = paused;
            t.physicalA = Controller::IsKeyDown(Key::A);
            g_padTrace = t;
            if (t.physicalA)
            {
                ++g_padPhysicalA;
                if (t.kdButtonsAndStick & 1)
                    ++g_padAMatches;
                if (t.vehicleControls & 1)
                    ++g_vehicleAMatches;
                if ((t.kdButtonsAndStick & 1) &&
                    (t.vehicleControls & 1))
                    ++g_allAMatches;
            }
        }


        // mk7re/MK7-Memory/System/KDPad{,AddBase,Setter}.hpp:
        // KDPad +0x14 active addin pointer; active->previous at +0x0C;
        // common addin base holds m_data_on_frame(+0x1C) / m_pad(+0x20).
        // Stop traversal at 4 nodes and reject mismatched backlinks/cycles.
        // Setter-specific fields are *hypotheses*: buttons +0x24,
        // stick x +0x28, disable flag +0x30 and update flag +0x3C.
        // Never write or assume a matching node is definitively setter.
        void ReadNativeSetterPreflight()
        {
            ++g_setterPolls;
            SetterPreflight out;
            u32 pad = 0, active = 0, mask = 0;
            if (!g_vehicleAddress || !g_observation.candidate ||
                !ReadPointer(g_vehicleAddress + 0xE0, pad) ||
                !ReadWord(pad + 0x18, mask))
            {
                g_setter = out;
                ++g_setterFailures;
                return;
            }

            out.playerPad = pad;
            out.playerPadValid = true;
            out.addinMask = mask;
            if (!ReadPointer(pad + 0x14, active))
            {
                g_setter = out;
                ++g_setterFailures;
                return;
            }
            out.activeAddin = active;
            u32 ptr = active;
            bool structurallyValid = true;
            for (unsigned i = 0; i < 4 && ptr; ++i)
            {
                SetterNode &node = out.nodes[i];
                u32 backlink = 0, framePtr = 0, prev = 0;
                // Validate each pointer and two independent backlinks.
                if (!PlausibleAddress(ptr) ||
                    !ReadWord(ptr + 0x20, backlink) ||
                    !ReadWord(ptr + 0x1C, framePtr) ||
                    !ReadWord(ptr + 0x0C, prev) ||
                    backlink != pad || framePtr != pad + 0x1C)
                {
                    structurallyValid = false;
                    break;
                }
                node.address = ptr;
                node.previous = prev;
                node.backPad = backlink;
                node.onFramePtr = framePtr;
                node.valid = true;
                out.chainLength = i + 1;

                // Raw candidate setter readings, not safe for injection yet.
                u32 buttons = 0;
                float stick = 0;
                u8 disable = 255, update = 255;
                if (ReadWord(ptr + 0x24, buttons) &&
                    ReadNumber(ptr + 0x28, stick) &&
                    Process::Read8(ptr + 0x30, disable) &&
                    Process::Read8(ptr + 0x3C, update))
                {
                    node.candidateButtons = buttons;
                    node.candidateStickX = stick;
                    node.disableFlag = disable;
                    node.updateFlag = update;
                    if (buttons <= 0xFFFF && std::fabs(stick) <= 1.5f &&
                        disable <= 1 && update <= 1)
                        ++out.plausibleSetterNodes;
                }

                if (prev == 0)
                {
                    ptr = 0;
                    break;
                }
                // Cycles would make the next-node lookup unsafe.
                for (unsigned j = 0; j <= i; ++j)
                    if (out.nodes[j].address == prev)
                        structurallyValid = false;
                if (!structurallyValid)
                    break;
                ptr = prev;
            }
            if (ptr != 0)
                structurallyValid = false; // chain too long or broken
            out.chainValid = structurallyValid && out.chainLength > 0;
            g_setter = out;
            if (out.chainValid && (out.addinMask & 0x10))
                ++g_setterStructuralMatches;
            else
                ++g_setterFailures;

            if (Controller::IsKeyDown(Key::A))
                ++g_setterAActivePolls;
            else
                ++g_setterAInactivePolls;
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
            const u64 perfTick = svcGetSystemTick();
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
            g_environmentReadUs = ElapsedUs(perfTick);
        }


        void AbortControl(const char *reason)
        {
            if (g_control == WAIT_CLOSE || g_control == COUNTDOWN ||
                g_control == PULSE || g_control == OBSERVE)
            {
                g_control = ABORTED;
                g_lastAbortReason = reason;
                ++g_controlAborts;
                if (reason && reason[0] == 'B' && reason[1] == '/')
                    g_physicalAbortVerified = true;
                // Do not inject any further artificial input.
                g_controlNextAt = 0;
                g_controlPulseStart = 0;
                g_controlWaitStartedAt = g_controlNeutralSince = 0;
                g_controlObserveEndAt = 0;
                g_nativePulseState = 0;
                g_nativeDeadline = 0;
            }
        }

        // Read MK7's candidate VehicleControl::m_controls (bit0=accelerate)
        // without changing game memory. A 2Hz screenshot may miss an 80ms key
        // pulse, so poll once per callback while PULSE/OBSERVE is active.
        void SampleGameControls(bool duringExperiment)
        {
            if (!g_vehicleAddress || !g_observation.candidate)
                return;
            u32 flags = 0;
            if (!ReadWord(g_vehicleAddress + 0xFC, flags))
                return;
            g_vehicleControlsObserved = flags;
            g_vehicleControlsValid = true;
            if (Controller::IsKeyDown(Key::A) && !duringExperiment)
            {
                ++g_manualAControlReads;
                if (flags & 1)
                    ++g_manualAMatchedReads;
            }
            if (duringExperiment)
            {
                ++g_pulseControlReads;
                if (flags & 1)
                    g_pulseMk7AccelerateSeen = true;
                if (g_observation.speedValid &&
                    g_observation.speed > 0.35f)
                    g_pulseMovedSeen = true;
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
                g_control != PULSE && g_control != OBSERVE)
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

            // WAIT_CLOSE is a RELEASE INTERLOCK, not part of the countdown.
            // At queue time, A is commonly held by the CTRPF menu itself.
            // Never test A/B/START synchronously in the menu callback.
            if (g_control == WAIT_CLOSE)
            {
                if (!g_controlWaitStartedAt)
                {
                    g_controlWaitStartedAt = now;
                    g_controlNeutralSince = 0;
                    return;
                }
                if (now < g_controlWaitStartedAt)
                {
                    AbortControl("Input timer reset");
                    return;
                }
                const u64 elapsed = now - g_controlWaitStartedAt;
                if (elapsed < FIRST_CONTACT_EXIT_GRACE_MS)
                    return;

                const u32 blockedKeys = static_cast<u32>(Key::A) |
                    static_cast<u32>(Key::B) | static_cast<u32>(Key::Start);
                if ((Controller::GetKeysDown(false) & blockedKeys) != 0)
                {
                    g_controlNeutralSince = 0;
                    if (elapsed >= FIRST_CONTACT_RELEASE_TIMEOUT_MS)
                        AbortControl("A/B/START remained held after menu");
                    return;
                }
                if (g_controlNeutralSince == 0)
                {
                    g_controlNeutralSince = now;
                    return;
                }
                if (now - g_controlNeutralSince <
                    FIRST_CONTACT_NEUTRAL_STABLE_MS)
                    return;
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
            // Physical B is the abort throughout countdown/pulse. START is
            // deliberately permitted early in countdown to unpause MK7.
            if (Controller::IsKeyDown(Key::B) ||
                ((g_control == PULSE || g_control == OBSERVE) &&
                 Controller::IsKeyDown(Key::Start)))
            {
                AbortControl("B/START physical override");
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
                // During the early part of 5s, allow A/START to resume the
                // game's own pause screen. Physical B always aborts.
                const u64 remain = now < g_controlNextAt
                    ? g_controlNextAt - now : 0;
                if (remain <= FIRST_CONTACT_FINAL_GUARD_MS)
                {
                    if (Controller::IsKeyDown(Key::A) ||
                        Controller::IsKeyDown(Key::Start))
                    {
                        g_controlNeutralSince = 0;
                    }
                    else if (g_controlNeutralSince == 0)
                        g_controlNeutralSince = now;
                }
                else
                    g_controlNeutralSince = 0;

                if (g_observation.speed > 0.35f ||
                    g_observation.speed < -0.35f)
                {
                    AbortControl("Moved before pulse");
                    return;
                }
                if (now >= g_controlNextAt)
                {
                    if (g_controlNeutralSince == 0 ||
                        now - g_controlNeutralSince <
                        FIRST_CONTACT_NEUTRAL_AT_PULSE_MS)
                    {
                        AbortControl("A/START not released before test");
                        return;
                    }
                    // A candidate pause flag is *logged*, not trusted as a
                    // universal game-paused detector until field-verified.
                    if (g_controlIsDryRun)
                    {
                        g_control = FINISHED;
                        g_dryRunComplete = true;
                    }
                    else
                    {
                        const u32 lastFrame = g_nativeLastFrameMs;
                        if (!g_nativeHookEnabled || !g_nativePlayerPad ||
                            g_nativeSeenFrames < 30 ||
                            g_nativeManualAFrames < 10 ||
                            lastFrame == 0 ||
                            (static_cast<u32>(now) - lastFrame) > 180)
                        {
                            AbortControl("Native hook not calibrated/active");
                            return;
                        }
                        g_nativeInjectedFrames = 0;
                        g_nativeReleaseFrames = 0;
                        g_nativeAbortSignal = 0;
                        g_nativeDeadline = static_cast<u32>(now) +
                            NATIVE_FRAME_TIMEOUT_MS;
                        g_nativePulseState = 1;
                        g_control = PULSE;
                        g_controlPulseStart = now;
                        g_controlObserveEndAt = 0;
                        ++g_nativePulseTrials;
                    }
                }
            }
            if (g_control == PULSE)
            {
                SampleGameControls(true);
                if (g_nativePulseState == 3)
                {
                    AbortControl(g_nativeAbortSignal == 2
                        ? "Physical B detected by native hook"
                        : "Native timeout or physical A during pulse");
                    return;
                }
                if (g_nativePulseState == 2)
                {
                    g_control = OBSERVE;
                    g_controlObserveEndAt = now + FIRST_CONTACT_OBSERVE_MS;
                    return;
                }
                if (now < g_controlPulseStart ||
                    now - g_controlPulseStart > NATIVE_FRAME_TIMEOUT_MS ||
                    g_nativePulseState != 1)
                {
                    AbortControl("Native pulse did not finish on time");
                    return;
                }
            }
            if (g_control == OBSERVE)
            {
                SampleGameControls(true);
                if (now >= g_controlObserveEndAt)
                {
                    if (!g_controlIsDryRun &&
                        g_nativeReleaseFrames < 3)
                        AbortControl("No confirmed normal-input frames");
                    else
                        g_control = FINISHED;
                }
            }
        }

        void QueueControlExperiment(bool dryRun)
        {
            if (g_control == WAIT_CLOSE || g_control == COUNTDOWN ||
                g_control == PULSE || g_control == OBSERVE)
            {
                MessageBox("P006 experiment already queued",
                    "Finish or abort the current experiment first.")();
                return;
            }
            if (!dryRun && (!g_dryRunComplete || !g_physicalAbortVerified ||
                            !g_nativeHookEnabled || g_nativeSeenFrames < 30 ||
                            g_nativeManualAFrames < 10))
            {
                MessageBox("P006.3a native A pulse LOCKED",
                    "Require installed pass-through hook, 30 game frames,"
                    " 10 physical-A frames, one complete 5s dry-run, and"
                    " another dry-run physically aborted by B.")();
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
            // Menu selection commonly uses A; check for key release later,
            // once the menu has fully closed and no longer owns the input.
            g_controlIsDryRun = dryRun;
            g_control = WAIT_CLOSE;
            g_controlNextAt = g_controlPulseStart = 0;
            g_controlLastCallbackAt = 0;
            g_controlWaitStartedAt = g_controlNeutralSince = 0;
            g_controlObserveEndAt = 0;
            g_pulseControlReads = 0;
            g_pulseMk7AccelerateSeen = g_pulseMovedSeen = false;
            g_nativePulseState = 0;
            g_nativeDeadline = 0;
            MessageBox(dryRun ? "P006.3a DRY RUN queued" :
                "P006.3a NATIVE 3-FRAME A TEST queued",
                dryRun
                ? "Release A/B/START after closing the menu."
                  " A neutral-input interlock runs before the FIVE-SECOND countdown."
                  " Resume MK7 using A/START in the first few seconds."
                  " Release controls near zero; B aborts anytime."
                : "Experimental A input for exactly 3 game frames."
                  " No HID injection. Close menu and resume MK7 during the"
                  " 5s countdown, then release A/START. B aborts."
                  " If controls stick, quit MK7 and disable Plugin Loader.")();
        }

        void QueueDryRun(MenuEntry *) { QueueControlExperiment(true); }
        // Safety audit gate: NEVER allow synthetic input on the first
        // real-hardware hook build. Follow-up actuation needs a separate review.
        void QueueNativePulse(MenuEntry *)
        {
            MessageBox("P006.3a actuator LOCKED",
                "PASS-THROUGH ONLY. No game-frame writes in this build."
                " First validate hook behavior on the New 3DS.")();
        }

        void CancelControl(MenuEntry *)
        {
            AbortControl("Manual menu cancellation");
            MessageBox("P006 control experiment",
                "Native experiment cancelled: normal physical input resumes."
                " If behavior is abnormal, quit MK7 and disable Plugin Loader.")();
        }

        void ToggleProfiler(MenuEntry *)
        {
            g_profilerEnabled = !g_profilerEnabled;
            g_prevCallbackAt = 0;
            g_profileCallbackGapMs = g_profileCallbackWorkMs = 0;
            g_profileProbeMs = g_profileMaxGapMs = g_profileMaxWorkMs = 0;
            g_profileProbeUs = g_profileCallbackWorkUs = 0;
            g_profileMaxWorkUs = g_environmentReadUs = 0;
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
                "Callback work: %lu us (max %lu)\n"
                "Probe/env read: %lu/%lu us\n"
                "Environment polls/fails: %u/%u\n"
                "Control phase: %s\n"
                "Dry-run passed: %s\n"
                "Physical abort verified: %s\n"
                "Release interlock: %s\n"
                "HID injection calls: %u\n"
                "Pulse control reads: %u\n"
                "Accel bit seen: %s  Speed: %s\n"
                "Manual A matches: %u/%u\n"
                "Pad paused?: %s\n"
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
                static_cast<unsigned long>(g_profileCallbackWorkUs),
                static_cast<unsigned long>(g_profileMaxWorkUs),
                static_cast<unsigned long>(g_profileProbeUs),
                static_cast<unsigned long>(g_environmentReadUs),
                g_envPolls, g_envReadFails, ControlName(g_control),
                g_dryRunComplete ? "YES" : "NO",
                g_physicalAbortVerified ? "YES" : "NO",
                g_control == WAIT_CLOSE ? "WAITING" : "NOT WAITING",
                g_hidInjectionCalls, g_pulseControlReads,
                g_pulseMk7AccelerateSeen ? "YES" : "NO",
                g_pulseMovedSeen ? "MOVED" : "NO",
                g_manualAMatchedReads, g_manualAControlReads,
                g_kdPadPauseKnown ? (g_kdPadPaused ? "YES*" : "NO*") : "UNKNOWN",
                g_controlAborts,
                g_lastAbortReason, g_vehicleControlsValid ? "READ" : "N/A",
                static_cast<unsigned long>(g_vehicleControlsObserved));
            MessageBox("P006 profiler and controls", msg)();
        }

        bool DrawStatus(const Screen &screen)
        {
            if (screen.IsTop || !g_verified)
                return true;

            const u64 callbackTickStarted = g_profilerEnabled
                ? svcGetSystemTick() : 0;
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
            screen.Draw(g_ready ? "CK P006.3a | READY" : "CK P006.3a | INACTIVE",
                        5, 226, g_ready ? Color::Green : Color::White, Color::Black);

            // Elapsed-time schedule: avoids assuming a specific MK7 FPS.
            // Only ~2Hz full pointer walks; <=10Hz lightweight flag reads.
            if (!menuOpen && g_ready &&
                (g_live || g_recording || g_control == WAIT_CLOSE ||
                 g_control == COUNTDOWN || g_control == PULSE ||
                 g_control == OBSERVE || g_traceInput ||
                 g_setterTrace))
            {
                const u64 now = osGetTime();
                bool dueTelemetry = g_lastTelemetryAt == 0 ||
                    now < g_lastTelemetryAt ||
                    now - g_lastTelemetryAt >= TELEMETRY_INTERVAL_MS;
                if (dueTelemetry)
                {
                    g_lastTelemetryAt = now;
                    const u64 probeStarted = osGetTime();
                    const u64 probeTickStarted = g_profilerEnabled
                        ? svcGetSystemTick() : 0;
                    Probe();
                    if (g_profilerEnabled)
                        g_profileProbeUs = ElapsedUs(probeTickStarted);
                    const u64 probeEnded = osGetTime();
                    g_profileProbeMs = probeEnded >= probeStarted
                        ? static_cast<u32>(probeEnded - probeStarted) : 0;
                    g_vehicleControlsValid = g_vehicleAddress != 0 &&
                        ReadWord(g_vehicleAddress + 0xFC, g_vehicleControlsObserved);
                    // KDPlayerPad::m_is_paused at +0x3D, candidate only.
                    g_kdPadPauseKnown = false;
                    g_kdPadPaused = false;
                    u32 gamePad = 0;
                    u8 gamePadPaused = 0;
                    if (g_vehicleAddress != 0 &&
                        ReadPointer(g_vehicleAddress + 0xE0, gamePad) &&
                        Process::Read8(gamePad + 0x3D, gamePadPaused))
                    {
                        g_kdPadPauseKnown = true;
                        g_kdPadPaused = gamePadPaused != 0;
                        if (g_nativeHookEnabled && g_nativePulseState == 0)
                            g_nativePlayerPad = gamePad;
                    }
                    // Compare physical A with MK7's own accelerate bit.
                    if (g_control == DISARMED || g_control == FINISHED ||
                        g_control == ABORTED)
                        SampleGameControls(false);
                }
                if (g_lastEnvironmentAt == 0 ||
                    now < g_lastEnvironmentAt ||
                    now - g_lastEnvironmentAt >= ENVIRONMENT_INTERVAL_MS)
                {
                    g_lastEnvironmentAt = now;
                    PollEnvironment();
                }
                if (g_traceInput && (g_lastInputTraceAt == 0 ||
                    now < g_lastInputTraceAt ||
                    now - g_lastInputTraceAt >= 100))
                {
                    g_lastInputTraceAt = now;
                    SampleKDPadReadOnly();
                }
                if (g_setterTrace && (g_lastSetterPollAt == 0 ||
                    now < g_lastSetterPollAt ||
                    now - g_lastSetterPollAt >= 200))
                {
                    g_lastSetterPollAt = now;
                    ReadNativeSetterPreflight();
                }
                ControlStep(osGetTime(), false);
                if (g_recording && dueTelemetry)
                    CaptureSample();
            }
            // A queued test cannot silently survive losing READY or menu reopen.
            else if (g_control == COUNTDOWN || g_control == PULSE ||
                     g_control == OBSERVE)
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
                g_control == PULSE || g_control == OBSERVE ||
                g_control == ABORTED)
            {
                screen.DrawRect(142, 196, 178, 13, Color::Black, true);
                char phase[42];
                if (g_control == COUNTDOWN && g_controlNextAt > osGetTime())
                    std::snprintf(phase, sizeof(phase), "RACCOON IN %lus",
                        static_cast<unsigned long>(
                            (g_controlNextAt - osGetTime() + 999) / 1000));
                else
                    std::snprintf(phase, sizeof(phase), "%s",
                        ControlName(g_control));
                screen.Draw(phase, 145, 197,
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
                g_profileCallbackWorkUs = ElapsedUs(callbackTickStarted);
                if (g_profileCallbackWorkUs > g_profileMaxWorkUs)
                    g_profileMaxWorkUs = g_profileCallbackWorkUs;
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
                g_nativePulseState = 0;
                // Stop collecting before unhooking. No actuator exists here.
                g_nativeHookEnabled.store(false, std::memory_order_release);
                g_nativePlayerPad.store(0, std::memory_order_release);
                if (g_nativeHook.IsEnabled())
                {
                    if (g_nativeHook.Disable() != HookResult::Success)
                        MessageBox("Hook removal FAILED",
                            "Stop testing. Close MK7 and disable Plugin Loader."
                            " Do not assume controller interception has stopped.")();
                }
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


        void ToggleInputTrace(MenuEntry *)
        {
            g_traceInput = !g_traceInput;
            g_lastInputTraceAt = 0;
            if (g_traceInput)
            {
                g_padTrace = PadTrace();
                g_padPolls = g_padFails = g_padPhysicalA = 0;
                g_padAMatches = g_vehicleAMatches = g_allAMatches = 0;
            }
            MessageBox("P006.1 KDPad trace", g_traceInput
                ? "Read-only, opt-in ~10Hz. Drive with physical A,"
                  " then turn left/right. Open input diagnostics after."
                : "Input trace disabled.")();
        }
        void InputTraceDiagnostics(MenuEntry *)
        {
            char msg[520];
            std::snprintf(msg, sizeof(msg),
                "Trace: %s | sample: %s\n"
                "Polls/fails: %u/%u\n"
                "Physical A polls: %u\n"
                "KDPad A matches: %u\n"
                "Vehicle A matches: %u\n"
                "All-three matches: %u\n"
                "KDPad buttons: 0x%08lX\n"
                "KDPad raw: 0x%08lX\n"
                "KDPad addin: 0x%08lX\n"
                "Pad type/pause: %u / %u\n"
                "Vehicle controls: 0x%08lX\n"
                "Vehicle steer X: %.3f\n"
                "NO NEW INPUT INJECTION",
                g_traceInput ? "ON" : "OFF",
                g_padTrace.valid ? "VALID" : "NO DATA",
                g_padPolls, g_padFails,
                g_padPhysicalA, g_padAMatches,
                g_vehicleAMatches, g_allAMatches,
                static_cast<unsigned long>(g_padTrace.kdButtonsAndStick),
                static_cast<unsigned long>(g_padTrace.kdRawButtons),
                static_cast<unsigned long>(g_padTrace.kdAddin),
                static_cast<unsigned>(g_padTrace.padType),
                static_cast<unsigned>(g_padTrace.padPaused),
                static_cast<unsigned long>(g_padTrace.vehicleControls),
                g_padTrace.steer);
            MessageBox("P006.1 input pipeline", msg)();
        }


        void ToggleSetterPreflight(MenuEntry *)
        {
            g_setterTrace = !g_setterTrace;
            g_lastSetterPollAt = 0;
            if (g_setterTrace)
            {
                g_setter = SetterPreflight();
                g_setterPolls = g_setterFailures = 0;
                g_setterStructuralMatches = 0;
                g_setterAActivePolls = g_setterAInactivePolls = 0;
            }
            MessageBox("P006.2 native setter preflight",
                g_setterTrace
                ? "ON, read-only 5Hz. Drive with physical A and steering."
                  " Then inspect the native setter diagnostic."
                  " NO artificial controls are enabled."
                : "OFF; no native game-memory writes were made.")();
        }

        void ShowSetterPreflight(MenuEntry *)
        {
            // A fresh snapshot while the menu is paused can help distinguish
            // the pointer chain from motion-specific changes.
            Probe();
            ReadNativeSetterPreflight();
            char msg[820];
            std::snprintf(msg, sizeof(msg),
                "Mode: READ ONLY / NATIVE DISABLED\n"
                "Player pad valid: %s\n"
                "Linked chain valid: %s\n"
                "Addon mask: 0x%08lX\n"
                "Chain nodes: %u (candidate: %u)\n"
                "Polls/fails: %u/%u\n"
                "Stable mask/chain samples: %u\n"
                "A held/released: %u/%u\n"
                "Node 0: 0x%08lX btn 0x%08lX upd %u\n"
                "Node 1: 0x%08lX btn 0x%08lX upd %u\n"
                "Node 2: 0x%08lX btn 0x%08lX upd %u\n"
                "Node 3: 0x%08lX btn 0x%08lX upd %u\n"
                "Actuation: NOT IMPLEMENTED",
                g_setter.playerPadValid ? "YES" : "NO",
                g_setter.chainValid ? "YES" : "NO",
                static_cast<unsigned long>(g_setter.addinMask),
                g_setter.chainLength, g_setter.plausibleSetterNodes,
                g_setterPolls, g_setterFailures,
                g_setterStructuralMatches,
                g_setterAActivePolls, g_setterAInactivePolls,
                static_cast<unsigned long>(g_setter.nodes[0].address),
                static_cast<unsigned long>(g_setter.nodes[0].candidateButtons),
                static_cast<unsigned>(g_setter.nodes[0].updateFlag),
                static_cast<unsigned long>(g_setter.nodes[1].address),
                static_cast<unsigned long>(g_setter.nodes[1].candidateButtons),
                static_cast<unsigned>(g_setter.nodes[1].updateFlag),
                static_cast<unsigned long>(g_setter.nodes[2].address),
                static_cast<unsigned long>(g_setter.nodes[2].candidateButtons),
                static_cast<unsigned>(g_setter.nodes[2].updateFlag),
                static_cast<unsigned long>(g_setter.nodes[3].address),
                static_cast<unsigned long>(g_setter.nodes[3].candidateButtons),
                static_cast<unsigned>(g_setter.nodes[3].updateFlag));
            MessageBox("P006.2 Setter Discovery", msg)();
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

    // The hook is installed only on explicit menu selection after signature
    // verification; it is never installed automatically on startup.
    void PatchProcess(FwkSettings &) {}
    void OnProcessExit(void)
    {
        g_nativePulseState = 0;
        g_nativePlayerPad = 0;
        g_nativeHookEnabled = false;
        if (g_nativeHook.IsEnabled())
            g_nativeHook.Disable();
    }

    int main(void)
    {
        const u64 title = Process::GetTitleID();
        if (title == MK7_USA) { g_verified = true; g_region = "USA"; }
        else if (title == MK7_EUR) { g_verified = true; g_region = "EUR"; }
        else if (title == MK7_JPN) { g_verified = true; g_region = "JPN"; }
        if (!g_verified)
            return 0;

        PluginMenu menu("Collie Kart P006.3a", 0, 0, 9,
            "Experimental MK7 game-frame input hook, OFF at boot.\n"
            "Pulse locked until native calibration and two dry runs.");
        menu.SynchronizeWithFrame(true);
        menu.ShowWelcomeMessage(false);
        menu += new MenuEntry("Toggle READY / INACTIVE", nullptr, ToggleReady,
            "Master safety toggle for telemetry and native control.");
        // P006.3a core tests stay at the top; the legacy diagnostics follow.
        menu += new MenuEntry("1: Install PASS-THROUGH hook", nullptr, HookPassThrough,
            "Native hook verification, no artificial input. Calibration first.");
        menu += new MenuEntry("2: Native hook diagnostics", nullptr, NativeHookDiagnostics,
            "Verify frame callbacks and real A input; inspect pulse results.");
        menu += new MenuEntry("3: Queue 5s SAFETY dry run", nullptr, QueueDryRun,
            "No injection; after a finish, repeat and press B to abort.");
        menu += new MenuEntry("4: NATIVE pulse LOCKED (P006.3a)", nullptr, QueueNativePulse,
            "No artificial input permitted until pass-through hardware validation.");
        menu += new MenuEntry("5: ABORT control experiment", nullptr, CancelControl,
            "Cancel countdown/pulse; physical B also aborts.");
        menu += new MenuEntry("Show diagnostics", nullptr, ShowDiagnostics,
            "Status of the last attempted telemetry probe.");
        menu += new MenuEntry("Read telemetry snapshot", nullptr, Snapshot,
            "Attempt a single read-only game-memory snapshot.");
        menu += new MenuEntry("Toggle live telemetry", nullptr, ToggleLive,
            "Opt-in sampled observations, only while READY.");
        menu += new MenuEntry("Environment & rewards", nullptr, ShowEnvironment,
            "Inspect grounded/gliding modes, fused pad signals, brake evidence and rewards.");
        menu += new MenuEntry("Toggle READ-ONLY input trace", nullptr, ToggleInputTrace,
            "Sample MK7 player KDPad and vehicle controls without writing memory.");
        menu += new MenuEntry("Input pipeline diagnostics", nullptr, InputTraceDiagnostics,
            "Check physical A, KDPad A and vehicle acceleration matches.");
        menu += new MenuEntry("Toggle NATIVE setter preflight", nullptr, ToggleSetterPreflight,
            "Read-only inspect the MK7 pad subsystem chain at up to 5Hz.");
        menu += new MenuEntry("Native setter diagnostics", nullptr, ShowSetterPreflight,
            "Check pad add-in chain validity and candidate native setter state.");


        menu += new MenuEntry("Start recording", nullptr, StartRecording,
            "Begin buffering telemetry in RAM. Requires READY.");
        menu += new MenuEntry("Stop & Save CSV", nullptr, StopAndSave,
            "Stop capturing and save a new CSV under SD:/CollieKart.");
        menu += new MenuEntry("Toggle performance profiler", nullptr, ToggleProfiler,
            "Read-only OSD callback, full-probe and environment timings.");
        menu += new MenuEntry("Profiler & control diagnostics", nullptr, ShowProfiler,
            "Frame callback timing, experimental control status and abort reason.");
        // No HID injection. Native A is a separate, opt-in frame experiment.
        OSD::Run(DrawStatus);
        menu.Run();
        g_nativePulseState = 0;
        g_nativePlayerPad = 0;
        g_nativeHookEnabled = false;
        if (g_nativeHook.IsEnabled())
            g_nativeHook.Disable();
        OSD::Stop(DrawStatus);
        return 0;
    }
}
