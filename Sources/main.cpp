// Collie Kart Portable P007a -- read-only manual demonstration capture.
// Fast observation, waypoint extraction, CSV export. No AI driving.
// No synthetic input menu actions; P006.4 remains preserved.
// This iteration only records manual driving.
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

        // P006.4: Native hook OFF until explicitly installed.
        // Source: MK7-Revolution KDPadInputer::calcInput signature (not yet
        // validated on MK7 USA console) and mk7re simulated-player frame hook.
        Hook g_nativeHook;
        // Cross-callback flags/counters: no unsynchronized volatile accesses.
        std::atomic<bool> g_nativeHookEnabled{false};
        u32 g_nativeHookAddress = 0, g_nativeScanMatches = 0;
        std::atomic<u32> g_nativePlayerPad{0};
        std::atomic<u32> g_nativeSeenFrames{0}, g_nativeManualAFrames{0};
        std::atomic<u32> g_nativeReadFailures{0};
        // State 0 OFF, 1 ARMED, 2 DONE, 3 ABORT, 4 WRITING.
        // compare-exchange prevents a concurrent abort rearming the writer.
        std::atomic<u32> g_nativePulseState{0};
        std::atomic<u32> g_nativeDeadline{0}, g_nativeAbortSignal{0};
        std::atomic<u32> g_nativeLastFrameMs{0};
        std::atomic<u32> g_nativeReleaseFrames{0}, g_nativeInjectedFrames{0};
        std::atomic<u32> g_nativeWriteFailures{0};
        // P006.4: one controlled hardware attempt per action each session.
        enum NativeAction : u32 { TEST_NONE = 0, TEST_THROTTLE = 1,
            TEST_LEFT = 2, TEST_RIGHT = 3, TEST_REVERSE = 4 };
        std::atomic<u32> g_nativeAction{TEST_NONE};
        std::atomic<u32> g_nativeAttemptMask{0};
        std::atomic<u32> g_nativeLastStick{7};
        unsigned g_nativePulseTrials = 0;
        float g_nativeSpeedBefore = 0.0f, g_nativeSpeedAfter = 0.0f;
        constexpr u32 NATIVE_FRAME_TIMEOUT_MS = 350; // legacy constant
        const char *NativeActionName(u32 a)
        {
            switch (a) {
                case TEST_THROTTLE: return "THROTTLE";
                case TEST_LEFT: return "STEER LEFT";
                case TEST_RIGHT: return "STEER RIGHT";
                case TEST_REVERSE: return "REVERSE";
                default: return "NONE";
            }
        }
        u32 ActionFrameLimit(u32 a)
        {
            switch (a) {
                case TEST_THROTTLE: return 18;   // ~0.3s at 60 fps
                case TEST_LEFT:
                case TEST_RIGHT: return 36;      // ~0.6s at 60 fps
                case TEST_REVERSE: return 90;    // ~1.5s at 60 fps
                default: return 0;
            }
        }
        u32 ActionTimeoutMs(u32 a)
        {
            switch (a) {
                case TEST_THROTTLE: return 700;
                case TEST_LEFT:
                case TEST_RIGHT: return 1100;
                case TEST_REVERSE: return 2300;
                default: return 0;
            }
        }

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

        // P007a: isolated in-RAM manual demonstration recorder. Never drives.
        // Movement heading is a DELTA-POSITION proxy, not vehicle facing yaw.
        constexpr unsigned MAX_DEMO_SAMPLES = 4800; // 4min at target 20Hz
        constexpr unsigned MAX_DEMO_WAYPOINTS = 960;
        constexpr u64 DEMO_INTERVAL_MS = 50;
        struct DemoFrame
        {
            u32 tMs = 0;
            float x = 0, y = 0, z = 0, speed = 0;
            float movementHeading = 0;
            u16 buttons = 0;
            u8 stickX = 7, lap = 0, checkpoint = 0;
            bool headingValid = false;
        };
        struct DemoWaypoint
        {
            unsigned sourceIndex = 0;
            float x = 0, z = 0, speed = 0;
            float movementHeading = 0;
            bool headingValid = false;
        };
        DemoFrame g_demoSamples[MAX_DEMO_SAMPLES];
        DemoWaypoint g_demoWaypoints[MAX_DEMO_WAYPOINTS];
        unsigned g_demoCount = 0, g_demoWaypointsCount = 0;
        unsigned g_demoReadFails = 0, g_demoGapStops = 0;
        unsigned g_demoHeadingCount = 0, g_demoLeftFrames = 0;
        unsigned g_demoRightFrames = 0, g_demoAFrames = 0, g_demoBFrames = 0;
        unsigned g_demoMaxGapMs = 0, g_demoMissedIntervals = 0;
        float g_demoDistance = 0, g_demoFinalDisplacement = 0;
        u32 g_demoKartAddress = 0;
        u64 g_demoStartAt = 0, g_demoLastAt = 0;
        bool g_demoRecording = false, g_demoComplete = false;
        bool g_demoAnalyzed = false, g_demoStoppedOnFault = false;
        std::string g_demoLastFile = "(none)";

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
            if (g_demoRecording)
            {
                MessageBox("Recorder unavailable",
                    "Stop the P007a demonstration recorder first.")();
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

        // P006.4: four independent one-shot actuator trials.
        // Every callback processes the ORIGINAL controller function first.
        // Inactive always means no artificial memory writes.
        void NativeInputHook(u32 inputer)
        {
            HookContext::GetCurrent().OriginalFunction<void>(inputer);
            if (!g_nativeHookEnabled.load(std::memory_order_acquire) ||
                inputer < 0x00100000 || inputer > 0x3FFF0000 ||
                !g_nativePlayerPad.load(std::memory_order_acquire))
                return;
            u32 pad = 0, frame = 0;
            u8 paused = 1, stickX = 7;
            u16 physical = 0;
            if (!Process::Read32(inputer + 0x20, pad) ||
                !Process::Read32(inputer + 0x1C, frame))
            {
                ++g_nativeReadFailures;
                return;
            }
            if (pad != g_nativePlayerPad.load(std::memory_order_acquire))
                return; // Not the local human kart; never modify CPU pads.
            if (pad > 0x3FFF0000 - 0x3D || frame != pad + 0x1C ||
                !Process::Read8(pad + 0x3D, paused) ||
                !Process::Read16(frame, physical) ||
                !Process::Read8(frame + 2, stickX))
            {
                ++g_nativeReadFailures;
                return;
            }
            if (paused)
                return;
            ++g_nativeSeenFrames;
            const u32 now = static_cast<u32>(osGetTime());
            g_nativeLastFrameMs.store(now, std::memory_order_release);
            if (physical & 0x0001)
                ++g_nativeManualAFrames;

            const u32 state = g_nativePulseState.load(std::memory_order_acquire);
            if (state == 2)
            {
                // Count actual original-input frames after the commanded run.
                // Both synthetic button and stick must have been released.
                if (!(physical & 0x0003) && stickX >= 6 && stickX <= 8 &&
                    g_nativeReleaseFrames.load() < 12)
                    ++g_nativeReleaseFrames;
                return;
            }
            if (state != 1)
                return;
            u32 expected = 1;
            const u32 action = g_nativeAction.load(std::memory_order_acquire);
            const u32 limit = ActionFrameLimit(action);
            const u32 deadline = g_nativeDeadline.load(std::memory_order_acquire);
            if (limit == 0 || static_cast<s32>(deadline - now) < 0)
            {
                g_nativeAbortSignal.store(1, std::memory_order_release);
                g_nativePulseState.compare_exchange_strong(expected, 3);
                return;
            }
            // Any physical A/B/START overrides the test, and physical
            // steering beyond the 6..8 neutral band cancels as well.
            if ((physical & 0x000B) || stickX < 6 || stickX > 8)
            {
                g_nativeAbortSignal.store(2, std::memory_order_release);
                g_nativePulseState.compare_exchange_strong(expected, 3);
                return;
            }
            if (!g_nativePulseState.compare_exchange_strong(
                    expected, 4, std::memory_order_acq_rel))
                return;

            if (!g_nativeHookEnabled.load(std::memory_order_acquire) ||
                g_nativePlayerPad.load(std::memory_order_acquire) != pad ||
                static_cast<s32>(g_nativeDeadline.load() -
                                 static_cast<u32>(osGetTime())) < 0)
            {
                g_nativeAbortSignal.store(1, std::memory_order_release);
                expected = 4;
                g_nativePulseState.compare_exchange_strong(expected, 3);
                return;
            }
            const u32 count = g_nativeInjectedFrames.load();
            if (count >= limit)
            {
                expected = 4;
                g_nativePulseState.compare_exchange_strong(expected, 2);
                return;
            }

            const bool turn = action == TEST_LEFT || action == TEST_RIGHT;
            const u8 targetStick = action == TEST_LEFT ? 2 :
                                   action == TEST_RIGHT ? 12 : 7;
            const u16 targetButton = action == TEST_REVERSE ? 0x0002 : 0x0001;

            // Steering writes are restricted to KDPadDataOnFrame::stick_x
            // byte +2. Zero synthetics outside a finite one-shot.
            if (turn && !Process::Write8(frame + 2, targetStick))
            {
                ++g_nativeWriteFailures;
                g_nativeAbortSignal.store(4, std::memory_order_release);
                expected = 4;
                g_nativePulseState.compare_exchange_strong(expected, 3);
                return;
            }
            // Only set A for throttle/turning, or B for standstill reverse.
            if (!Process::Write16(frame, static_cast<u16>(
                    physical | targetButton)))
            {
                if (turn)
                    Process::Write8(frame + 2, stickX); // best-effort restore
                ++g_nativeWriteFailures;
                g_nativeAbortSignal.store(4, std::memory_order_release);
                expected = 4;
                g_nativePulseState.compare_exchange_strong(expected, 3);
                return;
            }
            g_nativeLastStick.store(turn ? targetStick : stickX,
                                    std::memory_order_release);
            const u32 written = ++g_nativeInjectedFrames;
            expected = 4;
            g_nativePulseState.compare_exchange_strong(
                expected, written >= limit ? 2u : 1u,
                std::memory_order_acq_rel);
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
                MessageBox("P006.4 hook locked",
                    "Enter MK7 USA Time Trial and enable READY first.")();
                return;
            }
            if (g_nativeHookEnabled)
            {
                MessageBox("P006.4 hook running",
                    "Pass-through hook already installed. Test physical A,"
                    " then examine native hook diagnostics.")();
                return;
            }
            Probe();
            if (!g_observation.candidate || !g_observation.speedValid ||
                !g_vehicleAddress)
            {
                MessageBox("P006.4 hook locked",
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
                MessageBox("P006.4 hook locked",
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
                MessageBox("P006.4 incompatible signature", msg)();
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
                MessageBox("P006.4 hook installation failed",
                    "Hook framework rejected the address. No driving allowed.")();
                return;
            }
            g_nativeHookEnabled = true;
            MessageBox("P006.4 PASS-THROUGH enabled",
                "No artificial input is active. Resume Time Trial and"
                " hold physical A for 3 seconds; check native diagnostics."
                " If the game behaves oddly, quit and disable Plugin Loader.")();
        }

        void NativeHookDiagnostics(MenuEntry *)
        {
            char msg[950];
            const u32 a = g_nativeAction.load(std::memory_order_acquire);
            std::snprintf(msg, sizeof(msg),
                "Hook: %s | Signatures: %lu\n"
                "Hook address: 0x%08lX\n"
                "Local frames: %lu | physical A: %lu\n"
                "Action: %s\n"
                "Trials total: %u | attempt mask: 0x%02lX\n"
                "Written frames: %lu/%lu | last stick X: %lu\n"
                "Release frames: %lu | state: %lu\n"
                "Control phase: %s\n"
                "Dry-run / physical B: %s/%s\n"
                "Read/write failures: %lu/%lu\n"
                "Speed before/end: %.2f / %.2f\n"
                "Direction success: VISUAL CONFIRMATION REQUIRED",
                g_nativeHookEnabled.load(std::memory_order_acquire) ? "YES" : "NO",
                static_cast<unsigned long>(g_nativeScanMatches),
                static_cast<unsigned long>(g_nativeHookAddress),
                static_cast<unsigned long>(g_nativeSeenFrames.load()),
                static_cast<unsigned long>(g_nativeManualAFrames.load()),
                NativeActionName(a), g_nativePulseTrials,
                static_cast<unsigned long>(g_nativeAttemptMask.load()),
                static_cast<unsigned long>(g_nativeInjectedFrames.load()),
                static_cast<unsigned long>(ActionFrameLimit(a)),
                static_cast<unsigned long>(g_nativeLastStick.load()),
                static_cast<unsigned long>(g_nativeReleaseFrames.load()),
                static_cast<unsigned long>(g_nativePulseState.load()),
                ControlName(g_control),
                g_dryRunComplete ? "YES" : "NO",
                g_physicalAbortVerified ? "YES" : "NO",
                static_cast<unsigned long>(g_nativeReadFailures.load()),
                static_cast<unsigned long>(g_nativeWriteFailures.load()),
                g_nativeSpeedBefore, g_nativeSpeedAfter);
            MessageBox("P006.4 Four-Control Diagnostics", msg)();
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
            g_demoKartAddress = 0;

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

            // This pointer was validated by the existing full 2Hz scan.
            // The 20Hz recorder reads only these cached fields.
            g_demoKartAddress = kart;
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


        // P007a: single bounded 20Hz target demonstration stream.
        // This path NEVER patches MK7 memory or invokes native actuation.
        void SampleDemo(u64 now)
        {
            if (!g_demoRecording || (g_demoLastAt && now >= g_demoLastAt &&
                now - g_demoLastAt < DEMO_INTERVAL_MS))
                return;
            if (g_demoCount >= MAX_DEMO_SAMPLES)
            {
                g_demoRecording = false;
                g_demoComplete = true;
                return;
            }
            if (g_demoLastAt && now < g_demoLastAt)
            {
                g_demoRecording = false;
                g_demoComplete = true;
                g_demoStoppedOnFault = true;
                return;
            }
            if (g_demoLastAt)
            {
                const u64 gap = now - g_demoLastAt;
                if (gap > g_demoMaxGapMs)
                    g_demoMaxGapMs = static_cast<unsigned>(gap);
                if (gap > 150)
                    ++g_demoMissedIntervals;
            }
            g_demoLastAt = now;

            float x = 0, y = 0, z = 0, speed = 0;
            u32 pad = 0;
            u16 buttons = 0;
            u8 stick = 7, paused = 1, cp = 0;
            u32 lap = 0;
            if (!g_observation.candidate || !g_demoKartAddress ||
                !g_vehicleAddress ||
                !ReadNumber(g_demoKartAddress + 0x28, x) ||
                !ReadNumber(g_demoKartAddress + 0x2C, y) ||
                !ReadNumber(g_demoKartAddress + 0x30, z) ||
                !ReadWord(g_demoKartAddress + 0x1C, lap) || lap > 9 ||
                !Process::Read8(g_demoKartAddress + 0x0D, cp) ||
                !ReadNumber(g_vehicleAddress + 0xF2C, speed) ||
                speed < -20.0f || speed > 500.0f ||
                !ReadPointer(g_vehicleAddress + 0xE0, pad) ||
                !Process::Read8(pad + 0x3D, paused) ||
                !Process::Read16(pad + 0x1C, buttons) ||
                !Process::Read8(pad + 0x1E, stick) ||
                paused || stick > 14)
            {
                ++g_demoReadFails;
                // Stop rather than continue with possibly stale game pointers.
                if (g_demoReadFails > 8)
                {
                    g_demoRecording = false;
                    g_demoComplete = g_demoCount > 0;
                    g_demoStoppedOnFault = true;
                }
                return;
            }
            // Reset consecutive errors but retain total error count separately.
            g_demoReadFails = 0;

            // Time starts at first VALID gameplay sample, not while
            // CTRPF menu is still open following START selection.
            if (g_demoCount == 0)
                g_demoStartAt = now;
            DemoFrame &d = g_demoSamples[g_demoCount];
            d.tMs = static_cast<u32>(now - g_demoStartAt);
            d.x = x; d.y = y; d.z = z; d.speed = speed;
            d.buttons = buttons; d.stickX = stick;
            d.lap = static_cast<u8>(lap + 1); d.checkpoint = cp;
            d.headingValid = false;
            d.movementHeading = 0.0f;
            if (g_demoCount > 0)
            {
                const DemoFrame &prev = g_demoSamples[g_demoCount - 1];
                const float dx = d.x - prev.x, dz = d.z - prev.z;
                const float sq = dx * dx + dz * dz;
                // A large discontinuity indicates a scene reset, teleport
                // or invalid cached pointer: preserve RAM and stop.
                if (!std::isfinite(sq) || sq > 250000.0f ||
                    d.tMs <= prev.tMs || d.tMs - prev.tMs > 2000)
                {
                    g_demoRecording = false;
                    g_demoComplete = true;
                    g_demoStoppedOnFault = true;
                    ++g_demoGapStops;
                    return;
                }
                if (sq >= 0.04f)
                {
                    d.movementHeading = std::atan2(dz, dx);
                    d.headingValid = true;
                    ++g_demoHeadingCount;
                    g_demoDistance += std::sqrt(sq);
                }
                else
                    d.movementHeading = prev.movementHeading;
            }
            if (buttons & 0x1) ++g_demoAFrames;
            if (buttons & 0x2) ++g_demoBFrames;
            if (stick < 6) ++g_demoLeftFrames;
            if (stick > 8) ++g_demoRightFrames;
            ++g_demoCount;
            if (g_demoCount >= MAX_DEMO_SAMPLES)
            {
                g_demoRecording = false;
                g_demoComplete = true;
            }
        }

        // Downsample the sampled path into a sequence the later controller
        // can reference. This is trajectory extraction, NOT training yet.
        void AnalyzeDemoBuffer()
        {
            g_demoWaypointsCount = 0;
            g_demoFinalDisplacement = 0;
            if (!g_demoCount) { g_demoAnalyzed = false; return; }
            for (unsigned i = 0; i < g_demoCount &&
                    g_demoWaypointsCount + 1 < MAX_DEMO_WAYPOINTS; i += 5)
            {
                DemoWaypoint &w = g_demoWaypoints[g_demoWaypointsCount++];
                const DemoFrame &d = g_demoSamples[i];
                w.sourceIndex = i; w.x = d.x; w.z = d.z;
                w.speed = d.speed; w.movementHeading = d.movementHeading;
                w.headingValid = d.headingValid;
            }
            // Always include the demonstration endpoint.
            if (g_demoWaypoints[g_demoWaypointsCount - 1].sourceIndex !=
                    g_demoCount - 1)
            {
                const unsigned idx = g_demoCount - 1;
                const DemoFrame &d = g_demoSamples[idx];
                DemoWaypoint &w = g_demoWaypoints[g_demoWaypointsCount++];
                w.sourceIndex = idx; w.x = d.x; w.z = d.z;
                w.speed = d.speed; w.movementHeading = d.movementHeading;
                w.headingValid = d.headingValid;
            }
            const DemoFrame &first = g_demoSamples[0];
            const DemoFrame &last = g_demoSamples[g_demoCount - 1];
            const float dx = last.x - first.x, dz = last.z - first.z;
            g_demoFinalDisplacement = std::sqrt(dx * dx + dz * dz);
            g_demoAnalyzed = true;
        }

        void StartDemo(MenuEntry *)
        {
            if (Process::GetTitleID() != MK7_USA || !g_ready ||
                g_recording || g_demoRecording)
            {
                MessageBox("P007a demo unavailable",
                    "Need MK7 USA, READY, and no active recorder.")();
                return;
            }
            if (g_demoCount)
            {
                MessageBox("Existing demo in RAM",
                    "Save or explicitly DISCARD the prior demo first.")();
                return;
            }
            Probe(); // One full pointer walk; subsequent samples are lightweight.
            if (!g_observation.candidate || !g_observation.speedValid ||
                !g_vehicleAddress || !g_demoKartAddress)
            {
                MessageBox("P007a cannot record",
                    "Start offline Time Trial and verify position/speed.")();
                return;
            }
            g_demoCount = g_demoWaypointsCount = 0;
            g_demoReadFails = g_demoGapStops = g_demoMissedIntervals = 0;
            g_demoHeadingCount = g_demoLeftFrames = g_demoRightFrames = 0;
            g_demoAFrames = g_demoBFrames = g_demoMaxGapMs = 0;
            g_demoDistance = g_demoFinalDisplacement = 0;
            g_demoLastAt = 0;
            g_demoStartAt = osGetTime();
            g_demoComplete = g_demoAnalyzed = g_demoStoppedOnFault = false;
            g_demoRecording = true;
            MessageBox("P007a MANUAL demo started",
                "Close CTRPF and DRIVE YOURSELF. Nothing controls the kart."
                " When finished, open menu and select STOP & ANALYZE."
                " First test: 5-10s on a simple stretch.")();
        }
        void StopAnalyzeDemo(MenuEntry *)
        {
            g_demoRecording = false;
            g_demoComplete = g_demoCount > 0;
            AnalyzeDemoBuffer();
            if (!g_demoCount)
            {
                MessageBox("P007a no demo",
                    "No valid samples. See demo diagnostics.")();
                return;
            }
            MessageBox("P007a recorded",
                "Demonstration stopped and trajectory extracted in RAM."
                " Select DEMO DIAGNOSTICS and SAVE DEMO CSV.")();
        }
        void DemoDiagnostics(MenuEntry *)
        {
            if (g_demoComplete && !g_demoAnalyzed)
                AnalyzeDemoBuffer();
            char msg[650];
            const u32 duration = g_demoCount
                ? g_demoSamples[g_demoCount - 1].tMs : 0;
            std::snprintf(msg, sizeof(msg),
                "State: %s%s\n"
                "Valid samples: %u/%u\n"
                "Duration: %.2f sec (target 20Hz)\n"
                "Mean effective rate: %.1f Hz\n"
                "Max gap: %u ms | gaps >150ms: %u\n"
                "Headings from motion: %u/%u\n"
                "Manual A/B: %u/%u | stick L/R: %u/%u\n"
                "Path length: %.1f | endpoint delta: %.1f\n"
                "Reference points: %u\n"
                "Recent consecutive read faults: %u\n"
                "Stopped on anomaly: %s\n"
                "Last demo file: %s\n"
                "Travel heading != actual kart yaw.",
                g_demoRecording ? "RECORDING" :
                    g_demoComplete ? "STOPPED" : "EMPTY",
                g_demoAnalyzed ? " / ANALYZED" : "",
                g_demoCount, MAX_DEMO_SAMPLES,
                static_cast<float>(duration) / 1000.0f,
                duration ? (g_demoCount * 1000.0f / duration) : 0.0f,
                g_demoMaxGapMs, g_demoMissedIntervals,
                g_demoHeadingCount, g_demoCount,
                g_demoAFrames, g_demoBFrames,
                g_demoLeftFrames, g_demoRightFrames,
                g_demoDistance, g_demoFinalDisplacement,
                g_demoWaypointsCount, g_demoReadFails,
                g_demoStoppedOnFault ? "YES" : "NO",
                g_demoLastFile.c_str());
            MessageBox("P007a Demo Diagnostics", msg)();
        }
        void SaveDemoCSV(MenuEntry *)
        {
            if (g_demoRecording || !g_demoComplete || g_demoCount == 0)
            {
                MessageBox("Cannot save demo",
                    "Stop & analyze a non-empty demonstration first.")();
                return;
            }
            if (!g_demoAnalyzed)
                AnalyzeDemoBuffer();
            const std::string folder = "/CollieKart";
            if (Directory::IsExists(folder) != 1 &&
                Directory::Create(folder) != Directory::SUCCESS)
            {
                MessageBox("Demo save failed",
                    "Could not create SD:/CollieKart. RAM preserved.")();
                return;
            }
            std::string filePath;
            char name[55];
            for (unsigned i = 1; i <= 999; ++i)
            {
                std::snprintf(name, sizeof(name),
                              "/CollieKart/demo_%03u.csv", i);
                if (File::Exists(name) == 0) { filePath = name; break; }
            }
            if (filePath.empty() || File::Create(filePath) != File::SUCCESS)
            {
                MessageBox("Demo save failed",
                    "No free filename, or SD create failed. RAM preserved.")();
                return;
            }
            File csv;
            if (File::Open(csv, filePath, File::WRITE | File::TRUNCATE)
                != File::SUCCESS)
            {
                MessageBox("Demo save failed",
                    "SD open failed. RAM preserved.")();
                return;
            }
            bool saved = csv.WriteLine(
                "time_ms,pos_x,pos_y,pos_z,speed,buttons,stick_x,"
                "travel_heading_rad,heading_valid,lap,checkpoint,"
                "reference_waypoint") == File::SUCCESS;
            unsigned waypoint = 0;
            char row[256];
            for (unsigned i = 0; saved && i < g_demoCount; ++i)
            {
                const DemoFrame &d = g_demoSamples[i];
                const bool reference = waypoint < g_demoWaypointsCount &&
                    g_demoWaypoints[waypoint].sourceIndex == i;
                if (reference) ++waypoint;
                const int len = std::snprintf(row, sizeof(row),
                    "%lu,%.4f,%.4f,%.4f,%.5f,%u,%u,%.6f,%d,%u,%u,%d",
                    static_cast<unsigned long>(d.tMs),
                    d.x, d.y, d.z, d.speed,
                    static_cast<unsigned>(d.buttons),
                    static_cast<unsigned>(d.stickX),
                    d.movementHeading, d.headingValid ? 1 : 0,
                    static_cast<unsigned>(d.lap),
                    static_cast<unsigned>(d.checkpoint),
                    reference ? 1 : 0);
                if (len <= 0 || len >= static_cast<int>(sizeof(row)) ||
                    csv.WriteLine(row) != File::SUCCESS)
                    saved = false;
            }
            csv.Close();
            if (saved)
            {
                g_demoLastFile = filePath;
                MessageBox("Demo saved to SD", filePath)();
            }
            else
                MessageBox("Demo save incomplete",
                    "Check SD:/CollieKart/demo_*.csv. RAM retained."
                    " Do not use a partial CSV for training.")();
        }
        void DiscardDemo(MenuEntry *)
        {
            if (g_demoRecording)
            {
                MessageBox("Stop demo first",
                    "Use STOP & ANALYZE before discarding.")();
                return;
            }
            // Explicit menu command; never automatically delete unsaved RAM.
            g_demoCount = g_demoWaypointsCount = 0;
            g_demoComplete = g_demoAnalyzed = g_demoStoppedOnFault = false;
            g_demoReadFails = g_demoGapStops = g_demoMissedIntervals = 0;
            g_demoHeadingCount = g_demoLeftFrames = g_demoRightFrames = 0;
            g_demoAFrames = g_demoBFrames = g_demoMaxGapMs = 0;
            g_demoDistance = g_demoFinalDisplacement = 0;
            g_demoStartAt = g_demoLastAt = 0;
            MessageBox("Demo RAM cleared",
                "Previous SD files remain intact. Ready to record again.")();
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
                        const u32 action = g_nativeAction.load(std::memory_order_acquire);
                        if (ActionFrameLimit(action) == 0 ||
                            (g_nativeAttemptMask.load() & (1u << action)))
                        {
                            AbortControl("Action already tried / invalid");
                            return;
                        }
                        g_nativeInjectedFrames.store(0, std::memory_order_release);
                        g_nativeReleaseFrames.store(0, std::memory_order_release);
                        g_nativeAbortSignal.store(0, std::memory_order_release);
                        g_nativeLastStick.store(7, std::memory_order_release);
                        g_nativeSpeedBefore = g_observation.speed;
                        g_nativeSpeedAfter = g_nativeSpeedBefore;
                        g_nativeDeadline.store(static_cast<u32>(now) +
                            ActionTimeoutMs(action), std::memory_order_release);
                        // Mark attempted before enabling hook actuation.
                        g_nativeAttemptMask.fetch_or(1u << action);
                        g_nativePulseState.store(1, std::memory_order_release);
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
                    now - g_controlPulseStart >
                        ActionTimeoutMs(g_nativeAction.load()) ||
                    (g_nativePulseState != 1 && g_nativePulseState != 4))
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
                    g_nativeSpeedAfter = g_observation.speed;
                    if (!g_controlIsDryRun && g_nativeReleaseFrames < 3)
                        AbortControl("No confirmed physical-release frames");
                    else
                        g_control = FINISHED;
                }
            }
        }

        void QueueControlExperiment(bool dryRun, u32 requestedAction = TEST_NONE)
        {
            if (g_control == WAIT_CLOSE || g_control == COUNTDOWN ||
                g_control == PULSE || g_control == OBSERVE)
            {
                MessageBox("P006 experiment already queued",
                    "Finish or abort the current experiment first.")();
                return;
            }
            if (!dryRun && (ActionFrameLimit(requestedAction) == 0 ||
                            (g_nativeAttemptMask.load() & (1u << requestedAction)) ||
                            !g_dryRunComplete || !g_physicalAbortVerified ||
                            !g_nativeHookEnabled || g_nativeSeenFrames < 30 ||
                            g_nativeManualAFrames < 10 ||
                            g_nativeReadFailures != 0 ||
                            g_nativeWriteFailures != 0))
            {
                MessageBox("P006.4 control test LOCKED",
                    "Need native hook, 30+ player frames, 10+ physical A,"
                    " successful dry run plus B-abort, zero read/write"
                    " faults, and no prior attempt for this action.")();
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
            if (!dryRun)
                g_nativeAction.store(requestedAction, std::memory_order_release);
            g_control = WAIT_CLOSE;
            g_controlNextAt = g_controlPulseStart = 0;
            g_controlLastCallbackAt = 0;
            g_controlWaitStartedAt = g_controlNeutralSince = 0;
            g_controlObserveEndAt = 0;
            g_pulseControlReads = 0;
            g_pulseMk7AccelerateSeen = g_pulseMovedSeen = false;
            g_nativePulseState = 0;
            g_nativeDeadline = 0;
            char title[96];
            std::snprintf(title, sizeof(title), "P006.4 %s queued",
                dryRun ? "DRY RUN" : NativeActionName(requestedAction));
            MessageBox(title, dryRun
                ? "Release A/B/START after closing the plugin menu."
                  " The neutral interlock and five-second countdown begin."
                  " Resume game early, release controls. B aborts."
                : "One-shot experimental input, limited by frames and time."
                  " Close CTRPF and unpause MK7 early in countdown."
                  " RELEASE A/B/START and center the Circle Pad."
                  " B/START or physical stick aborts during injection."
                  " If controls stick, exit MK7 and restore P006.3b.")();
        }

        void QueueDryRun(MenuEntry *) { QueueControlExperiment(true); }
        void QueueThrottle(MenuEntry *) { QueueControlExperiment(false, TEST_THROTTLE); }
        void QueueLeft(MenuEntry *) { QueueControlExperiment(false, TEST_LEFT); }
        void QueueRight(MenuEntry *) { QueueControlExperiment(false, TEST_RIGHT); }
        void QueueReverse(MenuEntry *) { QueueControlExperiment(false, TEST_REVERSE); }

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
            screen.Draw(g_ready ? "CK P007a | READY" : "CK P007a | INACTIVE",
                        5, 226, g_ready ? Color::Green : Color::White, Color::Black);

            // Elapsed-time schedule: avoids assuming a specific MK7 FPS.
            // Only ~2Hz full pointer walks; <=10Hz lightweight flag reads.
            if (!menuOpen && g_ready &&
                (g_live || g_recording || g_demoRecording ||
                 g_control == WAIT_CLOSE ||
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
                if (g_demoRecording)
                    SampleDemo(osGetTime());
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
            if (g_demoRecording)
            {
                char demoHud[45];
                std::snprintf(demoHud, sizeof(demoHud), "DEMO %u/%u",
                    g_demoCount, MAX_DEMO_SAMPLES);
                screen.DrawRect(166, 211, 152, 12, Color::Black, true);
                screen.Draw(demoHud, 170, 212, Color::Green, Color::Black);
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
                // Disarm synthetic input before disabling the hook.
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
                if (g_demoRecording)
                {
                    g_demoRecording = false;
                    g_demoComplete = g_demoCount > 0;
                }
                g_havePreviousFlags = false;
                g_previousModeAt = 0;
            }
            ++g_toggles;
        }
        void ToggleLive(MenuEntry *)
        {
            g_live = !g_live;
            if (!g_recording && !g_demoRecording)
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

        PluginMenu menu("Collie Kart P007a", 0, 0, 1,
            "READ-ONLY demonstration recorder + path analysis.\n"
            "No autonomous driving or synthetic control.");
        menu.SynchronizeWithFrame(true);
        menu.ShowWelcomeMessage(false);
        menu += new MenuEntry("Toggle READY / INACTIVE", nullptr, ToggleReady,
            "Master toggle for read-only observation and demonstration capture.");
        // P007a is RECORD-ONLY. No synthetic input menu actions.
        menu += new MenuEntry("1: START manual demonstration", nullptr, StartDemo,
            "20Hz target in-RAM recording. Drive manually, no AI input.");
        menu += new MenuEntry("2: STOP & ANALYZE demonstration", nullptr, StopAnalyzeDemo,
            "Stop recording; extract reference path in RAM.");
        menu += new MenuEntry("3: DEMO diagnostics", nullptr, DemoDiagnostics,
            "Sample rate, input count, distance and motion-heading quality.");
        menu += new MenuEntry("4: SAVE demo CSV to SD", nullptr, SaveDemoCSV,
            "Write new SD:/CollieKart/demo_###.csv after stopping.");
        menu += new MenuEntry("5: DISCARD demo in RAM", nullptr, DiscardDemo,
            "Clears the RAM buffer only; existing SD files are untouched.");
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
        // No HID injection. Four controlled tests, never automatic driving.
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
