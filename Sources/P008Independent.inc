// P008 v1: independent, route-free, on-console TD control learner.
// Included inside main.cpp's anonymous namespace after P007bLearning.inc.
// HARD RULE: this file never reads g_demoSamples, g_demoWaypoints or g_aiWeights.
// Current radar returns INVALID until a separately validated collision query exists.

enum IndMode { IND_MANUAL=0, IND_GUIDED=1, IND_TRAIN=2, IND_EVAL=3 };
enum IndPhase { IND_IDLE=0, IND_WAIT=1, IND_DRIVE=2, IND_END=3 };
enum IndStop { IND_NONE=0, IND_TIMEOUT=1, IND_LAP=2, IND_ABORT=3,
               IND_BAD_SENSOR=4, IND_INPUT_FAULT=5, IND_MENU=6,
               IND_READY_OFF=7, IND_BUFFER=8, IND_BAD_START=9 };
const char* IndStopName(int n) {
    switch(n) {
        case IND_TIMEOUT:return "TIME LIMIT";
        case IND_LAP:return "LAP COMPLETE";
        case IND_ABORT:return "PHYSICAL ABORT";
        case IND_BAD_SENSOR:return "INVALID/STALLED OBSERVATION";
        case IND_INPUT_FAULT:return "NATIVE INPUT FAULT";
        case IND_MENU:return "MENU REOPENED";
        case IND_READY_OFF:return "READY OFF";
        case IND_BUFFER:return "BUFFER FULL";
        case IND_BAD_START:return "INVALID START";
        default:return "NONE";
    }
}
const char* IndModeName(IndMode m) {
    switch(m){case IND_MANUAL:return "MANUAL";
        case IND_GUIDED:return "GUIDED";
        case IND_TRAIN:return "TRAIN";
        case IND_EVAL:return "EVALUATE";
        default:return "UNKNOWN";}
}
constexpr unsigned IND_ACTIONS=9, IND_HASH=256, IND_FEATURES=18;
constexpr unsigned IND_ATTEMPTS=6, IND_TICKS=2000;
constexpr u64 IND_TICK_MS=65, IND_COMMAND_TTL=180, IND_GRACE=1200;
constexpr u64 IND_HOLD_MS=390; // six sensor ticks per learned action
constexpr float IND_ALPHA=0.12f, IND_DISCOUNT=0.985f;
struct IndSensor {
    bool valid=false, faceValid=false, surfaceValid=false, wallValid=false;
    bool managerValid=false, radarValid=false; // radar MUST remain invalid in v1.
    float x=0,z=0,speed=0,faceX=0,faceZ=1;
    float yaw=0, progress=0, maximum=0;
    // LapRankChecker::KartInfo read-only checkpoint telemetry.
    // Direction is a local checkpoint heading, NOT a target bearing.
    bool checkpointDirValid=false;
    u8 checkpointPrev=0,checkpointCurrent=0;
    float checkpointDirX=0,checkpointDirZ=0,checkpointHeadingError=0;
    float slip=0; // difference between heading of travel and facing
    u32 surface=0, wall=0, lap=0, flags=0;
    int mode=0;
    u32 manager=0;
    float radar[12]={};
    u16 radarMask=0;
};
struct IndDecision {
    u32 t=0,updates=0;
    float x=0,z=0,speed=0,progress=0,reward=0,total=0,q=0;
    float yaw=0,slip=0;
    u32 surface=0,wall=0,lap=0;
    u8 action=0,explored=0,validFace=0;
};
struct IndAttempt {
    IndDecision log[IND_TICKS];
    unsigned count=0;
    int reason=IND_NONE;
    float reward=0,bestProgress=0,finalProgress=0;
    u32 startLap=0, endLap=0,updates=0,explorations=0,frames=0;
    bool saved=false;
};
static IndAttempt g_indAttempts[IND_ATTEMPTS];
static IndMode g_indMode=IND_GUIDED; // preserve legacy behavior at first launch
static IndPhase g_indPhase=IND_IDLE;
static unsigned g_indCount=0,g_indUpdates=0,g_indPolicyVersion=0;
static float g_indWeights[IND_ACTIONS][IND_HASH]={};
static bool g_indModelDirty=false;
static unsigned g_indStartCalls=0,g_indDenials=0,g_indSensorFailures=0;
// Distinguish observation-read failure, heading failure, coordinate jump and timing stall.
static unsigned g_indReadFailures=0,g_indFacingFailures=0,g_indWarpFailures=0,g_indTimingFailures=0;
static u32 g_indRng=0x21AD489Bu;
static u64 g_indWaitAt=0,g_indNeutralAt=0,g_indStarted=0,g_indLastTick=0;
static u32 g_indFrameStart=0,g_indFaultStart=0,g_indLimit=12000;
static int g_indLastReason=IND_NONE;
static IndSensor g_indLastSensor;
static float g_indRewardMaximum=0,g_indProgressAt=0,g_indLastReward=0;
// P008.1: current-progress delta is distinct from historical best progress.
// A reward is paid only for NEW legitimate territory; retreat is penalized.
static float g_indPreviousProgress=0;
static unsigned g_indStartLap=0,g_indPrevWall=0,g_indNoAdvance=0;
static bool g_indPrevious=false;
static u64 g_indActionAt=0;
static float g_indHeldReward=0;
static unsigned g_indHeldTicks=0;
static u8 g_indPrevAction=0;
static u16 g_indPrevFeatures[IND_FEATURES]={};
static unsigned g_indModelSnapshot=0;
static const u16 IND_BUTTONS[IND_ACTIONS]={1,1,1,1,1,0,2,2,2};
static const u8 IND_STICKS[IND_ACTIONS]={7,4,10,1,13,7,7,4,10};

u32 IndRandom() {
    u32 x=g_indRng;
    x ^= x<<13;x ^= x>>17;x ^= x<<5;
    g_indRng=x?x:0xD2B74407u;
    return g_indRng;
}
float IndUnit() {return (IndRandom()&0xFFFFFF)/16777216.0f;}
float IndClamp(float v,float lo,float hi) {return v<lo?lo:v>hi?hi:v;}
int IndBin(float v,float scale,int lo,int hi) {
    int n=static_cast<int>(std::floor(v*scale));
    return n<lo?lo:n>hi?hi:n;
}
u16 IndHash(int channel,int value,int offset) {
    u32 h=2166136261u;
    h=(h^static_cast<u32>(channel+31))*16777619u;
    h=(h^static_cast<u32>(value+4096))*16777619u;
    h=(h^static_cast<u32>(offset+7))*16777619u;
    h^=h>>16;
    return static_cast<u16>(h%IND_HASH);
}
// Bounded, readonly. Never assume a failed sensor means empty space.
bool IndObserve(IndSensor &s,bool requireUnpaused=true) {
    s=IndSensor();
    if(!g_observation.candidate || !g_vehicleAddress || !g_demoKartAddress)
        return false;
    u32 pad=0;u8 paused=1;
    if(!ReadNumber(g_demoKartAddress+0x28,s.x)||
       !ReadNumber(g_demoKartAddress+0x30,s.z)||
       !ReadNumber(g_vehicleAddress+0xF2C,s.speed)||
       !ReadNumber(g_demoKartAddress+0x14,s.progress)||
       !ReadNumber(g_demoKartAddress+0x18,s.maximum)||
       !ReadWord(g_demoKartAddress+0x1C,s.lap)||
       !ReadWord(g_demoKartAddress+0x24,s.flags)||
       !ReadPointer(g_vehicleAddress+0xE0,pad)||
       !Process::Read8(pad+0x3D,paused) || (requireUnpaused&&paused))
        return false;
    // Native KartInfo offsets from mk7re/MK7-Memory. No unknown
    // checkpoint accessor pointers or invented next-target geometry.
    u8 cpPrev=0,cpCurrent=0;
    float cpX=0,cpZ=0;
    if(Process::Read8(g_demoKartAddress+0xC,cpPrev)&&
       Process::Read8(g_demoKartAddress+0xD,cpCurrent)&&
       ReadNumber(g_demoKartAddress+0x4,cpX)&&
       ReadNumber(g_demoKartAddress+0x8,cpZ)&&
       std::isfinite(cpX)&&std::isfinite(cpZ)&&
       cpX*cpX+cpZ*cpZ>0.25f&&
       cpX*cpX+cpZ*cpZ<2.25f) {
        const float n=std::sqrt(cpX*cpX+cpZ*cpZ);
        s.checkpointPrev=cpPrev;s.checkpointCurrent=cpCurrent;
        s.checkpointDirX=cpX/n;s.checkpointDirZ=cpZ/n;
        s.checkpointDirValid=true;
    }
    if(!std::isfinite(s.x)||!std::isfinite(s.z)||
       !std::isfinite(s.speed)||!std::isfinite(s.progress)||
       !std::isfinite(s.maximum)||std::fabs(s.x)>1000000.0f||
       std::fabs(s.z)>1000000.0f||std::fabs(s.speed)>100.0f||
       s.progress<-.5f||s.progress>10.0f||
       s.maximum<-.5f||s.maximum>10.0f||s.lap>9)
        return false;
    float fx=0,fz=0;
    if(ReadNumber(g_vehicleAddress+0xF44,fx)&&
       ReadNumber(g_vehicleAddress+0xF4C,fz)) {
        const float len=std::sqrt(fx*fx+fz*fz);
        if(std::isfinite(len) && len>.5f && len<1.5f) {
            s.faceX=fx/len;s.faceZ=fz/len;s.faceValid=true;
            if(s.checkpointDirValid) {
                const float cross=s.faceX*s.checkpointDirZ-
                                  s.faceZ*s.checkpointDirX;
                const float dot=s.faceX*s.checkpointDirX+
                                s.faceZ*s.checkpointDirZ;
                s.checkpointHeadingError=std::atan2(cross,dot);
            }
        }
    }
    // Distinguish kart facing from actual movement direction (side-slip).
    // Vehicle+0x38 is documented Rigid::m_velocity, not a demo heading.
    float vx=0,vz=0;
    if(s.faceValid&&ReadNumber(g_vehicleAddress+0x38,vx)&&
       ReadNumber(g_vehicleAddress+0x40,vz)&&
       std::isfinite(vx)&&std::isfinite(vz) &&
       vx*vx+vz*vz>0.0001f &&
       vx*vx+vz*vz<100000.0f) {
        const float travel=std::atan2(vz,vx);
        const float facing=std::atan2(s.faceZ,s.faceX);
        s.slip=std::atan2(std::sin(travel-facing),
                          std::cos(travel-facing));
    }
    float yaw=0;
    if(ReadNumber(g_vehicleAddress+0xF24,yaw)&&std::fabs(yaw)<100.0f)
        s.yaw=yaw;
    u32 road=0;
    if(ReadWord(g_vehicleAddress+0xD14,road)&&road<=31) {
        s.surface=road;s.surfaceValid=true;
    }
    u32 wall=0;
    if(ReadWord(g_vehicleAddress+0xC94,wall)) {
        s.wall=(wall&1u)?1u:0u;s.wallValid=true;
    }
    s.mode=static_cast<int>(g_environment.mode);
    // Known game-object chain: root -> scene manager -> game scene ->
    // character engine -> director list -> ground collision manager.
    // Read pointer only. DO NOT invoke unknown collision functions.
    u32 sceneMgr=0,scene=0,encoded=0,engine=0,collection=0,manager=0;
    if(ReadPointer(ROOTSYSTEM_USA_REV2+0x04,sceneMgr)&&
       ReadPointer(sceneMgr+0x04,scene)&&
       ReadWord(scene+0x1E4,encoded)) {
        engine=encoded^ENGINE_XOR;
        if(PlausibleAddress(engine)&&
           ReadPointer(engine+0x1C,collection)&&
           ReadPointer(collection+0x08,manager)) {
            s.manager=manager;s.managerValid=true;
        }
    }
    // P008 v1 radar deliberately invalid. A native ray/sphere ABI
    // or verified KCL parser must be implemented before setting any bit.
    s.radarValid=false;s.radarMask=0;
    s.valid=true;
    return true;
}
void IndFeatures(const IndSensor &s,u16 out[IND_FEATURES]) {
    const int speed=IndBin(s.speed,.35f,-3,9);
    const int yaw=IndBin(s.yaw,2.0f,-12,12);
    const int surf=s.surfaceValid?static_cast<int>(s.surface):32;
    const int wall=s.wallValid?static_cast<int>(s.wall):2;
    const int face=s.faceValid?1:0;
    out[0]=IndHash(0,1,0); // bias
    out[1]=IndHash(1,speed,0);
    out[2]=IndHash(2,speed,1);
    out[3]=IndHash(3,yaw,0);
    out[4]=IndHash(4,yaw,1);
    out[5]=IndHash(5,surf,0);
    out[6]=IndHash(6,wall,0);
    out[7]=IndHash(7,s.mode,0);
    out[8]=IndHash(8,speed*3+yaw,0);
    out[9]=IndHash(9,speed*3+surf,0);
    out[10]=IndHash(10,speed*3+wall,0);
    out[11]=IndHash(11,face,0);
    out[12]=IndHash(12,s.flags&1u?1:0,0); // reverse progress flag
    out[13]=IndHash(13,IndBin(s.slip,2.0f,-7,7),0);
    // P008.2a: track travel heading as an observation, never as a
    // hardcoded steering command. This is NOT full corridor geometry.
    const int heading=s.checkpointDirValid&&s.faceValid?
        IndBin(s.checkpointHeadingError,.35f,-9,9):99;
    out[14]=IndHash(14,heading,0);
    out[15]=IndHash(15,heading,speed);
    out[16]=IndHash(16,heading,yaw);
    out[17]=IndHash(17,s.checkpointDirValid?1:0,0);
}
// Exactly zero reference to human or Nintendo routes.
float IndQ(u8 action,const u16 f[IND_FEATURES]) {
    float q=0;
    for(unsigned i=0;i<IND_FEATURES;++i)
        q+=g_indWeights[action][f[i]];
    return q/static_cast<float>(IND_FEATURES);
}
u8 IndSelect(const u16 f[IND_FEATURES],bool explore,bool &explored,float &q) {
    explored=false;
    const float eps=explore?IndClamp(.32f-g_indUpdates*.00004f,.07f,.32f):0;
    if(eps>0 && IndUnit()<eps) {
        // Initial exploration should primarily discover forward driving;
        // recovery/reverse remains available to learned greedy actions.
        const u8 a=static_cast<u8>(IndRandom()%6u);
        explored=true;q=IndQ(a,f);return a;
    }
    float best=-1e20f;
    u8 chosen=0;
    for(unsigned a=0;a<IND_ACTIONS;++a) {
        const float v=IndQ(static_cast<u8>(a),f);
        // Random unbiased tie-break avoids always choosing index zero.
        if(v>best+0.00001f || (std::fabs(v-best)<=0.00001f &&
            (IndRandom()&1u))) {best=v;chosen=static_cast<u8>(a);}
    }
    q=best;return chosen;
}
void IndUpdate(const u16 f[IND_FEATURES],u8 action,float reward,float nextQ) {
    const float prior=IndQ(action,f);
    const float error=IndClamp(reward+IND_DISCOUNT*nextQ-prior,-8.0f,8.0f);
    // Since Q is an average of the feature weights, modifying each active
    // weight by alpha*TD yields an aggregate update of alpha*TD.
    for(unsigned i=0;i<IND_FEATURES;++i) {
        float &w=g_indWeights[action][f[i]];
        w=IndClamp(w+IND_ALPHA*error,-20.0f,20.0f);
    }
    ++g_indUpdates;
    ++g_indPolicyVersion;
    g_indModelDirty=true;
}
float IndReward(const IndSensor &s,bool &lapFinished) {
    lapFinished=s.lap>g_indStartLap;
    float reward=-0.008f; // time cost, not speed reward
    const float delta=s.progress-g_indPreviousProgress;
    const float newGround=s.maximum-g_indRewardMaximum;
    // Ignore large discontinuities; do not reward repeated traversal of
    // already visited track. Legitimate lap transition is handled below.
    if(delta>0.000001f && delta<=0.025f &&
       newGround>0.000001f && newGround<=0.025f) {
        reward+=IndClamp(newGround*100.0f,0,2.5f);
        g_indNoAdvance=0;
    } else {
        if(delta<-.0001f && delta>=-.025f)
            reward-=IndClamp(-delta*35.0f,0,0.45f);
        if(++g_indNoAdvance>50)reward-=.055f;
    }
    g_indPreviousProgress=s.progress;
    if(s.maximum>=g_indRewardMaximum)
        g_indRewardMaximum=s.maximum;
    if(s.wallValid && s.wall && !g_indPrevWall) reward-=1.0f;
    g_indPrevWall=s.wallValid?s.wall:0;
    if(s.flags&1u) reward-=.025f; // reverse checkpoint direction
    if(lapFinished) reward+=20.0f;
    return reward;
}
void IndStopNow(int reason) {
    // Release synthetic input before touching logs or learning state.
    g_aiAuthority.store(false,std::memory_order_release);
    g_aiDeadlineMs.store(0,std::memory_order_release);
    if(g_indPhase!=IND_WAIT && g_indPhase!=IND_DRIVE)return;
    g_indLastReason=reason;
    if(g_indPhase==IND_DRIVE && g_indCount<IND_ATTEMPTS) {
        IndAttempt &a=g_indAttempts[g_indCount];
        a.reason=reason;
        a.frames=g_aiActuatorFrames.load()-g_indFrameStart;
        a.endLap=g_indLastSensor.lap;
        a.finalProgress=g_indLastSensor.maximum;
        a.updates=g_indUpdates;
        if(g_indPrevious && g_indMode==IND_TRAIN) {
            const float terminal=reason==IND_LAP?20.0f:0.0f;
            IndUpdate(g_indPrevFeatures,g_indPrevAction,
                      g_indHeldReward+terminal,0);
        }
        if(a.count)++g_indCount;
    }
    g_indPrevious=false;g_indActionAt=0;
    g_indHeldReward=0;g_indHeldTicks=0;
    g_indPhase=IND_END;
}
void IndCycleMode(MenuEntry *) {
    if(g_indPhase==IND_WAIT||g_indPhase==IND_DRIVE||
       g_aiPhase==AI_COUNTDOWN||g_aiPhase==AI_RUNNING||
       g_demoRecording||g_recording) {
        MessageBox("Mode locked","Stop any attempt or recording before changing mode.")();
        return;
    }
    g_aiAuthority.store(false,std::memory_order_release);
    g_indMode=static_cast<IndMode>((static_cast<int>(g_indMode)+1)%4);
    MessageBox("P008.1 driver mode",std::string(IndModeName(g_indMode))+
       "\nMANUAL: no AI. GUIDED: proven P007.\n"
       "TRAIN: independent learning. EVALUATE: no policy updates.")();
}
void IndGuidedQueue(MenuEntry *entry) {
    if(g_indMode!=IND_GUIDED) {
        MessageBox("GUIDED mode required",
          "Cycle P008 mode to GUIDED to run P007b.2.")();
        return;
    }
    AiQueueAttempt(entry);
}
void IndStart(MenuEntry *) {
    if((g_indMode!=IND_TRAIN && g_indMode!=IND_EVAL)||
       !g_ready||g_indPhase==IND_WAIT||g_indPhase==IND_DRIVE||
       g_aiPhase==AI_COUNTDOWN||g_aiPhase==AI_RUNNING||
       g_demoRecording||g_recording||g_indCount>=IND_ATTEMPTS) {
        ++g_indDenials;
        MessageBox("Independent start denied",
            "Select TRAIN/EVALUATE, READY, stop recorders and free a slot.")();
        return;
    }
    if(!g_nativeHookEnabled.load()||g_nativeSeenFrames<30||
       g_nativeReadFailures!=0||g_nativeWriteFailures!=0||
       g_aiActuatorFaults!=0) {
        ++g_indDenials;
        MessageBox("Native preflight required",
            "Native hook must be active for 30 frames with zero faults."
            " Physical controls still abort and the watchdog remains active.")();
        return;
    }
    Probe();
    IndSensor s;
    if(!IndObserve(s,false)||std::fabs(s.speed)>.35f) {
        ++g_indDenials;
        MessageBox("Invalid race start",
           "Park stationary in an unpaused offline MK7 USA Time Trial."
           " A valid kart, velocity and race-progress signal are required.")();
        return;
    }
    g_indPhase=IND_WAIT;g_indWaitAt=0;g_indNeutralAt=0;
    g_indStarted=0;g_indLastTick=0;
    g_indPrevious=false;g_indNoAdvance=0;
    g_indActionAt=0;g_indHeldReward=0;g_indHeldTicks=0;
    g_aiAuthority.store(false,std::memory_order_release);
    g_aiPhysicalOverride.store(0);
    ++g_indStartCalls;
    MessageBox("Independent attempt queued",
        "No demo needed. Close CTRPF, resume race, release A/B/START"
        " and center Circle Pad during the short neutral check."
        " Physical controls abort. OFFLINE USA Rev2 Time Trial ONLY."
        " Radar is NOT available in P008 v1.")();
}
void IndStep(u64 now,bool menuOpen) {
    if(g_indPhase!=IND_WAIT&&g_indPhase!=IND_DRIVE)return;
    if(menuOpen) {
        if(g_indPhase==IND_DRIVE || g_indWaitAt)
            IndStopNow(IND_MENU);
        return;
    }
    if(!g_ready){IndStopNow(IND_READY_OFF);return;}
    if(g_indPhase==IND_WAIT) {
        if(!g_indWaitAt)g_indWaitAt=now;
        if(Controller::IsKeyDown(Key::B)){IndStopNow(IND_ABORT);return;}
        if(now<g_indWaitAt||now-g_indWaitAt>12000) {
            IndStopNow(IND_BAD_START);return;
        }
        if(now-g_indWaitAt<IND_GRACE-350)return;
        const u32 blocked=static_cast<u32>(Key::A) |
          static_cast<u32>(Key::B)|static_cast<u32>(Key::Start)|
          static_cast<u32>(Key::CPadLeft)|static_cast<u32>(Key::CPadRight);
        if(Controller::GetKeysDown(false)&blocked){g_indNeutralAt=0;return;}
        if(!g_indNeutralAt)g_indNeutralAt=now;
        if(now-g_indWaitAt<IND_GRACE||now-g_indNeutralAt<350)return;
        Probe();IndSensor s;
        if(!IndObserve(s)||std::fabs(s.speed)>.5f) {
            IndStopNow(IND_BAD_START);return;
        }
        g_indPhase=IND_DRIVE;g_indStarted=now;g_indLastTick=0;
        g_indLastSensor=s;g_indRewardMaximum=s.maximum;
        g_indPreviousProgress=s.progress;
        g_indProgressAt=s.maximum;g_indStartLap=s.lap;
        g_indPrevWall=0;g_indNoAdvance=0;
        g_indActionAt=0;g_indHeldReward=0;g_indHeldTicks=0;
        g_indFrameStart=g_aiActuatorFrames.load();
        g_indFaultStart=g_aiActuatorFaults.load();
        g_aiPassRelease.store(0);
        IndAttempt &a=g_indAttempts[g_indCount];
        a.count=0;a.reason=IND_NONE;a.reward=0;
        a.bestProgress=s.maximum;a.finalProgress=s.maximum;
        a.startLap=s.lap;a.endLap=s.lap;a.updates=g_indUpdates;
        a.explorations=0;a.saved=false;
        g_indRng^=static_cast<u32>(now)|1u;
        return;
    }
    if(g_aiPhysicalOverride.load()){IndStopNow(IND_ABORT);return;}
    if(g_aiActuatorFaults.load()!=g_indFaultStart||
       g_nativeReadFailures.load()!=0){IndStopNow(IND_INPUT_FAULT);return;}
    if(now<g_indStarted||now-g_indStarted>g_indLimit) {
        IndStopNow(IND_TIMEOUT);return;
    }
    if(g_indLastTick&&now>=g_indLastTick&&
       now-g_indLastTick<IND_TICK_MS)return;
    if(g_indLastTick&&(now<g_indLastTick||now-g_indLastTick>400)) {
        ++g_indTimingFailures;IndStopNow(IND_BAD_SENSOR);return;
    }
    g_indLastTick=now;
    if(g_indCount>=IND_ATTEMPTS){IndStopNow(IND_BUFFER);return;}
    IndAttempt &a=g_indAttempts[g_indCount];
    if(a.count>=IND_TICKS){IndStopNow(IND_BUFFER);return;}
    IndSensor s;
    if(!IndObserve(s)){++g_indSensorFailures;++g_indReadFailures;IndStopNow(IND_BAD_SENSOR);return;}
    if(!s.faceValid) {
        // Valid forward direction is mandatory for the first autonomous
        // release. Falling back to a demo heading is forbidden.
        ++g_indSensorFailures;++g_indFacingFailures;IndStopNow(IND_BAD_SENSOR);return;
    }
    // Detect implausible coordinate discontinuities (rescue/warp) and stop;
    // do not train on reward across teleport-like transitions.
    const float dx=s.x-g_indLastSensor.x,dz=s.z-g_indLastSensor.z;
    if(dx*dx+dz*dz>250000.0f){
        ++g_indSensorFailures;++g_indWarpFailures;IndStopNow(IND_BAD_SENSOR);return;
    }
    const bool training=g_indMode==IND_TRAIN;
    bool lapFinished=false;
    const float reward=IndReward(s,lapFinished);
    a.reward+=reward;
    if(s.maximum>a.bestProgress)a.bestProgress=s.maximum;
    g_indLastReward=reward;g_indLastSensor=s;
    if(lapFinished) {
        // Terminal SARSA update uses the lap reward, without bootstrapping
        // from an action that will never execute.
        if(g_indPrevious&&training)
            IndUpdate(g_indPrevFeatures,g_indPrevAction,
                      g_indHeldReward+reward,0);
        g_indPrevious=false;
        IndStopNow(IND_LAP);
        return;
    }
    // Keep observing and renewing the short native-input watchdog on
    // every tick, but do not change direction during a held maneuver.
    if(g_indPrevious && now>=g_indActionAt &&
       now-g_indActionAt<IND_HOLD_MS) {
        g_indHeldReward+=reward;
        ++g_indHeldTicks;
        g_aiCommand.store((static_cast<u32>(IND_STICKS[g_indPrevAction])<<16)|
            IND_BUTTONS[g_indPrevAction],std::memory_order_release);
        g_aiDeadlineMs.store(static_cast<u32>(now+IND_COMMAND_TTL),
            std::memory_order_release);
        g_aiAuthority.store(true,std::memory_order_release);
        return;
    }
    u16 features[IND_FEATURES];
    IndFeatures(s,features);
    bool explored=false;float q=0;
    const u8 action=IndSelect(features,training,explored,q);
    if(g_indPrevious&&training)
        IndUpdate(g_indPrevFeatures,g_indPrevAction,
                  g_indHeldReward+reward,q);
    g_indHeldReward=0;g_indHeldTicks=0;g_indActionAt=now;
    if(g_indNoAdvance>270){IndStopNow(IND_BAD_SENSOR);return;}
    IndDecision &d=a.log[a.count++];
    d.t=static_cast<u32>(now-g_indStarted);
    d.x=s.x;d.z=s.z;d.speed=s.speed;d.progress=s.maximum;
    d.reward=reward;d.total=a.reward;d.q=q;
    d.yaw=s.yaw;d.slip=s.slip;d.surface=s.surface;
    d.wall=s.wall;d.lap=s.lap;d.action=action;
    d.explored=explored?1:0;d.validFace=s.faceValid?1:0;
    d.updates=g_indUpdates;
    if(explored)++a.explorations;
    for(unsigned k=0;k<IND_FEATURES;++k)
        g_indPrevFeatures[k]=features[k];
    g_indPrevAction=action;g_indPrevious=true;
    g_aiCommand.store((static_cast<u32>(IND_STICKS[action])<<16)|
        IND_BUTTONS[action],std::memory_order_release);
    g_aiDeadlineMs.store(static_cast<u32>(now+IND_COMMAND_TTL),
        std::memory_order_release);
    g_aiAuthority.store(true,std::memory_order_release);
}
void IndCycleLimit(MenuEntry *) {
    if(g_indPhase==IND_WAIT||g_indPhase==IND_DRIVE)return;
    if(g_indLimit==12000)g_indLimit=30000;
    else if(g_indLimit==30000)g_indLimit=75000;
    else if(g_indLimit==75000)g_indLimit=120000;
    else g_indLimit=12000;
    char b[120];
    std::snprintf(b,sizeof(b),"Independent test limit: %lu seconds.",
        static_cast<unsigned long>(g_indLimit/1000));
    MessageBox("P008 duration",b)();
}
void IndAbort(MenuEntry *) {
    if(g_indPhase==IND_WAIT||g_indPhase==IND_DRIVE)
        IndStopNow(IND_ABORT);
}
void IndSensors(MenuEntry *) {
    Probe();IndSensor s;
    const bool ok=IndObserve(s,false);
    g_indLastSensor=s;
    char b[700];
    std::snprintf(b,sizeof(b),
        "Sample: %s | facing: %s\n"
        "Facing X/Z: %.3f / %.3f\n"
        "Speed: %.3f | yaw: %.3f slip: %.3f\n"
        "Surface: %s (%lu)\n"
        "Wall: %s (%lu)\n"
        "Progress: %.4f max %.4f lap %lu\n"
        "CP heading: %s prev %u current %u\n"
        "CP dir X/Z: %.3f / %.3f relative %.1f deg\n"
        "NEXT TARGET BEARING: UNAVAILABLE\n"
        "Collision mgr: %s 0x%08lX\n"
        "RADAR: UNAVAILABLE (0/12 rays)\n"
        "Never use dummy ray readings to steer.",
        ok?"VALID":"INVALID",s.faceValid?"VALID":"NOT VERIFIED",
        s.faceX,s.faceZ,s.speed,s.yaw,s.slip,
        s.surfaceValid?"VALID":"UNKNOWN",
        static_cast<unsigned long>(s.surface),
        s.wallValid?"VALID":"UNKNOWN",
        static_cast<unsigned long>(s.wall),s.progress,s.maximum,
        static_cast<unsigned long>(s.lap+1),
        s.checkpointDirValid?"READ ONLY":"INVALID",
        static_cast<unsigned>(s.checkpointPrev),
        static_cast<unsigned>(s.checkpointCurrent),
        s.checkpointDirX,s.checkpointDirZ,
        s.checkpointHeadingError*57.2957795f,
        s.managerValid?"PTR ONLY":"UNKNOWN",
        static_cast<unsigned long>(s.manager));
    MessageBox("P008.1 checkpoint sensor diagnostic",b)();
}
// S1 first-contact: read-only snapshot of MK7's active collision state.
// This address is from mk7re's USA Rev2 Citra reference; hardware is unverified.
// S2 look-ahead remains UNKNOWN until a native query ABI is validated.
void IndSurfaceSnapshot(MenuEntry *) {
    g_aiAuthority.store(false,std::memory_order_release);
    if (g_indPhase==IND_WAIT || g_indPhase==IND_DRIVE ||
        g_aiPhase==AI_RUNNING || g_aiPhase==AI_COUNTDOWN) {
        MessageBox("Surface diagnostic locked",
            "Stop autonomous/guided driving before inspecting collision memory.")();
        return;
    }
    if (!g_verified || Process::GetTitleID()!=MK7_USA) {
        MessageBox("Surface diagnostic unavailable",
            "Only MK7 USA is supported by this diagnostic.")();
        return;
    }
    constexpr u32 slot=0x0065C584u;
    u32 ptr=0,flags=0,rawWord=0;
    const bool slotOk=ReadWord(slot,ptr);
    const bool ptrOk=slotOk && PlausibleAddress(ptr) &&
        Process::CheckAddress(ptr,MEMPERM_READ) &&
        Process::CheckAddress(ptr+12,MEMPERM_READ);
    const bool flagsOk=ptrOk && ReadWord(ptr,flags);
    // Read a 32-bit word at +4 to avoid unverified Read16 API usage.
    const bool rawOk=flagsOk && ReadWord(ptr+4,rawWord);
    const u16 raw=static_cast<u16>(rawWord&0xFFFFu);
    const unsigned type=raw&31u;
    static const char* names[32]={
        "ROAD","ROAD2","ROAD3","SAND","LDIRT","DIRT","HDIRT","ICE",
        "DASH","GLIDE","ITROAD","RESQ","PULL","LAROAD","BELT","DUMMY0",
        "WALL","WALL2","OUTF","ITWALL","LWALL","BWALL","OUTMH","DUMMY1",
        "DUMMY2","VALLEY","TRIGGER","SOUND","CANNON","VALLEY2",
        "DUMMY3","ZONE"};
    char msg[430];
    std::snprintf(msg,sizeof(msg),
        "Slot 0x%08lX: %s\n"
        "Object 0x%08lX: %s\n"
        "Collision flags: %s 0x%08lX\n"
        "Raw flags: %s 0x%04X\n"
        "Base type: %s\n"
        "LOOK-AHEAD: UNKNOWN (not queried)\n"
        "READ ONLY. Manual driving only.\n"
        "Citra-derived address: hardware validation required.",
        static_cast<unsigned long>(slot),slotOk?"READ":"INVALID",
        static_cast<unsigned long>(ptr),ptrOk?"VALID PTR":"INVALID",
        flagsOk?"READ":"INVALID",static_cast<unsigned long>(flags),
        rawOk?"READ":"INVALID",rawOk?raw:0,
        rawOk?names[type]:"UNKNOWN");
    MessageBox("P008.2-S1 live surface snapshot",msg)();
}
void IndReport(MenuEntry *) {
    char b[280];
    std::string msg;
    std::snprintf(b,sizeof(b),
        "Mode %s | phase %d | limit %lus\n"
        "Started/denied %u/%u | captured %u/%u\n"
        "Policy v%u | updates %u | dirty %s\n"
        "Last end: %s | sensor failures: %u\n"
        "Failures: read %u, face %u, warp %u, timing %u\n"
        "RADAR UNAVAILABLE: 0/12 (not invented)\n",
        IndModeName(g_indMode),static_cast<int>(g_indPhase),
        static_cast<unsigned long>(g_indLimit/1000),
        g_indStartCalls,g_indDenials,g_indCount,IND_ATTEMPTS,
        g_indPolicyVersion,g_indUpdates,g_indModelDirty?"YES":"NO",
        IndStopName(g_indLastReason),g_indSensorFailures,
        g_indReadFailures,g_indFacingFailures,g_indWarpFailures,g_indTimingFailures);
    msg=b;
    for(unsigned i=0;i<g_indCount;++i) {
        const IndAttempt &a=g_indAttempts[i];
        std::snprintf(b,sizeof(b),
            "#%u %s | ticks %u | reward %.2f\n"
            " progress %.4f -> %.4f | explore %u\n",
            i+1,IndStopName(a.reason),a.count,a.reward,
            a.log[0].progress,a.finalProgress,a.explorations);
        msg+=b;
    }
    MessageBox("P008 independent learning",msg)();
}
bool IndSaveAttempt(IndAttempt &a) {
    const std::string folder="/CollieKart";
    if(Directory::IsExists(folder)!=1 &&
       Directory::Create(folder)!=Directory::SUCCESS)return false;
    char path[80];bool found=false;
    for(unsigned i=1;i<=999;++i) {
        std::snprintf(path,sizeof(path),"/CollieKart/ind_attempt_%03u.csv",i);
        if(File::Exists(path)==0){found=true;break;}
    }
    if(!found||File::Create(path)!=File::SUCCESS)return false;
    File f;
    if(File::Open(f,path,File::WRITE|File::TRUNCATE)!=File::SUCCESS)
        return false;
    char b[384];
    std::snprintf(b,sizeof(b),
        "reason=%s,mode=%s,reward=%.5f,start_lap=%lu,end_lap=%lu,"
        "best_progress=%.5f,final_progress=%.5f,updates=%lu",
        IndStopName(a.reason),IndModeName(g_indMode),a.reward,
        static_cast<unsigned long>(a.startLap+1),
        static_cast<unsigned long>(a.endLap+1),
        a.bestProgress,a.finalProgress,static_cast<unsigned long>(a.updates));
    bool ok=f.WriteLine(b)==File::SUCCESS;
    if(ok)ok=f.WriteLine("time_ms,pos_x,pos_z,speed,max_progress,"
       "reward,total_reward,action,exploration,q_value,policy_updates,"
       "yaw,surface,wall,lap,facing_valid")==File::SUCCESS;
    for(unsigned i=0;i<a.count&&ok;++i) {
        const IndDecision &d=a.log[i];
        const int n=std::snprintf(b,sizeof(b),
            "%lu,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%u,%u,%.5f,"
            "%lu,%.5f,%lu,%lu,%lu,%u",
            static_cast<unsigned long>(d.t),d.x,d.z,d.speed,d.progress,
            d.reward,d.total,d.action,d.explored,d.q,
            static_cast<unsigned long>(d.updates),d.yaw,
            static_cast<unsigned long>(d.surface),
            static_cast<unsigned long>(d.wall),
            static_cast<unsigned long>(d.lap+1),d.validFace);
        if(n<=0||n>=static_cast<int>(sizeof(b))||
           f.WriteLine(b)!=File::SUCCESS)ok=false;
    }
    if(f.Flush()!=File::SUCCESS)ok=false;
    if(f.Close()!=File::SUCCESS)ok=false;
    if(ok)a.saved=true;
    return ok;
}
void IndSaveAttempts(MenuEntry *) {
    if(g_indPhase==IND_WAIT||g_indPhase==IND_DRIVE)return;
    unsigned saved=0,failed=0;
    for(unsigned i=0;i<g_indCount;++i) {
        if(g_indAttempts[i].saved)continue;
        if(IndSaveAttempt(g_indAttempts[i]))++saved;
        else ++failed;
    }
    char b[130];std::snprintf(b,sizeof(b),
        "Saved %u independent attempt CSVs; failed %u.",saved,failed);
    MessageBox("P008.1 flight-recorder save",b)();
}
void IndClear(MenuEntry *) {
    if(g_indPhase==IND_WAIT||g_indPhase==IND_DRIVE)return;
    for(unsigned i=0;i<g_indCount;++i)
        if(!g_indAttempts[i].saved) {
            MessageBox("Save first",
                "Save all independent attempts before clearing RAM.")();
            return;
        }
    g_indCount=0;g_indPhase=IND_IDLE;
    MessageBox("P008.1 RAM batch cleared","Learned policy was retained.")();
}
u32 IndChecksumStep(u32 hash,unsigned a,unsigned i,s32 scaled) {
    hash=(hash^static_cast<u32>(a+1))*16777619u;
    hash=(hash^static_cast<u32>(i+1))*16777619u;
    hash=(hash^static_cast<u32>(scaled))*16777619u;
    return hash;
}
// Versioned immutable numbered snapshots: an interrupted write cannot
// overwrite the last good saved policy. Only menu actions perform SD I/O.
void IndSaveModel(MenuEntry *) {
    if(g_indPhase==IND_WAIT||g_indPhase==IND_DRIVE)return;
    const std::string folder="/CollieKart";
    if(Directory::IsExists(folder)!=1 &&
       Directory::Create(folder)!=Directory::SUCCESS) {
        MessageBox("Model save failed","Cannot create /CollieKart.")();return;
    }
    char path[80];bool found=false;unsigned num=0;
    for(unsigned i=1;i<=999;++i) {
        std::snprintf(path,sizeof(path),"/CollieKart/ind_model_%03u.csv",i);
        if(File::Exists(path)==0){found=true;num=i;break;}
    }
    if(!found||File::Create(path)!=File::SUCCESS) {
        MessageBox("Model save failed","No free policy file.")();return;
    }
    File f;
    if(File::Open(f,path,File::WRITE|File::TRUNCATE)!=File::SUCCESS) {
        MessageBox("Model save failed","File open error.")();return;
    }
    char b[110];u32 hash=2166136261u;
    bool ok=f.WriteLine("COLLIE_P0082A_V1,9,256")==File::SUCCESS;
    for(unsigned a=0;a<IND_ACTIONS&&ok;++a) {
        for(unsigned i=0;i<IND_HASH&&ok;++i) {
            const s32 v=static_cast<s32>(
                std::round(g_indWeights[a][i]*100000.0f));
            hash=IndChecksumStep(hash,a,i,v);
            std::snprintf(b,sizeof(b),"%u,%u,%ld",a,i,
                static_cast<long>(v));
            ok=f.WriteLine(b)==File::SUCCESS;
        }
    }
    if(ok) {
        std::snprintf(b,sizeof(b),"CHECKSUM,%lu,%u,%u",
            static_cast<unsigned long>(hash),g_indPolicyVersion,g_indUpdates);
        ok=f.WriteLine(b)==File::SUCCESS;
    }
    if(f.Flush()!=File::SUCCESS)ok=false;
    if(f.Close()!=File::SUCCESS)ok=false;
    if(ok) {
        g_indModelDirty=false;g_indModelSnapshot=num;
        MessageBox("P008 model saved",path)();
    } else MessageBox("Model save INCOMPLETE",
        "Do not reload that snapshot. Earlier model files remain intact.")();
}
bool IndLoadModelFile(const std::string &path) {
    File f;
    if(File::Open(f,path,File::READ)!=File::SUCCESS)return false;
    const u64 sz=f.GetSize();
    if(sz<2000||sz>250000){f.Close();return false;}
    std::vector<char> data(static_cast<unsigned>(sz)+1,0);
    const bool read=f.Read(data.data(),static_cast<u32>(sz))==File::SUCCESS;
    f.Close();
    if(!read)return false;
    char *p=data.data(),*end=data.data()+static_cast<unsigned>(sz);
    char *nl=static_cast<char*>(std::memchr(p,'\n',end-p));
    if(!nl)return false;
    *nl=0;
    if(std::strcmp(p,"COLLIE_P0082A_V1,9,256")!=0)return false;
    p=nl+1;
    // Validate the entire snapshot into temporary storage before applying.
    static float candidate[IND_ACTIONS][IND_HASH];
    u32 hash=2166136261u;
    for(unsigned a=0;a<IND_ACTIONS;++a)for(unsigned i=0;i<IND_HASH;++i) {
        nl=static_cast<char*>(std::memchr(p,'\n',end-p));
        if(!nl||nl-p>100)return false;
        *nl=0;
        unsigned gotA=0,gotI=0;long scaled=0;
        if(std::sscanf(p,"%u,%u,%ld",&gotA,&gotI,&scaled)!=3||
           gotA!=a||gotI!=i||scaled < -2000001 || scaled>2000001)
            return false;
        candidate[a][i]=static_cast<float>(scaled)*.00001f;
        hash=IndChecksumStep(hash,a,i,static_cast<s32>(scaled));
        p=nl+1;
    }
    unsigned long savedHash=0;
    unsigned version=0,updates=0;
    if(std::sscanf(p,"CHECKSUM,%lu,%u,%u",
       &savedHash,&version,&updates)!=3 ||
       savedHash!=hash)return false;
    std::memcpy(g_indWeights,candidate,sizeof(g_indWeights));
    g_indPolicyVersion=version;g_indUpdates=updates;
    g_indModelDirty=false;
    return true;
}
void IndLoadModel(MenuEntry *) {
    if(g_indPhase==IND_WAIT||g_indPhase==IND_DRIVE)return;
    char path[80];unsigned loaded=0,skipped=0;
    // Newest VALID snapshot wins; corrupt partial later saves are skipped.
    for(unsigned i=999;i>=1;--i) {
        std::snprintf(path,sizeof(path),"/CollieKart/ind_model_%03u.csv",i);
        if(File::Exists(path)==1) {
            if(IndLoadModelFile(path)) {loaded=i;break;}
            ++skipped;
        }
        if(i==1)break;
    }
    if(!loaded) {
        MessageBox("No valid P008 model",
            "Policy unchanged. Missing or corrupt numbered snapshots.")();
        return;
    }
    g_indModelSnapshot=loaded;
    char b[160];std::snprintf(b,sizeof(b),
        "Loaded snapshot %03u. Policy v%u, %u TD updates."
        " Rejected corrupt/newer files: %u.",
        loaded,g_indPolicyVersion,g_indUpdates,skipped);
    MessageBox("P008 model restored",b)();
}
void IndResetModel(MenuEntry *) {
    if(g_indPhase==IND_WAIT||g_indPhase==IND_DRIVE||
       g_aiPhase==AI_RUNNING||g_aiPhase==AI_COUNTDOWN)return;
    std::memset(g_indWeights,0,sizeof(g_indWeights));
    g_indUpdates=0;++g_indPolicyVersion;
    g_indModelDirty=true;
    MessageBox("P008 policy reset in RAM",
        "Fresh learning will start next attempt. Existing SD snapshots"
        " remain available; load a snapshot to restore.")();
}
