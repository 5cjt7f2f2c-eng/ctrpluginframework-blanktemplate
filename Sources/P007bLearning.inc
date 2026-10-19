// P007b: lightweight on-device imitation policy + multi-attempt flight recorder.
// This file is included INSIDE main.cpp's anonymous namespace after DemoFrame,
// after Probe and after the P007a recorder routines. No entry points in a hook.
constexpr unsigned AI_MAX_ATTEMPTS = 6;
constexpr unsigned AI_MAX_TICKS = 2000; // ~2 minutes at demonstrated ~15Hz
constexpr u64 AI_TICK_MS = 65;
constexpr u64 AI_COMMAND_EXPIRY_MS = 180;
constexpr u64 AI_MENU_GRACE_MS = 5000;
constexpr u64 AI_NEUTRAL_MS = 350;
constexpr float AI_MAX_START_DIST = 65.0f;
constexpr float AI_MAX_ROUTE_ERROR = 95.0f;

enum AiPhase { AI_IDLE=0, AI_COUNTDOWN=1, AI_RUNNING=2, AI_ENDED=3 };
enum AiEndReason { AI_NONE=0, AI_ROUTE_DONE=1, AI_TIME=2,
    AI_OFF_PATH=3, AI_PHYSICAL=4, AI_INPUT_FAULT=5, AI_MEMORY=6,
    AI_MENU=7, AI_MASTER_OFF=8, AI_BUFFER=9, AI_BAD_START=10 };
const char *AiEndName(unsigned n)
{
    switch(n) {
        case AI_ROUTE_DONE: return "FINISHED ROUTE";
        case AI_TIME: return "TIME LIMIT";
        case AI_OFF_PATH: return "OFF TRACK REFERENCE";
        case AI_PHYSICAL: return "PHYSICAL OVERRIDE";
        case AI_INPUT_FAULT: return "NATIVE ACTUATOR FAULT";
        case AI_MEMORY: return "STALE OR INVALID OBSERVATION";
        case AI_MENU: return "CTRPF MENU OPENED";
        case AI_MASTER_OFF: return "READY DISABLED";
        case AI_BUFFER: return "ATTEMPT BUFFER FULL";
        case AI_BAD_START: return "INVALID START / UNPAUSED";
        default: return "NONE";
    }
}
float AiClamp(float v,float lo,float hi)
{
    return v<lo?lo:v>hi?hi:v;
}
float AiAngle(float v) { return std::atan2(std::sin(v), std::cos(v)); }

struct AiTrace
{
    u32 tMs=0, delivered=0;
    float x=0,z=0,speed=0,routeX=0,routeZ=0;
    float pathError=0,angleError=0,policySteer=0,cmdSteer=0;
    u16 waypoint=0,buttons=0;
    u8 stick=7;
};
struct AiAttempt
{
    AiTrace log[AI_MAX_TICKS];
    unsigned count=0, reason=AI_NONE, attemptNo=0;
    u32 durationMs=0, frames=0, nativeFaults=0;
    float gain=1.0f, meanError=0,maxError=0,progress=0,score=0;
    float policyWeights[4]={0,0,0,0};
    bool saved=false;
    std::string file;
};
AiAttempt g_aiAttempts[AI_MAX_ATTEMPTS];
unsigned g_aiCount=0;
AiPhase g_aiPhase=AI_IDLE;
u64 g_aiLastTick=0, g_aiStartAt=0, g_aiMenuClosedAt=0, g_aiNeutralAt=0;
u32 g_aiLimitMs=12000;
float g_aiWeights[4]={0,0,0,0};
float g_aiTrainingMSE=0, g_aiBestScore=-1000000.0f;
float g_aiBestGain=1.0f, g_aiNextGain=1.0f;
unsigned g_aiTrainingPoints=0, g_aiPolicyVersion=0;
bool g_aiTrained=false;
unsigned g_aiCursor=0, g_aiBadObservations=0;
float g_aiPrevX=0,g_aiPrevZ=0,g_aiHeading=0;
bool g_aiPrevValid=false;
u32 g_aiFrameStart=0, g_aiFaultStart=0;

bool AiLoadDemoFromCSV(const std::string &path)
{
    if (g_demoRecording || g_aiPhase==AI_COUNTDOWN ||
        g_aiPhase==AI_RUNNING || g_aiCount) return false;
    File f;
    if (File::Open(f,path,File::READ)!=File::SUCCESS) return false;
    const u64 size=f.GetSize();
    if (size<100 || size>1000000) { f.Close(); return false; }
    std::vector<char> buf(static_cast<unsigned>(size)+1,0);
    bool success=f.Read(buf.data(),static_cast<u32>(size))==File::SUCCESS;
    f.Close();
    if(!success) return false;
    buf[static_cast<unsigned>(size)]=0;
    unsigned n=0;
    char *p=buf.data(), *end=buf.data()+static_cast<unsigned>(size);
    char *newline=static_cast<char*>(std::memchr(p,'\n',end-p));
    if (!newline || std::strncmp(p,"time_ms,pos_x,pos_y,pos_z,speed,",32)!=0)
        return false;
    p=newline+1;
    while(p<end)
    {
        newline=static_cast<char*>(std::memchr(p,'\n',end-p));
        if(!newline) newline=end;
        if(newline==p) { p=newline+1; continue; }
        if(newline-p>220 || n>=MAX_DEMO_SAMPLES) {g_demoCount=0;return false;}
        const char saved=*newline; *newline=0;
        unsigned long t=0;
        unsigned buttons=0,stick=7,lap=0,cp=0;
        int head=0,reference=0;
        float x=0,y=0,z=0,speed=0,heading=0;
        int fields=std::sscanf(p,
            "%lu,%f,%f,%f,%f,%u,%u,%f,%d,%u,%u,%d",
            &t,&x,&y,&z,&speed,&buttons,&stick,&heading,&head,&lap,&cp,&reference);
        *newline=saved;
        if(fields!=12 || stick>14 || buttons>65535 ||
            lap<1 || lap>10 || cp>255 ||
            !std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||
            !std::isfinite(speed)||!std::isfinite(heading)||
            (n && t<=g_demoSamples[n-1].tMs))
        { g_demoCount=0; return false; }
        DemoFrame &d=g_demoSamples[n++];
        d.tMs=static_cast<u32>(t);
        d.x=x;d.y=y;d.z=z;d.speed=speed;
        d.buttons=static_cast<u16>(buttons);
        d.stickX=static_cast<u8>(stick);
        d.lap=static_cast<u8>(lap);d.checkpoint=static_cast<u8>(cp);
        d.movementHeading=heading;d.headingValid=head!=0;
        p=newline+1;
    }
    g_demoCount=n;
    if(n<20) { g_demoCount=0;return false; }
    // Rebuild the SAME statistics the live recorder accumulated.
    // A saved CSV imported in a later game process must not inherit
    // empty/stale counters or a zero path-distance normalizer.
    g_demoDistance=0;
    g_demoHeadingCount=g_demoAFrames=g_demoBFrames=0;
    g_demoLeftFrames=g_demoRightFrames=0;
    g_demoReadFails=g_demoReadFaultTotal=g_demoPauseWaits=0;
    g_demoStoppedOnFault=false;
    for(unsigned i=0;i<n;++i)
    {
        const DemoFrame &d=g_demoSamples[i];
        if(d.headingValid)++g_demoHeadingCount;
        if(d.buttons&1)++g_demoAFrames;
        if(d.buttons&2)++g_demoBFrames;
        if(d.stickX<6)++g_demoLeftFrames;
        if(d.stickX>8)++g_demoRightFrames;
        if(i)
        {
            const float dx=d.x-g_demoSamples[i-1].x;
            const float dz=d.z-g_demoSamples[i-1].z;
            const float sq=dx*dx+dz*dz;
            if(!std::isfinite(sq)||sq>250000.0f)
            {g_demoCount=0;return false;}
            if(sq>=0.04f)g_demoDistance+=std::sqrt(sq);
        }
    }
    g_demoRecording=false;g_demoComplete=true;
    AnalyzeDemoBuffer();
    g_demoLastFile=path;
    g_aiTrained=false;
    return true;
}
void AiLoadNewestDemo(MenuEntry *)
{
    if(g_aiCount)
    {
        MessageBox("P007b already has attempts",
            "Save and CLEAR prior attempts before changing the demonstration.")();
        return;
    }
    char name[64]; std::string path;
    for(unsigned i=1;i<=999;++i)
    {
        std::snprintf(name,sizeof(name),"/CollieKart/demo_%03u.csv",i);
        if(File::Exists(name)==1) path=name;
    }
    if(path.empty() || !AiLoadDemoFromCSV(path))
    {
        MessageBox("P007b import failed",
            "No compatible demo CSV found. Use your P007a.1 file in"
            " SD:/CollieKart/demo_###.csv or record a fresh manual demo.")();
        return;
    }
    MessageBox("P007b demonstration loaded",
        "CSV parsed on the 3DS. Select TRAIN MODEL next.")();
}
void AiTrain(MenuEntry *)
{
    if (g_aiPhase==AI_RUNNING || g_aiPhase==AI_COUNTDOWN ||
        g_aiCount || g_demoRecording)
    {
        MessageBox("Training locked",
            "Stop recording / finish and clear attempts before retraining.")();
        return;
    }
    if(!g_demoComplete || g_demoCount<35)
    {
        MessageBox("Training needs demonstration",
            "Load an existing demo CSV or record 35+ valid samples.")();
        return;
    }
    if(!g_demoAnalyzed) AnalyzeDemoBuffer();
    float w[4]={0,0,0,0};
    unsigned used=0;
    // A tiny supervised regression: learn the signed analog stick from
    // local speed and the upcoming target's angular displacement.
    for(unsigned epoch=0;epoch<75;++epoch)
    {
        for(unsigned i=0;i+7<g_demoCount;++i)
        {
            const DemoFrame &d=g_demoSamples[i];
            const DemoFrame &look=g_demoSamples[i+7];
            if(!d.headingValid || std::fabs(d.speed)<0.2f) continue;
            const float dx=look.x-d.x,dz=look.z-d.z;
            if(dx*dx+dz*dz<1.0f) continue;
            const float angle=AiClamp(AiAngle(std::atan2(dz,dx)-
                              d.movementHeading),-1.5f,1.5f);
            const float velocity=AiClamp(std::fabs(d.speed)/10.0f,0,1);
            const float f[4]={1.0f,angle,velocity,angle*velocity};
            const float actual=(static_cast<int>(d.stickX)-7)/7.0f;
            const float pred=w[0]*f[0]+w[1]*f[1]+w[2]*f[2]+w[3]*f[3];
            const float err=AiClamp(actual-pred,-2,2);
            const float lr=(d.stickX==7?0.018f:0.030f);
            for(int k=0;k<4;++k) w[k]=AiClamp(w[k]+
                lr*(err*f[k]-0.0004f*w[k]),-3,3);
        }
    }
    float sq=0;
    for(unsigned i=0;i+7<g_demoCount;++i)
    {
        const DemoFrame &d=g_demoSamples[i];
        const DemoFrame &look=g_demoSamples[i+7];
        if(!d.headingValid || std::fabs(d.speed)<0.2f) continue;
        const float dx=look.x-d.x,dz=look.z-d.z;
        if(dx*dx+dz*dz<1.0f) continue;
        const float a=AiClamp(AiAngle(std::atan2(dz,dx)-
                      d.movementHeading),-1.5f,1.5f);
        const float v=AiClamp(std::fabs(d.speed)/10.0f,0,1);
        const float predicted=w[0]+w[1]*a+w[2]*v+w[3]*a*v;
        const float expected=(static_cast<int>(d.stickX)-7)/7.0f;
        sq+=(predicted-expected)*(predicted-expected);++used;
    }
    if(used<12)
    {
        g_aiTrained=false;
        MessageBox("P007b insufficient examples",
            "Need at least 12 usable moving/heading examples.")();
        return;
    }
    for(int k=0;k<4;++k)g_aiWeights[k]=w[k];
    g_aiTrainingMSE=sq/used;g_aiTrainingPoints=used;
    g_aiTrained=true;++g_aiPolicyVersion;
    g_aiBestScore=-1000000.0f;
    g_aiBestGain=g_aiNextGain=1.0f;
    char msg[300];
    std::snprintf(msg,sizeof(msg),
        "Learnt from %u samples, MSE %.4f.\n"
        "Steering weights: %.3f %.3f %.3f %.3f\n"
        "Policy v%u. Uses path feedback while driving.",
        used,g_aiTrainingMSE,w[0],w[1],w[2],w[3],g_aiPolicyVersion);
    MessageBox("P007b on-device training complete",msg)();
}
void AiStop(unsigned reason)
{
    // Must disable native input FIRST, before building reports.
    g_aiAuthority.store(false,std::memory_order_release);
    g_aiDeadlineMs.store(0,std::memory_order_release);
    if(g_aiPhase!=AI_COUNTDOWN && g_aiPhase!=AI_RUNNING)
        return;
    if(g_aiPhase==AI_RUNNING && g_aiCount<AI_MAX_ATTEMPTS)
    {
        AiAttempt &a=g_aiAttempts[g_aiCount];
        a.reason=reason;
        a.frames=g_aiActuatorFrames.load()-g_aiFrameStart;
        a.nativeFaults=g_aiActuatorFaults.load()-g_aiFaultStart;
        a.durationMs=a.count?a.log[a.count-1].tMs:0;
        float sum=0; a.maxError=0;
        for(unsigned i=0;i<a.count;++i)
        {
            sum+=a.log[i].pathError;
            if(a.log[i].pathError>a.maxError)
                a.maxError=a.log[i].pathError;
        }
        a.meanError=a.count?sum/a.count:0;
        a.progress=g_demoWaypointsCount>1?
           static_cast<float>(g_aiCursor)/(g_demoWaypointsCount-1):0;
        if(a.progress>1)a.progress=1;
        const float scale=g_demoDistance/100.0f+1.0f;
        a.score=100*a.progress-8*a.meanError/scale-
           (reason==AI_ROUTE_DONE?0:10);
        if(a.count>0)
        {
            ++g_aiCount;
            // Empirical parameter learning: retain the best tested gain,
            // propose a nearby variation next run, never claim an improvement
            // until another comparable trial actually scores better.
            if(a.score>g_aiBestScore)
            {
                g_aiBestScore=a.score;
                g_aiBestGain=a.gain;
            }
            const float step=(g_aiCount%4<2?0.14f:-0.14f)*
                 (1.0f+static_cast<float>(g_aiCount/4)*0.5f);
            g_aiNextGain=AiClamp(g_aiBestGain+step,0.65f,1.8f);
        }
    }
    g_aiPhase=AI_ENDED;
}
void AiQueueAttempt(MenuEntry *)
{
    if(!g_aiTrained || g_demoCount<35 || g_demoWaypointsCount<3 ||
       !g_ready || g_demoRecording || g_recording ||
       g_aiPhase==AI_COUNTDOWN || g_aiPhase==AI_RUNNING ||
       g_aiCount>=AI_MAX_ATTEMPTS)
    {
        MessageBox("AI attempt locked",
          "Need READY, trained demo, idle recorders and a free attempt slot.")();
        return;
    }
    if(!g_nativeHookEnabled.load() || g_nativeSeenFrames<30 ||
       g_nativeManualAFrames<10 || g_nativeReadFailures!=0 ||
       g_nativeWriteFailures!=0 || g_aiActuatorFaults!=0 ||
       !g_dryRunComplete || !g_physicalAbortVerified)
    {
        MessageBox("AI actuator not yet certified",
            "Install PASS-THROUGH hook, manually test A, complete the"
            " five-second safety dry run, separately abort with physical B,"
            " and require zero native read/write faults.")();
        return;
    }
    Probe();
    float speed=0;
    if(!g_observation.candidate || !g_demoKartAddress ||
       !g_vehicleAddress ||
       !ReadNumber(g_vehicleAddress+0xF2C,speed) ||
       std::fabs(speed)>0.35f)
    {
        MessageBox("Invalid AI starting position",
            "Park stationary on the ground near the recorded start.")();
        return;
    }
    const float dx=g_observation.x-g_demoSamples[0].x;
    const float dz=g_observation.z-g_demoSamples[0].z;
    if(dx*dx+dz*dz>AI_MAX_START_DIST*AI_MAX_START_DIST)
    {
        MessageBox("Too far from demonstration start",
            "Bring kart within about 65 game-coordinate units of"
            " the original starting point. No controls changed.")();
        return;
    }
    g_aiMenuClosedAt=g_aiNeutralAt=0;
    g_aiStartAt=g_aiLastTick=0;
    g_aiPrevValid=false;g_aiCursor=0;g_aiBadObservations=0;
    g_aiAuthority.store(false);
    g_aiPhysicalOverride.store(0);
    g_aiPhase=AI_COUNTDOWN;
    MessageBox("AI attempt queued",
        "Offline Toad Circuit Time Trial ONLY. Close CTRPF and resume"
        " game early in FIVE-SECOND countdown. Release A/B/START"
        " and center Circle Pad for final 350ms."
        " Any physical A/B/START/stick during driving aborts AI."
        " If controls feel stuck, exit game and disable Plugin Loader.")();
}
void AiCycleLimit(MenuEntry *)
{
    if(g_aiPhase==AI_RUNNING||g_aiPhase==AI_COUNTDOWN) return;
    if(g_aiLimitMs==12000)g_aiLimitMs=30000;
    else if(g_aiLimitMs==30000)g_aiLimitMs=75000;
    else if(g_aiLimitMs==75000)g_aiLimitMs=120000;
    else g_aiLimitMs=12000;
    char msg[110];
    std::snprintf(msg,sizeof(msg),
       "Next attempt max time: %lu seconds. Always supervision required.",
       static_cast<unsigned long>(g_aiLimitMs/1000));
    MessageBox("P007b attempt horizon",msg)();
}
void AiStep(u64 now,bool menuOpen)
{
    if(g_aiPhase!=AI_COUNTDOWN && g_aiPhase!=AI_RUNNING) return;
    if(menuOpen)
    {
        if(g_aiPhase==AI_RUNNING)AiStop(AI_MENU);
        return; // keep countdown while initial CTRPF menu closes
    }
    if(!g_ready) {AiStop(AI_MASTER_OFF);return;}
    if(g_aiPhase==AI_COUNTDOWN)
    {
        if(!g_aiMenuClosedAt) g_aiMenuClosedAt=now;
        if(now<g_aiMenuClosedAt) {AiStop(AI_BAD_START);return;}
        if(Controller::IsKeyDown(Key::B)) {AiStop(AI_PHYSICAL);return;}
        const u64 elapsed=now-g_aiMenuClosedAt;
        if(elapsed<AI_MENU_GRACE_MS-AI_NEUTRAL_MS) return;
        const u32 blocked=static_cast<u32>(Key::A) |
             static_cast<u32>(Key::B)|static_cast<u32>(Key::Start)|
             static_cast<u32>(Key::CPadLeft)|static_cast<u32>(Key::CPadRight);
        if(Controller::GetKeysDown(false)&blocked) {g_aiNeutralAt=0;return;}
        if(!g_aiNeutralAt)g_aiNeutralAt=now;
        if(elapsed<AI_MENU_GRACE_MS ||
           now-g_aiNeutralAt<AI_NEUTRAL_MS) return;
        Probe();
        float speed=0;
        u32 pad=0;u8 paused=1;
        if(!g_observation.candidate || !g_vehicleAddress ||
           !ReadNumber(g_vehicleAddress+0xF2C,speed) ||
           !ReadPointer(g_vehicleAddress+0xE0,pad) ||
           !Process::Read8(pad+0x3D,paused) || paused ||
           std::fabs(speed)>0.5f) {AiStop(AI_BAD_START);return;}
        g_nativePlayerPad.store(pad);
        g_aiPhase=AI_RUNNING;
        AiAttempt &a=g_aiAttempts[g_aiCount];
        a.count=0;a.reason=AI_NONE;a.saved=false;a.file.clear();
        a.attemptNo=g_aiCount+1;a.gain=g_aiNextGain;
        for(int k=0;k<4;++k)a.policyWeights[k]=g_aiWeights[k];
        g_aiPrevValid=false;g_aiStartAt=now;g_aiLastTick=0;
        g_aiFrameStart=g_aiActuatorFrames.load();
        g_aiFaultStart=g_aiActuatorFaults.load();
        g_aiPassRelease.store(0);
        return;
    }
    if(g_aiPhysicalOverride.load()) {AiStop(AI_PHYSICAL);return;}
    if(g_aiActuatorFaults.load()!=g_aiFaultStart ||
       g_nativeReadFailures.load()!=0)
    {AiStop(AI_INPUT_FAULT);return;}
    if(now<g_aiStartAt || now-g_aiStartAt>g_aiLimitMs)
    {AiStop(AI_TIME);return;}
    if(g_aiLastTick && now>=g_aiLastTick &&
       now-g_aiLastTick<AI_TICK_MS) return;
    if(g_aiLastTick && (now<g_aiLastTick || now-g_aiLastTick>400))
    {AiStop(AI_MEMORY);return;}
    g_aiLastTick=now;
    if(g_aiCount>=AI_MAX_ATTEMPTS) {AiStop(AI_BUFFER);return;}
    AiAttempt &a=g_aiAttempts[g_aiCount];
    if(a.count>=AI_MAX_TICKS) {AiStop(AI_BUFFER);return;}
    if(!g_demoKartAddress || !g_vehicleAddress ||
       !g_observation.candidate) {AiStop(AI_MEMORY);return;}
    float x=0,z=0,speed=0;
    u32 pad=0;u8 paused=1;
    if(!ReadNumber(g_demoKartAddress+0x28,x)||
       !ReadNumber(g_demoKartAddress+0x30,z)||
       !ReadNumber(g_vehicleAddress+0xF2C,speed)||
       !ReadPointer(g_vehicleAddress+0xE0,pad)||
       !Process::Read8(pad+0x3D,paused)||paused)
    {AiStop(AI_MEMORY);return;}
    float travel=g_aiHeading;
    if(g_aiPrevValid)
    {
        const float dx=x-g_aiPrevX,dz=z-g_aiPrevZ;
        if(dx*dx+dz*dz>0.04f) travel=std::atan2(dz,dx);
    }
    else
    {
        // Stationary start: reference motion bearing until the first
        // actual position displacement is measurable.
        for(unsigned k=0;k<g_demoCount;++k)
        if(g_demoSamples[k].headingValid)
        {travel=g_demoSamples[k].movementHeading;break;}
    }
    g_aiHeading=travel;g_aiPrevX=x;g_aiPrevZ=z;g_aiPrevValid=true;
    float best=1e15f;unsigned nearest=g_aiCursor;
    const unsigned n=g_demoWaypointsCount;
    if(n<3){AiStop(AI_MEMORY);return;}
    // Match MONOTONIC forward segments, not arbitrary closest points.
    const unsigned end=g_aiCursor+15<n-1?g_aiCursor+15:n-2;
    for(unsigned i=g_aiCursor;i<=end;++i)
    {
        const DemoWaypoint &p=g_demoWaypoints[i];
        const DemoWaypoint &q=g_demoWaypoints[i+1];
        const float vx=q.x-p.x,vz=q.z-p.z, len=vx*vx+vz*vz;
        if(len<1e-6f)continue;
        const float t=AiClamp(((x-p.x)*vx+(z-p.z)*vz)/len,0,1);
        const float ox=x-(p.x+vx*t),oz=z-(p.z+vz*t);
        const float err=ox*ox+oz*oz;
        if(err<best){best=err;nearest=i;}
    }
    if(!std::isfinite(best) || best>AI_MAX_ROUTE_ERROR*AI_MAX_ROUTE_ERROR)
    {AiStop(AI_OFF_PATH);return;}
    g_aiCursor=nearest;
    const float distance=std::sqrt(best);
    const unsigned look=nearest+3<n?nearest+3:n-1;
    const DemoWaypoint &target=g_demoWaypoints[look];
    const float tx=target.x-x,tz=target.z-z;
    const float aerr=AiClamp(AiAngle(std::atan2(tz,tx)-travel),-1.5f,1.5f);
    const float v=AiClamp(std::fabs(speed)/10.0f,0,1);
    float policy=g_aiWeights[0]+g_aiWeights[1]*aerr+
                 g_aiWeights[2]*v+g_aiWeights[3]*aerr*v;
    policy=AiClamp(policy,-0.9f,0.9f);
    // The learned gain is empirically optimized between attempts.
    const float steer=AiClamp(policy+
         (g_aiAttempts[g_aiCount].gain-1.0f)*aerr,-0.85f,0.85f);
    const int stick=static_cast<int>(std::round(steer*7.0f))+7;
    const u8 targetStick=static_cast<u8>(AiClamp(stick,1,13));
    const float desired=target.speed;
    const u16 buttons=speed>desired+0.9f?0x2:0x1;
    AiTrace &t=a.log[a.count++];
    t.tMs=static_cast<u32>(now-g_aiStartAt);
    t.x=x;t.z=z;t.speed=speed;
    t.routeX=target.x;t.routeZ=target.z;
    t.pathError=distance;t.angleError=aerr;
    t.policySteer=policy;t.cmdSteer=steer;
    t.waypoint=static_cast<u16>(nearest);
    t.buttons=buttons;t.stick=targetStick;
    t.delivered=g_aiActuatorFrames.load()-g_aiFrameStart;
    // Atomic release order: publish command and deadline before enabling.
    g_aiCommand.store((static_cast<u32>(targetStick)<<16)|buttons,
        std::memory_order_release);
    g_aiDeadlineMs.store(static_cast<u32>(now+AI_COMMAND_EXPIRY_MS),
        std::memory_order_release);
    g_aiAuthority.store(true,std::memory_order_release);
    const float endpointDx=x-g_demoSamples[g_demoCount-1].x;
    const float endpointDz=z-g_demoSamples[g_demoCount-1].z;
    if(nearest>=n-2 &&
       endpointDx*endpointDx+endpointDz*endpointDz<35.0f*35.0f)
       AiStop(AI_ROUTE_DONE);
}
void AiAbortMenu(MenuEntry *)
{
    AiStop(AI_MENU);
    MessageBox("P007b AI stopped",
       "Synthetic input authority has been revoked.")();
}
void AiAttemptReport(MenuEntry *)
{
    char msg[780];
    std::snprintf(msg,sizeof(msg),
        "Trained: %s | model v%u (%u examples)\n"
        "MSE: %.4f | Best gain: %.2f\n"
        "Attempts stored: %u/%u | next gain: %.2f\n"
        "Current: %s | max runtime: %lus\n"
        "Native synthetic frames: %lu | faults: %lu\n"
        "Manual override: %s | release: %lu\n"
        "Latest trial metrics:\n",
        g_aiTrained?"YES":"NO",g_aiPolicyVersion,g_aiTrainingPoints,
        g_aiTrainingMSE,g_aiBestGain,g_aiCount,AI_MAX_ATTEMPTS,
        g_aiNextGain,g_aiPhase==AI_RUNNING?"DRIVING":
        g_aiPhase==AI_COUNTDOWN?"COUNTDOWN":"IDLE",
        static_cast<unsigned long>(g_aiLimitMs/1000),
        static_cast<unsigned long>(g_aiActuatorFrames.load()),
        static_cast<unsigned long>(g_aiActuatorFaults.load()),
        g_aiPhysicalOverride.load()?"YES":"NO",
        static_cast<unsigned long>(g_aiPassRelease.load()));
    std::string result=msg;
    for(unsigned i=0;i<g_aiCount;++i)
    {
        const AiAttempt &a=g_aiAttempts[i];
        char row[200];
        std::snprintf(row,sizeof(row),
            "\n#%u %s | %.0f%% route | avg/max %.1f/%.1f"
            "\nscore %.1f gain %.2f | %u ticks / %lu writes | %s",
            a.attemptNo,AiEndName(a.reason),a.progress*100,
            a.meanError,a.maxError,a.score,a.gain,a.count,
            static_cast<unsigned long>(a.frames),
            a.saved?"SAVED":"UNSAVED");
        result+=row;
    }
    if(!g_aiCount)result+="\nNo AI attempts completed yet.";
    MessageBox("P007b Multi-Attempt Learning Report",result)();
}
bool AiSaveAttempt(AiAttempt &a)
{
    const std::string folder="/CollieKart";
    if(Directory::IsExists(folder)!=1 &&
       Directory::Create(folder)!=Directory::SUCCESS) return false;
    std::string path;char name[60];
    for(unsigned i=1;i<=999;++i)
    {
        std::snprintf(name,sizeof(name),
            "/CollieKart/ai_attempt_%03u.csv",i);
        if(File::Exists(name)==0){path=name;break;}
    }
    if(path.empty() || File::Create(path)!=File::SUCCESS)return false;
    File f;
    if(File::Open(f,path,File::WRITE|File::TRUNCATE)!=File::SUCCESS)
        return false;
    char row[340];
    std::snprintf(row,sizeof(row),
        "attempt=%u,reason=%s,gain=%.4f,score=%.3f,"
        "completion=%.3f,mean_error=%.3f,max_error=%.3f,"
        "frames=%lu,faults=%lu,policy_v=%u,model_mse=%.5f,"
        "weights=%.4f;%.4f;%.4f;%.4f",
        a.attemptNo,AiEndName(a.reason),a.gain,a.score,
        a.progress,a.meanError,a.maxError,
        static_cast<unsigned long>(a.frames),
        static_cast<unsigned long>(a.nativeFaults),g_aiPolicyVersion,
        g_aiTrainingMSE,a.policyWeights[0],a.policyWeights[1],
        a.policyWeights[2],a.policyWeights[3]);
    bool ok=f.WriteLine(row)==File::SUCCESS;
    if(ok)ok=f.WriteLine(
        "time_ms,pos_x,pos_z,speed,waypoint,target_x,target_z,"
        "path_error,heading_error,model_steer,final_steer,"
        "buttons,stick_x,cumulative_native_frames")==File::SUCCESS;
    for(unsigned i=0;i<a.count&&ok;++i)
    {
        const AiTrace &t=a.log[i];
        const int n=std::snprintf(row,sizeof(row),
            "%lu,%.4f,%.4f,%.4f,%u,%.4f,%.4f,%.4f,%.6f,"
            "%.5f,%.5f,%u,%u,%lu",
            static_cast<unsigned long>(t.tMs),t.x,t.z,t.speed,
            t.waypoint,t.routeX,t.routeZ,t.pathError,t.angleError,
            t.policySteer,t.cmdSteer,static_cast<unsigned>(t.buttons),
            static_cast<unsigned>(t.stick),
            static_cast<unsigned long>(t.delivered));
        if(n<=0||n>=static_cast<int>(sizeof(row))||
           f.WriteLine(row)!=File::SUCCESS)ok=false;
    }
    if(f.Flush()!=File::SUCCESS)ok=false;
    if(f.Close()!=File::SUCCESS)ok=false;
    if(ok){a.saved=true;a.file=path;}
    return ok;
}
void AiSaveAll(MenuEntry *)
{
    if(g_aiPhase==AI_RUNNING || g_aiPhase==AI_COUNTDOWN)
    {MessageBox("Save locked","Stop the AI before writing SD files.")();return;}
    unsigned saved=0,failed=0;
    for(unsigned i=0;i<g_aiCount;++i)
    {
        if(g_aiAttempts[i].saved)continue;
        if(AiSaveAttempt(g_aiAttempts[i]))++saved;
        else ++failed;
    }
    char msg[155];std::snprintf(msg,sizeof(msg),
        "Saved %u attempts; failed %u.\n"
        "Each has its own SD:/CollieKart/ai_attempt_###.csv.\n"
        "RAM remains until explicit CLEAR.",saved,failed);
    MessageBox("P007b attempt save report",msg)();
}
void AiClearSaved(MenuEntry *)
{
    if(g_aiPhase==AI_COUNTDOWN||g_aiPhase==AI_RUNNING) return;
    for(unsigned i=0;i<g_aiCount;++i)
        if(!g_aiAttempts[i].saved)
        {
            MessageBox("Unsaved attempt exists",
                "Use SAVE ALL ATTEMPTS before clearing; nothing deleted.")();
            return;
        }
    g_aiCount=0;
    g_aiPhase=AI_IDLE;
    MessageBox("New attempt batch ready",
       "Prior CSVs remain on SD. Trained policy and best gain retained.")();
}
