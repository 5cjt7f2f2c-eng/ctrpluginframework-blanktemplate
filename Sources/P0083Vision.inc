// P008.3d: geometry-backed vision mathematics (no unknown MK7 calls).
// Backend-neutral: triangles must be supplied by a separately validated KCL decoder.
// This file intentionally DOES NOT treat fabricated geometry as live game vision.
struct CKVisionPoint { float x,y,z; };
struct CKVisionTriangle {
    CKVisionPoint a,b,c;
    u16 material;
};
struct CKVisionHit {
    bool valid=false;
    float height=0;
    u16 material=0;
    unsigned triangle=0;
};
static bool CKVisionFinite(float v) {
    return std::isfinite(v) && std::fabs(v)<1000000.0f;
}
static bool CKVisionIntersectXZ(const CKVisionTriangle &t,float x,float z,
                                float referenceY,CKVisionHit &out,unsigned index) {
    const float ax=t.a.x,az=t.a.z,bx=t.b.x,bz=t.b.z,cx=t.c.x,cz=t.c.z;
    if(!CKVisionFinite(x)||!CKVisionFinite(z)||!CKVisionFinite(referenceY)||
       !CKVisionFinite(ax)||!CKVisionFinite(az)||
       !CKVisionFinite(bx)||!CKVisionFinite(bz)||
       !CKVisionFinite(cx)||!CKVisionFinite(cz)||
       !CKVisionFinite(t.a.y)||!CKVisionFinite(t.b.y)||
       !CKVisionFinite(t.c.y))return false;
    const float den=(bz-cz)*(ax-cx)+(cx-bx)*(az-cz);
    if(!std::isfinite(den)||std::fabs(den)<0.000001f)return false;
    const float u=((bz-cz)*(x-cx)+(cx-bx)*(z-cz))/den;
    const float v=((cz-az)*(x-cx)+(ax-cx)*(z-cz))/den;
    const float w=1.0f-u-v;
    if(u<-.0001f||v<-.0001f||w<-.0001f)return false;
    const float y=u*t.a.y+v*t.b.y+w*t.c.y;
    if(!CKVisionFinite(y))return false;
    // Prefer the closest plausible ground below the kart; never select a bridge above it.
    if(y>referenceY+0.5f||referenceY-y>30.0f)return false;
    if(!out.valid||y>out.height) {
        out.valid=true;out.height=y;out.material=t.material;out.triangle=index;
    }
    return true;
}
static CKVisionHit CKVisionGround(const CKVisionTriangle *tri,unsigned count,
                                 float x,float z,float referenceY) {
    CKVisionHit h;
    if(!tri||count>8192)return h;
    for(unsigned i=0;i<count;++i)
        CKVisionIntersectXZ(tri[i],x,z,referenceY,h,i);
    return h;
}
struct CKVisionScan {
    CKVisionHit hit[3][3]; // distance near/mid/far; angle left/center/right
    unsigned validCount=0;
};
static CKVisionScan CKVisionLook(const CKVisionTriangle *tri,unsigned count,
    float x,float z,float y,float faceX,float faceZ) {
    CKVisionScan scan;
    const float norm=std::sqrt(faceX*faceX+faceZ*faceZ);
    if(!CKVisionFinite(norm)||norm<0.5f||norm>1.5f)return scan;
    const float fx=faceX/norm,fz=faceZ/norm;
    const float distances[3]={3.0f,8.0f,16.0f};
    const float lateral[3]={-0.55f,0.0f,0.55f};
    for(unsigned d=0;d<3;++d)for(unsigned a=0;a<3;++a) {
        const float px=x+distances[d]*(fx-lateral[a]*fz);
        const float pz=z+distances[d]*(fz+lateral[a]*fx);
        scan.hit[d][a]=CKVisionGround(tri,count,px,pz,y);
        if(scan.hit[d][a].valid)++scan.validCount;
    }
    return scan;
}
void IndVisionGeometrySelfTest(MenuEntry *) {
    // Two adjacent triangles forming a flat 20x20 road, plus elevated bridge.
    // Tests mathematical behavior ONLY, not live course geometry access.
    const CKVisionTriangle t[3]={
        {{-10,0,-10},{10,0,-10},{-10,0,10},0},
        {{10,0,-10},{10,0,10},{-10,0,10},0},
        {{-10,8,-10},{10,8,-10},{-10,8,10},4}
    };
    const CKVisionHit inside=CKVisionGround(t,3,0,0,1);
    const CKVisionHit outside=CKVisionGround(t,3,30,0,1);
    const CKVisionHit bridge=CKVisionGround(t,3,-5,-5,9);
    const CKVisionScan scan=CKVisionLook(t,3,0,0,1,0,1);
    const bool a=inside.valid&&inside.material==0&&std::fabs(inside.height)<.01f;
    const bool b=!outside.valid;
    const bool c=bridge.valid&&bridge.material==4&&std::fabs(bridge.height-8)<.01f;
    const bool d=scan.validCount>0&&scan.validCount<9;
    char msg[340];
    std::snprintf(msg,sizeof(msg),
        "P008.3 GEOMETRY MATH (SYNTHETIC ONLY)\\n"
        "Flat road: %s\\nOutside road: %s\\n"
        "Bridge height: %s\\n3x3 scan: %s (%u/9 hits)\\n"
        "LIVE VISION: NOT CONNECTED.\\n"
        "Requires validated course KCL and kart Y.",
        a?"PASS":"FAIL",b?"PASS":"FAIL",c?"PASS":"FAIL",
        d?"PASS":"FAIL",scan.validCount);
    MessageBox("P008.3 geometry self-test",msg)();
}


// P008.3b: user-provided Toad Circuit geometry asset, NOT live KCL memory.
// Fixed versioned format: CKV3, u32 version,count,crc32; records: 9*f32,u32.
static unsigned g_ckCourseCount=0;
static bool g_ckCourseReady=false;
static u32 CKVisionCrc(const unsigned char *p,unsigned n) {
    u32 crc=0xFFFFFFFFu;
    for(unsigned i=0;i<n;++i) {
        crc^=p[i];
        for(unsigned j=0;j<8;++j)
            crc=(crc>>1) ^ ((crc&1)?0xEDB88320u:0u);
    }
    return ~crc;
}
static u32 CKVisionU32(const unsigned char *p) {
    return static_cast<u32>(p[0]) | (static_cast<u32>(p[1])<<8) |
           (static_cast<u32>(p[2])<<16) | (static_cast<u32>(p[3])<<24);
}
static float CKVisionF32(const unsigned char *p) {
    const u32 bits=CKVisionU32(p);
    float f=0;std::memcpy(&f,&bits,sizeof(f));return f;
}
void IndVisionLoadCourse(MenuEntry *) {
    if(g_indPhase==IND_WAIT||g_indPhase==IND_DRIVE||
       g_aiPhase==AI_RUNNING||g_aiPhase==AI_COUNTDOWN) {
        MessageBox("Vision loader locked","Stop all automated driving first.")();return;
    }
    g_ckCourseReady=false;g_ckCourseCount=0;
    const char *path="/CollieKart/ToadCircuit_CKV3.bin";
    File f;
    if(File::Open(f,path,File::READ)!=File::SUCCESS) {
        MessageBox("Missing Toad Circuit data",
          "Copy ToadCircuit_CKV3.bin into /CollieKart/ on the SD card.")();return;
    }
    const u64 size=f.GetSize();
    if(size<56 || size>400016 || (size-16)%40!=0) {
        f.Close();MessageBox("Invalid vision file","File size invalid.")();return;
    }
    // P008.3c: no duplicate full-file allocation. Stream in bounded batches.
    // A 3DS plugin has limited heap headroom; catch allocation failures rather
    // than allowing std::terminate -> abort() / svcBreak.
    unsigned char header[16]={};
    if(f.Read(header,16)!=File::SUCCESS) {
        f.Close();MessageBox("Vision file read failed","Cannot read header.")();return;
    }
    const u32 count=CKVisionU32(header+8);
    if(std::memcmp(header,"CKV3",4)!=0||
       CKVisionU32(header+4)!=1 || count==0 || count>8192 ||
       size!=16u+static_cast<u64>(count)*40u) {
        f.Close();MessageBox("Invalid vision file","Header or triangle count invalid.")();return;
    }
    u32 crc=0xFFFFFFFFu;
    unsigned char batch[40*32];
    u32 index=0;
    bool good=true;
    while(index<count && good) {
        const u32 n=(count-index)<32?(count-index):32;
        const u32 len=n*40;
        if(f.Read(batch,len)!=File::SUCCESS) {good=false;break;}
        for(u32 k=0;k<len;++k) {
            crc^=batch[k];
            for(unsigned j=0;j<8;++j)
                crc=(crc>>1)^((crc&1u)?0xEDB88320u:0u);
        }
        index+=n;
    }
    f.Close();
    if(!good || ~crc!=CKVisionU32(header+12)) {
        MessageBox("Vision geometry rejected",
            "Read error, invalid coordinates or checksum mismatch.")();
        return;
    }
    g_ckCourseCount=count;
    g_ckCourseReady=true;
    char b[160];std::snprintf(b,sizeof(b),
       "Verified %lu triangles on SD. CRC verified.\n"
       "Experimental MK7 geometry; surface labels not validated.",
       static_cast<unsigned long>(g_ckCourseCount));
    MessageBox("P008.3b geometry loaded",b)();
}
void IndVisionLiveSnapshot(MenuEntry *) {
    if(g_indPhase==IND_WAIT||g_indPhase==IND_DRIVE||
       g_aiPhase==AI_RUNNING||g_aiPhase==AI_COUNTDOWN) {
        MessageBox("Vision locked","Stop all automated driving first.")();return;
    }
    if(!g_ckCourseReady || !g_observation.candidate ||
       !g_demoKartAddress || !g_vehicleAddress) {
        MessageBox("Vision unavailable","Load ToadCircuit_CKV3.bin and enter active Time Trial.")();
        return;
    }
    float x=0,y=0,z=0,fx=0,fz=0;
    if(!ReadNumber(g_demoKartAddress+0x28,x) ||
       !ReadNumber(g_demoKartAddress+0x2C,y) ||
       !ReadNumber(g_demoKartAddress+0x30,z) ||
       !ReadNumber(g_vehicleAddress+0xF44,fx) ||
       !ReadNumber(g_vehicleAddress+0xF4C,fz) ||
       !CKVisionFinite(x)||!CKVisionFinite(y)||!CKVisionFinite(z)) {
        MessageBox("Vision unavailable","Live position/heading invalid.")();return;
    }
    // P008.3d: stream geometry directly from SD, bounded 1.25KB stack buffer.
    // Nine queries share one file traversal; no course-sized heap allocation.
    CKVisionScan scan;
    const float norm=std::sqrt(fx*fx+fz*fz);
    if(!CKVisionFinite(norm)||norm<0.5f||norm>1.5f) {
        MessageBox("Vision unavailable","Heading invalid.")();return;
    }
    const float ux=fx/norm,uz=fz/norm;
    const float distances[3]={3.0f,8.0f,16.0f};
    const float lateral[3]={-0.55f,0.0f,0.55f};
    float px[3][3],pz[3][3];
    for(unsigned d=0;d<3;++d)for(unsigned a=0;a<3;++a) {
        px[d][a]=x+distances[d]*(ux-lateral[a]*uz);
        pz[d][a]=z+distances[d]*(uz+lateral[a]*ux);
    }
    File f;
    if(File::Open(f,"/CollieKart/ToadCircuit_CKV3.bin",File::READ)!=File::SUCCESS) {
        MessageBox("Vision unavailable","Course file could not be reopened.")();return;
    }
    unsigned char header[16],batch[40*32];
    bool good=f.Read(header,16)==File::SUCCESS &&
        std::memcmp(header,"CKV3",4)==0 &&
        CKVisionU32(header+8)==g_ckCourseCount;
    u32 index=0;
    while(good&&index<g_ckCourseCount) {
        const u32 n=(g_ckCourseCount-index)<32?(g_ckCourseCount-index):32;
        if(f.Read(batch,n*40)!=File::SUCCESS){good=false;break;}
        for(u32 i=0;i<n;++i) {
            const unsigned char *p=batch+i*40;
            CKVisionTriangle t;
            t.a={CKVisionF32(p),CKVisionF32(p+4),CKVisionF32(p+8)};
            t.b={CKVisionF32(p+12),CKVisionF32(p+16),CKVisionF32(p+20)};
            t.c={CKVisionF32(p+24),CKVisionF32(p+28),CKVisionF32(p+32)};
            t.material=static_cast<u16>(CKVisionU32(p+36)&65535u);
            for(unsigned d=0;d<3;++d)for(unsigned a=0;a<3;++a)
                CKVisionIntersectXZ(t,px[d][a],pz[d][a],y,
                    scan.hit[d][a],index+i);
        }
        index+=n;
    }
    f.Close();
    if(!good) {
        MessageBox("Vision read error","Scan incomplete; no results trusted.")();return;
    }
    for(unsigned d=0;d<3;++d)for(unsigned a=0;a<3;++a)
        if(scan.hit[d][a].valid)++scan.validCount;
    const char* classes[3][3];
    for(unsigned d=0;d<3;++d)for(unsigned a=0;a<3;++a) {
        const CKVisionHit &h=scan.hit[d][a];
        // Raw collision types are displayed without pretending classification
        // has been hardware-correlated.
        classes[d][a]=!h.valid?"---":
            ((h.material&31u)==0?"R00":
            ((h.material&31u)==5?"D05":
            ((h.material&31u)==6?"D06":"OTH")));
    }
    char b[430];
    std::snprintf(b,sizeof(b),
        "KCL-based EXPERIMENTAL scan\n"
        "XYZ %.1f %.1f %.1f\n"
        "Heading %.2f %.2f\n"
        "LEFT CENTER RIGHT\n"
        "Near  %s %s %s\n"
        "Mid   %s %s %s\n"
        "Far   %s %s %s\n"
        "Hits %u/9. --- means UNKNOWN, not safe.\n"
        "Requires coordinate and flag validation.",
        x,y,z,fx,fz,
        classes[0][0],classes[0][1],classes[0][2],
        classes[1][0],classes[1][1],classes[1][2],
        classes[2][0],classes[2][1],classes[2][2],scan.validCount);
    MessageBox("P008.3b live course scan",b)();
}
