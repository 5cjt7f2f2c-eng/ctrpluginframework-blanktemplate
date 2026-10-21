// P008.3: geometry-backed vision mathematics (no unknown MK7 calls).
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
static std::vector<CKVisionTriangle> g_ckCourseTriangles;
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
    g_ckCourseReady=false;g_ckCourseTriangles.clear();
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
    std::vector<unsigned char> bytes(static_cast<unsigned>(size));
    const bool read=f.Read(bytes.data(),static_cast<u32>(size))==File::SUCCESS;
    f.Close();
    const u32 count=CKVisionU32(bytes.data()+8);
    if(!read||std::memcmp(bytes.data(),"CKV3",4)!=0||
       CKVisionU32(bytes.data()+4)!=1 || count==0 || count>10000 ||
       size!=16u+static_cast<u64>(count)*40u ||
       CKVisionCrc(bytes.data()+16,static_cast<unsigned>(size-16))!=
           CKVisionU32(bytes.data()+12)) {
        MessageBox("Invalid vision file","Header, count or checksum mismatch.")();return;
    }
    g_ckCourseTriangles.reserve(count);
    for(u32 i=0;i<count;++i) {
        const unsigned char *p=bytes.data()+16+i*40;
        CKVisionTriangle t;
        t.a={CKVisionF32(p),CKVisionF32(p+4),CKVisionF32(p+8)};
        t.b={CKVisionF32(p+12),CKVisionF32(p+16),CKVisionF32(p+20)};
        t.c={CKVisionF32(p+24),CKVisionF32(p+28),CKVisionF32(p+32)};
        t.material=static_cast<u16>(CKVisionU32(p+36)&65535u);
        if(!CKVisionFinite(t.a.x)||!CKVisionFinite(t.a.y)||!CKVisionFinite(t.a.z)||
           !CKVisionFinite(t.b.x)||!CKVisionFinite(t.b.y)||!CKVisionFinite(t.b.z)||
           !CKVisionFinite(t.c.x)||!CKVisionFinite(t.c.y)||!CKVisionFinite(t.c.z)) {
            g_ckCourseTriangles.clear();
            MessageBox("Invalid vision file","Nonfinite triangle coordinates.")();return;
        }
        g_ckCourseTriangles.push_back(t);
    }
    g_ckCourseReady=true;
    char b[160];std::snprintf(b,sizeof(b),
       "Loaded %lu course triangles. CRC verified.\n"
       "Experimental MK7 geometry; surface labels not validated.",
       static_cast<unsigned long>(g_ckCourseTriangles.size()));
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
    const CKVisionScan scan=CKVisionLook(g_ckCourseTriangles.data(),
        static_cast<unsigned>(g_ckCourseTriangles.size()),x,z,y,fx,fz);
    const char* classes[3][3];
    for(unsigned d=0;d<3;++d)for(unsigned a=0;a<3;++a) {
        const CKVisionHit &h=scan.hit[d][a];
        // Raw collision types are displayed without pretending classification
        // has been hardware-correlated.
        classes[d][a]=!h.valid?"---":
            ((h.material&31u)==0?"R00":
            ((h.material&31u)==5?"D05":
            ((h.material&31u)==6?"D06":"OTH"));
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
