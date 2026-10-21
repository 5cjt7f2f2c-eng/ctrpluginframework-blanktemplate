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
