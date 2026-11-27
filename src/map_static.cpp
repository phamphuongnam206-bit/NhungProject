#include <stdint.h>
#include <initializer_list>
#include <math.h>
#include <stdio.h>
#include <string.h>
#ifndef MAP_NAM_HOST_TEST
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#endif

#include "map_config.h"
namespace MapNam {

constexpr float Pi = 3.14159265f, Rad = Pi / 180.0f;
constexpr int SensorCount = 4;
constexpr uint8_t All = 15, None = 255;
constexpr uint32_t FreshMs = 135;
constexpr uint16_t MaxRay = 1100;

#if defined(MAP_NAM_DIAMETER_L_MM) || defined(MAP_NAM_DIAMETER_R_MM) || defined(MAP_NAM_CPR_L) || defined(MAP_NAM_CPR_R)
constexpr float CountsPerMm[] = {MAP_NAM_CPR_L/(Pi*MAP_NAM_DIAMETER_L_MM),
                                 MAP_NAM_CPR_R/(Pi*MAP_NAM_DIAMETER_R_MM)};
#else
constexpr float CountsPerMm[] = {4.5f,4.5f};
#endif
#ifndef MAP_NAM_TRACK_MM
#define MAP_NAM_TRACK_MM 90.0f
#endif
constexpr float Track=MAP_NAM_TRACK_MM;
static_assert(Track>0 && CountsPerMm[0]>0 && CountsPerMm[1]>0,"Invalid wheel calibration");
constexpr int EncoderSign[] = {-1, 1};
constexpr float GyroSign = 1, GyroScale = 1;
constexpr float HalfLength=StaticMap::ROBOT_LENGTH/2.f, HalfWidth=StaticMap::ROBOT_WIDTH/2.f;
constexpr float StartX=StaticMap::START_X, StartY=StaticMap::START_Y, StartHeading=StaticMap::START_HEADING_DEG*Rad;
struct Mount { float forward, left, yaw, opticalBias; };

constexpr Mount Mounts[] = {{45,25,35.43f*Rad,0}, {45,25,10*Rad,0},
                            {45,-25,-10*Rad,0}, {45,-25,-35.43f*Rad,0}};

float bound(float v, float lo, float hi) { return fmaxf(lo, fminf(hi, v)); }
float wrap(float a) {
    while (a > Pi) a -= 2*Pi;
    while (a < -Pi) a += 2*Pi;
    return a;
}
struct Pose { float x, y, h; };
struct Motion { float v, w; };
struct Measurement { Pose origin; uint32_t at; uint16_t mm; };
struct Frame {
    Measurement beam[SensorCount];
    uint32_t number;
    uint8_t mask;
    uint8_t observed; // Fresh, qualified ranges, including returns beyond MaxRay.
};
struct Scan { uint16_t mm[SensorCount]; uint8_t mask; };
int countBits(uint8_t mask) {
    int n = 0;
    for (; mask; mask &= mask-1) ++n;
    return n;
}
int32_t counterDelta(uint16_t cur, uint16_t prev) {
    const uint16_t delta = uint16_t(cur-prev);
    return delta < 32768U ? int32_t(delta) : int32_t(delta)-65536;
}

enum class Mode : uint8_t { IDLE, BLUETOOTH_MAPPING, AUTO_NAVIGATION };
enum class Phase : uint8_t { INIT, WAIT_BUTTON, START_SCAN, MANUAL_MAPPING, MAP_READY,
    INITIAL_LOCALIZATION, TRACKING, OBSTACLE_AVOIDANCE, RELOCALIZATION, FINISHED, FAULT_STOP, START_DELAY };
Mode mode = Mode::IDLE;
Phase phase = Phase::INIT;
const char *reason = "BOOT";
uint32_t phaseAt = 0, bootAt = 0;
bool ledOn(uint32_t now) {
    if (phase == Phase::INIT) {
        const uint32_t elapsed = now - bootAt;
        return elapsed < 1200 && (elapsed / 150) % 2 == 0;
    }
    if (phase==Phase::WAIT_BUTTON || phase==Phase::MAP_READY || phase==Phase::START_DELAY)
        return ((now-phaseAt)/350)%2==0;
    if (mode == Mode::BLUETOOTH_MAPPING) return ((now-phaseAt)/300)%2 == 0;
    return mode == Mode::AUTO_NAVIGATION && phase!=Phase::FAULT_STOP;
}
struct Button {
    bool raw = false, stable = false, armed = false;
    uint32_t changed = 0;
    bool press(bool down, uint32_t now) {
        if (down != raw) { raw = down; changed = now; }
        if (now-changed < 35) return false;
        stable = raw;
        if (!stable) { armed = true; return false; }
        if (!armed) return false;
        armed = false;
        return true;
    }
} button;

Pose pose{StartX, StartY, StartHeading};
float gyroBias = 0, yawRate = 0;
int32_t encoderTotal[2]{};
Motion request{};
Frame latest{};
uint8_t liveMask = 0;
enum class Reading : uint8_t { Hit, NoReturn, Invalid, Stale, Fault };
uint8_t misses[4]{};
uint8_t limitedVisibility=0;
Reading readings[4]={Reading::Stale,Reading::Stale,Reading::Stale,Reading::Stale};
uint32_t motionRangeAge(const Frame &f,uint32_t now) {
    const uint8_t mask=f.observed & liveMask;
    if (!f.number || (mask&6)!=6 || !(mask&9)) return UINT32_MAX;
    const uint32_t left=(mask&1)?now-f.beam[0].at:UINT32_MAX;
    const uint32_t right=(mask&8)?now-f.beam[3].at:UINT32_MAX;
    uint32_t side=left<right?left:right;
    if (request.w>0.03f) side=left;
    if (request.w<-0.03f) side=right;
    const uint32_t a=now-f.beam[1].at,b=now-f.beam[2].at;
    return side>a && side>b?side:(a>b?a:b);
}
bool fresh(const Frame &f,uint32_t now) {
    // Leave a control cycle to brake before the interrupt guard expires.
    return motionRangeAge(f,now)<=FreshMs-20;
}
void enter(Phase next, const char *why, uint32_t now) {
    phase = next; reason = why; phaseAt = now; request = {};
}
void fault(const char *why, uint32_t now) {
    if (phase != Phase::FAULT_STOP) enter(Phase::FAULT_STOP, why, now);
}

#include "navigation/wheel_odometry.inc"
constexpr int Cell=StaticMap::CELL_SIZE, Nx=StaticMap::MAP_WIDTH/Cell, Ymin=-225;
constexpr int Ny=(StaticMap::MAP_HEIGHT-Ymin)/Cell, Cells=Nx*Ny;
constexpr int NodeLimit=96;
constexpr float FinishLine=StaticMap::FINISH_LINE,FinishLeft=StaticMap::FINISH_LEFT,FinishRight=StaticMap::FINISH_RIGHT;
int cellAt(float x, float y) {
    if (x<0 || x>=Nx*Cell || y<Ymin || y>=Ymin+Ny*Cell) return -1;
    return int(x/Cell)+Nx*int((y-Ymin)/Cell);
}
struct Bits {
    uint8_t data[(Cells+7)/8];
    bool get(int i) const { return data[i>>3] & (1U<<(i&7)); }
    void set(int i) { data[i>>3] |= 1U<<(i&7); }
};
struct Footprint {
    float u,v,au,av,a,b,ex,ey;
    Footprint(float h, float margin) : u(cosf(h)),v(sinf(h)),au(fabsf(u)),av(fabsf(v)),
        a(HalfLength+margin),b(HalfWidth+margin),ex(a*au+b*av),ey(a*av+b*au) {}
    bool touches(float dx,float dy,float hx,float hy) const {
        return fabsf(dx)<=ex+hx && fabsf(dy)<=ey+hy &&
            fabsf(dx*u+dy*v)<=a+hx*au+hy*av && fabsf(dy*u-dx*v)<=b+hx*av+hy*au;
    }
};
bool legal(const Pose &p,const Footprint &s) {
    const float left=p.x-s.ex, right=p.x+s.ex;
    if (left<0 || right>StaticMap::MAP_WIDTH || p.y+s.ey>StaticMap::MAP_HEIGHT) return false;
    if (p.y-s.ey<FinishLine && !(left>=FinishLeft && right<=FinishRight)) return false;
    const auto &r=StaticMap::FORBIDDEN;
    return !s.touches((r.xmin+r.xmax)*0.5f-p.x,(r.ymin+r.ymax)*0.5f-p.y,
                      (r.xmax-r.xmin)*0.5f,(r.ymax-r.ymin)*0.5f);
}
bool acrossFinish(Pose p,float margin) {
    const Footprint s(p.h,margin);
    return p.y+s.ey<FinishLine && p.x-s.ex>=FinishLeft && p.x+s.ex<=FinishRight && sinf(p.h)<-0.5f;
}
float intersect(const Ray &r,const StaticMap::Obstacle &box,int &face) {
    float near=0,far=MaxRay;
    face=0;
    for (int axis=0;axis<2;++axis) {
        const float d=axis?r.dy:r.dx, origin=axis?r.y:r.x;
        const float centre=axis?box.y:box.x, half=0.5f*(axis?box.length:box.width);
        if (fabsf(d)<0.00001f) {
            if (fabsf(centre-origin)>half) return -1;
            continue;
        }
        const float a=(centre-half-origin)/d,b=(centre+half-origin)/d;
        if (fminf(a,b)>near) { near=fminf(a,b); face=axis+1; }
        far=fminf(far,fmaxf(a,b));
        if (near>far) return -1;
    }
    return near;
}
struct Terrain {
    Bits newObstacle{},otherObstacle{};
    StaticMap::Obstacle confirmed{};
    bool clearShape(Pose p,const Footprint &s) const {
        if (!legal(p,s)) return false;
        for (const auto &b:StaticMap::FIXED_OBSTACLES)
            if (s.touches(b.x-p.x,b.y-p.y,b.width*0.5f,b.length*0.5f)) return false;
        return !confirmed.id || !s.touches(confirmed.x-p.x,confirmed.y-p.y,confirmed.width*0.5f,confirmed.length*0.5f);
    }
    bool footprint(Pose p,float margin,bool) const { return clearShape(p,Footprint(p.h,margin)); }
    float expected(Pose p,int sensor,int &face) const {
        const Ray ray=rayFrom(p,sensor);
        float nearest=-1;
        face=0;
        for (const auto &b:StaticMap::FIXED_OBSTACLES) {
            int side=0;
            const float distance=intersect(ray,b,side);
            if (distance>=0 && (nearest<0 || distance<nearest)) { nearest=distance; face=side; }
        }
        return nearest;
    }
} terrain;
struct Contact { float x,y; bool valid; };
Contact contacts[SensorCount]{};
bool touchesReturn(const Pose &p,const Footprint &s) {
    for (const Contact &hit : contacts)
        if (hit.valid && s.touches(hit.x-p.x,hit.y-p.y,5,5)) return true;
    return false;
}

struct Node {
    int16_t x,y,angle10;
    uint16_t distance;
    bool important;
    Pose point() const { return {float(x),float(y),angle10*(Rad/10)}; }
};
struct Edge { uint16_t length; };
struct Route {
    Node nodes[NodeLimit]{};
    Edge edges[NodeLimit-1]{};
    uint8_t count=0;
} route;
constexpr float SearchRadius=StaticMap::START_SEARCH_MM, SearchAngle=StaticMap::START_SEARCH_DEG*Rad;
#include "navigation/localization.inc"
struct NewObstacle {
    Contact first{};
    Contact previous[4]{};
    uint8_t repeats[4]{};
    void observe(const Frame &f) {
        excludedBeams=0;
        for (int i=0;i<4;++i) {
            const auto &m=f.beam[i];
            const Ray r=rayFrom(m.origin,i);
            Contact &c=contacts[i];
            c={r.x+r.dx*m.mm,r.y+r.dy*m.mm,bool((f.mask&(1<<i)) && m.mm<=MaxRay)};
            int face;
            const float expected=terrain.expected(m.origin,i,face);
            bool fixedFace=false;
            for (const auto &box:StaticMap::FIXED_OBSTACLES) {
                const float x=fabsf(c.x-box.x)-box.width*0.5f, y=fabsf(c.y-box.y)-box.length*0.5f;
                if (fabsf(fmaxf(x,y))<=positionError+12) { fixedFace=true; break; }
            }
            if (!c.valid || fixedFace || (expected>=0 && m.mm+32>=expected)) { repeats[i]=0; continue; }
            excludedBeams|=1<<i;
            if (!previous[i].valid || hypotf(c.x-previous[i].x,c.y-previous[i].y)>28) repeats[i]=0;
            previous[i]=c;
            if (repeats[i]<3) ++repeats[i];
            if (repeats[i]<3 || terrain.confirmed.id || positionError>32) continue;
            first=c;
            float closest=positionError+22;
            for (const auto &b:StaticMap::POSSIBLE_OBSTACLES) {
                int side;
                const float predicted=intersect(r,b,side);
                const float sideError=fabsf(fmaxf(fabsf(c.x-b.x)-b.width*0.5f,fabsf(c.y-b.y)-b.length*0.5f));
                const float err=predicted>=0?fminf(sideError,fabsf(predicted-m.mm)):sideError;
                if (err<closest) { closest=err; terrain.confirmed=b; }
            }
            if (!terrain.confirmed.id) {
                const int size=StaticMap::BOX_SIZE;
                terrain.confirmed={255,int16_t(lroundf(c.x+r.dx*size/2)),int16_t(lroundf(c.y+r.dy*size/2)),
                                   int16_t(size+40),int16_t(size+40)};
            }
            const auto &b=terrain.confirmed;
            for (int y=b.y-b.length/2;y<=b.y+b.length/2;y+=Cell/2)
                for (int x=b.x-b.width/2;x<=b.x+b.width/2;x+=Cell/2) {
                    const int k=cellAt(x,y);
                    if (k>=0) terrain.newObstacle.set(k);
                }
        }
    }
} obstacle;
// Use a single wall normal only if the same face survives a heading perturbation.
void correctObservedAxis(const Frame &f) {
    float sum[2]{}, weight[2]{};
    for (int i=0;i<4;++i) {
        if (!(f.mask & ~excludedBeams & (1<<i))) continue;
        const Pose origin=f.beam[i].origin;
        int face=0;
        const float predicted=terrain.expected(origin,i,face);
        if (predicted<0 || !face || fabsf(predicted-f.beam[i].mm)>25) continue;
        bool stable=true;
        for (int sign=-1;sign<=1;sign+=2) {
            Pose test=origin; test.h+=sign*headingError;
            int side=0;
            const float range=terrain.expected(test,i,side);
            if (side!=face || fabsf(range-predicted)>20) stable=false;
        }
        if (!stable) continue;
        const Ray ray=rayFrom(origin,i);
        const float component=face==1?ray.dx:ray.dy;
        if (fabsf(component)<0.45f) continue;
        sum[face-1]+=(predicted-f.beam[i].mm)*component;
        weight[face-1]+=1;
    }
    Pose next=pose;
    if (weight[0]) next.x+=bound(0.18f*sum[0]/weight[0],-1.2f,1.2f);
    if (weight[1]) next.y+=bound(0.18f*sum[1]/weight[1],-1.2f,1.2f);
    if (terrain.footprint(next,0,true)) pose=next;
}
void trackScan(const Frame &f,uint32_t now) {
    for (const Measurement &b:f.beam) if (int32_t(b.at-registeredAt)<0) return;
    obstacle.observe(f);
    correctObservedAxis(f);
    int inconsistent=0;
    for (int i=0;i<SensorCount;++i) if ((f.mask&~excludedBeams)&(1<<i)) {
        int face=0;
        const float predicted=terrain.expected(f.beam[i].origin,i,face);
        if (predicted>0 && fabsf(predicted-f.beam[i].mm)>100) ++inconsistent;
    }
    if (inconsistent>=3) ++disagreementFrames;
    else disagreementFrames=0;
    if (disagreementFrames>=5) { relocalize("MAP_SCAN_DISAGREE",now); return; }
    if (matcher.active || now-lastMatchTry<320 || !fresh(f,now)) return;
    Scan s{}; s.mask=f.mask & ~excludedBeams;
    if (countBits(s.mask)<3) return;
    for (int i=0;i<SensorCount;++i) s.mm[i]=f.beam[i].mm;
    const Pose reference=f.beam[1].origin;
    matcher.start(reference,s,24,7*Rad,now); lastMatchTry=now;
    for (int i=0;i<SensorCount;++i) matcher.offset[i]=relative(reference,f.beam[i].origin);
}

char manual='0';
float manualSpeed=100;
uint32_t manualAt=0;
void loadRoute() {
    route={};
    // Subdivide edges so the local planner can rejoin before the next corner.
    for (int i=0;i<StaticMap::ROUTE_COUNT;++i) {
        const auto end=StaticMap::ROUTE[i],start=i?StaticMap::ROUTE[i-1]:end;
        const float dx=end.x-start.x,dy=end.y-start.y,length=hypotf(dx,dy);
        const int steps=1+int(length/90);
        for (int j=1;j<=steps;++j) {
            if (route.count>=NodeLimit) { route.count=0; return; }
            const int index=route.count++;
            Node &n=route.nodes[index];
            n.x=lroundf(start.x+dx*j/steps); n.y=lroundf(start.y+dy*j/steps);
            n.angle10=lroundf((i?atan2f(dy,dx):StartHeading)*10/Rad);
            if (index) {
                const auto &prev=route.nodes[index-1];
                auto &edge=route.edges[index-1];
                edge.length=lroundf(hypotf(n.x-prev.x,n.y-prev.y));
                n.distance=prev.distance+edge.length;
            }
            n.important=(j==steps);
        }
    }
}
void startAuto(uint32_t now);
void onPress(uint32_t now) {
    if (phase==Phase::FAULT_STOP) return;
    if (mode==Mode::AUTO_NAVIGATION) { fault("SW1_EMERGENCY",now); return; }
    if (phase==Phase::WAIT_BUTTON) {
#if defined(COMPETITION_2_LAPS) && COMPETITION_2_LAPS
        resetOdometry({StartX,StartY,StartHeading},now); lastFresh=now;
        mode=Mode::BLUETOOTH_MAPPING; manual='0'; manualSpeed=100;
        enter(Phase::MANUAL_MAPPING,"MANUAL",now);
#else
        startAuto(now);
#endif
    } else if (phase==Phase::MAP_READY) startAuto(now);
}
void onCommand(char c,uint32_t now) {
    if (phase!=Phase::MANUAL_MAPPING) return;
    if (c=='S') { manual='0'; mode=Mode::IDLE; enter(Phase::MAP_READY,"WAIT_AUTO",now); }
    else if (c=='U' || c=='D' || c=='M') manualSpeed=c=='M'?100:bound(manualSpeed+(c=='U'?15:-15),45,155);
    else if (c && strchr("FBLRGIHJ0XP",c)) { manual=c; manualAt=now; }
}
void onFrame(const Frame &f,uint32_t now) {
    if (phase==Phase::INITIAL_LOCALIZATION || phase==Phase::RELOCALIZATION) localScan(f,now);
    else if (phase==Phase::TRACKING || phase==Phase::OBSTACLE_AVOIDANCE) trackScan(f,now);
}
Motion manualMotion() {
    switch (manual) {
        case 'F': return {manualSpeed,0};
        case 'B': return {-manualSpeed*0.6f,0};
        case 'L': return {0,0.85f};
        case 'R': return {0,-0.85f};
        case 'G': return {manualSpeed*0.72f,0.72f};
        case 'I': return {manualSpeed*0.72f,-0.72f};
        case 'H': return {-manualSpeed*0.45f,-0.65f};
        case 'J': return {-manualSpeed*0.45f,0.65f};
        default: return {};
    }
}
#include "navigation/path_control.inc"
// For a box on a straight, validate a forward bypass before entering braking range.
// The flash route remains authoritative. The join point is on the same straight.
Pose bypass[4]{};
int bypassStep=-1;
uint8_t bypassJoin=0;
bool bypassClear(Pose a,Pose b,float margin) {
    const float bearing=atan2f(b.y-a.y,b.x-a.x);
    // Enclose every intermediate heading during the pivot, then test the exact
    // swept rectangle of the translation. Cost is independent of segment length.
    const float radius=hypotf(HalfLength+margin,HalfWidth+margin);
    Footprint pivot(0,0);
    pivot.a=pivot.b=pivot.ex=pivot.ey=radius;
    if (!terrain.clearShape(a,pivot) || touchesReturn(a,pivot)) return false;
    Pose mid{(a.x+b.x)*0.5f,(a.y+b.y)*0.5f,bearing};
    Footprint body(bearing,margin);
    body.a+=separation(a,b)*0.5f;
    body.ex=body.a*body.au+body.b*body.av;
    body.ey=body.a*body.av+body.b*body.au;
    return terrain.clearShape(mid,body) && !touchesReturn(mid,body);
}
bool beginBypass(Pose a,Pose b) {
    const auto &box=terrain.confirmed;
    if (!box.id || bypassStep>=0) return false;
    const float h=atan2f(b.y-a.y,b.x-a.x),u=cosf(h),v=sinf(h);
    const float dx=box.x-pose.x,dy=box.y-pose.y,along=u*dx+v*dy,side=-v*dx+u*dy;
    const float half=0.5f*(fabsf(v)*box.width+fabsf(u)*box.length);
    if (along<135 || along>365 || fabsf(side)>half+HalfWidth+safetyMargin()) return false;
    int join=targetNode;
    for (;join<route.count;++join) {
        const Pose p=route.nodes[join].point();
        if (fabsf(wrap(p.h-h))>8*Rad) return false;
        if ((p.x-box.x)*u+(p.y-box.y)*v>half+HalfLength+safetyMargin()+8) break;
    }
    if (join>=route.count) return false;
    const float space=half+hypotf(HalfLength,HalfWidth)+safetyMargin()+25;
    for (int sign : {-1,1}) {
        const float shift=side+sign*space;
        auto point=[&](float ahead) -> Pose {
            return {pose.x+u*ahead-v*shift,pose.y+v*ahead+u*shift,h};
        };
        bypass[0]=pose;
        bypass[1]=point(fmaxf(15,along-space));
        bypass[2]=point(along+space);
        bypass[3]=route.nodes[join].point();
        // If the obstruction is next to the gate, finish in the clear parallel lane.
        const auto finish=route.nodes[route.count-1].point();
        const Pose exit{bypass[2].x,finish.y,h};
        const bool nearGate=join+2>=route.count && v<-0.98f && acrossFinish(exit,safetyMargin());
        if (nearGate) bypass[3]=exit;
        bool clear=true;
        for (int i=1;i<=3;++i) {
            if (!bypassClear(bypass[i-1],bypass[i],safetyMargin()+3)) { clear=false; break; }
            bypass[i].h=atan2f(bypass[i].y-bypass[i-1].y,bypass[i].x-bypass[i-1].x);
        }
        if (clear) { bypassStep=1; bypassJoin=join; return true; }
    }
    return false;
}

void followRoute(float dt,uint32_t now) {
    Pose a=route.nodes[currentNode].point(), b=route.nodes[targetNode].point();
    if (bypassStep<0 && beginBypass(a,b)) {
        progressWatch.reset(now); enter(Phase::OBSTACLE_AVOIDANCE,"LOCAL_BYPASS",now);
    }
    if (bypassStep>0) { a=bypass[bypassStep-1]; b=bypass[bypassStep]; }
    if ((targetNode==route.count-1 || (bypassStep>0 && bypassJoin+2>=route.count)) && acrossFinish(pose,safetyMargin())) { finishRun(now); return; }
    const float dx=b.x-a.x,dy=b.y-a.y,len=hypotf(dx,dy);
    const float along=((pose.x-a.x)*dx+(pose.y-a.y)*dy)/len;
    trackingError=((pose.y-a.y)*dx-(pose.x-a.x)*dy)/len;
    const float remaining=len-along;
    if (remaining<4 && bypassStep>0) {
        if (++bypassStep>3) {
            bypassStep=-1; targetNode=bypassJoin; currentNode=targetNode-1;
            enter(Phase::TRACKING,"REJOINED",now);
        }
        progressWatch.reset(now); return;
    }
    if (remaining<4 && targetNode+1<route.count) {
        currentNode=targetNode++; progressWatch.reset(now); headingIntegral=0; return;
    }
    const float heading=atan2f(dy,dx)-atanf(trackingError/35);
    const float error=wrap(heading-pose.h);
    const float turn=bound(3.4f*error-0.15f*yawRate,-1.4f,1.4f);
    const bool open=terrain.footprint(pose,safetyMargin()+30,true) && fabsf(error)<8*Rad;
    float speed=fminf(open?StaticMap::CRUISE_MM_S:StaticMap::NARROW_MM_S,forwardLimit());
    if (limitedVisibility || positionError>38 || headingError>10*Rad || (latest.observed & liveMask)!=All) speed=fminf(speed,45);
    if (route.nodes[targetNode].important) speed=fminf(speed,35+fmaxf(0,remaining)*0.75f);
    speed*=bound(1-fabsf(error)/(50*Rad),0,1);
    if (fabsf(error)>12*Rad) speed=0;
    Motion next{speed,turn};
    if (!progressWatch.advancing(fmaxf(0,remaining)+50*fabsf(error),now)) { fault("NO_TRACK_PROGRESS",now); return; }
    const float margin=safetyMargin();
    // Preview the tracking arc before committing to acceleration in a narrow passage.
    while (next.v>12 && !swept(pose,next,0.35f+next.v/320,margin,true)) {
        next.v*=0.65f;
        if (fabsf(error)<4*Rad) next.w*=0.65f;
    }
    if (!issue(next,dt)) { bypassStep=-1; startAvoidance(now); }
}
void startAuto(uint32_t now) {
    if (route.count<2) { fault("BAD_ROUTE",now); return; }
    bypassStep=-1; terrain={}; obstacle={}; memset(contacts,0,sizeof(contacts)); memset(blockedEdge,0,sizeof(blockedEdge));
    batch={}; matcher.active=false; attempts=0; excludedBeams=disagreementFrames=0;
    headingIntegral=edgeStartDistance=0; detour.reset(); detourRetries=0; progressWatch.reset(now);
    resetOdometry({StartX,StartY,StartHeading},now);
    mode=Mode::AUTO_NAVIGATION; stillSince=now;
    if (StaticMap::LOCALIZE_AT_START) enter(Phase::INITIAL_LOCALIZATION,"LOCATE_START",now);
    else enter(Phase::START_DELAY,"START_DELAY",now);
}
float turnEncoderDeg=0, turnGyroDeg=0;
float slipEncoderDeg=0, slipGyroDeg=0;
void onMotion(int left, int right, float dt, uint32_t now) {
    if (phase==Phase::START_DELAY) {
        if (now-phaseAt<2000) return;
        turnEncoderDeg=turnGyroDeg=slipEncoderDeg=slipGyroDeg=0;
        currentNode=0; targetNode=1;
        registeredAt=phaseAt; lastFresh=lastMatchTry=now; lastMatchAt=0;
        lastFixDistance=0; matchConfidence=0; matchError=-1;
        positionError=7; headingError=2.5f*Rad;
        progressWatch.reset(now);
        enter(Phase::TRACKING,"ASSUMED_START",now);
    }
    if (phase==Phase::FINISHED) { enter(Phase::WAIT_BUTTON,"FINISH_BODY_CLEAR",now); return; }
    if (phase==Phase::INITIAL_LOCALIZATION || phase==Phase::RELOCALIZATION) {
        if (left || right || fabsf(yawRate)>1.5f*Rad) {
            stillSince=now; batch={}; matcher.active=false;
        }
        if (now-phaseAt>6500) fault("LOCALIZATION_TIMEOUT",now);
    }
    if (!predicting()) return;
    turnEncoderDeg+=(right/CountsPerMm[1]-left/CountsPerMm[0])/Track/Rad;
    turnGyroDeg+=(yawRate+lastRate)*0.5f*dt/Rad;
    predict(left,right,dt,now);
    if (phase!=Phase::MANUAL_MAPPING) {
        if (!fresh(latest,now)) {
            request={};
            if (now-lastFresh>750) {
                for (Reading r:readings) if (r==Reading::Fault) { fault("VL53_I2C",now); return; }
                reason="WAIT_SENSOR_COVERAGE";
            }
            return;
        }
        lastFresh=now;
        for (const Measurement &b:latest.beam)
            if (int32_t(b.at-registeredAt)<0) { request={}; return; }
        const bool uncertain=positionError>38 || headingError>10*Rad;
        if (slipTime>0.22f || headingError>15*Rad || positionError>55 ||
            (uncertain && !terrain.footprint(pose,safetyMargin()+20,true))) {
            if (slipTime>0.22f) { slipEncoderDeg=turnEncoderDeg; slipGyroDeg=turnGyroDeg; }
            relocalize(slipTime>0.22f?"WHEEL_SLIP":"FIX_REQUIRED",now); return;
        }
        if (phase==Phase::TRACKING || bypassStep>0) followRoute(dt,now);
        else if (phase==Phase::OBSTACLE_AVOIDANCE) followDetour(dt,now);
        return;
    }
    if (!terrain.footprint(pose,0,false)) { fault("FIELD_LIMIT",now); return; }
    if (slipTime>0.32f) { fault("TEACH_WHEEL_SLIP",now); return; }
    if (now-manualAt>667) manual='0';
    request=manualMotion();
    if (!fresh(latest,now)) {
        request={};
        if (now-lastFresh>850) fault("TEACH_RANGES_LOST",now);
        return;
    }
    lastFresh=now;
    if ((request.v>0 && nearFront(latest,45)) ||
        !swept(pose,request,0.24f,0,request.v<=0)) request={};
}
bool startConsistent() {
    if (matcher.scan.mask!=All || !matcher.accepted || !terrain.footprint(matcher.best,0,true)) return false;
    for (int i=0;i<4;++i) {
        int face;
        const float d=terrain.expected(matcher.best,i,face);
        if (d<0 || fabsf(d-matcher.scan.mm[i])>3) return false;
    }
    return true;
}
void work(uint32_t now,int candidates=3) {
    if (phase!=Phase::INITIAL_LOCALIZATION && phase!=Phase::RELOCALIZATION &&
        phase!=Phase::TRACKING && phase!=Phase::OBSTACLE_AVOIDANCE) return;
    for (int i=0;i<candidates && matcher.active;++i) if (matcher.step()) {
        if (phase==Phase::INITIAL_LOCALIZATION) matcher.accepted=startConsistent();
        applyMatch(now);
    }
    if (phase==Phase::OBSTACLE_AVOIDANCE && bypassStep<0 && fresh(latest,now)) detour.expand(now);
}

#ifndef MAP_NAM_HOST_TEST
#include "navigation/stm32_hardware.inc"
void pollRange(uint32_t now) {
    if (!rangesRunning || now-lastPoll<2) return;
    lastPoll=now;
    const uint8_t i=nextPoll, bit=uint8_t(1U<<i);
    nextPoll=(i+1)%SensorCount;
    uint8_t ready=0, result[13]{};
    const uint32_t prev=lastProbe[i];
    bool ok=readRegs(rangeBus,Addr[i],0x13,&ready,1);
    lastProbe[i]=ok ? now : 0;
    if (ok && !(ready&7)) return;
    if (ok) {

        ok=readRegs(rangeBus,Addr[i],0x14,result,sizeof(result));
        ok=writeReg(rangeBus,Addr[i],0x0b,1) && ok;
    }
    const int raw=(result[10]<<8)|result[11], signal=(result[6]<<8)|result[7];
    const uint8_t status=(result[0]>>3)&15;
    Reading sample=Reading::Invalid;
    if (!ok) sample=Reading::Fault;
    else if (!prev || now-prev>45) sample=Reading::Stale;
    else if (status==11 && signal>=32 && raw>0 && raw<8190)
        sample=raw>MaxRay?Reading::NoReturn:Reading::Hit;
    else if (raw>=8190 && (status==4 || status==11)) sample=Reading::NoReturn;
    readings[i]=sample;
    const uint32_t at=prev?prev-33:now;
    pending.beam[i]={sampleOrigin(at),at,uint16_t(sample==Reading::NoReturn?MaxRay+1:bound(raw+Mounts[i].opticalBias,1,8189))};
    const bool longReturn=sample==Reading::NoReturn && status==11 && raw<8190 && signal>=32;
    bool observed=sample==Reading::Hit || longReturn;
    limitedVisibility &= ~bit;
    if (sample==Reading::NoReturn && !longReturn) {
        if (misses[i]<3) ++misses[i];
        int face;
        const float expected=terrain.expected(pending.beam[i].origin,i,face);
        // Repeated no-target reports may support slow travel only where the
        // static map also predicts no nearby reflector. Invalid data never do.
        observed=misses[i]>=3 && (expected<0 || expected>500);
        limitedVisibility|=bit;
    } else misses[i]=0;
    if (observed) liveMask|=bit;
    else liveMask &= ~bit;
    // Motor control consumes each completed measurement immediately.
    // Localization still processes one frame after all four sensors report.
    latest.beam[i]=pending.beam[i];
    latest.observed=liveMask;
    if (sample==Reading::Hit) latest.mask|=bit;
    else latest.mask &= ~bit;
    collected |= bit;
    if (collected!=All) return;
    collected=0; pending.observed=liveMask; pending.mask=liveMask;
    for (int k=0;k<SensorCount;++k)
        if (readings[k]!=Reading::Hit || now-pending.beam[k].at>95) pending.mask &= ~(1U<<k);
    pending.number=latest.number+1;
    latest=pending;
    onFrame(latest,now);
}
#include "navigation/control_tick.inc"
uint32_t maxControlUs=0, maxTickGapUs=0, lastTickUs=0;
struct Telemetry {
    bool debug=false;
    char buffer[560]{};
    unsigned queued=0, sent=0;
    void diagnostics(uint32_t now) {
        char status[5]{};
        for (int i=0;i<4;++i)
            status[i]=now-latest.beam[i].at>FreshMs?'T':"HNITF"[int(readings[i])];
        const int n=snprintf(buffer,sizeof(buffer),
            "MODE=%u PHASE=%u REASON=%s X_MM=%d Y_MM=%d HEADING_DEG=%d ENC_L=%ld ENC_R=%ld "
            "RAW16_L=%u RAW16_R=%u GYRO_Z=%d J8=%u J9=%u J10=%u J11=%u SENSOR_STATUS=%s "
            "CURRENT_NODE=%u TARGET_NODE=%u CROSS_TRACK_ERROR=%d POSE_UNCERTAINTY=%d,%d "
            "OBSTACLE_ID=%u PWM_L=%d PWM_R=%d SW1=%d MAX_CONTROL_US=%lu MAX_TICK_GAP_US=%lu TRIP_CONTROL_MS=%lu TRIP_RANGE_MS=%lu SLIP_ENC_DEG=%d SLIP_GYRO_DEG=%d\r\n",
            unsigned(mode),unsigned(phase),reason,int(pose.x),int(pose.y),int(pose.h/Rad),
            long(encoderTotal[0]),long(encoderTotal[1]),unsigned(uint16_t(TIM3->CNT)),unsigned(uint16_t(TIM2->CNT)),
            int(yawRate/Rad),unsigned(latest.beam[0].mm),unsigned(latest.beam[1].mm),unsigned(latest.beam[2].mm),
            unsigned(latest.beam[3].mm),status,unsigned(currentNode),unsigned(targetNode),int(trackingError),
            int(ceilf(positionError)),int(ceilf(headingError/Rad)),unsigned(terrain.confirmed.id),
            wheels[0].applied,wheels[1].applied,int(button.stable),(unsigned long)maxControlUs,(unsigned long)maxTickGapUs,(unsigned long)tripControlMs,(unsigned long)tripRangeMs,int(slipEncoderDeg),int(slipGyroDeg));
        queued=n>0?min(unsigned(n),unsigned(sizeof(buffer)-1)):0;
        sent=0; debug=false;
    }
    static constexpr uint32_t ReportIntervalMs=1000;
    uint32_t lastReportAt=0;
    bool requested=true;
    Phase reported=Phase::INIT;
    void service(uint32_t now) {
        static constexpr char states[][32]={
            "KHOI DONG", "CHO NHAN SW1", "DANG QUET START", "CHAY BLUETOOTH",
            "CHO AUTO", "DINH VI START", "CHAY TU DONG", "TRANH VAT CAN",
            "DINH VI LAI", "DA VE DICH", "DUNG DO LOI", "CHO 2 GIAY"
        };
        if (debug && sent==queued) diagnostics(now);
        if (sent<queued) {
            const int room=Serial.availableForWrite();
            if (room>0) sent+=Serial.write((const uint8_t*)buffer+sent,min(unsigned(room),min(24U,queued-sent)));
            return;
        }
        if (!requested && reported==phase && now-lastReportAt<ReportIntervalMs) return;
        const char *state=states[static_cast<unsigned>(phase)];
        if (Serial.availableForWrite()<int(strlen(state)+2)) return;
        Serial.println(state);
        reported=phase;
        requested=false;
        lastReportAt=now;
    }
} telemetry;
void beginHardware() {
    low(PB0);
    digitalWrite(PC13,HIGH); pinMode(PC13,OUTPUT);
    analogWriteResolution(8);
    loadRoute();
    for (int i=0;i<2;++i) { low(In1[i]); low(In2[i]); low(Pwm[i]); analogWrite(Pwm[i],0); }
    for (uint32_t pin : Shut) low(pin);
    pinMode(SwitchPin,INPUT_PULLUP);
    Serial.setRx(PA10); Serial.setTx(PA9); Serial.begin(38400,SERIAL_8N1);
    Wire.setSDA(PB7); Wire.setSCL(PB6); Wire.begin(); Wire.setClock(100000);
    rangeBus.begin(); rangeBus.setClock(100000);
    bootAt=bootStepAt=lastTick=millis();
    if (!initEncoders()) fault("ENCODER_INIT",bootAt);
    if (!initImu()) fault("MPU_INIT",bootAt);
}
void serviceControl() {
    const uint32_t tickBefore=lastTick, started=micros();
    control(millis());
    if (lastTick!=tickBefore) {
        maxControlUs=max(maxControlUs,uint32_t(micros()-started));
        if (lastTickUs) maxTickGapUs=max(maxTickGapUs,uint32_t(started-lastTickUs));
        lastTickUs=started;
    }
}
void service() {
    const uint32_t now=millis();
    if (button.press(digitalRead(SwitchPin)==LOW,now))
        onPress(now);
    for (int n=0;n<12 && Serial.available();++n) {
        const char c=char(Serial.read());
        if (c=='?') telemetry.debug=true; else onCommand(c,now);
    }
    digitalWrite(PC13,ledOn(now)?LOW:HIGH);
    bootRanges(now);
    serviceControl();
    pollRange(millis());
    serviceControl();
    if (millis()-lastTick<7) work(millis(),1);
    if (phase==Phase::FAULT_STOP || (!request.v && !request.w && armed)) disableDrive(millis());
    telemetry.service(millis());
}
#endif
}

#ifndef MAP_NAM_HOST_TEST
void setup() { MapNam::beginHardware(); }
void loop() { MapNam::service(); }
#endif
