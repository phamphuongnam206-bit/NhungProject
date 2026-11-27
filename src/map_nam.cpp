#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#ifndef MAP_NAM_HOST_TEST
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#endif

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
constexpr float HalfLength = 68, HalfWidth = 58;
constexpr float StartX = 330, StartY = 75, StartHeading = Pi / 2;
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
    INITIAL_LOCALIZATION, TRACKING, OBSTACLE_AVOIDANCE, RELOCALIZATION, FINISHED, FAULT_STOP };
Mode mode = Mode::IDLE;
Phase phase = Phase::INIT;
const char *reason = "BOOT";
uint32_t phaseAt = 0, bootAt = 0;
bool ledOn(uint32_t now) {
    if (phase == Phase::INIT) {
        const uint32_t elapsed = now - bootAt;
        return elapsed < 1200 && (elapsed / 150) % 2 == 0;
    }
    if (mode == Mode::BLUETOOTH_MAPPING) return ((now-phaseAt)/300)%2 == 0;
    return mode == Mode::AUTO_NAVIGATION;
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
bool fresh(const Frame &f, uint32_t now) {
    if (!f.number || (f.mask & liveMask) != All) return false;
    for (const auto &b : f.beam) if (now-b.at > FreshMs) return false;
    return true;
}
void enter(Phase next, const char *why, uint32_t now) {
    phase = next; reason = why; phaseAt = now; request = {};
}
void fault(const char *why, uint32_t now) {
    if (phase != Phase::FAULT_STOP) enter(Phase::FAULT_STOP, why, now);
}

#include "navigation/wheel_odometry.inc"
constexpr int Cell=25, Nx=72, Ny=57, Ymin=-225, Cells=Nx*Ny;
constexpr float FinishLine=70, FinishLeft=1250, FinishRight=1750;
constexpr int NodeLimit=96;
constexpr float NodeStep=85, NodeTurn=20*Rad, NodeRangeChange=110;
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
bool legal(const Pose &p, const Footprint &s) {
    const float left=p.x-s.ex,right=p.x+s.ex,bottom=p.y-s.ey;
    if (left<50 || right>1750 || p.y+s.ey>1150 || bottom<Ymin+Cell) return false;
    if (bottom<50 && !(left>=50 && right<=550 && bottom>=-100) &&
        !(left>=FinishLeft && right<=FinishRight)) return false;
    return !s.touches(900-p.x,300-p.y,300,300);
}
bool acrossFinish(Pose p, float margin) {
    const Footprint s(p.h,margin);
    return p.x-s.ex>FinishLeft && p.x+s.ex<FinishRight && p.y+s.ey<FinishLine && sinf(p.h)<-0.5f;
}
struct RayWalk {
    int x,y,sx,sy,face=0;
    float tx,ty,stepX,stepY,entry=0;
    explicit RayWalk(const Ray &r) {
        const int cell=cellAt(r.x,r.y);
        x=cell<0?-1:cell%Nx; y=cell<0?-1:cell/Nx;
        sx=r.dx>=0?1:-1; sy=r.dy>=0?1:-1;
        stepX=fabsf(r.dx)<0.00001f?1e9f:Cell/fabsf(r.dx);
        stepY=fabsf(r.dy)<0.00001f?1e9f:Cell/fabsf(r.dy);
        tx=stepX==1e9f?1e9f:fabsf((x+(sx>0))*Cell-r.x)*stepX/Cell;
        ty=stepY==1e9f?1e9f:fabsf(Ymin+(y+(sy>0))*Cell-r.y)*stepY/Cell;
    }
    bool inside() const { return x>=0 && x<Nx && y>=0 && y<Ny; }
    int index() const { return y*Nx+x; }
    void next() {
        if (tx<ty) { entry=tx; tx+=stepX; x+=sx; face=1; }
        else { entry=ty; ty+=stepY; y+=sy; face=2; }
    }
};
struct Terrain {
    Bits surveyed, currentFree, newObstacle, otherObstacle;
    uint8_t evidence[(Cells+3)/4];
    uint16_t landmarks;
    uint8_t hits(int k) const { return (evidence[k>>2] >> (2*(k&3))) & 3; }
    void hit(int k) {
        const uint8_t n=hits(k);
        if (n<3) {
            evidence[k>>2]+=uint8_t(1U<<(2*(k&3)));
            if (n==1) ++landmarks;
        }
    }
    void clearRay(const Ray &ray, float mm, Bits &dest) {
        const float end=fminf(float(MaxRay),mm-Cell);
        for (RayWalk w(ray); w.inside() && w.entry<end; w.next()) {
            if (hits(w.index())>=2 || newObstacle.get(w.index()) || otherObstacle.get(w.index())) break;
            dest.set(w.index());
        }
    }
    void learn(const Frame &f,bool repeated=false) {
        int used[SensorCount]={-1,-1,-1,-1};
        for (int i=0;i<SensorCount;++i) {
            if (!(f.mask&(1<<i))) continue;
            const Measurement &m=f.beam[i];
            const Ray ray=rayFrom(m.origin,i);
            clearRay(ray,m.mm,surveyed);
            if (m.mm>MaxRay) continue;
            const int k=cellAt(ray.x+ray.dx*m.mm,ray.y+ray.dy*m.mm);
            if (k<0) continue;
            bool seen=false;
            for (int j=0;j<i;++j) seen |= used[j]==k;
            if (!seen) { hit(k); if (repeated) hit(k); }
            used[i]=k;
        }
    }
    bool footprint(Pose p, float margin, bool requireFree, bool record=false) {
        const Footprint s(p.h,margin);
        if (!legal(p,s)) return false;
        const int lo=cellAt(p.x-s.ex,p.y-s.ey),hi=cellAt(p.x+s.ex,p.y+s.ey);
        if (lo<0 || hi<0) return false;
        for (int y=lo/Nx;y<=hi/Nx;++y) {
            for (int x=lo%Nx;x<=hi%Nx;++x) {
                const int k=y*Nx+x;
                if (!s.touches((x+0.5f)*Cell-p.x,Ymin+(y+0.5f)*Cell-p.y,Cell*0.5f,Cell*0.5f))
                    continue;
                if (record) surveyed.set(k);
                else if (hits(k)>=2 || newObstacle.get(k) || otherObstacle.get(k) ||
                         (requireFree && !surveyed.get(k) && !currentFree.get(k))) return false;
            }
        }
        return true;
    }
    float expected(Pose p,int sensor,int &face) const {
        const Ray ray=rayFrom(p,sensor);
        for (RayWalk w(ray); w.inside() && w.entry<=MaxRay; w.next()) {
            const int k=w.index();
            if (hits(k)<2) continue;
            const bool vertical=(w.y>0 && hits(k-Nx)>=2) || (w.y+1<Ny && hits(k+Nx)>=2);
            const bool horizontal=(w.x>0 && hits(k-1)>=2) || (w.x+1<Nx && hits(k+1)>=2);
            face=vertical!=horizontal ? (vertical?1:2) : w.face;
            float distance=(w.entry+fminf(w.tx,w.ty))*0.5f;
            if (vertical && !horizontal && fabsf(ray.dx)>0.12f)
                distance=((w.x+0.5f)*Cell-ray.x)/ray.dx;
            if (horizontal && !vertical && fabsf(ray.dy)>0.12f)
                distance=(Ymin+(w.y+0.5f)*Cell-ray.y)/ray.dy;
            return bound(distance,w.entry-Cell*0.5f,fminf(float(MaxRay),w.entry+2*Cell));
        }
        face=0; return -1;
    }
} terrain{};

struct Contact { float x,y; bool valid; };
Contact contacts[SensorCount]{};
bool touchesReturn(const Pose &p,const Footprint &s) {
    for (const Contact &hit : contacts)
        if (hit.valid && s.touches(hit.x-p.x,hit.y-p.y,5,5)) return true;
    return false;
}

struct Node {
    int16_t x,y,angle10;
    uint16_t distance,mm[SensorCount],landmark[SensorCount];
    uint8_t id,previous,mask,confidence,important;
    Pose point() const { return {float(x),float(y),angle10*(Rad/10)}; }
};
struct Edge {
    uint16_t length,clearance,cost;
    int16_t bearing10;
    uint8_t from,to;
    int8_t direction;
};
struct Route {
    Node nodes[NodeLimit];
    Edge edges[NodeLimit-1];
    uint8_t count;
    bool locked;
    uint16_t clearance;
    float signedDistance;
    bool record(const Frame &f,bool force,bool important) {
        if (locked) return false;
        const Node *prev=count?&nodes[count-1]:nullptr;
        const float gap=prev?separation(pose,prev->point()):0;
        const float turn=prev?fabsf(wrap(pose.h-prev->point().h)):0;
        bool change=false;
        for (int i=0;i<SensorCount;++i) {
            if (!(f.mask&(1<<i))) continue;
            if (f.beam[i].mm<clearance) clearance=f.beam[i].mm;
            if (prev && (prev->mask&(1<<i)) && abs(int(f.beam[i].mm)-prev->mm[i])>NodeRangeChange)
                change=true;
        }
        if (prev && gap<9 && turn<5*Rad && travelled-prev->distance<15) {
            if (force) nodes[count-1].important |= important;
            return true;
        }
        const float step=count>NodeLimit*3/4 ? NodeStep*1.6f : NodeStep;
        if (prev && !force && travelled-prev->distance<step && turn<NodeTurn && !(change && gap>=45))
            return true;
        if (count>=NodeLimit || travelled>65000) return false;
        Node &n=nodes[count];
        n.x=lroundf(pose.x); n.y=lroundf(pose.y); n.angle10=lroundf(pose.h*10/Rad);
        n.distance=lroundf(travelled); n.id=count; n.previous=count?count-1:None;
        n.mask=f.mask; n.important=important || turn>=NodeTurn;
        n.confidence=uint8_t(bound(100-positionError+3*countBits(f.mask),0,100));
        for (int i=0;i<SensorCount;++i) {
            n.mm[i]=(f.mask&(1<<i))?f.beam[i].mm:0;
            n.landmark[i]=UINT16_MAX;
            if (!n.mm[i] || n.mm[i]>MaxRay) continue;
            const Ray ray=rayFrom(f.beam[i].origin,i);
            const int k=cellAt(ray.x+ray.dx*n.mm[i],ray.y+ray.dy*n.mm[i]);
            if (k>=0 && terrain.hits(k)>=2) n.landmark[i]=k;
        }
        if (prev) {
            Edge &e=edges[count-1];
            e.from=prev->id; e.to=count; e.length=n.distance-prev->distance;
            e.bearing10=gap>2?lroundf(atan2f(n.y-prev->y,n.x-prev->x)*10/Rad):n.angle10;
            e.clearance=clearance;
            e.cost=uint16_t(bound(e.length+turn*Track+(clearance<180?40:0),1,65535));
            e.direction=signedDistance < -2 ? -1 : 1;
        }
        ++count; clearance=UINT16_MAX; signedDistance=0;
        return true;
    }
} route{};

constexpr float SearchRadius=50, SearchAngle=15*Rad;
#include "navigation/localization.inc"
struct NewObstacle {
    Contact first{};
    struct Surface { float x,y,dx,dy; } samples[14]{};
    uint8_t samplesUsed=0;
    struct TrackHit { Contact centre{}; uint8_t seen=0; } tracks[SensorCount];
    uint32_t lastFrame=0;
    bool closeToFixed(float x,float y) const {
        const int k=cellAt(x,y);
        if (k<0) return false;
        for (int dy=-1;dy<=1;++dy) for (int dx=-1;dx<=1;++dx) {
            const int ix=k%Nx+dx,iy=k/Nx+dy;
            if (ix>=0 && ix<Nx && iy>=0 && iy<Ny && terrain.hits(ix+iy*Nx)>=2) return true;
        }
        return false;
    }
    void mark(Contact point,Ray ray) {
        if (!first.valid) first=point;
        if (hypotf(point.x-first.x,point.y-first.y)>165) return;
        for (int i=0;i<samplesUsed;++i) {
            if (hypotf(samples[i].x-point.x,samples[i].y-point.y)<14) return;
        }
        if (samplesUsed>=14) return;
        samples[samplesUsed++]={point.x,point.y,ray.dx,ray.dy};
        float tx=-samples[0].dy,ty=samples[0].dx,nx=samples[0].dx,ny=samples[0].dy;
        float cx=first.x+48*nx,cy=first.y+48*ny,halfT=69,halfN=61;
        float bestSupport=0;
        for (int i=0;i<samplesUsed;++i) for (int j=i+1;j<samplesUsed;++j) {
            const Surface &a=samples[i],&b=samples[j];
            const float dx=b.x-a.x,dy=b.y-a.y,length=hypotf(dx,dy);
            if (length<36 || length>116) continue;
            const float u=dx/length,v=dy/length;
            float fx=-v,fy=u;
            if (fx*a.dx+fy*a.dy<0) { fx=-fx; fy=-fy; }
            float lo=0,hi=length,support=0;
            bool fits=true;
            for (int k=0;k<samplesUsed;++k) {
                const auto &s=samples[k];
                const float px=s.x-a.x,py=s.y-a.y;
                const float depth=px*fx+py*fy,side=px*u+py*v;
                if (depth < -12 || depth>112) { fits=false; break; }
                lo=fminf(lo,side); hi=fmaxf(hi,side);
                if (fabsf(depth)<9 && s.dx*fx+s.dy*fy>0.18f) support+=24;
            }
            if (!fits || hi-lo>116 || support<48 || support+length<=bestSupport) continue;
            bestSupport=support+length;
            tx=u; ty=v; nx=fx; ny=fy;
            cx=a.x+(lo+hi)*0.5f*u+50*fx; cy=a.y+(lo+hi)*0.5f*v+50*fy;
            halfT=fmaxf(50,100-(hi-lo)*0.5f)+5; halfN=55;
        }
        memset(&terrain.newObstacle,0,sizeof(terrain.newObstacle));
        const int centre=cellAt(cx,cy);
        if (centre>=0) for (int dy=-6;dy<=6;++dy) for (int dx=-6;dx<=6;++dx) {
            const int x=centre%Nx+dx,y=centre/Nx+dy;
            if (x<0 || x>=Nx || y<0 || y>=Ny) continue;
            const float rx=(x+0.5f)*Cell-cx,ry=Ymin+(y+0.5f)*Cell-cy;
            if (fabsf(rx*tx+ry*ty)<=halfT+0.5f*Cell*(fabsf(tx)+fabsf(ty)) &&
                fabsf(rx*nx+ry*ny)<=halfN+0.5f*Cell*(fabsf(nx)+fabsf(ny)))
                terrain.newObstacle.set(x+Nx*y);
        }
        for (int k=0;k<samplesUsed;++k) {
            const int index=cellAt(samples[k].x,samples[k].y);
            if (index>=0) terrain.newObstacle.set(index);
        }
    }
    void observe(const Frame &f) {
        if (f.number==lastFrame) return;
        lastFrame=f.number; excludedBeams=0;
        const bool trusted=positionError<=32 && slipTime<0.10f && matchConfidence>=45;
        for (int i=0;i<SensorCount;++i) {
            TrackHit &track=tracks[i]; Contact &point=contacts[i];
            point.valid=(f.mask&(1<<i)) && f.beam[i].mm<=MaxRay;
            if (!(f.mask&(1<<i))) { track.seen=0; continue; }
            const Measurement &m=f.beam[i]; const Ray ray=rayFrom(m.origin,i);
            point.x=ray.x+m.mm*ray.dx; point.y=ray.y+m.mm*ray.dy;
            if (trusted) terrain.clearRay(ray,m.mm,terrain.currentFree);
            if (!point.valid || !trusted || closeToFixed(point.x,point.y)) { track.seen=0; continue; }
            const int k=cellAt(point.x,point.y);
            if (k<0) { track.seen=0; continue; }
            int face=0;
            const float expected=terrain.expected(m.origin,i,face);
            const bool shorter=expected>m.mm+55;
            excludedBeams|=1U<<i;
            if (!track.seen || hypotf(point.x-track.centre.x,point.y-track.centre.y)>38) {
                track.centre=point; track.seen=1;
            } else if (track.seen<3) ++track.seen;
            if (track.seen>=3) {
                if (terrain.surveyed.get(k) || shorter || terrain.newObstacle.get(k)) mark(point,ray);
                else terrain.otherObstacle.set(k);
            }
        }
    }
} obstacle;

void trackScan(const Frame &f,uint32_t now) {
    for (const Measurement &b:f.beam) if (int32_t(b.at-registeredAt)<0) return;
    obstacle.observe(f);
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
void onPress(uint32_t now) {
    if (mode==Mode::IDLE && phase==Phase::WAIT_BUTTON) {
        memset(&terrain,0,sizeof(terrain)); memset(&route,0,sizeof(route));
        obstacle={}; memset(contacts,0,sizeof(contacts)); memset(blockedEdge,0,sizeof(blockedEdge));
        excludedBeams=disagreementFrames=0;
        route.clearance=UINT16_MAX;
        resetOdometry({StartX,StartY,StartHeading},now);
        batch={}; stillSince=now; manual='0'; manualSpeed=100;
        matcher.active=false; matchError=-1; matchConfidence=0; attempts=0;
        currentNode=targetNode=0;
        mode=Mode::BLUETOOTH_MAPPING; enter(Phase::START_SCAN,"START_SAMPLING",now);
    } else if (mode==Mode::BLUETOOTH_MAPPING && phase==Phase::MAP_READY && route.locked) {
        mode=Mode::AUTO_NAVIGATION; batch={}; stillSince=now;
        matcher.active=false; attempts=0;
        enter(Phase::INITIAL_LOCALIZATION,"LOCATE_START",now);
    }
}
void onCommand(char c,uint32_t now) {
    if (phase!=Phase::MANUAL_MAPPING) return;
    if (c=='S') {
        manual='0'; request={};
        Frame f=latest;
        if (!fresh(f,now)) f.mask=0;
        if (route.count<1 || travelled<45) { reason="ROUTE_TOO_SHORT"; return; }
        if (!route.record(f,true,true)) { fault("MAP_CAPACITY",now); return; }
        route.locked=true; enter(Phase::MAP_READY,"MAP_LOCKED",now);
    } else if (c=='U' || c=='D' || c=='M' || c=='Q') {
        manualSpeed=c=='M'?100:c=='Q'?155:bound(manualSpeed+(c=='U'?15:-15),45,155);
    } else if (c=='0' || c=='X' || c=='P') {
        if (manual!='0' && !route.record(latest,true,false)) fault("MAP_CAPACITY",now);
        manual='0'; request={};
    } else if (c && strchr("FBLRGIHJ",c)) {
        if (manual!='0' && c!=manual && !route.record(latest,true,true)) {
            fault("MAP_CAPACITY",now); return;
        }
        manual=c; manualAt=now;
    }
}
void onFrame(const Frame &f,uint32_t now) {
    if (phase==Phase::START_SCAN) {
        Scan s{};
        if (!batch.collect(f,stillSince,s)) return;
        if (s.mask!=All) { batch={}; reason="START_INCOMPLETE"; return; }
        Frame start=f;
        for (int i=0;i<SensorCount;++i) {
            start.beam[i].mm=s.mm[i]; start.beam[i].origin=pose;
        }
        start.mask=s.mask;
        terrain.learn(start,true);
        terrain.footprint(pose,0,false,true);
        route.record(start,true,true);
        lastFresh=now;
        enter(Phase::MANUAL_MAPPING,"BT_READY",now);
    } else if (phase==Phase::MANUAL_MAPPING) {
        terrain.learn(f);
        if (!route.record(f,false,false)) fault("MAP_CAPACITY",now);
    } else if (phase==Phase::INITIAL_LOCALIZATION || phase==Phase::RELOCALIZATION) localScan(f,now);
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
void followRoute(float dt,uint32_t now) {
    if (!targetNode || targetNode>=route.count) { fault("BAD_ROUTE",now); return; }
    const Node &target=route.nodes[targetNode];
    const Pose a=route.nodes[currentNode].point(),b=target.point();
    const Edge &edge=route.edges[targetNode-1];
    const float dx=b.x-a.x,dy=b.y-a.y,len=hypotf(dx,dy);
    const float dist=separation(pose,b),angleLeft=fabsf(wrap(b.h-pose.h));
    const float along=len>1?((pose.x-a.x)*dx+(pose.y-a.y)*dy)/len:0;
    trackingError=len>1?((pose.y-a.y)*dx-(pose.x-a.x)*dy)/len:dist;
    const bool last=targetNode==route.count-1;
    if (last && matchConfidence>=45 && acrossFinish(pose,safetyMargin())) { finishRun(now); return; }
    const bool pivot=len<15 && fabsf(wrap(b.h-a.h))>8*Rad;
    const bool atPoint=dist<(target.important?24:32) ||
        (!target.important && along>=len && fabsf(trackingError)<22 && dist<45);
    const bool covered=pivot || edge.length<32 ||
        travelled-edgeStartDistance+42>=fminf(edge.length,len)*0.65f;
    if (blockedEdge[targetNode-1] && !atPoint) { startAvoidance(now); return; }
    if (!last && atPoint && covered && matchConfidence>=45 && (!pivot || angleLeft<8*Rad)) {
        currentNode=targetNode++; edgeStartDistance=travelled;
        headingIntegral=0; progressWatch.reset(now); return;
    }
    Motion desired{};
    float remaining=dist;
    if (last && atPoint) {
        if (pose.x<=FinishLeft || pose.x>=FinishRight || pose.y>FinishLine+180) {
            fault("TAUGHT_END_OUTSIDE_FINISH",now); return;
        }
        remaining=fmaxf(0,pose.y+Footprint(pose.h,safetyMargin()).ey-FinishLine);
        desired=steer(-Pi/2,remaining+45,1,58,dt);
    } else if (pivot && dist<30) {
        desired=steer(b.h,0,0,0,dt); remaining=Track*angleLeft;
    } else {
        const float look=bound(along+70,0,len);
        const float x=len>1?a.x+look*dx/len:b.x, y=len>1?a.y+look*dy/len:b.y;
        const float bearing=atan2f(y-pose.y,x-pose.x);
        const float heading=wrap(bearing+(edge.direction<0?Pi:0));
        const float maxSpeed=edge.direction<0?45:(target.important?92:145);
        desired=steer(heading,hypotf(x-pose.x,y-pose.y),edge.direction,maxSpeed,dt);
        remaining+=20*fabsf(wrap(heading-pose.h));
    }
    if (!progressWatch.advancing(remaining,now)) { fault("NO_TRACK_PROGRESS",now); return; }
    if ((obstacle.first.valid && desired.v>0 &&
         !swept(pose,desired,0.40f+desired.v/350,safetyMargin(),true)) ||
         !issue(desired,dt)) startAvoidance(now);
}

void onMotion(int left, int right, float dt, uint32_t now) {
    if (phase==Phase::FINISHED) { enter(Phase::WAIT_BUTTON,"FINISH_BODY_CLEAR",now); return; }
    if (phase==Phase::START_SCAN && now-phaseAt>6500) fault("START_SAMPLES_UNSTABLE",now);
    if (phase==Phase::START_SCAN || phase==Phase::INITIAL_LOCALIZATION || phase==Phase::RELOCALIZATION) {
        if (left || right || fabsf(yawRate)>1.5f*Rad) {
            stillSince=now; batch={}; matcher.active=false;
        }
        if (phase!=Phase::START_SCAN && now-phaseAt>6500) fault("LOCALIZATION_TIMEOUT",now);
    }
    if (!predicting()) return;
    predict(left,right,dt,now);
    if (phase!=Phase::MANUAL_MAPPING) {
        if (!fresh(latest,now)) {
            request={};
            if (now-lastFresh>750) fault("VL53_STALE_OR_INVALID",now);
            return;
        }
        lastFresh=now;
        for (const Measurement &b:latest.beam)
            if (int32_t(b.at-registeredAt)<0) { request={}; return; }
        if (slipTime>0.22f || positionError>38 || headingError>10*Rad || travelled-lastFixDistance>950) {
            relocalize(slipTime>0.22f?"WHEEL_SLIP":"FIX_REQUIRED",now); return;
        }
        if (phase==Phase::TRACKING) followRoute(dt,now);
        else if (phase==Phase::OBSTACLE_AVOIDANCE) followDetour(dt,now);
        return;
    }
    route.signedDistance+=(left/CountsPerMm[0]+right/CountsPerMm[1])*0.5f;
    if (!terrain.footprint(pose,0,false,true)) { fault("FIELD_LIMIT",now); return; }
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
void work(uint32_t now) {
    if (phase!=Phase::INITIAL_LOCALIZATION && phase!=Phase::RELOCALIZATION &&
        phase!=Phase::TRACKING && phase!=Phase::OBSTACLE_AVOIDANCE) return;
    for (int i=0;i<3 && matcher.active;++i) if (matcher.step()) applyMatch(now);
    if (phase==Phase::OBSTACLE_AVOIDANCE && fresh(latest,now)) detour.expand(now);
}

#ifndef MAP_NAM_HOST_TEST
uint32_t motionRangeAge(const Frame &frame,uint32_t now) {
    uint32_t age=0;
    for (const auto &beam:frame.beam) if (now-beam.at>age) age=now-beam.at;
    return age;
}
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
    ok = ok && prev && now-prev<=45 && ((result[0]>>3)&15)==11 && signal>=32 && raw>0 && raw<8190;
    const uint32_t at = prev-33;
    pending.beam[i]={sampleOrigin(at),at,uint16_t(bound(raw+Mounts[i].opticalBias,1,8189))};
    if (ok) liveMask|=bit;
    else liveMask &= ~bit;
    collected |= bit;
    if (collected!=All) return;
    collected=0; pending.mask=liveMask;
    for (int k=0;k<SensorCount;++k)
        if (now-pending.beam[k].at>95) pending.mask &= ~(1U<<k);
    pending.number=latest.number+1;
    latest=pending;
    onFrame(latest,now);
}
#include "navigation/control_tick.inc"
struct Telemetry {
    static constexpr uint32_t ReportIntervalMs=1000;
    uint32_t lastReportAt=0;
    bool requested=true;
    Phase reported=Phase::INIT;
    void service(uint32_t now) {
        static constexpr char states[][32]={
            "KHOI DONG", "CHO NHAN SW1", "DANG QUET START", "CHAY BLUETOOTH",
            "DA LUU BAN DO", "DINH VI START", "CHAY TU DONG", "TRANH VAT CAN",
            "DINH VI LAI", "DA VE DICH", "DUNG DO LOI"
        };
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
void service() {
    const uint32_t now=millis();
    if (button.press(digitalRead(SwitchPin)==LOW,now))
        onPress(now);
    for (int n=0;n<12 && Serial.available();++n) {
        const char c=char(Serial.read());
        if (c=='?') telemetry.requested=true; else onCommand(c,now);
    }
    digitalWrite(PC13,ledOn(now)?LOW:HIGH);
    bootRanges(now);
    control(millis());
    pollRange(millis());
    work(millis());
    if (phase==Phase::FAULT_STOP || (!request.v && !request.w && armed)) disableDrive(millis());
    telemetry.service(millis());
}
#endif
}

#ifndef MAP_NAM_HOST_TEST
void setup() { MapNam::beginHardware(); }
void loop() { MapNam::service(); }
#endif
