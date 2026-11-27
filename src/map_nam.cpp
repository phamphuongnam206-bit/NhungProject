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

struct Wheel {
    float cps = 0, integral = 0, duty = 0;
    uint32_t lastPulse = 0, stoppedAt = 0, startedAt = 0;
    int direction = 0, wrong = 0, applied = 0;
    void stop(uint32_t now) {
        if (duty > 0) stoppedAt = now;
        duty = integral = 0; wrong = applied = 0;
    }
    bool observe(int ticks, float dt, uint32_t now) {
        cps += dt/(0.055f+dt) * (ticks/dt-cps);
        if (!applied) return true;
        const int forward = ticks*direction;
        if (forward > 0) { lastPulse = now; wrong = 0; }
        if (forward < 0) wrong -= forward;
        return now-lastPulse < 750 && (now-startedAt < 125 || wrong < 24);
    }
    int output(float ref, float cps160, float dt, uint32_t now) {
        if (fabsf(ref) < 5) { stop(now); return 0; }
        const int sign = ref > 0 ? 1 : -1;
        if (duty > 0 && sign != direction) stop(now);
        if (direction && sign != direction && now-stoppedAt < 25) return 0;
        if (!duty) { direction = sign; lastPulse = startedAt = now; }
        const float error = fabsf(ref)-sign*cps;
        const float floor = (now-lastPulse > 55 || now-startedAt < 65) ? 78 : 42;
        const float ff = 42 + 118*fabsf(ref)/cps160 + 0.014f*error;
        if ((error > 0 && ff+integral < 140) || (error < 0 && ff+integral > floor))
            integral = bound(integral + 0.055f*error*dt, -26, 28);
        duty += bound(bound(ff+integral, floor, 140)-duty, -900*dt, 700*dt);
        applied = sign*int(lroundf(duty));
        return applied;
    }
} wheels[2];

struct BiasEstimator {
    unsigned n=0;
    float sum=0, minimum=0, maximum=0;
    bool sample(float raw, bool moved) {
        if (moved || fabsf(raw)>8) { n=0; return false; }
        if (!n) { sum=0; minimum=maximum=raw; }
        minimum=fminf(minimum,raw); maximum=fmaxf(maximum,raw);
        if (maximum-minimum>1.8f) { n=0; return false; }
        sum+=raw;
        if (++n<180) return false;
        gyroBias=sum/n;
        return true;
    }
} bias;

struct Ray { float x, y, dx, dy; };
Ray rayFrom(const Pose &p, int i) {
    const Mount &m=Mounts[i];
    const float u=cosf(p.h), v=sinf(p.h);
    return {p.x+m.forward*u-m.left*v,p.y+m.forward*v+m.left*u,
            cosf(p.h+m.yaw),sinf(p.h+m.yaw)};
}
float separation(const Pose &a, const Pose &b) {
    return hypotf(a.x-b.x,a.y-b.y);
}
float travelled=0, positionError=5, headingError=2*Rad, slipTime=0, lastRate=0;
struct TimedPose { Pose value; uint32_t at; };
TimedPose history[12]{};
uint8_t historyNext=0, historySize=0;
void rememberPose(uint32_t now) {
    history[historyNext]={pose,now};
    historyNext=(historyNext+1)%12;
    if (historySize<12) ++historySize;
}
Pose sampleOrigin(uint32_t at) {
    Pose result=pose;
    for (int i=0;i<historySize;++i) {
        const TimedPose &p=history[(historyNext+11-i)%12];
        result=p.value;
        if (int32_t(at-p.at)>=0) break;
    }
    return result;
}
void resetOdometry(Pose origin, uint32_t now) {
    pose=origin; travelled=0; encoderTotal[0]=encoderTotal[1]=0;
    slipTime=0; positionError=5; headingError=2*Rad; lastRate=yawRate;
    historySize=historyNext=0; rememberPose(now);
}
bool predicting() {
    return phase==Phase::MANUAL_MAPPING || phase==Phase::TRACKING ||
           phase==Phase::OBSTACLE_AVOIDANCE;
}
void predict(int left, int right, float dt, uint32_t now) {
    const float dl=left/CountsPerMm[0], dr=right/CountsPerMm[1];
    const float ds=(dl+dr)*0.5f, encTurn=(dr-dl)/Track;
    const float gyroTurn=(yawRate+lastRate)*0.5f*dt;
    lastRate=yawRate;
    const float mismatch=fabsf(encTurn-gyroTurn);
    const bool slipping=mismatch>0.012f+0.55f*dt;
    slipTime=bound(slipTime+(slipping?dt:-0.45f*dt),0,1);
    const bool still=!left && !right && fabsf(yawRate)<0.45f*Rad;
    const float gyroWeight=slipping?0.96f:0.82f;
    const float dh=still?0:gyroWeight*gyroTurn+(1-gyroWeight)*encTurn;
    pose.x+=ds*cosf(pose.h+dh*0.5f);
    pose.y+=ds*sinf(pose.h+dh*0.5f);
    pose.h=wrap(pose.h+dh);
    travelled+=fabsf(ds);
    headingError=fminf(0.6f,headingError+0.003f*fabsf(dh)+0.00018f*dt+0.027f*mismatch);
    positionError=fminf(150,positionError+0.0035f*fabsf(ds)+0.12f*mismatch+0.012f*slipping);
    encoderTotal[0]+=left; encoderTotal[1]+=right;
    rememberPose(now);
}
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

struct Batch {
    static constexpr int Size=7;
    uint16_t samples[SensorCount][Size]{};
    uint8_t n[SensorCount]{}, frames=0;
    uint32_t last=0;
    bool collect(const Frame &f, uint32_t after, Scan &out) {
        if (!f.number || f.number==last || frames>=Size) return false;
        for (const auto &b : f.beam) if (int32_t(b.at-after)<0) return false;
        last=f.number; ++frames;
        for (int i=0;i<SensorCount;++i) if (f.mask&(1<<i)) samples[i][n[i]++]=f.beam[i].mm;
        if (frames<Size) return false;
        out={};
        for (int i=0;i<SensorCount;++i) {
            if (n[i]<5) continue;
            auto *v=samples[i];
            for (int a=1;a<n[i];++a)
                for (int b=a;b && v[b]<v[b-1];--b) { uint16_t t=v[b]; v[b]=v[b-1]; v[b-1]=t; }
            if (v[n[i]-2]-v[1]>40) continue;
            out.mask|=1U<<i;
            out.mm[i]=v[n[i]/2];
        }
        return true;
    }
} batch;

Pose transform(Pose base,Pose local) {
    const float c=cosf(base.h),s=sinf(base.h);
    return {base.x+c*local.x-s*local.y,base.y+s*local.x+c*local.y,wrap(base.h+local.h)};
}
Pose relative(Pose base,Pose p) {
    const float dx=p.x-base.x,dy=p.y-base.y,c=cosf(base.h),s=sinf(base.h);
    return {dx*c+dy*s,-dx*s+dy*c,wrap(p.h-base.h)};
}
struct ScanMatcher {
    static constexpr int Side=5, Cube=Side*Side*Side, Bad=65000;
    Scan scan{};
    Pose anchor{},offset[SensorCount]{},centres[3]{},best{};
    uint16_t scores[3][Cube]{};
    int level=0,index=0,bestScore=Bad,rival=Bad;
    float radius=50,angle=15*Rad;
    bool active=false,accepted=false;
    uint32_t began=0;
    Pose candidate(int l,int k) const {
        const float step=radius*(l?0.125f:0.5f),dh=angle*(l?0.125f:0.5f);
        Pose p=centres[l];
        p.x+=((k%5)-2)*step; p.y+=((k/5%5)-2)*step;
        p.h=wrap(p.h+((k/25)-2)*dh);
        return p;
    }
    bool separate(Pose p,Pose q) const {
        const float dx=q.x-p.x,dy=q.y-p.y;
        return dx*dx+dy*dy>fmaxf(float(Cell*Cell),radius*radius*0.42f) ||
               fabsf(wrap(p.h-q.h))>fmaxf(5*Rad,angle*0.52f);
    }
    int evaluate(Pose p,bool verify=false) const {
        if (fabsf(p.x-anchor.x)>radius+0.1f || fabsf(p.y-anchor.y)>radius+0.1f ||
            fabsf(wrap(p.h-anchor.h))>angle+0.001f || !legal(p,Footprint(p.h,0))) return Bad;
        float errors[4]={200,200,200,200},j[4][3]{};
        int support=0,axes=0;
        for (int i=0;i<SensorCount;++i) {
            if (!(scan.mask&(1<<i)) || scan.mm[i]>MaxRay) continue;
            const Pose origin=transform(p,offset[i]);
            const Ray ray=rayFrom(origin,i);
            const int endpoint=cellAt(ray.x+ray.dx*scan.mm[i],ray.y+ray.dy*scan.mm[i]);
            if (endpoint>=0 && (terrain.newObstacle.get(endpoint) || terrain.otherObstacle.get(endpoint)))
                continue;
            int face=0;
            const float range=terrain.expected(origin,i,face);
            if (range<0 || !face) continue;
            const float err=fabsf(range-scan.mm[i]);
            errors[i]=err;
            if (err>(verify?36:95)) continue;
            axes|=face;
            if (verify) {
                const float hx=ray.x-p.x+range*ray.dx,hy=ray.y-p.y+range*ray.dy;
                const float x=face==1?1.f:0.f,y=face==2?1.f:0.f;
                const float a=(-hy*x+hx*y)/180.f;
                const float norm=sqrtf(1+a*a);
                j[support][0]=x/norm; j[support][1]=y/norm; j[support][2]=a/norm;
            }
            ++support;
        }
        if (support<3 || axes!=3) return Bad;
        if (verify) {
            float rank=0;
            for (int a=0;a<support-2;++a) for (int b=a+1;b<support-1;++b) {
                for (int c=b+1;c<support;++c) {
                    const float *u=j[a],*v=j[b],*w=j[c];
                    const float det=u[0]*(v[1]*w[2]-v[2]*w[1])-
                        u[1]*(v[0]*w[2]-v[2]*w[0])+u[2]*(v[0]*w[1]-v[1]*w[0]);
                    rank=fmaxf(rank,fabsf(det));
                }
            }
            if (rank<0.065f) return Bad;
        }
        for (int i=1;i<4;++i) for (int k=i;k && errors[k]<errors[k-1];--k) {
            const float t=errors[k]; errors[k]=errors[k-1]; errors[k-1]=t;
        }
        return int(lroundf((errors[0]+errors[1]+errors[2]+fminf(errors[3],48)*0.45f)*4/3.45f));
    }
    void start(Pose origin,Scan s,float mm,float radians,uint32_t now) {
        anchor=centres[0]=best=origin; scan=s; radius=mm; angle=radians;
        level=index=0; bestScore=rival=Bad; active=true; accepted=false; began=now;
        for (auto &o:offset) o={};
    }
    int alternative() const {
        int kbest=-1,score=Bad;
        for (int k=0;k<Cube;++k) {
            if (scores[0][k]<score && separate(candidate(0,k),best)) {
                score=scores[0][k]; kbest=k;
            }
        }
        return kbest;
    }
    bool step() {
        if (!active) return false;
        const Pose p=candidate(level,index);
        const int score=evaluate(p);
        scores[level][index++]=score;
        if (score<bestScore) { best=p; bestScore=score; }
        if (index<Cube) return false;
        if (level==0 && bestScore<Bad) {
            centres[1]=best;
            const int k=alternative();
            centres[2]=k>=0?candidate(0,k):best;
            level=1; index=0; return false;
        }
        if (level==1 && separate(centres[2],centres[1])) { level=2; index=0; return false; }
        for (int l=0;l<=level;++l) for (int k=0;k<Cube;++k) {
            if (scores[l][k]<rival && separate(candidate(l,k),best)) rival=scores[l][k];
        }
        active=false;
        accepted=bestScore<=92 && rival-bestScore>=14 && evaluate(best,true)<Bad;
        return true;
    }
} matcher;
uint8_t currentNode=0,targetNode=0,attempts=0;
uint32_t stillSince=0;
float matchError=-1;
int matchConfidence=0;
uint32_t lastMatchAt=0, lastMatchTry=0, lastFresh=0;
uint32_t registeredAt=0;
float lastFixDistance=0;
uint8_t excludedBeams=0;
uint8_t disagreementFrames=0;
uint8_t blockedEdge[NodeLimit-1]{};

void relocalize(const char *why,uint32_t now) {
    matcher.active=false; batch={}; attempts=0; stillSince=now;
    enter(Phase::RELOCALIZATION,why,now);
}

bool scanCompatible(const Frame &f,const Scan &s,uint32_t now) {
    int agreeing=0;
    for (int i=0;i<SensorCount;++i) {
        if ((s.mask&f.mask&liveMask&(1<<i)) && now-f.beam[i].at<FreshMs &&
            abs(int(s.mm[i])-int(f.beam[i].mm))<32) ++agreeing;
    }
    return agreeing>=3;
}
void localScan(const Frame &f,uint32_t now) {
    if (matcher.active) return;
    Scan s{};
    if (!batch.collect(f,stillSince,s)) return;
    batch={};
    if (countBits(s.mask)<3) { reason="LOC_INVALID_RANGES"; return; }
    const bool start=phase==Phase::INITIAL_LOCALIZATION;
    matcher.start(start?route.nodes[0].point():pose,s,50,15*Rad,now);
}
void applyMatch(uint32_t now) {
    matchError=matcher.bestScore<ScanMatcher::Bad?matcher.bestScore*0.25f:-1;
    if (phase==Phase::TRACKING || phase==Phase::OBSTACLE_AVOIDANCE) {
        if (!matcher.accepted || now-matcher.began>500 || !fresh(latest,now) ||
            (excludedBeams & matcher.scan.mask) || slipTime>0.10f) return;
        const Pose corrected=transform(matcher.best,relative(matcher.anchor,pose));
        const float dx=corrected.x-pose.x,dy=corrected.y-pose.y;
        const float gain=fminf(0.30f,3.5f/fmaxf(1,hypotf(dx,dy)));
        const Pose step{pose.x+gain*dx,pose.y+gain*dy,
            wrap(pose.h+bound(gain*wrap(corrected.h-pose.h),-0.7f*Rad,0.7f*Rad))};
        if (!terrain.footprint(step,0,true)) return;
        pose=step; rememberPose(now);
        positionError=fmaxf(8,0.82f*positionError+0.18f*(matchError+separation(pose,corrected)));
        headingError=fmaxf(2*Rad,0.85f*headingError);
        lastFixDistance=travelled; lastMatchAt=now; matchConfidence=int(bound(94-matchError,1,98));
        return;
    }
    matchConfidence=matcher.accepted?int(bound(94-matchError,1,98)):0;
    if (!matcher.accepted || !scanCompatible(latest,matcher.scan,now)) {
        reason=matcher.bestScore<ScanMatcher::Bad?"LOC_AMBIGUOUS":"LOC_NO_LANDMARKS";
        if (++attempts>=3) fault(reason,now);
        return;
    }
    if (phase==Phase::INITIAL_LOCALIZATION) {
        resetOdometry(matcher.best,now); currentNode=0; targetNode=1;
    } else {
        pose=matcher.best; historySize=0; rememberPose(now); slipTime=0;
    }
    positionError=bound(7+0.4f*matchError,7,18); headingError=2.5f*Rad;
    lastFixDistance=travelled; registeredAt=lastMatchAt=lastMatchTry=lastFresh=now;
    disagreementFrames=0;
    attempts=0; enter(Phase::TRACKING,"LOCALIZED",now);
}

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
Pose advance(Pose p,Motion m,float dt) {
    const float dh=m.w*dt;
    p.x+=m.v*dt*cosf(p.h+dh*0.5f); p.y+=m.v*dt*sinf(p.h+dh*0.5f);
    p.h=wrap(p.h+dh); return p;
}
bool swept(Pose p,Motion m,float seconds,float margin,bool known) {
    const float reach=(fabsf(m.v)+fabsf(m.w)*(HalfLength+HalfWidth))*seconds;
    const int steps=1+int(reach/8);
    const float dt=seconds/steps;
    for (int i=0;i<=steps;++i) {
        if (!terrain.footprint(p,margin+2,known) ||
            touchesReturn(p,Footprint(p.h,margin+2))) return false;
        p=advance(p,m,dt);
    }
    return true;
}
bool nearFront(const Frame &f, float reserve) {
    for (int i=0;i<SensorCount;++i) {
        const float buffer=(i==1 || i==2)?reserve:reserve*0.60f;
        if ((f.mask&(1<<i)) && f.beam[i].mm < (HalfLength-Mounts[i].forward)+buffer) return true;
    }
    return false;
}

float trackingError=0, headingIntegral=0;
float safetyMargin() {
    return positionError+HalfLength*sinf(headingError);
}
float observedSpeed() {
    return (wheels[0].cps/CountsPerMm[0]+wheels[1].cps/CountsPerMm[1])*0.5f;
}
float forwardLimit() {
    const float mm=fminf(latest.beam[1].mm,latest.beam[2].mm)-(HalfLength-Mounts[1].forward)-30;
    if (mm<=0) return 0;
    return bound(sqrtf(2*320*mm+45*45)-45,0,155);
}
Motion steer(float desiredHeading,float distance,int direction,float speed,float dt) {
    const float error=wrap(desiredHeading-pose.h);
    headingIntegral=bound(headingIntegral+error*dt,-0.28f,0.28f);
    const float turn=2.25f*error+0.28f*headingIntegral-0.18f*yawRate;
    if (fabsf(error)>62*Rad || !direction) return {0,bound(turn,-1.05f,1.05f)};
    float v=fminf(speed,40+distance*0.9f)/(1+1.5f*fabsf(error));
    if (direction>0) v=fminf(v,forwardLimit());
    if (v<1) return {};
    const float w=1.7f*v*sinf(error)/fmaxf(distance,55)+0.55f*error+
                  0.22f*headingIntegral-0.13f*yawRate;
    const float limit=fminf(1.35f,1.65f*v/Track);
    return {direction*v,bound(w,-limit,limit)};
}
bool issue(Motion desired,float dt) {
    if (!desired.v && !desired.w) { request={}; return false; }
    const float margin=safetyMargin();
    const Motion actual{observedSpeed(),yawRate};
    const float stopTime=0.12f+fabsf(actual.v)/320;
    if ((fabsf(actual.v)>15 || fabsf(actual.w)>0.2f) && !swept(pose,actual,stopTime,margin,true)) {
        request={}; return false;
    }
    const float rising=fabsf(desired.v)>fabsf(request.v)?230:450;
    Motion next{request.v+bound(desired.v-request.v,-rising*dt,rising*dt),
                request.w+bound(desired.w-request.w,-3.2f*dt,3.2f*dt)};
    if (fabsf(next.v)<0.5f) next.v=0;
    if (!swept(pose,next,0.18f+fabsf(next.v)/320,margin,true)) { request={}; return false; }
    request=next; return true;
}
struct ProgressWatch {
    float best=1e6f;
    uint32_t improved=0;
    void reset(uint32_t now) { best=1e6f; improved=now; }
    bool advancing(float distance,uint32_t now) {
        if (distance<best-3) { best=distance; improved=now; }
        return now-improved<4500;
    }
} progressWatch;
float edgeStartDistance=0;
struct Detour {
    static constexpr int Limit=288,Depth=36;
    static constexpr int GuideSide=19,GuideCell=45;
    uint8_t guide[GuideSide*GuideSide];
    int qRead=0,qWrite=0,guideX=0,guideY=0;
    bool guiding=false;
    struct Vertex {
        int16_t x,y,h1000,parent;
        uint16_t cost,estimate;
        uint8_t action,closed;
        Pose point() const { return {float(x),float(y),h1000*0.001f}; }
    };
    union {
        Vertex vertices[Limit];
        uint16_t queue[GuideSide*GuideSide];
    };
    uint16_t chain[Depth];
    int used=0,current=-1,action=0,pathSize=0,stepIndex=0;
    uint8_t rejoin=0;
    bool searching=false,following=false;
    uint32_t started=0;
    Pose goal{};
    static Motion primitive(int action) {
        switch (action) {
            case 0: return {58,0};
            case 1: return {55,0.48f};
            case 2: return {55,-0.48f};
            case 3: return {0,0.70f};
            default: return {0,-0.70f};
        }
    }
    void reset() {
        searching=following=guiding=false; used=pathSize=stepIndex=0; current=-1; action=0;
    }
    int guideCell(Pose p) const {
        const float x=(p.x-guideX)/GuideCell,y=(p.y-guideY)/GuideCell;
        return x>=0 && y>=0 && x<GuideSide && y<GuideSide ? int(y)*GuideSide+int(x) : -1;
    }
    float toGoal(Pose p) const {
        const float direct=separation(p,goal);
        const int k=guideCell(p);
        if (k<0 || guide[k]>=254) return direct+150;
        return fmaxf(direct,guide[k]*float(GuideCell));
    }
    void guideStep() {
        if (qRead==qWrite) {
            guiding=false; add(pose,-1,0,0); return;
        }
        const int k=queue[qRead++];
        for (int dy=-1;dy<=1;++dy) for (int dx=-1;dx<=1;++dx) {
            const int x=k%GuideSide+dx,y=k/GuideSide+dy;
            if (x<0 || x>=GuideSide || y<0 || y>=GuideSide) continue;
            const int idx=y*GuideSide+x;
            if (guide[idx]!=255) continue;
            const Pose p{guideX+(x+0.5f)*GuideCell,guideY+(y+0.5f)*GuideCell,goal.h};
            if (terrain.footprint(p,safetyMargin(),true)) {
                guide[idx]=guide[k]+1; queue[qWrite++]=idx;
            } else guide[idx]=254;
        }
    }
    bool chooseRejoin() {
        int best=-1;
        const float initial=route.nodes[currentNode].distance;
        for (int i=targetNode;i<route.count;++i) {
            const Node &n=route.nodes[i];
            if (n.distance-initial>650) break;
            const Pose p=n.point();
            const bool clear=terrain.footprint(p,safetyMargin()+2,true) &&
                             !touchesReturn(p,Footprint(p.h,safetyMargin()+2));
            if (!clear) best=-1;
            else if (separation(p,pose)>30) best=i;
            if (n.important || (best>=0 && n.distance-initial>390)) break;
        }
        if (best<0) return false;
        rejoin=uint8_t(best); goal=route.nodes[rejoin].point(); return true;
    }
    void add(Pose p,int parent,int act,int cost) {
        const int16_t x=lroundf(p.x),y=lroundf(p.y),h=lroundf(p.h*1000);
        const int estimate=cost+int(1.5f*toGoal(p)+18*fabsf(wrap(goal.h-p.h)));
        int index=used;
        for (int i=0;i<used;++i) {
            const Vertex &v=vertices[i];
            if (abs(x-v.x)>21 || abs(y-v.y)>21 || fabsf(wrap(p.h-v.h1000*0.001f))>0.21f) continue;
            if (v.closed || v.cost<=cost) return;
            index=i; break;
        }
        if (index>=Limit) return;
        if (index==used) ++used;
        vertices[index]={x,y,h,int16_t(parent),uint16_t(cost),uint16_t(estimate),uint8_t(act),0};
    }
    bool begin(uint32_t now) {
        reset(); started=now;
        if (!chooseRejoin() || !terrain.footprint(pose,safetyMargin()+2,true)) return false;
        guideX=int((pose.x+goal.x)/2)-GuideSide*GuideCell/2;
        guideY=int((pose.y+goal.y)/2)-GuideSide*GuideCell/2;
        const int cell=guideCell(goal);
        if (cell<0) return false;
        memset(guide,255,sizeof(guide)); guide[cell]=0;
        qRead=0; qWrite=1; queue[0]=cell; guiding=searching=true; return true;
    }
    bool connected(Pose p) const {
        const float d=separation(p,goal);
        if (d>38) return false;
        if (d<9) return true;
        const float angle=atan2f(goal.y-p.y,goal.x-p.x);
        if (fabsf(wrap(angle-p.h))>0.70f) return false;
        return swept(p,{0,wrap(angle-p.h)},1,safetyMargin(),true) &&
            swept({p.x,p.y,angle},{d,0},1,safetyMargin(),true);
    }
    bool extract() {
        for (int v=current;vertices[v].parent>=0;v=vertices[v].parent) {
            if (pathSize>=Depth) return false;
            chain[pathSize++]=uint16_t(v);
        }
        searching=false; following=true; stepIndex=pathSize-1; return true;
    }
    void expand(uint32_t now) {
        if (!searching) return;
        if (guiding) { guideStep(); return; }
        if (current<0) {
            int score=65536;
            for (int i=0;i<used;++i) {
                if (!vertices[i].closed && vertices[i].estimate<score) {
                    current=i; score=vertices[i].estimate;
                }
            }
            if (current<0 || now-started>4300) { searching=false; fault("NO_SAFE_DETOUR",now); return; }
            vertices[current].closed=1; action=0;
            if (connected(vertices[current].point())) {
                if (!extract()) fault("DETOUR_CAPACITY",now);
                progressWatch.reset(now); return;
            }
        }
        const Pose p=vertices[current].point();
        const Motion m=primitive(action);
        if (swept(p,m,1,safetyMargin()+5,true)) {
            const Pose q=advance(p,m,1);
            const int tight=terrain.footprint(q,safetyMargin()+13,true)?0:48;
            const int cost=(action==0?58:action<3?64:85)+tight;
            add(q,current,action,vertices[current].cost+cost);
        }
        if (++action==5) current=-1;
    }
} detour;

uint8_t detourRetries=0;
void finishRun(uint32_t now) {
    request={}; manual='0'; matcher.active=false; batch={}; attempts=0;
    headingIntegral=trackingError=edgeStartDistance=lastFixDistance=0;
    travelled=slipTime=lastRate=0; encoderTotal[0]=encoderTotal[1]=0;
    disagreementFrames=excludedBeams=0;
    currentNode=targetNode=0; progressWatch.reset(now);
    detour.reset(); detourRetries=0;
    for (auto &w:wheels) w.stop(now);
    button.armed=false; button.changed=now;
    mode=Mode::IDLE; enter(Phase::FINISHED,"FINISH_BODY_CLEAR",now);
}
void startAvoidance(uint32_t now) {
    if (targetNode>0) blockedEdge[targetNode-1]=1;
    matcher.active=false; detour.reset(); detourRetries=0; headingIntegral=0;
    enter(Phase::OBSTACLE_AVOIDANCE,"ROUTE_OBSTRUCTED",now);
}
void followDetour(float dt,uint32_t now) {
    if (!detour.searching && !detour.following) {
        request={};
        if (now-phaseAt<150 || fabsf(observedSpeed())>16 || fabsf(yawRate)>0.23f) {
            if (now-phaseAt>1600) fault("DETOUR_NOT_STILL",now);
            return;
        }
        if (!detour.begin(now)) { fault("NO_KNOWN_REJOIN",now); return; }
        reason="SEARCHING_DETOUR"; return;
    }
    if (!detour.following) { request={}; return; }
    const bool joining=detour.stepIndex<0;
    const Detour::Vertex &vertex=detour.vertices[joining?0:detour.chain[detour.stepIndex]];
    const Pose dest=joining?detour.goal:vertex.point();
    const bool pivot=!joining && vertex.action>=3;
    const float dist=separation(pose,dest),dh=wrap(dest.h-pose.h);
    if ((pivot && fabsf(dh)<3*Rad && dist<26) || (!pivot && dist<(joining?18:9))) {
        headingIntegral=0; progressWatch.reset(now);
        if (!joining) { --detour.stepIndex; return; }
        targetNode=detour.rejoin;
        currentNode=targetNode-1; edgeStartDistance=travelled-route.edges[currentNode].length;
        detour.reset(); detourRetries=0; enter(Phase::TRACKING,"REJOINED",now); return;
    }
    const float heading=pivot?dest.h:atan2f(dest.y-pose.y,dest.x-pose.x);
    Motion next{};
    if (pivot || joining) next=steer(heading,dist,pivot?0:1,55,dt);
    else {
        const Pose start=detour.vertices[vertex.parent].point();
        const Motion arc=detour.primitive(vertex.action);
        const float dx=dest.x-start.x,dy=dest.y-start.y;
        const float q=bound(((pose.x-start.x)*dx+(pose.y-start.y)*dy)/(dx*dx+dy*dy),0,1);
        const Pose ref=advance(start,arc,q);
        const float cross=(pose.y-ref.y)*cosf(ref.h)-(pose.x-ref.x)*sinf(ref.h);
        const float v=fminf(55,forwardLimit());
        const float w=v*arc.w/arc.v + 3.0f*wrap(ref.h-pose.h)-0.045f*cross;
        next={v,bound(w,-1.2f,1.2f)};
    }
    if (!progressWatch.advancing(dist+35*fabsf(wrap(heading-pose.h)),now) || !issue(next,dt)) {
        request={}; detour.reset(); phaseAt=now;
        if (++detourRetries>2) fault("DETOUR_BLOCKED",now);
        else reason="DETOUR_REPLAN";
    }
}
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
constexpr uint32_t In1[] = {PB14,PB12}, In2[] = {PB15,PB13}, Pwm[] = {PB9,PB8};
constexpr uint32_t Shut[] = {PA2,PA3,PA4,PA5};
constexpr uint8_t Addr[] = {0x30,0x31,0x32,0x33};
constexpr uint32_t SwitchPin=PB1;
TwoWire rangeBus(PB11, PB10);
VL53L0X tof[SensorCount];
uint8_t mpu = 0, bootSensor = 0;
bool sensorReleased = false, rangesRunning = false;
uint32_t bootStepAt = 0, lastTick = 0, lastPoll = 0;
uint16_t previousCount[2]{};
Frame pending{};
uint8_t collected = 0, nextPoll = 0;
uint32_t lastProbe[SensorCount]{};
volatile uint32_t controlGuardAt = 0, rangeGuardAt = 0;
volatile bool armed = false, tripped = false, tickSeen = false;
extern "C" void HAL_SYSTICK_Callback(void) {
    tickSeen=true;
    if (armed && (HAL_GetTick()-controlGuardAt > 65 || HAL_GetTick()-rangeGuardAt > FreshMs)) {
        GPIOB->BRR = 1U;
        armed = false;
        tripped = true;
    }
}
void low(uint32_t pin) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
void disableDrive(uint32_t now) {
    const bool wasArmed=armed;
    digitalWrite(PB0, LOW); armed = false;
    for (int i=0; i<2; ++i) {
        if (wasArmed || wheels[i].applied) analogWrite(Pwm[i], 0);
        wheels[i].stop(now);
    }
}
void drive(uint32_t now, float dt) {
    if (tripped || (!request.v && !request.w) || !fresh(latest, now)) {
        disableDrive(now); return;
    }
    uint32_t age = 0;
    for (const auto &b : latest.beam) if (now-b.at > age) age = now-b.at;
    bool driven = false;
    for (int i=0; i<2; ++i) {
        const float ref = (request.v + (i ? 0.5f : -0.5f)*Track*request.w)*CountsPerMm[i];
        const int pwm = wheels[i].output(ref, i ? 5563 : 5666, dt, now);
        const bool in1High = pwm && ((pwm > 0) == (i == 1));
        digitalWrite(In1[i], in1High ? HIGH : LOW);
        digitalWrite(In2[i], pwm && !in1High ? HIGH : LOW);
        analogWrite(Pwm[i], abs(pwm));
        driven |= pwm != 0;
    }
    const uint32_t flags = __get_PRIMASK(); __disable_irq();
    controlGuardAt = now; rangeGuardAt = now-age;
    armed = driven && !tripped && HAL_GetTick()-rangeGuardAt <= FreshMs;
    digitalWrite(PB0, armed ? HIGH : LOW); __set_PRIMASK(flags);
}
bool setupTimer(TIM_TypeDef *t) {
    t->CR1 = t->CR2 = t->DIER = t->CCER = t->PSC = 0;
    t->ARR = 65535; t->CCMR1 = 1U|(8U<<4)|(1U<<8)|(8U<<12);
    t->SMCR = 3; t->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    t->EGR = TIM_EGR_UG; t->CNT = t->SR = 0; t->CR1 = TIM_CR1_CEN;
    return (t->CR1 & 1) && (t->SMCR & 7) == 3 && t->ARR == 65535 && !t->DIER;
}
bool initEncoders() {
    __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_DisableIRQ(TIM2_IRQn); HAL_NVIC_DisableIRQ(TIM3_IRQn);
    __HAL_RCC_TIM2_FORCE_RESET(); __HAL_RCC_TIM2_RELEASE_RESET();
    __HAL_RCC_TIM3_FORCE_RESET(); __HAL_RCC_TIM3_RELEASE_RESET();
    const uint32_t remap = AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP;
    AFIO->MAPR = (AFIO->MAPR & ~(remap | AFIO_MAPR_SWJ_CFG)) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    pinMode(PA6,INPUT_PULLUP); pinMode(PA7,INPUT_PULLUP);
    pinMode(PA0,INPUT_PULLUP); pinMode(PA1,INPUT_PULLUP);
    return !((remap|AFIO_MAPR_TIM4_REMAP) & AFIO->MAPR) && setupTimer(TIM3) && setupTimer(TIM2);
}
bool readRegs(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t n) {
    bus.beginTransmission(addr); bus.write(reg);
    if (bus.endTransmission(false) || bus.requestFrom(addr,n) != n) return false;
    for (int i=0; i<n; ++i) { int b = bus.read(); if (b<0) return false; data[i]=uint8_t(b); }
    return true;
}
bool writeReg(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t value) {
    bus.beginTransmission(addr); bus.write(reg); bus.write(value);
    return bus.endTransmission() == 0;
}
bool initImu() {
    for (mpu=0x68; mpu<=0x69; ++mpu) {
        uint8_t who = 0;
        if (readRegs(Wire,mpu,0x75,&who,1) && who==0x68)
            return writeReg(Wire,mpu,0x6b,1) && writeReg(Wire,mpu,0x6c,0) &&
                   writeReg(Wire,mpu,0x1a,3) && writeReg(Wire,mpu,0x19,9) &&
                   writeReg(Wire,mpu,0x1b,0);
    }
    return false;
}
void bootRanges(uint32_t now) {
    if (rangesRunning || phase!=Phase::INIT || now-bootStepAt<50) return;
    const uint8_t i = bootSensor;
    if (!sensorReleased) {
        pinMode(Shut[i],INPUT); sensorReleased = true; bootStepAt=now; return;
    }
    VL53L0X &s=tof[i];
    s.setBus(&rangeBus); s.setTimeout(45);
    rangeBus.beginTransmission(0x29);
    bool ok = rangeBus.endTransmission()==0 && s.init() && !s.last_status;
    if (ok) {
        s.setAddress(Addr[i]);
        rangeBus.beginTransmission(Addr[i]);
        ok = !s.last_status && !rangeBus.endTransmission() && s.setMeasurementTimingBudget(33000);
    }
    if (!ok) { low(Shut[i]); fault("VL53_INIT",now); return; }
    sensorReleased=false; bootStepAt=now;
    if (++bootSensor == SensorCount) {
        for (auto &v : tof) { v.startContinuous(); if (v.last_status) { fault("VL53_START",now); return; } }
        rangesRunning=true;
    }
}
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
void control(uint32_t now) {
    const uint32_t elapsed=now-lastTick;
    if (elapsed<10) return;
    lastTick=now;
    const float dt=elapsed*0.001f;
    const uint16_t hardware[2]={uint16_t(TIM3->CNT),uint16_t(TIM2->CNT)};
    int delta[2];
    for (int i=0;i<2;++i) {
        delta[i]=EncoderSign[i]*counterDelta(hardware[i],previousCount[i]);
        previousCount[i]=hardware[i];
        if (!wheels[i].observe(delta[i],dt,now)) fault(i?"WHEEL_R_STALL_DIR":"WHEEL_L_STALL_DIR",now);
    }
    if (tripped) fault("CONTROL_OR_RANGE_TIMEOUT",now);
    if (phase==Phase::FAULT_STOP) { disableDrive(now); return; }
    uint8_t z[3]{};
    if (!readRegs(Wire,mpu,0x47,z,3)) { fault("MPU_READ",now); disableDrive(now); return; }
    const float raw=int16_t(uint16_t(z[0]<<8)|z[1])/131.0f;
    yawRate=(raw-gyroBias)*Rad*GyroSign*GyroScale;
    if (phase==Phase::INIT) {
        if (rangesRunning && bias.sample(raw,delta[0]||delta[1]) && now-bootAt>=1200) {
            if (!tickSeen) fault("SYSTICK_GUARD",now);
            else enter(Phase::WAIT_BUTTON,"READY",now);
        }
        else if (now-bootAt>12000) fault("GYRO_NOT_STILL",now);
    } else {
        onMotion(delta[0],delta[1],dt,now);
    }
    drive(millis(),dt);
}
struct Telemetry {
    char bytes[490]{};
    unsigned size=0,sent=0;
    bool requested=true;
    const char *lastReason=nullptr;
    void put(char c) { if (size<sizeof(bytes)) bytes[size++]=c; }
    void label(const char *s) { while (*s) put(*s++); }
    void number(int32_t value) {
        uint32_t n=uint32_t(value);
        if (value<0) { put('-'); n=0U-n; }
        char digits[10]; int len=0;
        do { digits[len++]=char('0'+n%10); n/=10; } while (n);
        while (len) put(digits[--len]);
    }
    void field(const char *name,int32_t value) { label(name); number(value); }
    void snapshot(uint32_t now) {
        static const char *const modes[]={"IDLE","BLUETOOTH_MAPPING","AUTO_NAVIGATION"};
        static const char *const phases[]={"INIT","WAIT_BUTTON","START_SCAN","MANUAL_MAPPING","MAP_READY",
            "INITIAL_LOCALIZATION","TRACKING","OBSTACLE_AVOIDANCE","RELOCALIZATION","FINISHED","FAULT_STOP"};
        size=sent=0; lastReason=reason; requested=false;
        label(modes[int(mode)]); put(' '); label(phases[int(phase)]); put(' '); label(reason);
        field(" Xmm=",lroundf(pose.x)); field(" Ymm=",lroundf(pose.y)); field(" Hdeg10=",lroundf(pose.h*10/Rad));
        field(" Vmm_s=",lroundf(wheels[0].cps/CountsPerMm[0])); field(",",lroundf(wheels[1].cps/CountsPerMm[1]));
        field(" ENC=",encoderTotal[0]); field(",",encoderTotal[1]);
        field(" RAW16=",uint16_t(TIM3->CNT)); field(",",uint16_t(TIM2->CNT));
        field(" GZdeg10_s=",lroundf(yawRate*10/Rad)); field(" Bias100=",lroundf(gyroBias*100));
        label(" J8_11mm=");
        for (int i=0;i<SensorCount;++i) {
            if (i) put(',');
            const bool ok=(latest.mask&liveMask&(1<<i)) && now-latest.beam[i].at<=FreshMs;
            number(ok?int(latest.beam[i].mm):-1);
        }
        field(" node=",currentNode); field(" target=",targetNode); field(" N=",route.count);
        field(" E=",route.count?route.count-1:0); field(" landmarks=",terrain.landmarks);
        field(" cross_mm=",lroundf(trackingError)); field(" scan_mm=",lroundf(matchError));
        field(" confidence=",matchConfidence); field(" posErr_mm=",lroundf(positionError));
        field(" yawErr_deg10=",lroundf(headingError*10/Rad));
        field(" fixAge_ms=",lastMatchAt?int32_t(now-lastMatchAt):-1);
        field(" edgeBlocked=",targetNode?blockedEdge[targetNode-1]:0);
        field(" new=",obstacle.first.valid);
        if (obstacle.first.valid) { field(" hit_mm=",lroundf(obstacle.first.x)); field(",",lroundf(obstacle.first.y)); }
        field(" PWM=",wheels[0].applied); field(",",wheels[1].applied);
        field(" SW1=",button.stable);
        put('\r'); put('\n');
    }
    void service(uint32_t now) {
        if (sent==size && (requested || lastReason!=reason)) snapshot(now);
        const int room=Serial.availableForWrite();
        if (room>0 && sent<size) {
            const unsigned n=unsigned(fminf(24,fminf(room,size-sent)));
            sent+=Serial.write(reinterpret_cast<const uint8_t *>(bytes+sent),n);
        }
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
