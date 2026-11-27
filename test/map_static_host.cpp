#define MAP_NAM_HOST_TEST
#include "../src/map_static.cpp"
#include <cassert>
#include <cstdlib>
#include <cstdio>

using namespace MapNam;

static Scan scanAt(Pose truth,const StaticMap::Obstacle &extra) {
    Scan scan{};
    for (int i=0;i<SensorCount;++i) {
        int face;
        float d=terrain.expected(truth,i,face);
        if (extra.id) {
            const float hit=intersect(rayFrom(truth,i),extra,face);
            if (hit>=0 && (d<0 || hit<d)) d=hit;
        }
        if (d>=0) { scan.mask|=1<<i; scan.mm[i]=lroundf(d); }
    }
    return scan;
}

static void localizationGrid() {
    int accepted=0, rejected=0, illegal=0;
    for (int x=-50;x<=50;x+=25) for (int y=-50;y<=50;y+=25) for (int h=-15;h<=15;h+=5) {
        Pose truth{StartX+x,StartY+y,StartHeading+h*Rad};
        if (!terrain.footprint(truth,0,true)) { ++illegal; continue; }
        matcher.start({StartX,StartY,StartHeading},scanAt(truth,{}),SearchRadius,SearchAngle,0);
        while (!matcher.step()) {}
        if (!startConsistent()) { ++rejected; continue; }
        assert(separation(truth,matcher.best)<8 && fabsf(wrap(truth.h-matcher.best.h))<3*Rad);
        ++accepted;
    }
    assert(accepted>0 && rejected>0);
    printf("START accepted=%d rejected=%d physically invalid=%d\n",accepted,rejected,illegal);
}

static void safetyTests() {
    using namespace StaticMap;
    const int coordinates[][2]={{50,150},{550,250},{300,350},{550,450},{50,550},{50,750},
        {350,900},{450,1150},{650,750},{750,650},{850,1000},{950,650},{1150,1150},
        {1250,850},{1450,450},{1450,150},{1750,250}};
    for (int i=0;i<17;++i) {
        const auto &b=i<FIXED_COUNT?FIXED_OBSTACLES[i]:POSSIBLE_OBSTACLES[i-FIXED_COUNT];
        assert(b.id==i+1 && b.x==coordinates[i][0] && b.y==coordinates[i][1]);
        assert(b.width==100 && b.length==100);
    }
    assert(MAP_WIDTH==1800 && MAP_HEIGHT==1200 && FINISH_LINE==0);
    int face=0;
    assert(intersect({300,120,0,1},FIXED_OBSTACLES[2],face)==180 && face==2);
    assert(intersect({50,150,1,0},FIXED_OBSTACLES[2],face)<0);
    assert(!terrain.footprint({600,200,0},0,true));
    assert(!terrain.footprint({300,0,StartHeading},0,true));
    assert(!acrossFinish({1600,-60,-Pi/2},0) && acrossFinish({1600,-75,-Pi/2},0));
    Frame f{}; f.number=1; f.observed=All; liveMask=All;
    for (auto &beam:f.beam) beam.at=100;
    f.mask=0; assert(fresh(f,120)); // Long, qualified returns are not landmarks.
    phase=Phase::WAIT_BUTTON; phaseAt=0;
    assert(ledOn(0) && !ledOn(350) && ledOn(700));
    phase=Phase::TRACKING; mode=Mode::AUTO_NAVIGATION;
    assert(ledOn(0) && ledOn(350));
    mode=Mode::IDLE;
    f.beam[3].at=0;
    assert(motionRangeAge(f,120)==20);
    liveMask=7; assert(fresh(f,120));
    request.w=-1; assert(!fresh(f,120)); request={};
    liveMask=13; assert(!fresh(f,120)); liveMask=All; assert(!fresh(f,300));
    loadRoute(); phase=Phase::WAIT_BUTTON; onPress(100);
#if defined(COMPETITION_2_LAPS) && COMPETITION_2_LAPS
    assert(phase==Phase::MANUAL_MAPPING); onCommand('F',120);
    onMotion(0,0,0.01f,800); assert(manual=='0');
    onCommand('S',820); assert(phase==Phase::MAP_READY);
    onPress(1000);
#endif
    if (StaticMap::LOCALIZE_AT_START) assert(phase==Phase::INITIAL_LOCALIZATION);
    else {
        assert(phase==Phase::START_DELAY);
        onMotion(0,0,0.01f,phaseAt+1999);
        assert(phase==Phase::START_DELAY && !request.v && !request.w);
    }
    onCommand('F',1020); assert(!request.v);
    onPress(1050); onPress(1200); assert(phase==Phase::FAULT_STOP && !request.v && !request.w);
    assert(route.count>2 && route.nodes[0].x==START_X);
    localizationGrid();
}

static float clearance(Pose p,const StaticMap::Obstacle &extra) {
    float low=0,high=100;
    for (int j=0;j<12;++j) {
        const float margin=(low+high)/2;
        Footprint f(p.h,margin);
        bool ok=legal(p,f);
        for (const auto &b:StaticMap::FIXED_OBSTACLES)
            ok=ok && !f.touches(b.x-p.x,b.y-p.y,b.width/2.f,b.length/2.f);
        if (extra.id) ok=ok && !f.touches(extra.x-p.x,extra.y-p.y,extra.width/2.f,extra.length/2.f);
        if (ok) low=margin; else high=margin;
    }
    return low;
}

static void driveCase(int scenario) {
    loadRoute(); phase=Phase::WAIT_BUTTON; onPress(0);
#if defined(COMPETITION_2_LAPS) && COMPETITION_2_LAPS
    onCommand('S',0); onPress(0);
#endif
    const StaticMap::Obstacle extra=scenario==0?StaticMap::Obstacle{}:scenario==5?
        StaticMap::Obstacle{255,1600,185,100,100}:StaticMap::POSSIBLE_OBSTACLES[scenario-1];
    Pose truth{StartX,StartY,StartHeading};
    float residual[2]{}, minimum=clearance(truth,extra), peak=0;
    bool avoided=false;
    for (unsigned now=10;now<130000;now+=10) {
        const Motion m=request;
        assert(m.v>=0); peak=fmaxf(peak,m.v);
        truth=advance(truth,m,0.01f); yawRate=m.w;
        // Bound the travel of any corner during this sample, including rotation.
        const float excursion=(m.v+fabsf(m.w)*hypotf(HalfLength,HalfWidth))*0.01f;
        const float gap=clearance(truth,extra);
        assert(gap>excursion);
        minimum=fminf(minimum,gap-excursion);
        int ticks[2];
        for (int i=0;i<2;++i) {
            residual[i]+=(m.v+(i?0.5f:-0.5f)*Track*m.w)*0.01f*CountsPerMm[i];
            ticks[i]=lroundf(residual[i]); residual[i]-=ticks[i];
            wheels[i].observe(ticks[i],0.01f,now);
        }
        if (now%40==0) {
            Frame f{}; f.number=latest.number+1; liveMask=f.observed=All;
            const Scan scan=scanAt(truth,extra); f.mask=scan.mask;
            for (int i=0;i<4;++i) f.beam[i]={pose,now,uint16_t(scan.mask&(1<<i)?scan.mm[i]:MaxRay+1)};
            latest=f; onFrame(f,now);
        }
        onMotion(ticks[0],ticks[1],0.01f,now);
        for (int i=0;i<10;++i) work(now);
        avoided|=phase==Phase::OBSTACLE_AVOIDANCE;
        if (phase==Phase::FAULT_STOP) fprintf(stderr,"case %d: %s at %u ms\n",scenario,reason,now);
        assert(phase!=Phase::FAULT_STOP);
        if (phase==Phase::FINISHED) {
            assert(acrossFinish(truth,0) && !request.v && !request.w);
            assert(terrain.confirmed.id==extra.id);
            if (scenario==5) assert(avoided);
            printf("case %d time %.2f s peak %.1f mm/s swept clearance >= %.1f mm id %u\n",
                   scenario,now/1000.,peak,minimum,extra.id);
            return;
        }
    }
    assert(false && "course timeout");
}
int main(int argc,char **argv) {
    if (argc==1) safetyTests();
    else { const int n=atoi(argv[1]); assert(n>=0 && n<=5); driveCase(n); }
}
