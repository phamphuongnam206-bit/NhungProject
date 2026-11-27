#define MAP_NAM_HOST_TEST
#include "../src/map_nam.cpp"
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace MapNam;

static void hardwareAndOdometry() {
    phase=Phase::INIT; bootAt=120; mode=Mode::IDLE;
    for (unsigned i=0;i<8;++i) assert(ledOn(120+i*150)==(i%2==0));
    assert(!ledOn(1320));
    assert(counterDelta(1,65534)==3 && counterDelta(65534,1)==-3);
    Button sw;
    assert(!sw.press(false,40) && !sw.press(true,45) && !sw.press(false,49));
    assert(!sw.press(true,55) && sw.press(true,95) && !sw.press(true,400));
    assert(!sw.press(false,420) && !sw.press(false,460));
    assert(!sw.press(true,500) && sw.press(true,540));
    BiasEstimator estimator;
    for (int i=0;i<100;++i) assert(!estimator.sample(1,false));
    assert(!estimator.sample(1,true));
    for (int i=0;i<179;++i) assert(!estimator.sample(1,false));
    assert(estimator.sample(1,false) && gyroBias==1);
    for (float deg : {30.f,45.f,90.f,180.f}) {
        const int ticks=lroundf(CountsPerMm[0]*Track*deg*Rad*0.5f);
        yawRate=deg*Rad; resetOdometry({330,75,0},0);
        int done=0;
        for (int i=1;i<=100;++i) {
            const int next=ticks*i/100;
            predict(done-next,next-done,0.01f,i*10); done=next;
        }
        assert(fabsf(wrap(pose.h-deg*Rad))<0.008f && separation(pose,{330,75,0})<0.01f);
    }
    yawRate=0; resetOdometry({330,75,Pi/2},0);
    for (int i=1;i<=100;++i) predict(9,9,0.01f,i*10);
    assert(fabsf(pose.y-275)<0.1f && fabsf(pose.x-330)<0.1f);
    for (int i=0;i<3000;++i) predict(0,0,0.01f,i*10+1010);
    assert(fabsf(pose.h-Pi/2)<0.001f);
    Wheel w;
    for (int i=0;i<70;++i) { w.output(450,5666,0.01f,i*10); assert(w.observe(0,0.01f,i*10)); }
    assert(!w.observe(0,0.01f,800));
}

static Scan fixtureRanges(Pose p) {
    Scan out{}; out.mask=All;
    for (int i=0;i<4;++i) {
        const Ray r=rayFrom(p,i);
        float best=1400;
        for (float x : {112.5f,562.5f}) {
            if (fabsf(r.dx)<0.001f) continue;
            const float t=(x-r.x)/r.dx;
            const float y=r.y+t*r.dy;
            if (t>0 && y>155 && y<800) best=fminf(best,t);
        }
        if (r.dy>0.01f) {
            const float t=(512.5f-r.y)/r.dy;
            const float x=r.x+t*r.dx;
            if (t>0 && x>110 && x<565) best=fminf(best,t);
        }
        out.mm[i]=uint16_t(lroundf(best));
    }
    return out;
}
static void fixtureMap() {
    memset(&terrain,0,sizeof(terrain));
    for (int y=162;y<800;y+=25) for (int x : {112,562}) {
        const int k=cellAt(x,y); terrain.hit(k); terrain.hit(k);
    }
    for (int x=112;x<566;x+=25) {
        const int k=cellAt(x,512); terrain.hit(k); terrain.hit(k);
    }
}
static Frame frameAt(Pose p,Scan s,uint32_t time,uint32_t sequence) {
    Frame f{}; f.number=sequence; f.mask=s.mask;
    for (int i=0;i<4;++i) f.beam[i]={p,time,s.mm[i]};
    return f;
}
static void mappingAndFreeze() {
    phase=Phase::WAIT_BUTTON; mode=Mode::IDLE; onPress(100); yawRate=0; liveMask=All;
    for (int i=1;i<=Batch::Size;++i) {
        latest=frameAt(pose,fixtureRanges(pose),100+i*40,i);
        onFrame(latest,100+i*40);
    }
    assert(phase==Phase::MANUAL_MAPPING && route.count==1);
    for (int i=0;i<75;++i) {
        const uint32_t now=430+i*10;
        onMotion(9,9,0.01f,now);
        if (i%4==0) { latest=frameAt(pose,fixtureRanges(pose),now,10+i); onFrame(latest,now); }
    }
    assert(route.count>=2 && terrain.landmarks>0);
    onCommand('S',1200);
    assert(route.locked && phase==Phase::MAP_READY);
    const Route saved=route;
    const Terrain learned=terrain;
    const Pose end=pose;
    onMotion(300,-200,0.01f,1250);
    onFrame(frameAt(pose,fixtureRanges(pose),1300,200),1300);
    onCommand('F',1301);
    assert(separation(end,pose)==0 && !memcmp(&saved,&route,sizeof(route)));
    assert(!memcmp(&learned,&terrain,sizeof(terrain)));
    onPress(1350);
    assert(mode==Mode::AUTO_NAVIGATION && phase==Phase::INITIAL_LOCALIZATION && ledOn(1350));
}
static void matching() {
    fixtureMap();
    const Pose start{StartX,StartY,StartHeading};
    const Pose cases[]={start,{StartX+30,StartY-20,StartHeading},
        {StartX,StartY,StartHeading+9*Rad},{StartX-25,StartY+32,StartHeading-8*Rad}};
    for (const Pose &actual : cases) {
        matcher.start(start,fixtureRanges(actual),50,15*Rad,0);
        while (matcher.active) matcher.step();
        if (!matcher.accepted || separation(actual,matcher.best)>24 ||
            fabsf(wrap(actual.h-matcher.best.h))>5*Rad) {
            fprintf(stderr,"loc true %.0f %.0f %.1f got %.0f %.0f %.1f scores %d %d\n",
                actual.x,actual.y,actual.h/Rad,matcher.best.x,matcher.best.y,matcher.best.h/Rad,
                matcher.bestScore,matcher.rival);
            assert(false);
        }
    }
    Scan corrupted=fixtureRanges(start); corrupted.mask=3;
    matcher.start(start,corrupted,50,15*Rad,0);
    while (matcher.active) matcher.step();
    assert(!matcher.accepted);
    memset(&terrain,0,sizeof(terrain));
    for (int x=0;x<1800;x+=Cell) {
        const int k=cellAt(x,512); terrain.hit(k); terrain.hit(k);
    }
    Scan wall{}; wall.mask=All;
    for (int i=0;i<4;++i) {
        const Ray ray=rayFrom(start,i); wall.mm[i]=lroundf((512.5f-ray.y)/ray.dy);
    }
    matcher.start(start,wall,50,15*Rad,0);
    while (matcher.active) matcher.step();
    assert(!matcher.accepted);
}

static void teachAndRelocate() {
    uint32_t now=500,sequence=0;
    phase=Phase::WAIT_BUTTON; mode=Mode::IDLE; liveMask=All; yawRate=0;
    onPress(now);
    auto capture=[&](Pose real) {
        latest=frameAt(real,fixtureRanges(real),now,++sequence); onFrame(latest,now);
    };
    for (int i=0;i<7;++i) { now+=40; capture(pose); }
    assert(phase==Phase::MANUAL_MAPPING);
    float fraction[2]{};
    auto teach=[&](int steps,float v,float w) {
        for (int i=0;i<steps;++i) {
            int ticks[2]; yawRate=w;
            for (int k=0;k<2;++k) {
                fraction[k]+=(v+(k?0.5f:-0.5f)*Track*w)*CountsPerMm[k]*0.01f;
                ticks[k]=lroundf(fraction[k]); fraction[k]-=ticks[k];
            }
            now+=10; onMotion(ticks[0],ticks[1],0.01f,now);
            if (i%4==0) capture(pose);
        }
    };
    teach(120,0,-0.42f); teach(240,0,0.42f); teach(120,0,-0.42f); teach(140,85,0);
    onCommand('S',now); assert(route.locked && terrain.landmarks>12);
    const Pose placed{StartX+24,StartY+18,StartHeading+7*Rad};
    const Terrain saved=terrain;
    now+=500; onPress(now);
    for (int i=0;i<600 && phase==Phase::INITIAL_LOCALIZATION;++i) {
        now+=10;
        if (i%4==0) capture(placed);
        onMotion(0,0,0.01f,now); work(now); work(now);
    }
    if (phase!=Phase::TRACKING || separation(pose,placed)>24 || fabsf(wrap(pose.h-placed.h))>5*Rad) {
        fprintf(stderr,"taught START %s pose %.0f %.0f %.1f scores %d %d\n",
            reason,pose.x,pose.y,pose.h/Rad,matcher.bestScore,matcher.rival);
        assert(false);
    }
    assert(!memcmp(saved.evidence,terrain.evidence,sizeof(terrain.evidence)));
    assert(!memcmp(&saved.surveyed,&terrain.surveyed,sizeof(terrain.surveyed)));
}

struct TestBox { float x,y; };
static float boxRange(Ray r,TestBox box) {
    float near=0,far=1600;
    const float origin[]={r.x,r.y},direction[]={r.dx,r.dy},centre[]={box.x,box.y};
    for (int i=0;i<2;++i) {
        const float d=direction[i],lo=centre[i]-50,hi=centre[i]+50;
        if (fabsf(d)<0.00001f) {
            if (origin[i]<lo || origin[i]>hi) return 1550;
        } else {
            const float a=(lo-origin[i])/d,b=(hi-origin[i])/d;
            near=fmaxf(near,fminf(a,b)); far=fminf(far,fmaxf(a,b));
        }
    }
    return near<=far?near:1550;
}
static void finishLane() {
    memset(&terrain,0,sizeof(terrain)); memset(&route,0,sizeof(route));
    memset(terrain.surveyed.data,255,sizeof(terrain.surveyed.data));
    memset(contacts,0,sizeof(contacts)); memset(blockedEdge,0,sizeof(blockedEdge));
    obstacle={}; detour.reset(); detourRetries=0; matcher.active=false;
    wheels[0]={}; wheels[1]={}; matchConfidence=88; request={}; yawRate=0;
    registeredAt=lastMatchTry=lastMatchAt=lastFresh=1000;
    lastFixDistance=edgeStartDistance=headingIntegral=0;
    const Pose a{1490,600,-Pi/2}; resetOdometry(a,1000);
    route.clearance=UINT16_MAX;
    for (int i=0;i<=8;++i) {
        pose.y=a.y-80*i; travelled=80*i;
        const Frame f=frameAt(pose,{{1550,1550,1550,1550},All},100,1);
        assert(route.record(f,true,i==0 || i==8));
    }
    route.locked=true; currentNode=0; targetNode=1;
    resetOdometry(a,1000); liveMask=All; positionError=8;
    mode=Mode::AUTO_NAVIGATION; enter(Phase::TRACKING,"TEST",1000);
    progressWatch.reset(1000);
}
static void navigation(TestBox box,bool hasObstacle,bool surveyed=true) {
    finishLane();
    if (!surveyed) memset(&terrain.surveyed,0,sizeof(terrain.surveyed));
    float remainder[2]{};
    bool avoided=false,rejoined=false;
    for (uint32_t now=1010;now<75000 && mode==Mode::AUTO_NAVIGATION;now+=10) {
        const Motion m=request; yawRate=m.w;
        const float dt=0.010f;
        if (now%40==10) {
            Scan s{}; s.mask=All;
            for (int i=0;i<4;++i) s.mm[i]=hasObstacle?int(lroundf(boxRange(rayFrom(pose,i),box))):1550;
            latest=frameAt(pose,s,now,now/40); onFrame(latest,now);
        }
        int counts[2]{};
        for (int i=0;i<2;++i) {
            wheels[i].cps=(m.v+(i?0.5f:-0.5f)*Track*m.w)*CountsPerMm[i];
            remainder[i]+=wheels[i].cps*dt;
            counts[i]=int(lroundf(remainder[i])); remainder[i]-=counts[i];
        }
        onMotion(counts[0],counts[1],dt,now);
        for (int slice=0;slice<8;++slice) work(now);
        if (phase==Phase::OBSTACLE_AVOIDANCE) avoided=true;
        if (avoided && phase==Phase::TRACKING) rejoined=true;
        assert(!hasObstacle || !Footprint(pose.h,0).touches(box.x-pose.x,box.y-pose.y,50,50));
        assert(legal(pose,Footprint(pose.h,0)));
        if (phase==Phase::FAULT_STOP) break;
    }
    if (!surveyed) {
        assert(phase==Phase::FAULT_STOP && !request.v && !request.w); return;
    }
    if (phase!=Phase::FINISHED || (hasObstacle && !rejoined)) {
        fprintf(stderr,"nav box %.0f,%.0f at %.0f,%.0f h=%.1f phase=%d reason=%s N=%d goal=%d used=%d avoided=%d rejoin=%d\n",
            box.x,box.y,pose.x,pose.y,pose.h/Rad,int(phase),reason,currentNode,targetNode,detour.used,avoided,rejoined);
        assert(false);
    }
    assert(acrossFinish(pose,safetyMargin()) && !ledOn(80000) && !request.v && !request.w);
    onMotion(0,0,0.01f,80000); assert(phase==Phase::WAIT_BUTTON);
    onCommand('F',80005); assert(phase==Phase::WAIT_BUTTON && !request.v);
    onPress(80050); assert(phase==Phase::START_SCAN && route.count==0);
}
static void guardedMotion() {
    finishLane();
    const Pose p{1490,15,-Pi/2};
    assert(!acrossFinish(p,8));
    assert(acrossFinish({p.x,-25,p.h},8));
    assert(!acrossFinish({FinishLeft+20,-25,p.h},8));
    onCommand('F',1200); onCommand('S',1201);
    assert(phase==Phase::TRACKING && !request.v);
    latest={}; lastFresh=1000; onMotion(0,0,0.01f,1900);
    assert(phase==Phase::FAULT_STOP && !request.v);
    finishLane(); memset(&terrain.surveyed,0,sizeof(terrain.surveyed));
    assert(!swept(pose,{0,Pi/2},1,8,true));
    assert(!swept(pose,{-50,0},1,8,true));
}
static void continuousFixAndSlip() {
    finishLane(); fixtureMap();
    memset(terrain.surveyed.data,255,sizeof(terrain.surveyed.data));
    const Pose actual{StartX,StartY,StartHeading};
    pose={actual.x+11,actual.y-10,actual.h+3*Rad};
    const Pose before=pose;
    registeredAt=lastMatchTry=1000; excludedBeams=0; matchConfidence=80;
    latest=frameAt(pose,fixtureRanges(actual),1500,15);
    trackScan(latest,1500);
    assert(matcher.active);
    while (matcher.active) work(1501);
    assert(lastMatchAt==1501 && separation(before,pose)<=3.51f &&
           separation(pose,actual)<separation(before,actual));
    assert(fabsf(wrap(before.h-pose.h))<=0.71f*Rad);
    const unsigned fixed=terrain.landmarks;
    for (int n=0;n<6;++n) {
        latest=frameAt(actual,fixtureRanges(actual),1550+n*40,20+n);
        latest.beam[1].mm=100; obstacle.observe(latest);
        if (n<2) assert(!obstacle.first.valid);
    }
    assert(obstacle.first.valid && (excludedBeams&2) && terrain.landmarks==fixed);
    finishLane();
    for (int i=0;i<40 && phase==Phase::TRACKING;++i) {
        const uint32_t now=1100+i*10;
        latest=frameAt(pose,{{1550,1550,1550,1550},All},now,i+1);
        onMotion(-9,9,0.01f,now);
    }
    assert(phase==Phase::RELOCALIZATION && !request.v && !request.w);
}
static void sealedCorridor() {
    finishLane();
    for (int y=Ymin;y<1200;y+=Cell) for (int x : {1390,1590}) {
        const int k=cellAt(x,y); terrain.hit(k); terrain.hit(k);
    }
    for (int x=1450;x<=1525;x+=Cell) for (int y=280;y<=355;y+=Cell)
        terrain.newObstacle.set(cellAt(x,y));
    startAvoidance(1100);
    assert(detour.begin(1300));
    for (uint32_t tick=0;tick<6000 && detour.searching;++tick) detour.expand(1300+tick/3);
    assert(phase==Phase::FAULT_STOP && !request.v && !request.w);
}
int main() {
    hardwareAndOdometry(); mappingAndFreeze(); matching(); teachAndRelocate();
    guardedMotion(); continuousFixAndSlip(); sealedCorridor();
    navigation({},false); navigation({},false,false);
    for (TestBox box : {TestBox{1480,320},{1510,345},{1470,285},{1500,260}}) navigation(box,true);
    puts("PASS encoder, gyro, odometry, switch, mapping, pickup, START offsets, ambiguity,");
    puts("     navigation, four obstacle placements, rejoin, unknown space, faults, FINISH, next round");
}
