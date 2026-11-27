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
