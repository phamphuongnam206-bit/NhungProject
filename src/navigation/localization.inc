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
    matcher.start(start?route.nodes[0].point():pose,s,SearchRadius,SearchAngle,now);
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

