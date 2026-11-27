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
