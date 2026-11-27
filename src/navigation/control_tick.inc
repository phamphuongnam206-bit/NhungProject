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
    if (tripped) fault(tripControlMs>65?"CONTROL_TIMEOUT":"RANGE_TIMEOUT",now);
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
