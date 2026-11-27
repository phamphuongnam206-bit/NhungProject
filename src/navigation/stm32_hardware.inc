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
volatile uint32_t tripControlMs=0, tripRangeMs=0;
volatile bool armed = false, tripped = false, tickSeen = false;
extern "C" void HAL_SYSTICK_Callback(void) {
    tickSeen=true;
    if (!armed) return;
    const uint32_t now=HAL_GetTick();
    const uint32_t controlAge=now-controlGuardAt, rangeAge=now-rangeGuardAt;
    if (controlAge>65 || rangeAge>FreshMs) {
        tripControlMs=controlAge;
        tripRangeMs=rangeAge;
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
    const uint32_t age=motionRangeAge(latest,now);
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
