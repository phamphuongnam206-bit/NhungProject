/* Standalone MPU6050 relative yaw PID test, STM32F103C8 / TB6612.
   UART PA9 TX / PA10 RX, 115200, newline. MPU SDA PB7 / SCL PB6.
   Boot: motors locked, 250 still samples for gyro bias. No VL53 dependency.
   Gyro sign +1 confirmed by physical right turn with negative motor command.
   No manual sign arming needed; LEFTOK only checks positive left yaw, never flips sign.
   TURN 30 / TURN -30: relative turn; STOP cancels. Max +/-90deg, 6s, PWM90; measured error <=0.30deg and rate <1.5deg/s for 400ms.
   Each TURN line is a separate request. Up to 16 turns can wait in FIFO order. RESULT reprints the last turn.
   LOG 0 = summaries, LOG 1 = continuous angle at 5 Hz, LOG 2 = live angle and turn trace.
   PID 2.0 0 0.5 sets trial angle gains in RAM (PWM/deg, PWM/(deg*s), PWM/(deg/s)).
   Direct angle PID -> opposing motor PWM; NOT the encoder speed PI controller.
   Encoder counts monitor motor stall/direction only; do not tune speed PI here.
   Short relative yaw tests only: gyro yaw drifts and scale is not calibrated.
   Host tests only; build/flash and real turn response require target validation.
*/
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
namespace AngleTest {
float bound(float x,float lo,float hi){return fmaxf(lo,fminf(hi,x));}
int direction(float x){return (x>0)-(x<0);}
struct Controller {
  float kp=2.0f,ki=0.0f,kd=0.5f,integral=0;
  static constexpr float MAX_PWM = 90, TOL_DEG = 0.30f, SETTLE_RATE = 1.5f;
  float frictionBoost = 0;
  int correctionSign = 0;
  void reset(){
    integral = 0;
    frictionBoost = 0;
    correctionSign = 0;
  }
  float update(float error, float rate, float dt){
    if (!isfinite(error) || !isfinite(rate) || !isfinite(dt) || dt <= 0) return 0;
    if (kp == 0 && ki == 0) { reset(); return 0; }
    const float remaining = fabsf(error);
    const int sign = direction(error);
    const float closingRate = sign * rate;
    if (remaining <= TOL_DEG || sign != correctionSign) reset();
    if (remaining <= TOL_DEG) return 0;
    correctionSign = sign;

    // Coast through the last fraction of a degree; let the chassis slow
    // before applying an opposite correction after an overshoot.
    if (remaining < 6 && (closingRate < -4 ||
        (closingRate > SETTLE_RATE && remaining <= TOL_DEG + 0.030f * closingRate))) {
      frictionBoost = 0;
      return 0;
    }

    // Shape the P term to reduce cruise speed and approach at a lower rate.
    // At default gains the P/D balance requests at most 95 deg/s,
    // and progressively less as the remaining angle shrinks.
    const float brakeRate = sqrtf(2.0f * 125.0f * remaining);
    const float shapedAngle = fminf(remaining, 0.25f * fminf(95.0f, brakeRate));
    // Increase torque progressively if static friction is holding the wheels.
    // A turn already in motion can receive much less than its breakaway PWM.
    const float approachRate = bound(4.0f * shapedAngle, 2.5f, 35.0f);
    float boostSlope = 0;
    if (closingRate < 0.55f * approachRate) boostSlope = 105.0f;
    if (closingRate > approachRate) boostSlope = -140.0f;
    frictionBoost = bound(frictionBoost + boostSlope * dt, 0, 32);
    const float effort = 40.0f + frictionBoost + kp * shapedAngle - kd * closingRate;
    if (ki > 0 && remaining < 15 && effort + integral < MAX_PWM)
      integral = bound(integral + ki * remaining * dt, 0, 10);
    return sign * bound(effort + integral, 0, MAX_PWM);
  }
};
bool decimal(const char *&p,float &value){
  while(*p==' '||*p=='\t')++p;
  if(*p=='+')++p;
  uint32_t whole=0,fraction=0,scale=1;bool seen=false;
  while(*p>='0'&&*p<='9'){seen=true;whole=whole*10+unsigned(*p++-'0');if(whole>100)return false;}
  if(*p=='.'){++p;while(*p>='0'&&*p<='9'){
    seen=true;if(scale>=1000000U)return false;
    fraction=fraction*10+unsigned(*p++-'0');scale*=10;
  }}
  if(!seen||(*p&&*p!=' '&&*p!='\t'))return false;
  value=float(whole)+float(fraction)/scale;return true;
}
bool gains(const char *line,float &kp,float &ki,float &kd){
  if(strncmp(line,"PID ",4))return false;
  const char *p=line+4;float a,b,c;
  if(!decimal(p,a)||!*p||!decimal(p,b)||!*p||!decimal(p,c))return false;
  while(*p==' '||*p=='\t')++p;
  if(*p||a>10||b>2||c>5)return false;
  kp=a;ki=b;kd=c;return true;
}
bool turnRequest(const char *line,int &degrees){
  if(strncmp(line,"TURN ",5))return false;
  const char *p=line+5;while(*p==' ')++p;
  int sign=1;if(*p=='-'||*p=='+'){if(*p=='-')sign=-1;++p;}
  if(*p<'0'||*p>'9')return false;
  unsigned magnitude=0;
  while(*p>='0'&&*p<='9'){
    magnitude=magnitude*10+unsigned(*p++-'0');
    if(magnitude>90)return false;
  }
  while(*p==' ')++p;
  if(*p||magnitude<5)return false;
  degrees=sign*int(magnitude);return true;
}
// Reject an entire corrupted line rather than turning its remaining bytes into a command.
struct CommandLine {
  char text[64]{};unsigned used=0;bool bad=false, nonAscii=false;uint32_t lastByte=0;
  bool feed(uint8_t c,uint32_t now){
    if((used||bad)&&uint32_t(now-lastByte)>2000){used=0;bad=nonAscii=false;}
    lastByte=now;
    if(c=='\r'||c=='\n'){
      if(bad){used=0;bad=nonAscii=false;return false;}
      while(used&&text[used-1]==' ')--used;
      text[used]=0;
      bool complete = used != 0;
      if (nonAscii) {
        // Only this UTF-8 stop word is allowed in non-ASCII input.
        complete = !strcmp(text,"dừng") || !strcmp(text,"Dừng") || !strcmp(text,"DỪNG");
        if (complete) strcpy(text,"STOP");
      }
      used=0; nonAscii=false;
      return complete;
    }
    if(bad)return false;
    if(c==8||c==127){if(used)--used;return false;}
    if(c=='\t')c=' ';
    if(c<32){bad=true;return false;}
    if(c>126)nonAscii=true;
    if(c==' '&&(!used||text[used-1]==' '))return false;
    if(used>=sizeof(text)-1){bad=true;return false;}
    text[used++]=char(c);return false;
  }
};

}
#ifndef ANGLE_NATIVE_TEST
#include <Arduino.h>
#include <Wire.h>
using namespace AngleTest;
constexpr uint32_t STBY=PB0,LPWM=PB9,RPWM=PB8;
constexpr int L_SIGN=-1,R_SIGN=1;
TwoWire imu(PB7,PB6);
uint8_t address=0x68;
Controller pid;
bool imuOK=false,ready=false,signChecked=true,running=false;
const char *fault=nullptr;
float gyroSign=1,bias=0,biasSum=0,yaw=0,rate=0,previousRate=0,target=0,startYaw=0;
float biasMin=0,biasMax=0;
uint16_t calibrationCount=0,oldL=0,oldR=0;
int32_t deltaL=0,deltaR=0;
uint32_t lastTick=0,startedAt=0,settleAt=0,lastYawMotion=0;
constexpr uint32_t STALL_DRIVE_MS = 500;
uint32_t silentDriveL = 0, silentDriveR = 0;
uint32_t reversedAt=0,lastLog=0;
int appliedDirection=0,previousDirection=0,lastPwm=0;
int32_t wrongL=0,wrongR=0;
volatile bool guardArmed=false,guardExpired=false,tickSeen=false;
volatile uint32_t guardAt=0;
char tx[320];size_t txLen=0,txAt=0;
// Use only with motors off: complete pending text before printing an acknowledgement.
void finishText(){while(txAt<txLen)Serial.write(uint8_t(tx[txAt++]));txAt=txLen=0;}

unsigned logMode = 1;

constexpr unsigned TURN_QUEUE_SIZE = 16;
int8_t pendingDegrees[TURN_QUEUE_SIZE];
unsigned pendingHead = 0, pendingCount = 0;
uint32_t turnNumber = 0;
struct TurnResult {
  const char *reason = nullptr;
  float finalYaw = 0, goal = 0, displacement = 0;
  uint32_t number = 0, silentLeftMs = 0, silentRightMs = 0;
  int pwmAtStop = 0;
} lastResult;

void printResult(){
  if (!lastResult.reason) {
    Serial.println("NO_RESULT");
    return;
  }
  Serial.print("STOPPED "); Serial.print(lastResult.reason);
  Serial.print(" ANGLE="); Serial.print(lastResult.finalYaw, 2);
  Serial.print(" TARGET="); Serial.print(lastResult.goal, 2);
  Serial.print(" ERROR="); Serial.print(lastResult.goal - lastResult.finalYaw, 2);
  Serial.print(" TURNED="); Serial.print(lastResult.displacement, 2);
  if (!strcmp(lastResult.reason, "ENCODER_STALL")) {
    const bool left = lastResult.silentLeftMs >= STALL_DRIVE_MS;
    const bool right = lastResult.silentRightMs >= STALL_DRIVE_MS;
    Serial.print(" SIDE="); Serial.print(left && right ? "BOTH" : left ? "L" : "R");
    Serial.print(" NO_PULSE_DRIVE_MS_L="); Serial.print(lastResult.silentLeftMs);
    Serial.print(" NO_PULSE_DRIVE_MS_R="); Serial.print(lastResult.silentRightMs);
    Serial.print(" PWM="); Serial.print(lastResult.pwmAtStop);
  }
  Serial.print(" ID="); Serial.println(lastResult.number);
}
extern "C" void HAL_SYSTICK_Callback(void){
  tickSeen=true;
  if(guardArmed && uint32_t(HAL_GetTick()-guardAt)>=100){
    GPIOB->BRR=1U;guardArmed=false;guardExpired=true;
  }
}
void low(uint32_t pin){digitalWrite(pin,LOW);pinMode(pin,OUTPUT);}
void outputsOff(){
  digitalWrite(STBY,LOW);guardArmed=false;
  analogWrite(LPWM,0);analogWrite(RPWM,0);lastPwm=0;
}
void stop(const char *why){
  if (strcmp(why, "DONE") != 0) pendingCount = 0;
  const bool wasTurning = running;
  if (wasTurning) lastResult.pwmAtStop = lastPwm;
  outputsOff();running=false;pid.reset();settleAt=0;
  previousDirection=0;appliedDirection=0;finishText();
  if (wasTurning) {
    lastResult.reason = why;
    lastResult.silentLeftMs = silentDriveL;
    lastResult.silentRightMs = silentDriveR;
    lastResult.finalYaw = yaw;
    lastResult.goal = target;
    lastResult.displacement = yaw - startYaw;
    lastResult.number = turnNumber;
    printResult();
  } else {
    Serial.print("STOPPED "); Serial.println(why);
  }
}
void fail(const char *why){fault=why;stop(why);}
void configureEncoder(TIM_TypeDef *t){
  t->CR1=0;t->CR2=0;t->SMCR=0;t->DIER=0;t->CCER=0;t->PSC=0;t->ARR=65535;
  t->CCMR1=1U|(8U<<4)|(1U<<8)|(8U<<12);t->CCMR2=0;t->SMCR=3U;
  t->CCER=TIM_CCER_CC1E|TIM_CCER_CC2E;t->EGR=TIM_EGR_UG;t->CNT=0;t->SR=0;t->CR1=TIM_CR1_CEN;
}
void initEncoders(){
  __HAL_RCC_AFIO_CLK_ENABLE();__HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();__HAL_RCC_TIM3_CLK_ENABLE();
  HAL_NVIC_DisableIRQ(TIM2_IRQn);HAL_NVIC_DisableIRQ(TIM3_IRQn);
  __HAL_RCC_TIM2_FORCE_RESET();__HAL_RCC_TIM2_RELEASE_RESET();
  __HAL_RCC_TIM3_FORCE_RESET();__HAL_RCC_TIM3_RELEASE_RESET();
  uint32_t m=AFIO->MAPR;
  m&=~(AFIO_MAPR_TIM2_REMAP|AFIO_MAPR_TIM3_REMAP|AFIO_MAPR_TIM4_REMAP|AFIO_MAPR_SWJ_CFG);
  AFIO->MAPR=m|AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  pinMode(PA6,INPUT_PULLUP);pinMode(PA7,INPUT_PULLUP);
  pinMode(PA0,INPUT_PULLUP);pinMode(PA1,INPUT_PULLUP);
  configureEncoder(TIM2);configureEncoder(TIM3);
  oldL=uint16_t(TIM3->CNT);oldR=uint16_t(TIM2->CNT);
}
int32_t difference(uint16_t a,uint16_t b){uint16_t v=a-b;return v<32768U?v:int32_t(v)-65536L;}
bool byteRead(uint8_t reg,uint8_t &v){
  imu.beginTransmission(address);imu.write(reg);
  if(imu.endTransmission()!=0)return false;
  if(imu.requestFrom(address,uint8_t(1))!=1){while(imu.available())imu.read();return false;}
  v=uint8_t(imu.read());return true;
}
bool writeReg(uint8_t reg,uint8_t value){
  imu.beginTransmission(address);imu.write(reg);imu.write(value);return imu.endTransmission()==0;
}
bool initImu(){
  imu.begin();imu.setClock(100000);
  for(uint8_t a=0x68;a<=0x69;++a){
    address=a;uint8_t id=0;
    if(!byteRead(0x75,id)||id!=0x68)continue;
    if(!writeReg(0x6B,0x80))return false;delay(100);
    return writeReg(0x6B,1)&&writeReg(0x6C,0)&&writeReg(0x1B,0)&&writeReg(0x1A,3)&&writeReg(0x19,9);
  }
  return false;
}
bool gyro(float &value){
  for(unsigned retry=0;retry<2;++retry){
    uint8_t h,l,check;
    if(!byteRead(0x47,h)||!byteRead(0x48,l)||!byteRead(0x47,check))return false;
    if(h==check){value=int16_t((uint16_t(h)<<8)|l)/131.0f;return true;}
  }
  return false;
}
void drive(float requested,uint32_t now,float dt){
  const int wanted=direction(requested);
  if(appliedDirection && wanted!=appliedDirection){
    previousDirection=appliedDirection;reversedAt=now;appliedDirection=0;outputsOff();return;
  }
  if(wanted && previousDirection && wanted!=previousDirection && now-reversedAt<25){outputsOff();return;}
  const float maxChange=420.0f*dt;
  int pwm=0;
  if(wanted){
    const float mag=fminf(fabsf(requested),fabsf(float(lastPwm))+maxChange);
    pwm=int(lroundf(mag))*wanted;
  }
  if(!pwm){
    if(appliedDirection){previousDirection=appliedDirection;reversedAt=now;}
    appliedDirection=0;outputsOff();return;
  }
  appliedDirection=wanted;previousDirection=wanted;
  // Positive angular PWM = left/CCW: left wheel reverse, right wheel forward.
  const int l=-pwm,r=pwm;
  digitalWrite(PB14,l>0?LOW:HIGH);digitalWrite(PB15,l>0?HIGH:LOW);
  digitalWrite(PB12,r>0?HIGH:LOW);digitalWrite(PB13,r>0?LOW:HIGH);
  analogWrite(LPWM,abs(l));analogWrite(RPWM,abs(r));
  guardAt=now;guardArmed=true;
  if(guardExpired){outputsOff();return;}
  digitalWrite(STBY,HIGH);lastPwm=pwm;
}
void status(){
  Serial.print("STATE=");Serial.print(fault?fault:!ready?"CALIBRATING":running?"TURNING":"LOCKED");
  Serial.print(" ANGLE=");Serial.print(yaw,2);Serial.print(" RATE=");Serial.print(rate,2);
  Serial.print(" BIAS=");Serial.print(bias,4);Serial.print(" SIGN=");Serial.print(gyroSign,0);
  Serial.print(" CHECKED=");Serial.println(signChecked?1:0);
}
void acceptTurn(const char *line){
  int degrees;
  if (!turnRequest(line, degrees)) {
    Serial.println("USE TURN integer +/-5..90");
    return;
  }
  if (!ready || fault || !signChecked || !tickSeen) {
    Serial.println("NOT_READY");
    return;
  }
  if (!running && fabsf(rate) > 4) {
    Serial.println("WAIT_STILL");
    return;
  }
  if (pendingCount == TURN_QUEUE_SIZE) {
    Serial.println("QUEUE_FULL");
    return;
  }
  pendingDegrees[(pendingHead + pendingCount) % TURN_QUEUE_SIZE] = int8_t(degrees);
  ++pendingCount;
}

void startPendingTurn(){
  if (running || !pendingCount || fault || !ready || guardExpired) return;
  if (fabsf(rate) > 4) {
    stop("WAIT_STILL");
    return;
  }
  finishText();
  const int degrees = pendingDegrees[pendingHead];
  pendingHead = (pendingHead + 1) % TURN_QUEUE_SIZE;
  --pendingCount;
  ++turnNumber;
  startYaw = yaw;
  target = startYaw + degrees;
  pid.reset();
  silentDriveL = silentDriveR = 0;
  startedAt = lastYawMotion = lastLog = millis();
  settleAt = 0;
  wrongL = wrongR = 0;
  previousDirection = appliedDirection = 0;
  Serial.print("TURN_START ID="); Serial.print(turnNumber);
  Serial.print(" DEG="); Serial.print(degrees);
  Serial.print(" TARGET="); Serial.println(target, 2);
  running = true;
}
void execute(char *line){
  while(*line==' '||*line=='\t')++line;
  size_t n=strlen(line);while(n&&(line[n-1]==' '||line[n-1]=='\t'))line[--n]=0;
  for(char *p=line;*p;++p)if(*p>='a'&&*p<='z')*p=char(*p-'a'+'A');
  if(!*line)return;
  if (!strcmp(line,"STOP") || !strcmp(line,"DUNG")) {
    logMode = 0;
    stop("USER");
    return;
  }
  if (!strncmp(line, "TURN ", 5)) {
    acceptTurn(line);
    return;
  }
  if (running || pendingCount) return;
  const bool known=!strcmp(line,"STATUS")||!strcmp(line,"HELP")||!strcmp(line,"ZERO")||
    !strcmp(line,"RESULT")||!strcmp(line,"LOG 2")||
    !strcmp(line,"LEFTOK")||!strcmp(line,"LOG 0")||!strcmp(line,"LOG 1")||
    !strncmp(line,"PID ",4);
  if(!known)return;
  finishText();
  if (!strcmp(line,"RESULT")) { printResult(); return; }
  if (!strncmp(line,"LOG ",4)) {
    logMode = unsigned(line[4] - '0');
    Serial.println(logMode == 0 ? "LOG_OFF" : logMode == 1 ? "LOG_LIVE" : "LOG_TRACE");
    return;
  }
  if(!strcmp(line,"STATUS")){status();return;}
  if(!strcmp(line,"HELP")){
    Serial.println("STATUS | STOP/DUNG | LEFTOK | TURN +/-5..90 | PID Kp Ki Kd | ZERO | LOG 0/1/2 | RESULT");return;
  }
  if(!strcmp(line,"ZERO")){
    if(!ready||fault){status();return;}
    yaw=target=0;pid.reset();Serial.println("ZERO_OK LOCKED");return;
  }
  if(!strcmp(line,"LEFTOK")){
    if(!ready||fault){status();return;}
    if(yaw<10 || fabsf(rate)>4){Serial.println("LEFTCHECK: rotate car LEFT by hand 20..30deg, STOP moving, then LEFTOK.");return;}
    yaw=target=0;signChecked=true;
    Serial.print("LEFT_SIGN_OK SIGN=");Serial.print(gyroSign,0);Serial.println(" ANGLE=0 LOCKED");return;
  }
  if(!strncmp(line,"PID ",4)){
    float p,i,d;if(!gains(line,p,i,d)){Serial.println("USE PID Kp[0..10] Ki[0..2] Kd[0..5], decimal numbers");return;}
    pid.kp=p;pid.ki=i;pid.kd=d;pid.reset();
    Serial.print("PID_SET ");Serial.print(p,3);Serial.print(' ');Serial.print(i,3);Serial.print(' ');Serial.println(d,3);return;
  }

}
void commands(){
  static CommandLine input;
  uint8_t budget=24;
  while(budget--&&Serial.available()){
    const uint32_t now=millis();
    if(input.feed(uint8_t(Serial.read()),now))execute(input.text);
  }
}
void pump(){int room=Serial.availableForWrite();if(room>32)room=32;
  while(room-->0&&txAt<txLen)Serial.write(uint8_t(tx[txAt++]));if(txAt==txLen)txAt=txLen=0;
}
void logLiveAngle(uint32_t now){
  if (!ready || fault || logMode == 0 || txLen || now-lastLog < 200) return;
  if (running && logMode == 2) return;  // Detailed turn trace uses the same buffer.
  lastLog=now;
  const long a=lroundf(yaw*100),r=lroundf(rate*100);
  const long aa=a<0?-a:a,rr=r<0?-r:r;
  const int n=snprintf(tx,sizeof(tx),
    "ANGLE=%s%ld.%02ld deg RATE=%s%ld.%02ld deg/s %s\r\n",
    a<0?"-":"",aa/100,aa%100,r<0?"-":"",rr/100,rr%100,running?"TURNING":"LOCKED");
  if(n>0&&size_t(n)<sizeof(tx)){txLen=size_t(n);txAt=0;}
}
void logTurn(uint32_t now){
  if (logMode != 2 || !running || txLen || now-lastLog < 200) return;
  lastLog=now;
  const int n=snprintf(tx,sizeof(tx),
    "ANGLE_MS=%lu ANGLE100=%ld TARGET100=%ld ERR100=%ld RATE100=%ld PWM=%d DL=%ld DR=%ld\r\n",
    (unsigned long)(now-startedAt),(long)lroundf(100*yaw),(long)lroundf(100*target),
    (long)lroundf(100*(target-yaw)),(long)lroundf(100*rate),lastPwm,(long)deltaL,(long)deltaR);
  if(n>0&&size_t(n)<sizeof(tx)){txLen=size_t(n);txAt=0;}
}
void control(uint32_t now){
  const uint32_t elapsed=now-lastTick;if(elapsed<10)return;lastTick=now;
  const uint16_t l=uint16_t(TIM3->CNT),r=uint16_t(TIM2->CNT);
  deltaL=difference(l,oldL)*L_SIGN;deltaR=difference(r,oldR)*R_SIGN;oldL=l;oldR=r;
  if(guardExpired){if(!fault)fail("WATCHDOG");return;}
  if(!imuOK||fault)return;
  if(running&&elapsed>40){fail("CONTROL_LATE");return;}
  float raw=0;if(!gyro(raw)){fail("MPU_READ_FAILED");return;}
  const float dt=elapsed*.001f;
  if(!ready){
    if(deltaL||deltaR||fabsf(raw)>5){calibrationCount=0;biasSum=0;return;}
    if(!calibrationCount)biasMin=biasMax=raw;
    biasMin=fminf(biasMin,raw);biasMax=fmaxf(biasMax,raw);
    if(biasMax-biasMin>1.5f){calibrationCount=0;biasSum=0;return;}
    biasSum+=raw;
    if(++calibrationCount>=250){bias=biasSum/calibrationCount;ready=true;yaw=rate=previousRate=0;
      Serial.print("READY LOCKED BIAS=");Serial.println(bias,4);
      Serial.println("SIGN=+1 fixed from motor turn test. TURN 30=LEFT, TURN -30=RIGHT. No LEFTOK needed.");}
    return;
  }
  const float angularRate=gyroSign*(raw-bias);
  yaw+=.5f*(angularRate+previousRate)*dt;previousRate=angularRate;
  rate+=dt/(.025f+dt)*(angularRate-rate);
  if(!running)return;
  if(now-startedAt>=6000){stop("TIMEOUT");return;}
  if(fabsf(angularRate)>220){fail("GYRO_RANGE");return;}
  // lastPwm was applied during the interval that produced these encoder deltas.
  // Coast time adds nothing; any observed motion clears that wheel's accumulator.
  const uint32_t driveMs = lastPwm ? elapsed : 0;
  silentDriveL = deltaL ? 0 : silentDriveL + driveMs;
  silentDriveR = deltaR ? 0 : silentDriveR + driveMs;
  const float error=target-yaw;
  if(fabsf(error)<=Controller::TOL_DEG && fabsf(rate)<Controller::SETTLE_RATE){
    drive(0,now,dt);pid.reset();
    if(!settleAt)settleAt=now;
    if(now-settleAt>=400)stop("DONE");return;
  }
  settleAt=0;
  if(fabsf(rate)>2)lastYawMotion=now;
  if(now-lastYawMotion>800){fail("NO_YAW_MOTION");return;}
  if (lastPwm && (silentDriveL >= STALL_DRIVE_MS || silentDriveR >= STALL_DRIVE_MS)) {
    fail("ENCODER_STALL");
    return;
  }
  if(lastPwm && now-startedAt>150){
    if(deltaL*direction(-lastPwm)<0)wrongL+=abs(deltaL);else if(deltaL)wrongL=0;
    if(deltaR*direction(lastPwm)<0)wrongR+=abs(deltaR);else if(deltaR)wrongR=0;
    if(wrongL>=40||wrongR>=40){fail("ENCODER_DIRECTION");return;}
  }
  // Net motion opposite the initial requested turn indicates a wrong gyro sign.
  // Use net displacement, not instantaneous rate: braking/countersteering is legitimate.
  if(now-startedAt<1000 && (yaw-startYaw)*direction(target-startYaw)<-5){
    fail("GYRO_DIRECTION");return;
  }
  drive(pid.update(error,rate,dt),now,dt);logTurn(now);
}
void setup(){
  low(STBY);low(PB14);low(PB15);low(PB12);low(PB13);low(LPWM);low(RPWM);
  low(PA2);low(PA3);low(PA4);low(PA5); // four VL53 held off; separate I2C bus unused
  pinMode(PC13,OUTPUT);digitalWrite(PC13,HIGH);analogWriteResolution(8);outputsOff();
  Serial.setRx(PA10);Serial.setTx(PA9);Serial.begin(115200,SERIAL_8N1);
  initEncoders();imuOK=initImu();delay(100);lastTick=millis();
  Serial.println("MPU_ANGLE_TEST_V11 SMOOTH TOL_0.30 SIGN_PLUS1 LOCKED UART115200 MPU PB7/PB6");
  if(!imuOK){fail("MPU_INIT_FAILED");return;}
  if(!tickSeen){fail("SYSTICK_GUARD_INACTIVE");return;}
  Serial.println("KEEP STILL ~3s for gyro bias. No automatic movement. HELP for commands.");
}
void loop(){commands();startPendingTurn();control(millis());logLiveAngle(millis());pump();digitalWrite(PC13,running?LOW:HIGH);}
#endif
