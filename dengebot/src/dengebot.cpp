#include "EDADengeBot.h"
#include <math.h>
static float clampf(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
EDAPIDConfig edaDefaultConfig(){
 EDAPIDConfig c={};
 c.kp=20.0f; c.ki=0.0f; c.kd=2.0f;
 c.iLimit=255.0f; c.iEnableAngle=10.0f; c.maxRateForI=1.0e9f; c.iLeakPerSec=0.0f;
 c.outputLimit=255.0f; c.gyroTauMs=4.0f; c.recoveryTauMs=20.0f; c.assistTauMs=30.0f;
 c.assistMinError=3.0f; c.assistGain=0.3f; c.assistMaxRate=20.0f;
 c.responseStartError=3.0f; c.responseFullError=12.0f; c.responseMaxGain=1.0f;
 c.recoveryMinError=3.0f; c.recoveryMinRate=10.0f; c.recoveryFullRate=60.0f; c.recoveryExtraGain=0.3f;
 c.floorStartError=6.0f; c.floorFullError=15.0f; c.floorMinRate=20.0f; c.floorMinOutput=40.0f; c.floorMaxOutput=120.0f;
 c.recoveryHysteresis=2.0f;
 c.centerEnterError=0.5f; c.centerExitError=1.5f; c.centerEnterRate=3.0f; c.centerExitRate=8.0f; c.centerCommandThreshold=6.0f;
 c.pushMinError=2.0f; c.pushMinRate=15.0f; c.pushFullRate=80.0f; c.pushMaxExtra=40.0f;
 c.brakeMaxError=3.0f; c.brakeMinRate=10.0f; c.brakeFullRate=60.0f; c.brakeIntegralLeak=3.0f;
 c.centerUrgentAwayRate=0.60f; c.centerBlockAwayRate=0.35f;
 // enableAssist/IRateGate/Recovery/Floor/CenterFilter/PushCatch/ApproachBrake: hepsi false
 return c;
}
static bool validConfig(const EDAPIDConfig &c);
EDABalancePID::EDABalancePID(const EDAPIDConfig &cfg):c_(cfg),configValid_(false),fault_(EDAFault::None),firstFault_(EDAFault::None),zeroOnFault_(false){
 configValid_=validConfig(cfg);
 if(!configValid_){c_=edaDefaultConfig(); fault_=EDAFault::InvalidConfig; firstFault_=EDAFault::InvalidConfig;}
 reset();
 if(!configValid_)valid_=false;
}
static bool edaFinite(float x){return (x==x)&&(x-x==0.0f);} // portable: no isfinite() macro/namespace issues on AVR/ESP cores
static bool validConfig(const EDAPIDConfig &c){
 // All float fields checked, including disabled-feature parameters.
 const float vals[]={c.kp,c.ki,c.kd,c.iLimit,c.iEnableAngle,c.maxRateForI,c.iLeakPerSec,
 c.outputLimit,c.gyroTauMs,c.recoveryTauMs,c.assistTauMs,c.assistMinError,c.assistGain,c.assistMaxRate,
 c.responseStartError,c.responseFullError,c.responseMaxGain,c.recoveryMinError,c.recoveryMinRate,
 c.recoveryFullRate,c.recoveryExtraGain,c.floorStartError,c.floorFullError,c.floorMinRate,
 c.floorMinOutput,c.floorMaxOutput,c.recoveryHysteresis,c.centerEnterError,c.centerExitError,
 c.centerEnterRate,c.centerExitRate,c.centerCommandThreshold,c.pushMinError,c.pushMinRate,
 c.pushFullRate,c.pushMaxExtra,c.brakeMaxError,c.brakeMinRate,c.brakeFullRate,
 c.brakeIntegralLeak,c.centerUrgentAwayRate,c.centerBlockAwayRate};
 for(unsigned i=0;i<sizeof(vals)/sizeof(vals[0]);++i)if(!edaFinite(vals[i]))return false;
 // Her zaman: isaret/aralik kurallari (kapali ozellikler icin de).
 if(c.kp<0||c.ki<0||c.kd<0||c.outputLimit<=0||c.iLimit<0||c.iEnableAngle<0||
 c.maxRateForI<0||c.iLeakPerSec<0||c.gyroTauMs<0||c.recoveryTauMs<0||c.assistTauMs<0||
 c.assistMinError<0||c.assistGain<0||c.assistMaxRate<0||
 c.responseStartError<0||c.responseFullError<c.responseStartError||c.responseMaxGain<0||
 c.recoveryMinError<0||c.recoveryMinRate<0||c.recoveryExtraGain<0||c.recoveryHysteresis<0||
 c.floorStartError<0||c.floorMinRate<0||c.floorMinOutput<0||
 c.centerEnterError<0||c.centerEnterRate<0||c.centerCommandThreshold<0||
 c.pushMinError<0||c.pushMinRate<0||c.pushMaxExtra<0||
 c.brakeMaxError<0||c.brakeMinRate<0||c.brakeIntegralLeak<0||
 c.centerUrgentAwayRate<0||c.centerBlockAwayRate<0)return false;
 // Yalniz ilgili ozellik ACIKSA: siralama / cikis siniri iliskileri.
 if(c.enableRecovery && c.recoveryFullRate<c.recoveryMinRate)return false;
 if(c.enableFloor && (c.floorFullError<c.floorStartError||c.floorMaxOutput<c.floorMinOutput||c.floorMaxOutput>c.outputLimit))return false;
 if(c.enableCenterFilter && (c.centerExitError<c.centerEnterError||c.centerExitRate<c.centerEnterRate))return false;
 if(c.enablePushCatch && c.pushFullRate<c.pushMinRate)return false;
 if(c.enableApproachBrake && c.brakeFullRate<c.brakeMinRate)return false;
 return true;
}
void EDABalancePID::fail(EDAFault reason){valid_=false;lastOutput_=0;fault_=reason;if(firstFault_==EDAFault::None)firstFault_=reason;}
void EDABalancePID::clearFault(){
 // Fault latch can only be cleared when a valid configuration is installed.
 if(configValid_){fault_=EDAFault::None;firstFault_=EDAFault::None;}
}

bool EDABalancePID::setConfig(const EDAPIDConfig &cfg,bool resetState){
 if(!validConfig(cfg)){fail(EDAFault::InvalidConfig);return false;}
 c_=cfg;configValid_=true; if(resetState)reset(); return true;
}
bool EDABalancePID::setTunings(float kp,float ki,float kd,bool resetIntegral){
 EDAPIDConfig n=c_;n.kp=kp;n.ki=ki;n.kd=kd;
 if(!validConfig(n)){fail(EDAFault::InvalidConfig);return false;}
 // Preserve integral contribution when Ki changes; Ki=0 cannot retain integral output.
 const float oldI=i_;
 c_=n;configValid_=true;
 if(resetIntegral || ki==0.0f){integral_=0;i_=0;}
 else if(ki>0.0f){i_=clampf(oldI,-c_.iLimit,c_.iLimit);integral_=i_/ki;}
 return true;
}
void EDABalancePID::reset(float initialRate){
 valid_=configValid_;lastOutput_=0;
 integral_=p_=i_=d_=assist_=peakError_=0;
 rate_=recoveryRate_=slowRate_=initialRate;
 holding_=centerActive_=false;holdSign_=0;
 saturated_=gated_=integrated_=0;
}
float EDABalancePID::update(float target,float pitch,float verifiedRate,float pitchRate,float dt){
 if(!configValid_){fail(EDAFault::InvalidConfig);return 0.0f;}
 if(zeroOnFault_ && fault_!=EDAFault::None){valid_=false;lastOutput_=0;return 0.0f;}
 if(!edaFinite(target)||!edaFinite(pitch)||!edaFinite(verifiedRate)||!edaFinite(pitchRate)){
   fail(EDAFault::InvalidSensor);return 0.0f;
 }
 if(!edaFinite(dt)||!(dt>=0.004f && dt<=0.050f)){
   fail(EDAFault::InvalidTimeStep);return 0.0f;
 }
 valid_=true;
 const float a=dt/(dt+c_.gyroTauMs*0.001f);
 const float b=dt/(dt+c_.recoveryTauMs*0.001f);
 const float s=dt/(dt+c_.assistTauMs*0.001f);
 rate_+=a*(verifiedRate-rate_);
 recoveryRate_+=b*(verifiedRate-recoveryRate_);
 slowRate_+=s*(pitchRate-slowRate_);
 const float error=target-pitch, ae=fabsf(error);
 if(ae>peakError_)peakError_=ae;
 p_=c_.kp*error;
 assist_=0;
 if(c_.enableAssist && ae>=c_.assistMinError)
   assist_=clampf(c_.assistGain*slowRate_,-c_.assistMaxRate,c_.assistMaxRate);
 d_=-c_.kd*(rate_+assist_);
 // Kosullu integral: cikis doyumunda daha da buyuten birikimi engeller.
 if(ae<=c_.iEnableAngle && c_.ki>0.0f){
   if(c_.enableIRateGate && fabsf(rate_)>c_.maxRateForI){
     integral_*=clampf(1.0f-c_.iLeakPerSec*dt,0.0f,1.0f);gated_++;
   }else{
     float candidate=clampf(integral_+error*dt,-c_.iLimit/c_.ki,c_.iLimit/c_.ki);
     float proposed=p_+d_+c_.ki*candidate;
     // Motor limitine ulasan ham cikista ayni yone integral birikimini durdur.
     // Tepki kazanci buyudugunde de tasma olmamasi icin kazanc oncesi limit korunur.
     if(!((proposed>c_.outputLimit && error>0)||(proposed< -c_.outputLimit && error<0))){
       integral_=candidate;integrated_++;
     }
   }
 }else integral_=0;
 i_=c_.ki*integral_;
 float raw=p_+i_+d_;
 if(fabsf(raw)>c_.outputLimit)saturated_++;
 float responseSpan=c_.responseFullError-c_.responseStartError;
 float blend=responseSpan>0.001f?clampf((ae-c_.responseStartError)/responseSpan,0,1):0.0f;
 float cmd=raw*(1.0f+(c_.responseMaxGain-1.0f)*blend);
 if(c_.enableRecovery && ae>=c_.recoveryMinError &&
    fabsf(recoveryRate_)>=c_.recoveryMinRate && error*recoveryRate_<0 && cmd*error>0){
   float recoverySpan=c_.recoveryFullRate-c_.recoveryMinRate;
   float r=recoverySpan>0.001f?clampf((fabsf(recoveryRate_)-c_.recoveryMinRate)/recoverySpan,0,1):0.0f;
   cmd*=1.0f+c_.recoveryExtraGain*r;
 }
 bool movingAway=error*verifiedRate<0;
 bool falling=ae>=c_.floorStartError && fabsf(verifiedRate)>=c_.floorMinRate && movingAway;
 if(!c_.enableFloor || ae<c_.recoveryHysteresis || !movingAway){holding_=false;holdSign_=0;}
 else if(falling){holding_=true;holdSign_=error>0?1:-1;}
 if(holding_ && (error>0?1:-1)==holdSign_){
   float floorSpan=c_.floorFullError-c_.floorStartError;
   float f=floorSpan>0.001f?clampf((ae-c_.floorStartError)/floorSpan,0,1):0.0f;
   float floorCmd=c_.floorMinOutput+(c_.floorMaxOutput-c_.floorMinOutput)*f;
   if(cmd*holdSign_<floorCmd)cmd=holdSign_*floorCmd;
 }
 // V1.3: Hizli aci degisiminde gecikmeden duzeltme ilkesi (PID_v1 tarzi hizli tepki);
 // PID_v1 kodu kopyalanmadi ve motor surucusu icermez.
 // Robot HEDEFTEN UZAKLASIRKEN (error * rate < 0) ek yakalama uygula.
 // Sadece PID komutu zaten dogru yone ise guclendir; ters yone zorlamaz.
 const bool escaping=(error*verifiedRate<0.0f);
 if(c_.enablePushCatch && escaping && ae>=c_.pushMinError &&
    fabsf(verifiedRate)>=c_.pushMinRate && cmd*error>0.0f){
   const float span=c_.pushFullRate-c_.pushMinRate;
   const float t=span>0.001f?clampf((fabsf(verifiedRate)-c_.pushMinRate)/span,0.0f,1.0f):1.0f;
   cmd+=(error>0.0f?1.0f:-1.0f)*c_.pushMaxExtra*t;
 }
 // Hedefe HIZLA YAKLASIRKEN integralin kisa sureli tasmasini azalt.
 // Oransal ve turev kontrolunu degistirme; frenlemek icin gecikme ekleme.
 if(c_.enableApproachBrake && !escaping && ae<=c_.brakeMaxError &&
    fabsf(verifiedRate)>=c_.brakeMinRate && error*verifiedRate>0.0f){
   const float span=c_.brakeFullRate-c_.brakeMinRate;
   const float t=span>0.001f?clampf((fabsf(verifiedRate)-c_.brakeMinRate)/span,0.0f,1.0f):1.0f;
   const float oldI=i_;
   integral_*=clampf(1.0f-c_.brakeIntegralLeak*t*dt,0.0f,1.0f);
   i_=c_.ki*integral_;
   cmd+=i_-oldI;
 }
 // V1.2: Merkez sakinlestirme yalnizca GERCEKTEN sakin durumlarda.
 // Hata buyurken ve govde hedeften uzaklasirken beklemeden kontrolu serbest birak.
 // Burada yeni motor yon gecikmesi / rampa YOKTUR.
 const bool movingAwayNow = (error * verifiedRate < 0.0f);
 const float absRate = fabsf(verifiedRate);
 const float urgentAway = c_.centerUrgentAwayRate>0.0f ? c_.centerUrgentAwayRate : 0.60f;
 const float blockAway  = c_.centerBlockAwayRate>0.0f  ? c_.centerBlockAwayRate  : 0.35f;
 const bool urgentMotion = holding_ ||
     (movingAwayNow && absRate >= urgentAway) ||
     (ae >= c_.centerExitError) ||
     (absRate >= c_.centerExitRate);
 if(c_.enableCenterFilter){
   if(urgentMotion){
     centerActive_=false;
   }else if(!centerActive_ && ae<=c_.centerEnterError &&
           absRate<=c_.centerEnterRate &&
           !(movingAwayNow && absRate>=blockAway)){
     centerActive_=true;
   }
   // Geri dusus basliyorsa, cok kucuk PID komutunu bile susturma.
   if(centerActive_ && !movingAwayNow &&
      fabsf(cmd)<c_.centerCommandThreshold){
     integral_=0.0f; i_=0.0f; cmd=0.0f;
   }
 }else centerActive_=false;
 lastOutput_=clampf(cmd,-c_.outputLimit,c_.outputLimit);
 if(!edaFinite(lastOutput_)){fail(EDAFault::InvalidOutput);}
 return lastOutput_;
}
