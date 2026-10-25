#ifndef EDA_DENGEBOT_H
#define EDA_DENGEBOT_H
#include <stdint.h>
// E.D.A Balance PID 1.5.2 - PID_v1 ilkelerinden esinlenen BAGIMSIZ kontrolcu.
// PID_v1 kutuphanesinin kodunu kopyalamaz ve yuklenmesini gerektirmez.
struct EDAPIDConfig {
 float kp,ki,kd, iLimit, iEnableAngle, maxRateForI, iLeakPerSec;
 float outputLimit, gyroTauMs, recoveryTauMs, assistTauMs;
 float assistMinError, assistGain, assistMaxRate;
 float responseStartError,responseFullError,responseMaxGain;
 float recoveryMinError,recoveryMinRate,recoveryFullRate,recoveryExtraGain;
 float floorStartError,floorFullError,floorMinRate,floorMinOutput,floorMaxOutput;
 float recoveryHysteresis;
 bool enableAssist,enableIRateGate,enableRecovery,enableFloor;
 bool enableCenterFilter;
 float centerEnterError,centerExitError,centerEnterRate,centerExitRate,centerCommandThreshold;
 // V1.3: itme aninda erken yakalama ve hedefe yaklasirken tasma azaltma.
 bool enablePushCatch;
 float pushMinError,pushMinRate,pushFullRate,pushMaxExtra;
 bool enableApproachBrake;
 float brakeMaxError,brakeMinRate,brakeFullRate,brakeIntegralLeak;
 // V1.4: merkez filtresinde onceden koda gomulu olan esikler (derece/saniye).
 // <=0 verilirse eski varsayilan kullanilir (geriye uyumluluk).
 float centerUrgentAwayRate;  // hedeften uzaklasma bu hizi gecince filtre hemen birakilir (varsayilan 0.60)
 float centerBlockAwayRate;   // uzaklasma bu hizdaysa filtre devreye girmez (varsayilan 0.35)
};
// V1.4: Guvenli BASLANGIC ayarlari. Tum gelismis ozellikler KAPALI, Ki=0.
// Kp/Kd robota gore ayarlanmalidir (birimler: cikis/derece, cikis/(derece/s)).
// Once yalniz P ve D ile dengele, sonra Ki, sonra ozellikleri tek tek ac.
EDAPIDConfig edaDefaultConfig();
enum class EDAFault : uint8_t { None, InvalidConfig, InvalidSensor, InvalidTimeStep, InvalidOutput };
class EDABalancePID {
public:
 explicit EDABalancePID(const EDAPIDConfig &cfg);
 void reset(float initialRate=0.0f);
 bool setConfig(const EDAPIDConfig &cfg, bool resetState=true);
 bool setTunings(float kp,float ki,float kd, bool resetIntegral=true);
 EDAPIDConfig getConfig() const {return c_;}
 bool lastUpdateValid() const {return valid_;}
 bool faulted() const {return fault_ != EDAFault::None;}
 EDAFault fault() const {return fault_;}            // en son hata
 EDAFault firstFault() const {return firstFault_;}  // clearFault()'a kadar ILK hata
 // Istege bagli: true ise hata kilitliyken update() 0 dondurur (state ilerlemez); clearFault() beklenir. Varsayilan false.
 void setZeroOutputOnFault(bool on){zeroOnFault_=on;}
 bool zeroOutputOnFault() const {return zeroOnFault_;}
 void clearFault();
 bool configValid() const {return configValid_;}
 float lastOutput() const {return lastOutput_;}
 // update(): yeni gecerli IMU ornegi icin dt=0.004..0.050 saniye. Hata motor komutu degildir.
 float update(float target,float pitch,float verifiedRate,float pitchRate,float dt);
 float pTerm()const{return p_;} float iTerm()const{return i_;}
 float dTerm()const{return d_;} float filteredRate()const{return rate_;}
 float recoveryRate()const{return recoveryRate_;}
 float slowRate()const{return slowRate_;} float assistRate()const{return assist_;}
 bool recoveryHolding()const{return holding_;}
 bool centerFiltered()const{return centerActive_;}
 uint32_t saturatedSamples()const{return saturated_;}
 uint32_t gatedSamples()const{return gated_;}
 uint32_t integratedSamples()const{return integrated_;}
 float peakError()const{return peakError_;}
private:
 EDAPIDConfig c_;
 float integral_,p_,i_,d_,rate_,recoveryRate_,slowRate_,assist_,peakError_;
 bool holding_,centerActive_;
 int8_t holdSign_;
 uint32_t saturated_,gated_,integrated_;
 bool valid_;
 bool configValid_;
 EDAFault fault_;
 EDAFault firstFault_;
 bool zeroOnFault_;
 void fail(EDAFault reason);
 float lastOutput_;
};
#endif
