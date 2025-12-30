#include "KrenCtrl.hpp"
#include <string>
#include <math.h>
#include <algorithm>
#include "imu.h"
#define _slCtrl_2PD 1
#define _slCtrl_Modal 2
#define _slCtrl_Slide 3
#define _slCtrl_MaxFs 4
#define __SelCtrl 3

KrenCtrl::KrenCtrl(float Tf, float Tv, float Tm, float Tmi){
  mTi = 0.2;
  mTe = 0.01; 
  setParam(Tf, Tv, Tm);
  mTmu = mTm;
  setCtrlParam();
}
KrenCtrl::KrenCtrl(){
  mTi = 0.2;
  mTe = 0.01; 
  mTmu = mTm;
  klmf = klmv = 0.1;
  setCtrlParam();
}
void KrenCtrl::setCtrlParam(){
#if (__SelCtrl==_slCtrl_2PD)
  // Kpv = mTe*mTv/(45.0*mTmu*mTmu);
  // Kdv = mTe*mTv/(2.0*mTmu); // SO 4--5
  // Kpf = mTf / (15 * mTmu); // MO  15-30
  Kpv = 0.03;
  Kdv = 0.02;
  Kpf = 2.0;
#endif
#if (__SelCtrl==_slCtrl_Modal)
  Kd  = mTmi / ( mTm * 2 );
  Kp  = mTv / ( 2.23 * 2 * mTm);
  Kpf = mTf / ( 2.23 * 4 * mTm );
  Kdf = 1.25 * mTm * Kpf / mTf;
#endif
#if (__SelCtrl==_slCtrl_Slide)
  // mTi = mTe;
  // mTmu = mTm;
  //Kd  = 1000;
  Kdv = 0.05;
  Kpv  = 0.1;
  Kpf = 1.0;
#endif
#if (__SelCtrl==_slCtrl_MaxFs)
/*   __       ___   __            __   __   ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi__Kp__|s |_|Kd|_|_1__|_uM_
 Fi_|- |       Vi__|- |  |AC|*Ac_|g |      |sTmi|
    |__|           |__|          |n_|      |____|
*/
  Kd  = mTmi / ( mTm * 2 );
  Kp  = mTv / ( 2 * mTm);
  Kpm  = 2*Kd/(mTv*mTmi);
  Kpf = mTf / ( 2.2 * 2.5 * mTm ); // opt
  //Kpf = mTf / ( 5 * 5 * mTm );
#endif
}

//#define stabkalm 1
//#define stabkalm 0.5
#define stabkalm 0.05
//#define stabkalm 0.01
//#define __Kalman
#ifdef  __Kalman
  #define __KlFmn 0.0042
  #define __KlFmx 0.52
  #define __KlVmn 0.042
  #define __KlVmx 0.52
#else
  #define __KlFmn stabkalm
  #define __KlFmx stabkalm
  #define __KlVmn stabkalm
  #define __KlVmx stabkalm
#endif // 

float lineTrans(float x, float x1, float x2, float y1, float y2){
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}
void KrenCtrl::UdateKalman(float Fi, float Vi){
  //mFi = Fi; mVi = Vi; return;
  float uI2 = ((Ui/100)*(Ui/100));
  float kKFi = lineTrans(uI2, 0 , 100 , __KlFmn, __KlFmx); //kKFi = 1.0;
  float kKVi = lineTrans(uI2, 0 , 100 , __KlVmn, __KlVmx); //kKVi = 1.0;
  mFi += (Fi - mFi) * klmf;
  mVi += (Vi - mVi) * klmv;
}

auto sat = [](float x){
    if (x >  1.0f) return  1.0f;
    if (x < -1.0f) return -1.0f;
    return x;
};
static float dVi = 0.0f;
float KrenCtrl::updateCtrl(float dt, float setFi, float Fi, float Vi){
  UdateKalman(Fi, Vi);  
  float erFi = setFi - mFi;
  float erVi = Kpf * erFi  - mVi;
  rldiff(dt, erVi - erVi1, 1.0, mTmu, mdVi); erVi1 = erVi;
#if (__SelCtrl==_slCtrl_2PD)
  Us =  erVi * Kpv + mdVi * Kdv;
  Us -= Uv;
#endif
#if (__SelCtrl==_slCtrl_Modal)
/*   __       ___   __       __   __         ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi_|Kv|_|+ |_|Kdv|_|_1__|_uM_
 Fi_|- |       Vi__|- |       Ac_|- |       |sTmi|
    |__|           |__|          |__|       |____|
*/
  //               {---eVi-|---eFi-----|------}
  uI = Kd * ( Kp * ( Kpf * (setFi - mFi) - mVi) - getTr() );
#endif //(__SelCtrl==_slCtrl_Modal)
#if (__SelCtrl==_slCtrl_Slide)
 //  Us = (Kpv * erVi + Kdv * mdVi - Uv > 0) ? 1000 : (-1000);
  // if((Us * oUi) < 0) {  oUi = Us;  Us = 0; } oUi = Us;

  float s = (Kpv * erVi + Kdv * mdVi - Uv); 
  float Umax = 1000.0f;
  float phi  = 50.0f;             
  Us = Umax * sat(s / phi);

#endif //(__SelCtrl==_slCtrl_Slide)
#if (__SelCtrl==_slCtrl_MaxFs)
/*   __       ___   __            __   ___   ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi__Kp__|s |_|Kdv|_|_1__|_uM_
 Fi_|- |       Vi__|- |  |AC|*Ac_|g |       |sTmi|
    |__|           |__|          |n_|       |____|
*/
  float eVi = Kpf * (setFi - mFi) - mVi;

  float xp = getTr();
  if( (eVi*eVi + 1.4*xp*xp) < 2000000 ){
    uI = Kd * ( Kp * eVi - getTr() );
  }
  else{
    // maxFast method
    if(xp > 0) xp = getTr(); else xp = -getTr();
    uI = ( ( Kpm * eVi  + xp ) > 0 )? 1000:(-1000);
    if((uI * oUi) < 0) {  oUi = uI;  uI = 0; } oUi = uI; // через ноль
 }
#endif //(__SelCtrl==_slCtrl_MaxFs)

saturate(Us, -1000.0f, 1000.0f);
// lọc Uv theo Te
integr(dt, Us, mTe, Uv);
saturate(Uv, -1000.0f, 1000.0f);
integr(dt, Uv, mTi, Ui);
saturate(Ui, -1000.0f, 1000.0f);

Um = Ui + Uv;
saturate(Um, -1000.0f, 1000.0f);
intert(dt, Um,1.0,mTmu,Umm);
rldiff(dt, Umm-uMold, mTmu, mTmu, muMd); uMold = Umm;
updateMdl(dt, muMd);
  //updateMdl(dt, Uv);
return Um;
}

float KrenCtrl::GetUi() { return Um; }