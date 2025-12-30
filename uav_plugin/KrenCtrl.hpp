#ifndef __KrenCtrl_HPP
#define __KrenCtrl_HPP

#include "KrenMdl.hpp"

class KrenCtrl : public KrenMdl {
public:
  float loldVi=0, Us=0, Us_prev = 0,Ui=0, mdVi=0, Uv=0, Um=0, Umm=0, erVi1= 0, oUi = 0; // Uv=0, 
  float mVid = 0;
  float mVold = 0;
  float muMd = 0;
  float uMold = 0;
  float klmf, /// коэффициент калмана углом
        klmv, /// коэффициент калмана скоростью угла
        Kpf, /// коэффициент в контуре управления углом
        Kdf, /// коэффициент дифференцирования в контуре управления углом
        Kpv, /// коэффициент в контуре управления скоростью изменения угла
        Kdv, /// коэффициент дифференцирования в контуре управления скоростью изменения угла
        mTmu,/// время инерции контура по модульному оптимуму
        mTe, /// время интергратора  ограничивающего скорость изменения управляющего сигнала ESC
        mTi; /// Время интегрирования контура по углу
        //mTvr,/// оценка времени интегрированя объекта контура по скорости изменения угла mTvr = mTv при настроенной системе
        //mTmr;/// оценка времени инерции двигателя и проеобразователя частоты mTmr = mTm при настроенной системе
        //mTmm;/// время дополнительной инерции
  int CtrlType  = 0;// _slCtrl_manual;
  void UdateKalman(float Fi, float Vi);
  void setCtrlParam();
  KrenCtrl(float Tf, float Tv, float Tm, float Tmi);
  KrenCtrl();
  float updateCtrl(float dt, float setFi, float Fi, float Vi);
  float GetUi();
  void reset();
};

#endif /* __KrenCtrl_HPP */