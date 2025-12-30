#ifndef __KrenMdl_HPP
#define __KrenMdl_HPP

void integr(float dt, float inp, float tm, float& out);
void intert(float dt, float inp, float k, float tm, float& out);
void rldiff(float dt, float dinp, float tdf, float tm, float& out);
void saturate(float& val, float min, float max);
void saturatePi(float& val);
float my_rand(float min, float max);


class KrenMdl {
protected:
public:
  float mTf;  /// Время интергирования от скорости до значения угла гиросокпа, и определяется его параметрами 
  float mTv;  /// Вhемя интергирования от приложеной силы на скорость, зависит от массы коптера и мощности двигателя
  float mTm;  /// время инерции двигателя и ESC проеобразователя частоты
  float mFi;  /// значение угла гиросокпа +-180 ` или -+ 3.141592654 Rad Tf = 1/34.907 = 0.02864789 s
  float mVi;  /// скорость изменения угла гиросокпа в диапазоне +-2000  `/s ( или можно 34.907  Rad/s 34907  mRad/s)
  float mAc;  /// ускорение, которое пропорционально силе тяги
  float mPreAc;  /// ускорение, которое пропорционально силе тяги

  KrenMdl();
  KrenMdl(float Tf, float Tv, float Tm);
  
  void setParam (float Tf, float Tv, float Tm);
  float getFi();
  float getVi();
  float getTr();
  float updateMdl(float dt, float inp);
  char* print(char* msg);
};

#endif /* __KrenMdl_HPP */