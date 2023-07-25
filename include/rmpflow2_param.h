#pragma once

#include <rmpflow/root.h>
#include <typeinfo>

namespace RMPflow2 {

void setAttractorType1(LegPoint &p) {
  p.at_.type_ = 1;
  p.at_.alpha_ =
      125000.0f*0.75f; // accelation. 大きくすると速くトップスピードになります．
  p.at_.beta_ =125.0f; // damper. 多くするとゆっくりになります．トップスピードは
                       // alpha/beta[mm/s]が目安になります
  p.at_.c_ = 1e10f; // large enough
  p.at_.sg_ =
      200.0f; // standard distribution of disttance between the tip and the
             // destination.
             // 目標点までの距離がこの値のときに重みが(wu+wl)/2になり，近づくほど重みは大きくなります．
  p.at_.wu_ = 10.0f;   // maximum norm of reimannian metric. 重みの最大値
  p.at_.wl_ = 1.0f;   // minimum norm of reimannian metric．重みの最小値
  p.at_.gain_ = 1.0f; // 1より大きくすると直線的でなく，山なりな軌道を描きます．
}

void setAttractorType2(LegPoint &p) {
  p.at_.type_ = 2;
  p.at_.alpha_ = 1000.0f; // gain300
  p.at_.beta_ = 110.0f;    // damper
  p.at_.gamma_ = 1.0f;    // inertia
}

// SOLでは使いません
void setDragAvoidance(LegPoint &p) {
  p.da_.alpha_ = 4.0f; // 10.0f
  p.da_.beta_ = 2.0f;  // 2.0f
  p.da_.rw_ = 40.0f;   // 40
  p.da_.sv_ = 20.0f;   // 10.0f
  p.da_.lambda_ = 1.0e-4f;
  p.da_.vel_margin_ = 0.0f; // 0.0f
  p.da_.d_th_ = 25.0f;      // 20.0f
}

// SOLでは使いません
void setObstacleAvoidance(LegPart &p) {
  p.oa_.alpha_ = 4.0f;       // 2.0f
  p.oa_.beta_ = 2.0f;        // 2.0f
  p.oa_.rw_ = 40.0f;         // 30.0f
  p.oa_.sv_ = 20.0f;         // 10.0f
  p.oa_.lambda_ = 1.0e-4f;   // 1.0e-4f
  p.oa_.vel_margin_ = 10.0f; // 0.0f
}

void setSelfCollisionAvoidance(LegPart &p) {
  p.sca_.alpha_ = 0.2f; // 大きくすると強く反発します
  p.sca_.beta_ = 5.0f; // 大きくすると近づく速度に比例して強い力が発生します．
  p.sca_.rw_ = 50.0f; // 脚同士の距離がこれ以下になると反発力が発生します．
  p.sca_.sv_ =
      30.0f; // 脚同士が近づく速度がこの値よりも大きいとシグモイド関数により重みが大きくなります．
  p.sca_.lambda_ =
      1.0e-4f; // 小さな値が推奨です．大きくすると反発と言うよりは回避対象の脚の周りを回るようにして衝突を回避します．
  p.sca_.vel_margin_ = 0.0f;
}

// 要調整
void setJointLimti(Root2 &root) {
  root.jl_.alpha_ = 1.0e8f; // 大きくすると強く反発します
  root.jl_.beta_ =
      1.0e1f; // 大きくすると近づく速度に比例して強い力が発生します．
  root.jl_.rw_ =
      M_PI * 5.0f /
      180.0f; // 制限値と現在の角度差がこれ以下になると反発力が発生します．
  root.jl_.sv_ =
      M_PI; // 制限値に近づく速度がこの値よりも大きいとシグモイド関数により重みが大きくなります．
  root.jl_.lambda_ = 1.0e4f; // ここは大きくても大丈夫です.

  root.jl_.ll_ = {{-M_PI / 3.0f, -10.0f * M_PI / 180.0f,
                   -50.0f * M_PI / 180.0f}}; // 各関節角度の下限値
  root.jl_.lu_ = {{M_PI / 3.0f, 135.0f * M_PI / 180.0f,
                   160.0f * M_PI / 180.0f}}; // 各関節角度の上限値
}

void setPoliciesParam(Root2 &r, const int attractor_type) {
  if (attractor_type == 1)
    setAttractorType1(r.tip_);
  else if (attractor_type == 2)
    setAttractorType2(r.tip_);
  setDragAvoidance(r.tip_);
  for (LegPart &p : r.leg_parts_) {
    setObstacleAvoidance(p);
    setSelfCollisionAvoidance(p);
  }
  setJointLimti(r);
}

} // namespace RMPflow2