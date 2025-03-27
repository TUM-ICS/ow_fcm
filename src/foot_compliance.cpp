/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#include <ow_fcm/foot_compliance.h>

namespace ow_fcm
{

  FootCompliance::FootCompliance() : Base("foot_compliance"),
                                     enabled_(false),
                                     Xoff_l_(ow::CartesianState::Zero()),
                                     Xoff_r_(ow::CartesianState::Zero()),
                                     kick_(ow::Moment::Zero())
  {
  }

  FootCompliance::~FootCompliance()
  {
  }

  bool FootCompliance::init(const ow::Parameter &parameter, ros::NodeHandle &nh)
  {
    // get global ow parameter
    ow::Scalar frequency = parameter.get<ow::Scalar>("loop_rate");
    ss_dur_ = ros::Duration(parameter.get<ow::Scalar>("t_single_support"));

    // load module parameter
    parameter_.add<bool>("enabled");
    parameter_.add<ow::Moment>("kick_torque");
    parameter_.add<ow::Scalar>("mu_tresh");
    parameter_.add<ow::Scalar>("f_normal_thesh");
    parameter_.add<ow::Vector6>("virtual_dynamics/M");
    parameter_.add<ow::Vector6>("virtual_dynamics/D");
    parameter_.add<ow::Vector6>("virtual_dynamics/S");
    parameter_.add<ow::Vector6>("gains/K_track");
    parameter_.add<ow::Vector3>("gains/Ko_ss_swing");
    parameter_.add<ow::Vector3>("gains/Ko_ss_support");
    parameter_.add<ow::Vector3>("gains/Ko_ds");
    parameter_.add<ow::Vector6>("limits/pos");
    parameter_.add<ow::Vector6>("limits/vel");

    // load module parameter
    if (!parameter_.load(nh, "foot_compliance"))
    {
      ROS_ERROR("%s::initialize: Config loading failed.", Base::name().c_str());
      return false;
    }

    // init members
    parameter_.get("enabled", enabled_);
    parameter_.get("kick_torque", kick_);
    parameter_.get("mu_tresh", mu_tresh_);
    parameter_.get("f_normal_thesh", f_normal_thesh_);

    // init virtual dynamics
    M_inv_ =
        parameter_.get<ow::Vector6>("virtual_dynamics/M").asDiagonal().inverse();
    D_ = parameter_.get<ow::Vector6>("virtual_dynamics/D").asDiagonal();
    S_ = parameter_.get<ow::Vector6>("virtual_dynamics/S").asDiagonal();

    // init gain matrices
    K_track_ = parameter_.get<ow::Vector6>("gains/K_track").asDiagonal();
    Ko_ss_swing_ = parameter_.get<ow::Vector3>("gains/Ko_ss_swing").asDiagonal();
    Ko_ss_support_ =
        parameter_.get<ow::Vector3>("gains/Ko_ss_support").asDiagonal();
    Ko_ds_ = parameter_.get<ow::Vector3>("gains/Ko_ds").asDiagonal();

    // get the integrator limits
    ow::CartesianVector pos_rpy = parameter_.get<ow::Vector6>("limits/pos");
    ow::CartesianVelocity XP = parameter_.get<ow::Vector6>("limits/vel");

    // convert euler rpy to to quaternions repesentation
    ow::CartesianPosition X_lo, X_ul;
    X_lo.linear() = -pos_rpy.linear();
    X_lo.angular() = ow::Rotation3::RPY(-pos_rpy.angular());
    X_ul.linear() = pos_rpy.linear();
    X_ul.angular() = ow::Rotation3::RPY(pos_rpy.angular());

    // state integrators for virtual system
    state_integ_l_.reset(new Integrator(1. / frequency, X_lo, X_ul, -XP, XP));
    state_integ_r_.reset(new Integrator(1. / frequency, X_lo, X_ul, -XP, XP));

    return true;
  }

  void FootCompliance::update(ow::Flags &flags,
                              const ow::CartesianState &Xreal_l_w,
                              const ow::CartesianState &Xref_l_w,
                              const ow::CartesianState &Xreal_r_w,
                              const ow::CartesianState &Xref_r_w,
                              const ow::Wrench &W_l,
                              const ow::Wrench &W_r)
  {
    if (!enabled_)
    {
      // not activated
      return;
    }

    // wrench applied to virtual system
    ow::Wrench Wv_l = ow::Wrench::Zero();
    ow::Wrench Wv_r = ow::Wrench::Zero();

    // single support phase, one foot on the ground
    if (flags.walkingPhase() == ow::Flags::SINGLE_SUPPORT)
    {
      if (flags.supportFoot().isLeft())
      {
        // right = swing, left = support
        updateSingleSupport(flags,
                            Wv_r, Wv_l,
                            Xreal_r_w.pos(),
                            Xref_r_w.pos(),
                            W_r, W_l);
      }
      else
      {
        // right = support, left = swing
        updateSingleSupport(flags,
                            Wv_l, Wv_r,
                            Xreal_l_w.pos(),
                            Xref_l_w.pos(),
                            W_l, W_r);
      }
    }

    // double support phase, both feet on the ground
    if (flags.walkingPhase() == ow::Flags::DOUBLE_SUPPORT)
    {
      updateDoubleSupport(flags, Wv_l, Wv_r, W_l, W_r);
    }

    // apply to virtual dynamics
    updateVirtualDynamics(Wv_l, Wv_r);
  }

  void FootCompliance::updateSingleSupport(
      ow::Flags &flags,
      ow::Wrench &Wv_swing,
      ow::Wrench &Wv_support,
      const ow::CartesianPosition &X_swing_w,
      const ow::CartesianPosition &X_ref_swing_w,
      const ow::Wrench &W_swing,
      const ow::Wrench &W_support)
  {
    ow::Scalar height_offset = 0.0;

    // swing leg compliance in the second half of single support phase
    if (flags.quater() == flags.SINGLE_SUPPORT_3 ||
        flags.quater() == flags.SINGLE_SUPPORT_4)
    {
      ow::Scalar f_normal = W_swing.force().z();
      ow::Scalar mu_norm = W_swing.moment().norm();

      // orientation compliance
      if (mu_norm > mu_tresh_)
      {
        Wv_swing.angular() += Ko_ss_swing_ * W_swing.angular();
      }

      // early stopping if swing leg hits something and not at the first/final step
      if (f_normal > f_normal_thesh_ && 
          !(flags.step() == ow::Flags::FIRST || flags.step() == ow::Flags::FINAL))
      {
        flags.eventIn() = ow::Flags::EARLY_TOUCH_DOWN;
        ROS_WARN_STREAM(
            "Early Contact Event: f_normal=" << f_normal << ">" << f_normal_thesh_);
      }

      // single support time is elapsed check if we have support
      if (flags.elapsedSingleSupport() > ss_dur_)                               
      { 
        if (f_normal > 50 &&  
            !(flags.step() == ow::Flags::FIRST || flags.step() == ow::Flags::FINAL)) 
        {
          flags.eventIn() = ow::Flags::STABLE_FOOT_HOLD;
          ROS_WARN_STREAM("STABLE_FOOT_HOLD EVENT");
        }
        else
        {
          flags.eventIn() = ow::Flags::STABLE_FOOT_HOLD;
        }

        // ROS_WARN_STREAM("DELAYED by: " << (flags.elapsedSingleSupport() - ss_dur_).toSec() << "s with fz=" << f_normal);
        // height_offset = -0.8;                                           
      }
    }

    // modify the reference height
    ow::CartesianPosition X_ref_swing_w_mod = X_ref_swing_w;
    X_ref_swing_w_mod.pos().z() += height_offset;

    // get the rotation of world into foot frame
    ow::AngularPosition Q_w_swing = X_swing_w.angular().inverse();

    // compute the cartesian tracking error of the swing foot in foot frame
    ow::CartesianVector E_foot = ow::cartesianError(X_ref_swing_w_mod, X_swing_w);

    E_foot.linear() = Q_w_swing * E_foot.linear();
    E_foot.angular() = Q_w_swing * E_foot.angular();
    Wv_swing += K_track_ * E_foot;

    // supporting leg compliance during the single support phase
    Wv_support.angular() += Ko_ss_support_ * W_support.angular();
  }

  void FootCompliance::updateDoubleSupport(ow::Flags &flags,
                                           ow::Wrench &Wv_l,
                                           ow::Wrench &Wv_r,
                                           const ow::Wrench &W_l,
                                           const ow::Wrench &W_r)
  {
    // orientation compliance
    Wv_l.angular() += Ko_ds_ * W_l.angular();
    Wv_r.angular() += Ko_ds_ * W_r.angular();

    // kick before starting single suport phase
    if (flags.quater() == ow::Flags::DOUBLE_SUPPORT_4 ||
        flags.quater() == ow::Flags::DOUBLE_SUPPORT_3)
    {
      Wv_l.angular() += kick_;
      Wv_r.angular() += kick_;
    }
  }

  void FootCompliance::updateVirtualDynamics(const ow::Wrench &W_l,
                                             const ow::Wrench &W_r)
  {
    // cartesian error for pulling system to zero offset                        
    ow::CartesianVector E_l =
        ow::cartesianError(ow::CartesianPosition::Identity(), Xoff_l_.pos());

    // integrate right foot dynamic in foot frame
    Xoff_l_.acc() = M_inv_ * (W_l - D_ * Xoff_l_.vel() + S_ * E_l);
    Xoff_l_.acc() = (Xoff_l_.acc().array().abs() < 1e-4).select(0, Xoff_l_.acc()).matrix();

    state_integ_l_->update(Xoff_l_);

    // cartesian error for pulling system to zero offset
    ow::CartesianVector E_r =
        ow::cartesianError(ow::CartesianPosition::Identity(), Xoff_r_.pos());

    // integrate right foot dynamic in foot frame
    Xoff_r_.acc() = M_inv_ * (W_r - D_ * Xoff_r_.vel() + S_ * E_r);
    Xoff_r_.acc() = (Xoff_r_.acc().array().abs() < 1e-4).select(0, Xoff_r_.acc()).matrix();
    state_integ_r_->update(Xoff_r_);
  }

  const ow::CartesianState &FootCompliance::Xoff_r() const
  {
    return Xoff_r_;
  }

  const ow::CartesianState &FootCompliance::Xoff_l() const
  {
    return Xoff_l_;
  }

} // namespace ow_fcm
