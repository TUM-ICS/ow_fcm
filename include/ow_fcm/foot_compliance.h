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

#ifndef OPEN_WALKER_FOOT_COMPLIANCE_H_
#define OPEN_WALKER_FOOT_COMPLIANCE_H_

#include <ow_core/interfaces/i_foot_compliance.h>

#include <ow_core/math.h>
#include <ow_core/algorithms.h>

/*!
 * \brief Open Walker foot compliance module namespace. These classes implement
 * the impedance-admitance controllers for the feet to track the reference
 * trajectories while adapting the foothold to the terrain conditions.
 */
namespace ow_fcm
{

/*!
 * \brief The FootCompliance class
 *
 * This class implements the FootCompliance module of the
 * openwalker framework. 
 * It adjusts the foot orientation in a compliant manner based on external 
 * forces measured by the FT sensors in both feet.
 */
class FootCompliance : 
  public ow::IFootCompliance
{
public:
  typedef ow::IFootCompliance Base;
  typedef ow_core::StateIntegrator<ow::CartesianState> Integrator;

protected:
  bool enabled_;
  ow::Parameter parameter_;     //!< configuration of this module

  ros::Duration ss_dur_;        //!< single support phase duration

  // outports
  ow::CartesianState Xoff_l_;   //!< the offset state of the left foot.
  ow::CartesianState Xoff_r_;   //!< the offset state of the right foot.

  // virtual system in foot frame
  ow::Matrix6 M_inv_;     //!< Inverted mass matrix for foot virtual dynamics.
  ow::Matrix6 D_;         //!< Damping matrix for foot virtual dynamics.
  ow::Matrix6 S_;         //!< Spring constant for foot virtual dynamics.

  // gains 
  ow::Matrix6 K_track_;       //!< cartesian spline tracking gains.
  ow::Matrix3 Ko_ss_swing_;   //!< Orientation gains for swing foot compliance.
  ow::Matrix3 Ko_ss_support_; //!< Orientation gains for single support phase.
  ow::Matrix3 Ko_ds_;         //!< Orientation gains for double support phase.

  ow::Moment kick_;           //!< Foot ankle kick before single support phase.
  ow::Scalar mu_tresh_;       //!< Force torque sensor mu threshold.
  ow::Scalar f_normal_thesh_; //!< Normal force theshold for stopping swing foot

  // algorithms
  std::unique_ptr<Integrator> state_integ_l_; //!< Integrator for left foot
  std::unique_ptr<Integrator> state_integ_r_;  //!< Integrator for right foot.

public:
  /*!
  * \brief FootCompliance Default constructor.
  *
  */
  FootCompliance();

  /*!
   * \brief Desturctor
   */
  virtual ~FootCompliance();

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState contains the offset pose off on the
  *    right foot with respect to the reference foot frame R.
  */
  virtual const ow::CartesianState& Xoff_r() const;

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState contains the offset pose off on the
  *    right foot with respect to the reference foot frame L.
  */
  virtual const ow::CartesianState& Xoff_l() const;

  /*!
   * \brief Update function of the module. It computes the feet offsets to
   *    increase the stability of the foot contacts.
   *
   * \param flags
   *    Open Walker Flags.
   *
   * \param Xreal_l_w
   *    CartesianState of the left foot.
   *
   * \param Xref_l_w
   *    Reference CartesianState for the left foot.
   *
   * \param Xreal_r_w
   *    CartesianState of the right foot.
   *
   * \param Xref_r_w
   *    Reference CartesianState for the right foot
   *
   * \param W_l
   *    Left foot ankle wrench.
   *
   * \param W_r
   *    Right foot ankle wrench.
   */
  void update(ow::Flags& flags,
    const ow::CartesianState& Xreal_l_w,
    const ow::CartesianState& Xref_l_w,
    const ow::CartesianState& Xreal_r_w,
    const ow::CartesianState& Xref_r_w,
    const ow::Wrench& W_l,
    const ow::Wrench& W_r);

protected:
  /*!
   * \brief Initialization of FootCompliance module
   */
  virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh);

  /*!
   * \brief Update the foot compliance in double support phase.
   *
   * \param flags
   *    Open Walker Flags.
   *
   * \param W_l
   *    Left foot compliance wrench.
   *
   * \param W_r
   *    Right foor compliance wrench.
   *
   * \param W_ft_l
   *    Left foot FT sensor wrench.
   *
   * \param W_ft_r
   *    Right foot FT sensor wrench.
   */
  void updateDoubleSupport(ow::Flags& flags,
    ow::Wrench& W_l,
    ow::Wrench& W_r,
    const ow::Wrench& W_ft_l,
    const ow::Wrench& W_ft_r);

  /*!
   * \brief Update foot compliance in single support phase.
   *
   * \param flags
   *    Open Walker Flags.
   *
   * \param W_swing
   *  Foot compliance wrench for swing foot in single support phase.
   *
   * \param W_support
   *    Foot compliance wrench for support foot in single support phase.
   *
   * \param X_swing_w
   *    CartesianState of swing foot.
   *
   * \param X_ref_swing_w
   *    Reference CartesianState of swing foot.
   *
   * \param W_ft_swing
   *    FT sensor wrench of swing foot.
   *
   * \param W_ft_support
   *    FT sensor wrench of support foot.
   */
  void updateSingleSupport(ow::Flags& flags,
    ow::Wrench& W_swing,
    ow::Wrench& W_support,
    const ow::CartesianPosition& X_swing_w,
    const ow::CartesianPosition& X_ref_swing_w,
    const ow::Wrench& W_ft_swing,
    const ow::Wrench& W_ft_support);

  /*!
   * \brief Update virtual mass spring damper of the feet.
   *
   * \param W_l
   *    Left footr compliance wrench.
   *
   * \param W_r
   *    Right foot compliance wrench.
   */
  void updateVirtualDynamics(
    const ow::Wrench& W_l,
    const ow::Wrench& W_r);

};

}

#endif
