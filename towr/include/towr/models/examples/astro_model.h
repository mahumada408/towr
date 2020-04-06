#pragma once

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot Astro.
 */
class AstroKinematicModel : public KinematicModel {
public:
  AstroKinematicModel () : KinematicModel(4)
  {
    const double x_nominal_b = 0.133;
    const double y_nominal_b = 0.131;
    const double z_nominal_b = -0.20;

    nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
    nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_ << 0.25, 0.20, 0.10;
  }
};

/**
 * @brief The Dynamics of the quadruped robot Astro.
 */
class AstroDynamicModel : public SingleRigidBodyDynamics {
public:
  AstroDynamicModel() : SingleRigidBodyDynamics(20,
                      0.1826, 0.03903, 0.04894, 0.0, 0.0, 0.0,
                      4) {}
};

} /* namespace towr */