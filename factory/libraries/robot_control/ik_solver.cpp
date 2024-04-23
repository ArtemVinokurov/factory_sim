#include "solver.h"




int ikSolver::CartToJnt(KDL::ChainIkSolverVelMimicSVD& ik_solver, const KDL::JntArray& q_init,
                                   const KDL::Frame& p_in, KDL::JntArray& q_out, const unsigned int max_iter,
                                   const Eigen::VectorXd& joint_weights, const Twist& cartesian_weights) const
{
  double last_delta_twist_norm = DBL_MAX;
  double step_size = 1.0;
  KDL::Frame f;
  KDL::Twist delta_twist;
  KDL::JntArray delta_q(q_out.rows()), q_backup(q_out.rows());
  Eigen::ArrayXd extra_joint_weights(joint_weights.rows());
  extra_joint_weights.setOnes();

  q_out = q_init;
  RCLCPP_DEBUG_STREAM(getLogger(), "Input: " << q_init);

  unsigned int i;
  bool success = false;
  for (i = 0; i < max_iter; ++i)
  {
    fk_solver_->JntToCart(q_out, f);
    delta_twist = diff(f, p_in);
    RCLCPP_DEBUG_STREAM(getLogger(), "[" << std::setw(3) << i << "] delta_twist: " << delta_twist);

    // check norms of position and orientation errors
    const double position_error = delta_twist.vel.Norm();
    const double orientation_error = ik_solver.isPositionOnly() ? 0 : delta_twist.rot.Norm();
    const double delta_twist_norm = std::max(position_error, orientation_error);
    if (delta_twist_norm <= params_.epsilon)
    {
      success = true;
      break;
    }

    if (delta_twist_norm >= last_delta_twist_norm)
    {
      // if the error increased, we are close to a singularity -> reduce step size
      double old_step_size = step_size;
      step_size *= std::min(0.2, last_delta_twist_norm / delta_twist_norm);  // reduce scale;
      KDL::Multiply(delta_q, step_size / old_step_size, delta_q);
      RCLCPP_DEBUG(getLogger(), "      error increased: %f -> %f, scale: %f", last_delta_twist_norm, delta_twist_norm,
                   step_size);
      q_out = q_backup;  // restore previous unclipped joint values
    }
    else
    {
      q_backup = q_out;  // remember joint values of last successful step
      step_size = 1.0;   // reset step size
      last_delta_twist_norm = delta_twist_norm;

      ik_solver.CartToJnt(q_out, delta_twist, delta_q, extra_joint_weights * joint_weights.array(), cartesian_weights);
    }

    clipToJointLimits(q_out, delta_q, extra_joint_weights);

    const double delta_q_norm = delta_q.data.lpNorm<1>();
    RCLCPP_DEBUG(getLogger(), "[%3d] pos err: %f  rot err: %f  delta_q: %f", i, position_error, orientation_error,
                 delta_q_norm);
    if (delta_q_norm < params_.epsilon)  // stuck in singularity
    {
      if (step_size < params_.epsilon)  // cannot reach target
        break;
      // wiggle joints
      last_delta_twist_norm = DBL_MAX;
      delta_q.data.setRandom();
      delta_q.data *= std::min(0.1, delta_twist_norm);
      clipToJointLimits(q_out, delta_q, extra_joint_weights);
      extra_joint_weights.setOnes();
    }

    KDL::Add(q_out, delta_q, q_out);

    RCLCPP_DEBUG_STREAM(getLogger(), "      delta_q: " << delta_q);
    RCLCPP_DEBUG_STREAM(getLogger(), "      q: " << q_out);
  }

  int result = (i == max_iter) ? -3 : (success ? 0 : -2);
  RCLCPP_DEBUG_STREAM(getLogger(), "Result " << result << " after " << i << " iterations: " << q_out);

  return result;
}