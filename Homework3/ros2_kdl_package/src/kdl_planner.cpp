#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}


/// 2a) New constructor - linear with trapezoidal shape
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    shape = TRAPEZOIDAL;
    path_type = LINEAR;

}

/// 2a) New constructor - circular with trapezoidal shape
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    shape = CUBIC;
    path_type = LINEAR;
}

/// 2a) New constructor - linear with cubic shape
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajRadius = _trajRadius;
    shape = TRAPEZOIDAL;
    path_type = CIRCULAR;

}


/// 2a) New constructor - circular with cubic shape
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius = _trajRadius;
    shape = CUBIC;
    path_type = CIRCULAR;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}


// 1a)
// New trapezoidal function
void KDLPlanner::trapezoidal_vel(double t_, double tc_, double tf_, double &s, double &s_dot, double &s_dotdot) {
  double sc_dotdot;
  sc_dotdot=1/(tf_*tc_-std::pow(tc_,2));

  if(0 <= t_ && t_<= tc_)
  {
    s = 0.5*sc_dotdot*std::pow(t_,2);
    s_dot = sc_dotdot*t_;
    s_dotdot = sc_dotdot;
  }
  else if(t_ <= tf_-tc_)
  {
    s =   sc_dotdot*tc_*(t_-tc_/2);
    s_dot = 0.5*sc_dotdot;
    s_dotdot = 0;
  }
  else if(t_ <= tf_)
  {
    s =  1 - 0.5*sc_dotdot*std::pow(tf_-t_,2);
    s_dot = 0;
    s_dotdot = 0;
  }


}

// 1b)
// New cubic polynomial function
void KDLPlanner::cubic_polynomial(double t_, double tf_, double &s, double &s_dot, double &s_dotdot) {
   
    double s0,s_final,v0,v_final,a0,a1,a2,a3;
    
    s0=0; //initial position
    s_final=1; //final position
    v0=0; //initial velocity
    v_final=0; //final velocity

    a0 = s0;
    a1 = v0;
    a2 = (3 * (s_final - s0) / std::pow(tf_,2)) - ((2 * v0 + v_final) / tf_);
    a3 = (-2 * (s_final - s0) / std::pow(tf_,3)) + ((v0 + v_final) / std::pow(tf_,2));

    s = a3 * std::pow(t_,3) + a2 * std::pow(t_,2) + a1 * t_ + a0; // position
    s_dot = 3 * a3 * std::pow(t_,2) + 2 * a2 * t_ + a1; // velocity
    s_dotdot = 6 * a3 * t_ + 2 * a2; // acceleration
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}



// 2b) and 2c)
trajectory_point KDLPlanner::compute_trajectory(double time)
{
  
    double s, s_dot, s_dotdot;
     trajectory_point traj;

    // Selection of the shape
    if (shape == CUBIC) {
        cubic_polynomial(time, trajDuration_, s, s_dot, s_dotdot);
    } else if (shape == TRAPEZOIDAL) {

        trapezoidal_vel(time, accDuration_, trajDuration_, s, s_dot, s_dotdot);
    }

    // Selection of the path
    if (path_type == CIRCULAR) // 2b) - it looks ok
    {
        // Circular trajectory in the y-z plane
        double theta = 2 * M_PI * s;
        double theta_dot = 2 * M_PI * s_dot;
        double theta_dotdot = 2 * M_PI * s_dotdot;

        traj.pos.x() = trajInit_.x(); // Fixed x-coordinate
        traj.pos.y() = trajInit_.y() - trajRadius * cos(theta);
        traj.pos.z() = trajInit_.z() - trajRadius * sin(theta);

        traj.vel.x() = 0; // No velocity in x
        traj.vel.y() = trajRadius * theta_dot * sin(theta);
        traj.vel.z() = -trajRadius * theta_dot * cos(theta);

        traj.acc.x() = 0; // No acceleration in x
        traj.acc.y() = trajRadius * (theta_dotdot * sin(theta) + theta_dot * theta_dot * cos(theta));
        traj.acc.z() = -trajRadius * (theta_dotdot * cos(theta) - theta_dot * theta_dot * sin(theta));
    }
    else if (path_type == LINEAR) // 2c) - check it
    {

      traj.pos = trajInit_ + (trajEnd_ - trajInit_) * s;
      traj.vel = s_dot * (trajEnd_ - trajInit_).normalized();
      traj.acc = Eigen::Vector3d::Zero();

	}
    return traj;
}