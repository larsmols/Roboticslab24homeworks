#ifndef KDLPlanner_H
#define KDLPlanner_H

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include <cmath>
#include "Eigen/Dense"

struct trajectory_point{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

//// New types of trajectories
enum Shape {TRAPEZOIDAL, CUBIC};
enum PathType {LINEAR, CIRCULAR};

class KDLPlanner
{

public:

    KDLPlanner();
    KDLPlanner(double _maxVel, double _maxAcc);

    // 2a) 
    // linear trapezoidal
    KDLPlanner(double _trajDuration, double _accDuration,
               Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);
    // linear cubic
    KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);
    // circular trapezoidal 
    KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, double _trajRadius);
    // circular cubic
    KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius);


    void CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                    double _radius, double _eqRadius);
    void createCircPath(KDL::Frame &_F_start,
                        KDL::Vector &_V_centre,
                        KDL::Vector &_V_base_p,
                        KDL::Rotation &_R_base_end,
                        double alpha,
                        double eqradius);

    // 1a) New trapezoidal function
    void trapezoidal_vel(double t_, double tc_, double tf_, double &s, double &s_dot, double &s_ddot);
    
    // 1b) New cubic polinomial function
    void cubic_polynomial(double t_, double tf_, double &s, double &s_dot, double &s_ddot);

    KDL::Trajectory* getTrajectory();

    //////////////////////////////////

    trajectory_point compute_trajectory(double time);

private:

    KDL::Path_RoundedComposite* path_;
    KDL::Path_Circle* path_circle_;
	KDL::VelocityProfile* velpref_;
	KDL::Trajectory* traject_;

    /////////////
    double trajDuration_, accDuration_;
    double trajRadius; 
    //// New types of trajectories
    Shape shape;
    PathType path_type;
    Eigen::Vector3d trajInit_, trajEnd_;
    trajectory_point p;

};

#endif
