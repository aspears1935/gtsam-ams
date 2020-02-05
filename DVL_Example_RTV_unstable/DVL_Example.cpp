/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LocalizationExample.cpp
 * @brief Simple robot localization example, with three "GPS-like" measurements
 * @author Frank Dellaert
 */

/**
 * A simple 2D pose slam example with "GPS" measurements
 *  - The robot moves forward 2 meter each iteration
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have "GPS-like" measurements implemented with a custom factor
 */

// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam_unstable/dynamics/DynamicsPriors.h>
#include <gtsam_unstable/dynamics/VelocityConstraint.h>
#include <gtsam_unstable/dynamics/PoseRTV.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// As in OdometryExample.cpp, we use a BetweenFactor to model odometry measurements.
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// We add all facors to a Nonlinear Factor Graph, as our factors are nonlinear.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

// Before we begin the example, we must create a custom unary factor to implement a
// "GPS-like" functionality. Because standard GPS measurements provide information
// only on the position, and not on the orientation, we cannot use a simple prior to
// properly model this measurement.
//
// The factor will be a unary factor, affect only a single system variable. It will
// also use a standard Gaussian noise model. Hence, we will derive our new factor from
// the NoiseModelFactor1.
#include <gtsam/nonlinear/NonlinearFactor.h>

int main(int argc, char** argv) {

  //Syntactic sugar to clarify velocity components
  typedef Vector3 Velocity3;
  Velocity3 DVL_velocities, prev_DVL_velocities;
    
  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add odometry factors
  // For simplicity, we will use the same noise model for each odometry factor
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(9) << 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001).finished());
  noiseModel::Diagonal::shared_ptr DVLNoise = noiseModel::Diagonal::Variances((Vector(9) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
  noiseModel::Diagonal::shared_ptr DVLVelNoise = noiseModel::Diagonal::Variances((Vector(3) << 0.01, 0.01, 0.01).finished());
  
  // Create odometry (Between) factors between consecutive poses
  Rot3 zeroRot3 = Rot3::ypr(0, 0, 0);
  Point3 zonlyPoint3(0,0,2);
  
  //  NonlinearEquality<gtsam::PoseRTV> poseHardPrior(x1, x1_v);  
  PriorFactor<gtsam::PoseRTV> posePrior(1, PoseRTV(Point3(0.0, 0.0, 0.0), Rot3::ypr(0.0, 0.0, 0.0), Velocity3(10.0, 0.0, 0.0)), priorNoise);

  //Add pre-integrated DVL velocity factors
  double dt =0.1; //10Hz
  double vx=10.0;
  double vy=0.0;
  double vz=0.0;
  prev_DVL_velocities = Velocity3(vx, vy, vz); // DVL_Velocities;
  DVL_velocities = Velocity3(vx, vy, vz);

  cout << DVL_velocities << endl;
  //Trapezoidal numerical integration
  Point3 preintDVL;
  preintDVL = Point3((DVL_velocities+prev_DVL_velocities)*dt*0.5);

  VelocityPrior velPrior(1, DVL_velocities, DVLVelNoise);
  //  BetweenFactor<gtsam::PoseRTV> odom(1, 2, x1_v, model9);
  //VelocityConstraint constraint(x1, x2, 0.1, 10000);
  
  //graph.emplace_shared<PriorFactor<PoseRTV> >(1, x1_v); //, priorNoise));
  //graph += NonlinearEquality<PoseRTV>(x1, pose1); //, priorNoise));
  graph += posePrior; //PriorFactor<PoseRTV>(x1, pose1); //, priorNoise));
  graph += velPrior; //PriorFactor<PoseRTV>(x1, pose1); //, priorNoise));  

  graph += BetweenFactor<PoseRTV>(1, 2, PoseRTV(preintDVL, zeroRot3, Velocity3(0,0,0)), DVLNoise);
  graph += BetweenFactor<PoseRTV>(2, 3, PoseRTV(preintDVL, zeroRot3, Velocity3(0,0,0)), DVLNoise);

  //Add DVL Velocity Constraints
  graph += VelocityConstraint(1, 2, dt, DVLVelNoise);
  graph += VelocityConstraint(2, 3, dt, DVLVelNoise);

  graph.print("\nFactor Graph:\n"); // print
  
  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;
  initialEstimate.insert(1, PoseRTV(Point3(0.5, 0.0, 0.0), Rot3::ypr(0.0, 0.0, 0.0), Velocity3(10.5, 0.0, 0.0)));
  initialEstimate.insert(2, PoseRTV(Point3(1.5, 0.0, 0.0), Rot3::ypr(0.0, 0.0, 0.0), Velocity3(10.5, 0.0, 0.0)));
  initialEstimate.insert(3, PoseRTV(Point3(2.5, 0.0, 0.0), Rot3::ypr(0.0, 0.0, 0.0), Velocity3(10.5, 0.0, 0.0)));
  initialEstimate.print("\nInitial Estimate:\n"); // print

 // 4. Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

  return 0;
}
