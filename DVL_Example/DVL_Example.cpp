/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DVL_Example.cpp
 * @brief Simple robot localization example, with Simple DVL velocity integrated odometry measurements
 * @author Anthony Spears
 */

/**
 * A simple 3D pose slam example with "DVL" velocity measurements
 *  - We have full linear velocity at each pose
 * Odometry is calculated from velocity measurements using trapezoidal numerical integration
 # This was adapted from the LocalizationExample.cpp example from the GTSAM library
 */

// We will use Pose3 variables (xyz, phi/theta/psi) to represent the robot positions
// We could also use VelocityConstraints for the velocity measurements but don't currently implement this and rely only on the numerically integrated BetweenFactor odometry factors for this simple implementation. A more advanced implementation is available the our DVL_Example_RTV.cpp example.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam_unstable/dynamics/VelocityConstraint.h>

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

int main(int argc, char** argv) {

  //Syntactic sugar to clarify velocity components
  typedef Vector3 Velocity3;
  Velocity3 DVL_velocities, prev_DVL_velocities;
    
  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add odometry factors
  // For simplicity, we will use the same noise model for each odometry factor
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001).finished());
  noiseModel::Diagonal::shared_ptr DVLNoise = noiseModel::Diagonal::Variances((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
  
  // Add prior factor
  Rot3 zeroRot3 = Rot3::ypr(0, 0, 0);
  graph.add(PriorFactor<Pose3>(1, Pose3(),priorNoise));

  //Add pre-integrated DVL velocity factors
  double dt =0.1; //10Hz
  double vx=10.0; //10m/s in X direction
  double vy=0.0;
  double vz=0.0;
  prev_DVL_velocities = Velocity3(vx, vy, vz); // Previous DVL Velocity measurements
  DVL_velocities = Velocity3(vx, vy, vz); // Current DVL Velocity measurements

  //Trapezoidal numerical integration
  Point3 preintDVL;
  preintDVL = Point3((DVL_velocities+prev_DVL_velocities)*dt*0.5);

  //Add DVL Factors
  graph.emplace_shared<BetweenFactor<Pose3> >(1, 2, Pose3(zeroRot3, preintDVL), DVLNoise);
  graph.emplace_shared<BetweenFactor<Pose3> >(2, 3, Pose3(zeroRot3, preintDVL), DVLNoise);

  //Print Factor Graph
  graph.print("\nFactor Graph:\n"); // print

 
  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;
  initialEstimate.insert(1, Pose3(Rot3::ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.5)));
  initialEstimate.insert(2, Pose3(Rot3::ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 1.5)));
  initialEstimate.insert(3, Pose3(Rot3::ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 2.5)));
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
