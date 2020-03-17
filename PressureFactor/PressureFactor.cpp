/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PressureFactor.cpp
 * @brief Simple Pressure/depth example, with three "GPS-like" measurements in the Z direction
 * @author Anthony Spears
 */

/**
 * A simple 3D pressure/depth factor example with "GPS" measurements in the Z direction
 *  - We have "GPS-like" measurements implemented with a custom factor
 * Adapted from the Pose2 2D LocalizationExample.cpp example in GTSAM 
 */

// We will use Pose3 variables (xyz, rpy) to represent the robot positions
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

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
// "GPS-like" Z direction functionality for the dpeth sensor. Because standard GPS
// measurements provide information only on the position, and not on the orientation,
//we cannot use a simple prior to properly model this measurement.
//
// The factor will be a unary factor, affect only a single system variable. It will
// also use a standard Gaussian noise model. Hence, we will derive our new factor from
// the NoiseModelFactor1.
#include <gtsam/nonlinear/NonlinearFactor.h>

class UnaryFactor: public NoiseModelFactor1<Pose3> {

  // The factor will hold a measurement consisting of a Z location
  double mz_;

public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<UnaryFactor> shared_ptr;

  // The constructor requires the variable key, the (Z) measurement value, and the noise model
  UnaryFactor(Key j, double z, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose3>(model, j), mz_(z) {}


  virtual ~UnaryFactor() {}

  // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
  // The first is the 'evaluateError' function. This function implements the desired measurement
  // function, returning a vector of errors when evaluated at the provided variable value. It
  // must also calculate the Jacobians for this measurement function, if requested.
  Vector evaluateError(const Pose3& q, boost::optional<Matrix&> H = boost::none) const
  {
    // The measurement function for a GPS-like measurement is simple:
    // error_z = pose.z - measurement.z
    // Consequently, the Jacobian is:
    // [ derror_z/dphi  derror_z/dtheta  derror_z/dpsi derror_z/dx derror_z/dy derror_z/dz] = [0 0 0 0 0 1]
    if (H) (*H) = (Matrix(1,6) << 0.0,0.0,0.0,0.0,0.0,1.0).finished();
    return (Vector(1) << q.z() - mz_).finished();
  }

  // The second is a 'clone' function that allows the factor to be copied. Under most
  // circumstances, the following code that employs the default copy constructor should
  // work fine.
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

}; // UnaryFactor


int main(int argc, char** argv) {

  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add prior factor and create noise and odometry models
  // For simplicity, we will use the same noise model for each odometry factor
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001).finished());
  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
  cout << odometryNoise << endl;
  // Create odometry (Between) factors between consecutive poses
  Rot3 zeroRot3 = Rot3::ypr(0, 0, 0);
  Point3 zonlyPoint3(0,0,0.11);
  graph.add(PriorFactor<Pose3>(0, Pose3(Rot3::ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0)),priorNoise));

  // 2b. Add "GPS-like" depth measurements
  // We will use our custom UnaryFactor for this.
  noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector1(0.000001)); //  std on z

  Values initialEstimate;
  initialEstimate.insert(0, Pose3(Rot3::ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0)));

  // Loop through and add depth factors and initial estimates
  double deltaT=0.1;
  for(double time=deltaT; time<=100; time+=deltaT)
    {
      int currentKey = round(1000*time);
      int prevKey = round(1000*(time-deltaT));
      double depth = 1*time; //Simulated depth reading assuming velocity of 1m/s diving in Z direction
      graph.emplace_shared<UnaryFactor>(currentKey, depth, unaryNoise);
      graph.emplace_shared<BetweenFactor<Pose3> >(prevKey, currentKey, Pose3(zeroRot3, zonlyPoint3), odometryNoise);
      graph.print("\nFactor Graph:\n"); // print

      // 3. Create the data structure to hold the initialEstimate estimate to the solution
      // For illustrative purposes, these have been deliberately set to incorrect values
      initialEstimate.insert(currentKey, Pose3(Rot3::ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, depth+0.3)));
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
      cout << "x covariance:\n" << marginals.marginalCovariance(currentKey) << endl;
    }
  
  return 0;
}
