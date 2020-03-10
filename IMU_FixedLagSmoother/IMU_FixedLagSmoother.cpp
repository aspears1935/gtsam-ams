/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose3FixedLagSmootherExample.cpp
 * @brief Demonstration of the fixed-lag smoothers using a full 3D (Pose3) robot example and multiple odometry-like sensors. This is based on the FixedLagSmootherExample.cpp example which uses 2D (Pose2) representations.
 * @author Anthony Spears
 */

/**
 * A simple 3D pose slam example with multiple odometry-like measurements
 *  - The robot initially faces along the X axis
 *  - The robot moves forward at 2m/s
 *  - We have measurements between each pose from multiple odometry sensors
 */

// This example demonstrates the use of the Fixed-Lag Smoothers in GTSAM unstable
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// We will use simple integer Keys to uniquely identify each robot pose.
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>

// We will use Pose3 variables (x, y, z, Rot(x, y, z)) to represent the robot positions
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <iomanip>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

// This will either be PreintegratedImuMeasurements (for ImuFactor) or
// PreintegratedCombinedMeasurements (for CombinedImuFactor).
PreintegrationType *imu_preintegrated_;

int main(int argc, char** argv) {
  bool use_combined_imu = false;
  
  // Define the smoother lag (in seconds)
  double lag = 2.0;

  // Create a fixed lag smoother
  // The Batch version uses Levenberg-Marquardt to perform the nonlinear optimization
  BatchFixedLagSmoother smootherBatch(lag);
  // The Incremental version uses iSAM2 to perform the nonlinear optimization
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.0; // Set the relin threshold to zero such that the batch estimate is recovered
  parameters.relinearizeSkip = 1; // Relinearize every time
  IncrementalFixedLagSmoother smootherISAM2(lag, parameters);

  // Create containers to store the factors and linearization points that
  // will be sent to the smoothers
  NonlinearFactorGraph newFactors;
  Values newValues;
  FixedLagSmoother::KeyTimestampMap newTimestamps;

  // Create a prior on the first pose, placing it at the origin
  ///  Pose3 priorMean = Pose3(Rot3::ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  //NOTE: noiseModel in Pose3 requires rpy first, then xyz. In this case 0.1 rad is rpy and 30cm in xyz
  ///  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 0.1, 0.1, 0.1, 0.3, 0.3, 0.3).finished());
  ///Key priorKey = 0;
  
  // Format is (N,E,D,qX,qY,qZ,qW,velN,velE,velD)
  Eigen::Matrix<double,10,1> initial_state = Eigen::Matrix<double,10,1>::Zero();
  cout << "initial state:\n" << initial_state.transpose() << "\n\n";

  // Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
  Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3),
					 initial_state(4), initial_state(5));
  Point3 prior_point(initial_state.head<3>());
  Pose3 prior_pose(prior_rotation, prior_point);
  Vector3 prior_velocity(initial_state.tail<3>());
  imuBias::ConstantBias prior_imu_bias; // assume zero initial bias 

  // Assemble prior noise model and add it the graph.
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

  //Add priors to list of values and factors
  newFactors.push_back(PriorFactor<Pose3>(X(0), prior_pose, pose_noise_model));
  newFactors.push_back(PriorFactor<Vector3>(V(0), prior_velocity,velocity_noise_model));
  newFactors.push_back(PriorFactor<imuBias::ConstantBias>(B(0), prior_imu_bias,bias_noise_model));

  newValues.insert(X(0), prior_pose);
  newValues.insert(V(0), prior_velocity);
  newValues.insert(B(0), prior_imu_bias);
  
  ///newValues.insert(priorKey, priorMean); // Initialize the first pose at the mean of the prior
  newTimestamps[X(0)] = 0.0; // Set the timestamp associated with this key to 0.0 seconds;

  newFactors.print();

  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
  Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
  Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration
  
  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
  p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;

  std::shared_ptr<PreintegrationType> imu_preintegrated_ = nullptr;
  if (use_combined_imu) {
        imu_preintegrated_ =
	  std::make_shared<PreintegratedCombinedMeasurements>(p, prior_imu_bias);
  } else {
        imu_preintegrated_ =
	  std::make_shared<PreintegratedImuMeasurements>(p, prior_imu_bias);
  }
  assert(imu_preintegrated_);
    
  // Store previous state for the imu integration and the latest predicted outcome.
  NavState prev_state(prior_pose, prior_velocity);
  NavState prop_state = prev_state;
  imuBias::ConstantBias prev_bias = prior_imu_bias;
    
  // Now, loop through several time steps, creating factors from different "sensors"
  // and adding them to the fixed-lag smoothers
  double deltaT = 0.01;
  
  for(double time = deltaT; time <= 3.0; time += deltaT) {

    // Define the keys related to this timestamp
    int previousKey(1000 * (time-deltaT));
    int currentKey(1000 * (time));

    // Assign the current key to the current timestamp
    newTimestamps[X(currentKey)] = time;

    if(currentKey % 100) //Limit IMU to 10Hz and preintegrate between
      {
	//Get new IMU reading and preintegrate
	Eigen::Matrix<double,6,1> imu = Eigen::Matrix<double,6,1>::Zero();
	Vector3 measuredAcc, measuredOmega;
	measuredAcc=Vector3(0.0,0.0,0.0);
	//      measuredOmega=Vector3(0.01*3.141592*dt,0.0,0.0);
	measuredOmega=Vector3(0.0,0.0,0.0);
	
	// Adding the IMU preintegration.
	///      imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
	imu_preintegrated_->integrateMeasurement(measuredAcc, measuredOmega, deltaT);
      }

    else { //add preintegrated factors every 10Hz
      ///correction_count++;

      // Adding IMU factor and optimizing.
      if (use_combined_imu) {
	const PreintegratedCombinedMeasurements& preint_imu_combined =
	  dynamic_cast<const PreintegratedCombinedMeasurements&>(
								 *imu_preintegrated_);
	CombinedImuFactor imu_factor(X(previousKey), V(previousKey),
				     X(currentKey), V(currentKey),
				     B(previousKey), B(currentKey),
				     preint_imu_combined);
	newFactors.push_back(imu_factor);
      } else {
	const PreintegratedImuMeasurements& preint_imu =
	  dynamic_cast<const PreintegratedImuMeasurements&>(
							    *imu_preintegrated_);
	ImuFactor imu_factor(X(previousKey), V(previousKey),
			     X(currentKey), V(currentKey),
			     B(previousKey),
			     preint_imu);
	newFactors.push_back(imu_factor);
	imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
	newFactors.push_back(BetweenFactor<imuBias::ConstantBias>(B(previousKey),
							B(currentKey),
							zero_bias, bias_noise_model));
      }

      // Now optimize and compare results.
      prop_state = imu_preintegrated_->predict(prev_state, prev_bias);
      newValues.insert(X(currentKey), prop_state.pose());
      newValues.insert(V(currentKey), prop_state.v());
      newValues.insert(B(currentKey), prev_bias);

      // Update the smoothers with the new factors. In this example, batch smoother needs one iteration
      // to accurately converge. The ISAM smoother doesn't, but we only start getting estiates when
      // both are ready for simplicity.
      Values resultISAM2;
      Values resultBatch;
      if (time >= 0.50) {
	smootherBatch.update(newFactors, newValues, newTimestamps);
	smootherISAM2.update(newFactors, newValues, newTimestamps);
	for(size_t i = 1; i < 2; ++i) { // Optionally perform multiple iSAM2 iterations
          smootherISAM2.update();
	}
	 
	// Print the optimized current pose
	cout << setprecision(5) << "Timestamp = " << time << endl;
	smootherBatch.calculateEstimate<Pose3>(currentKey).print("Batch Estimate:");
	smootherISAM2.calculateEstimate<Pose3>(currentKey).print("iSAM2 Estimate:");
	cout << endl;
	
	// Clear contains for the next iteration
	newTimestamps.clear();
	newValues.clear();
	newFactors.resize(0);
      }
      // Overwrite the beginning of the preintegration for the next step.
      prev_state = NavState(resultISAM2.at<Pose3>(X(currentKey)),
			    resultISAM2.at<Vector3>(V(currentKey)));
      prev_bias = resultISAM2.at<imuBias::ConstantBias>(B(currentKey));

      // Reset the preintegration object.
      imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);
    }
  }

  // And to demonstrate the fixed-lag aspect, print the keys contained in each smoother after 3.0 seconds
  cout << "After 3.0 seconds, " << endl;
  cout << "  Batch Smoother Keys: " << endl;
  for(const FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp: smootherBatch.timestamps()) {
    cout << setprecision(5) << "    Key: " << key_timestamp.first << "  Time: " << key_timestamp.second << endl;
  }
  cout << "  iSAM2 Smoother Keys: " << endl;
  for(const FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp: smootherISAM2.timestamps()) {
    cout << setprecision(5) << "    Key: " << key_timestamp.first << "  Time: " << key_timestamp.second << endl;
  }

  // Here is an example of how to get the full Jacobian of the problem.
  // First, get the linearization point.
  Values result = smootherISAM2.calculateEstimate();

  // Get the factor graph
  auto &factorGraph = smootherISAM2.getFactors();

  // Linearize to a Gaussian factor graph
  boost::shared_ptr<GaussianFactorGraph> linearGraph = factorGraph.linearize(result);

  // Converts the linear graph into a Jacobian factor and extracts the Jacobian matrix
  Matrix jacobian = linearGraph->jacobian().first;
  //  cout << " Jacobian: " << jacobian << endl;

  return 0;
}
