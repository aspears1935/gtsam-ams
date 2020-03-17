/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file IMU_ISAM2_Example.cpp
 * @brief Test example for using GTSAM ImuFactor and ImuCombinedFactor navigation code with ISAM2
 * @author Anthony Spears
 */

/**
 * Example of use of the imuFactors (imuFactor and combinedImuFactor) with ISAM2
 * This is developed from the imuFactorsExample.cpp example in the GTSAM library
 * Same as our IMU_Example.cpp example but using ISAM2 instead of LevenbergMarquardt
 *  - imuFactor is used by default. You can test combinedImuFactor by
 *  appending a `-c` flag at the end (see below for example command).
 *  N, E, D, qx, qY, qZ, qW, velN, velE, velD
 *  linAccN, linAccE, linAccD, angVelN, angVelE, angVelD
 */

// GTSAM related includes.
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <chrono>

using namespace gtsam;
using namespace std;
using namespace chrono;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

const string output_filename = "imuFactorExampleResults.csv";
const string use_combined_imu_flag = "-c";

// Create incremental ISAM2 solver
ISAM2 isam;

// This will either be PreintegratedImuMeasurements (for ImuFactor) or
// PreintegratedCombinedMeasurements (for CombinedImuFactor).
PreintegrationType *imu_preintegrated_;

int main(int argc, char* argv[])
{
  string data_filename;
  bool use_combined_imu = false;
  if (argc < 2) {
    printf("using default CSV file\n");
    data_filename = findExampleDataFile("imuAndGPSdata.csv");
  } else if (argc < 3){
    if (strcmp(argv[1], use_combined_imu_flag.c_str()) == 0) { // strcmp returns 0 for a match
      printf("using CombinedImuFactor\n");
      use_combined_imu = true;
      printf("using default CSV file\n");
      data_filename = findExampleDataFile("imuAndGPSdata.csv");
    } else {
      data_filename = argv[1];
    }
  } else {
    data_filename = argv[1];
    if (strcmp(argv[2], use_combined_imu_flag.c_str()) == 0) { // strcmp returns 0 for a match
      printf("using CombinedImuFactor\n");
      use_combined_imu = true;
    }
  }

  // Set up output file for plotting errors
  FILE* fp_out = fopen(output_filename.c_str(), "w+");
  fprintf(fp_out, "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m),gt_qx,gt_qy,gt_qz,gt_qw\n");

  string value;

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

  Values initial_values;
  int correction_count = 0;
  initial_values.insert(X(correction_count), prior_pose);
  initial_values.insert(V(correction_count), prior_velocity);
  initial_values.insert(B(correction_count), prior_imu_bias);

  // Assemble prior noise model and add it the graph.
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

  // Add all prior factors (pose, velocity, bias) to the graph.
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  graph->add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
  graph->add(PriorFactor<Vector3>(V(correction_count), prior_velocity,velocity_noise_model));
  graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias,bias_noise_model));

  isam.update(*graph, initial_values);
  Values result=isam.calculateEstimate();
  graph = new NonlinearFactorGraph();
  initial_values.clear();
  graph->print();
  
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

  // Keep track of the total error over the entire run for a simple performance metric.
  double current_position_error = 0.0, current_orientation_error = 0.0;

  double output_time = 0.0;
  double dt = 0.01;
  // All priors have been set up, now iterate
  for(int i=0; i<10000; i++) {

    auto start = high_resolution_clock::now(); //Get time at start to measure excecution time
      

    if (i%10) { // IMU measurement
      Eigen::Matrix<double,6,1> imu = Eigen::Matrix<double,6,1>::Zero();
      
      ///AMS Added:
      Vector3 measuredAcc, measuredOmega;
      measuredAcc=Vector3(0.0,0.0,0.0);
      //      measuredOmega=Vector3(0.01*3.141592*dt,0.0,0.0);
      measuredOmega=Vector3(0.0,0.0,0.0);

      // Adding the IMU preintegration.
      ///      imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
      imu_preintegrated_->integrateMeasurement(measuredAcc, measuredOmega, dt);

    } else { // Time to add preintegrated factors to graph (10Hz)
      Eigen::Matrix<double,7,1> gps = Eigen::Matrix<double,7,1>::Zero();
 
      correction_count++;

      // Adding IMU factor and GPS factor and optimizing.
      if (use_combined_imu) {
        const PreintegratedCombinedMeasurements& preint_imu_combined =
            dynamic_cast<const PreintegratedCombinedMeasurements&>(
              *imu_preintegrated_);
        CombinedImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                                     X(correction_count  ), V(correction_count  ),
                                     B(correction_count-1), B(correction_count  ),
                                     preint_imu_combined);
        graph->add(imu_factor);
      } else {
        const PreintegratedImuMeasurements& preint_imu =
            dynamic_cast<const PreintegratedImuMeasurements&>(
              *imu_preintegrated_);
        ImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                             X(correction_count  ), V(correction_count  ),
                             B(correction_count-1),
                             preint_imu);
        graph->add(imu_factor);
        imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
        graph->add(BetweenFactor<imuBias::ConstantBias>(B(correction_count-1),
                                                        B(correction_count  ),
                                                        zero_bias, bias_noise_model));
      }

      noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3,1.0);
      GPSFactor gps_factor(X(correction_count),
                           Point3(gps(0),  // N,
                                  gps(1),  // E,
                                  gps(2)), // D,
                           correction_noise);
      //graph->add(gps_factor);


      
      // Now optimize and compare results.
      cout << correction_count << endl;
      prop_state = imu_preintegrated_->predict(prev_state, prev_bias);
      initial_values.insert(X(correction_count), prop_state.pose());
      initial_values.insert(V(correction_count), prop_state.v());
      initial_values.insert(B(correction_count), prev_bias);

      /*
      LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
      Values result = optimizer.optimize();
      */
      
      graph->print();
      isam.update(*graph, initial_values);
      Values result=isam.calculateEstimate();
      
      
      // Overwrite the beginning of the preintegration for the next step.
      prev_state = NavState(result.at<Pose3>(X(correction_count)),
                            result.at<Vector3>(V(correction_count)));
      prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));

      // Reset the preintegration object.
      imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);

      // Print out the position and orientation error for comparison.
      Vector3 gtsam_position = prev_state.pose().translation();
      //Vector3 position_error = gtsam_position - gps.head<3>();
      //current_position_error = position_error.norm();

      //      cout << gtsam_position(0) << "," << gtsam_position(1) << "," << gtsam_position(2) << endl;
      
      Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();

      //Get time after excecution to get total excecution time
      auto stop = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>(stop-start);
      cout << duration.count() << endl;
            
      // cout << prev_state.pose().rotation().ypr() << endl;
      
      ///Quaternion gps_quat(gps(6), gps(3), gps(4), gps(5));
      ///Quaternion quat_error = gtsam_quat * gps_quat.inverse();
      ///quat_error.normalize();
      ///Vector3 euler_angle_error(quat_error.x()*2,
      ///                           quat_error.y()*2,
      ///                           quat_error.z()*2);
      /// current_orientation_error = euler_angle_error.norm();
      
      // display statistics
      ///      cout << "Position error:" << current_position_error << "\t " << "Angular error:" << current_orientation_error << "\n";

      /*      fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
              output_time, gtsam_position(0), gtsam_position(1), gtsam_position(2),
              gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w(),
              gps(0), gps(1), gps(2),
              gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());
      */
      output_time += 1.0;

      ///    } else {
      /// cerr << "ERROR parsing file\n";
      /// return 1;
    }

  }
  fclose(fp_out);
  cout << "Complete, results written to " << output_filename << "\n\n";;
  return 0;
}
