//
//  ground_truth_package.h
//  ExtendedKF
//
//  Created by Carlos on 4/7/17.
//
//

#ifndef ground_truth_package_h
#define ground_truth_package_h

#include "Eigen/Dense"

class GroundTruthPackage {
public:
  long timestamp_;
  
  enum SensorType {
    LASER, RADAR
  } sensor_type_;
  
  Eigen::VectorXd gt_values_;
  
};

#endif /* ground_truth_package_h */
