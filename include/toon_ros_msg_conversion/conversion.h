#ifndef TOON_ROS_CONVERSION_H
#define TOON_ROS_CONVERSION_H

#include "TooN/TooN.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "sun_math_toolbox/UnitQuaternion.h"

namespace sun {

TooN::Matrix<4, 4> pose2TooN(const geometry_msgs::Pose &pose) {
  TooN::Matrix<4, 4> T = TooN::Zeros;
  T(3, 3) = 1;
  T.slice<0, 3, 3, 1>() =
      TooN::Data(pose.position.x, pose.position.y, pose.position.z);
  T.slice<0, 0, 3, 3>() =
      sun::UnitQuaternion(pose.orientation.w,
                          TooN::makeVector(pose.orientation.x,
                                           pose.orientation.y,
                                           pose.orientation.z))
          .torot();
  return T;
}

TooN::Vector<6> wrench2TooN(const geometry_msgs::Wrench &wrench) {
  return TooN::makeVector(wrench.force.x, wrench.force.y, wrench.force.z,
                          wrench.torque.x, wrench.torque.y, wrench.torque.z);
}

geometry_msgs::Wrench TooN2wrench(const TooN::Vector<6> &w) {
  geometry_msgs::Wrench wrench;
  wrench.force.x = w[0];
  wrench.force.y = w[1];
  wrench.force.z = w[2];
  wrench.torque.x = w[3];
  wrench.torque.y = w[4];
  wrench.torque.z = w[5];
  return wrench;
}

} // namespace sun
#endif