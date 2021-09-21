#ifndef TOON_ROS_CONVERSION_H
#define TOON_ROS_CONVERSION_H

#include "TooN/TooN.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
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

geometry_msgs::Twist TooN2twist(const TooN::Vector<6> &t) {
  geometry_msgs::Twist twist;
  twist.linear.x = t[0];
  twist.linear.y = t[1];
  twist.linear.z = t[2];
  twist.angular.x = t[3];
  twist.angular.y = t[4];
  twist.angular.z = t[5];
  return twist;
}

TooN::Vector<3> vector3_2_TooN(const geometry_msgs::Vector3& msg)
{
  return TooN::makeVector(msg.x, msg.y, msg.z);
}

geometry_msgs::Vector3 TooN2Vector3(const TooN::Vector<3>& v)
{
  geometry_msgs::Vector3 out;
  out.x = v[0];
  out.y = v[1];
  out.z = v[2];
  return out;
}

} // namespace sun
#endif