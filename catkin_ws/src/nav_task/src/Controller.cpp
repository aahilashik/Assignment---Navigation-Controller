#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <sstream>
#include <stack>
#include <tf/tf.h>

using namespace std;

class TurtleBotNavController {
private:
  ros::Publisher pubVel;
  ros::Subscriber subOdom;

  geometry_msgs::Pose currentPose;
  geometry_msgs::Twist velMsg;

  const double PI = 3.14159265359;

  const double KpX = 0.1;
  const double KpZ = 0.3;

  vector<vector<double>> wayPoints = {};

public:
  TurtleBotNavController(string nodeName, int argc, char **argv) {
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;

    pubVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    subOdom = nh.subscribe("/odom", 1000, &TurtleBotNavController::odomCallback,
                           this);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg) {
    currentPose.position.x = odomMsg->pose.pose.position.x;
    currentPose.position.y = odomMsg->pose.pose.position.y;

    tf::Quaternion q(
        odomMsg->pose.pose.orientation.x, odomMsg->pose.pose.orientation.y,
        odomMsg->pose.pose.orientation.z, odomMsg->pose.pose.orientation.w);

    currentPose.orientation.z = tf::getYaw(q);
  }

  double getRestrictedVelocity(double velocity, double minVelocity,
                               double maxVelocity) {
    if (velocity < minVelocity)
      return minVelocity;
    else if (velocity > maxVelocity)
      return maxVelocity;
    else
      return velocity;
  }

  void addWayPoint(double x, double y, double theta) {
    wayPoints.push_back({x, y, theta});
  }
  double getEuclideanDistance(geometry_msgs::Pose currentPose,
                              geometry_msgs::Pose goalPose) {
    return sqrt(pow((goalPose.position.x - currentPose.position.x), 2) +
                pow((goalPose.position.y - currentPose.position.y), 2));
  }

  void startMove(double distanceTolerance = 0.1) {
    ros::Rate loopRate(10);
    geometry_msgs::Pose goalPose;

    ROS_INFO("Started to Move");
    for (int i = 0; i < wayPoints.size(); i++) {

      goalPose.position.x = wayPoints[i][0];
      goalPose.position.y = wayPoints[i][1];
      goalPose.orientation.z = wayPoints[i][2];

      do {
        double errorX = getEuclideanDistance(currentPose, goalPose);
        double errorZ = atan2(goalPose.position.y - currentPose.position.y,
                              goalPose.position.x - currentPose.position.x) -
                        currentPose.orientation.z;

        errorZ =
            (((PI > errorZ) && (errorZ > -PI))
                 ? errorZ
                 : ((errorZ < -PI) ? errorZ + (2 * PI) : errorZ - (2 * PI)));

        velMsg.linear.x = KpX * errorX;
        velMsg.angular.z = -KpZ * errorZ;

        velMsg.linear.x = getRestrictedVelocity(velMsg.linear.x, 0.2, 1);
        velMsg.angular.z = getRestrictedVelocity(velMsg.angular.z, -0.5, 0.5);

        pubVel.publish(velMsg);

        ROS_INFO("\nError:\n\tDistance: %0.4lf\t| Angle: "
                 "%0.4lf\nControl:\n\tLin.Vel: "
                 "%0.4lf\t| Ang.Vel: %0.4lf",
                 errorX, errorZ, velMsg.linear.x, velMsg.angular.z);

        ros::spinOnce();
        loopRate.sleep();
      } while (
          (getEuclideanDistance(currentPose, goalPose) > distanceTolerance) &&
          (ros::ok()));

      ROS_INFO("Reached %dth Goal!!", i + 1);
    }

    ROS_INFO("End Goal Reached!!");

    velMsg.linear.x = 0;
    velMsg.angular.z = 0;
    pubVel.publish(velMsg);

    loopRate.sleep();
  }
};

int main(int argc, char **argv) {
  TurtleBotNavController botNavCtrl =
      TurtleBotNavController("turtlebotNavController", argc, argv);
  double goals[4][3] = {{10, 3, 50}, {-10, 3, 50}, {-10, -3, 50}, {10, -3, 50}};

  for (int i = 0; i < 4; i++) {
    botNavCtrl.addWayPoint(goals[i][0], goals[i][1], goals[i][2]);
  }
  botNavCtrl.startMove();
  return 0;
}
