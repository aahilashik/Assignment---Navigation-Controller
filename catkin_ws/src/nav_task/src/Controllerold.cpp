#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <sstream>
#include <tf/tf.h>

using namespace std;

ros::Publisher pubVel;
ros::Subscriber subOdom;
geometry_msgs::Pose currentPose;

const double PI = 3.14159265359;

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool cloclwise);
double degrees2radians(double angle_in_degrees);
void setDesiredOrientation(double desired_angle_radians);
void poseCallback(const nav_msgs::Odometry::ConstPtr &pose_message);
void moveGoal(geometry_msgs::Pose goalPose, double distance_tolerance);
double getDistance(double x1, double y1, double x2, double y2);

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtlebotController");
  ros::NodeHandle n;
  double speed, angular_speed;
  double distance, angle;
  bool isForward, clockwise;

  pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  subOdom = n.subscribe("/odom", 10, poseCallback);
  ros::Rate loop_rate(10);

  //	/turtle1/cmd_vel is the Topic name
  //	/geometry_msgs::Twist is the msg type
  ROS_INFO("\n\n\n ********START TESTING*********\n");

  /*********This is to move and rotate the robot as the user.**************
  cout<<"enter speed: ";
  cin>>speed;
  cout<<"enter distance: ";
  cin>>distance;
  cout<<"forward?: ";
  cin>>isForward;
  move(speed, distance, isForward);

  cout<<"enter angular velocity: ";
  cin>>angular_speed;
  cout<<"enter angle: ";
  cin>>angle;
  cout<<"Clockwise?: ";
  cin>>clockwise;
  rotate(degrees2radians(angular_speed), degrees2radians(angle), clockwise);
  */

  /**************This is to change the Absolute Orientation***************
  setDesiredOrientation(degrees2radians(120));
  ros::Rate loop_rate(0.5);
  loop_rate.sleep();
  setDesiredOrientation(degrees2radians(-60));
  loop_rate.sleep();
  setDesiredOrientation(degrees2radians(0));
  */
  double goals[4][3] = {{10, 3, 50}, {-10, 3, 50}, {-10, -3, 50}, {10, -3, 50}};
  geometry_msgs::Pose goal_pose;

  for (int i = 0; i < 4; i++) {
    goal_pose.position.x = goals[i][0];
    goal_pose.position.y = goals[i][1];
    goal_pose.orientation.z = goals[i][2];
    moveGoal(goal_pose, 0.1);
    loop_rate.sleep();
  }
  //   ros::spin();

  return 0;
}

void move(double speed, double distance, bool isForward) {
  geometry_msgs::Twist vel_msg;
  // set a random linear velocity in the x-axis
  if (isForward)
    vel_msg.linear.x = abs(speed);
  else
    vel_msg.linear.x = -abs(speed);
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  // set a random angular velocity in the y-axis
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0;

  double t0 = ros::Time::now().toSec();
  double current_distance = 0.0;
  ros::Rate loop_rate(100);
  do {
    pubVel.publish(vel_msg);
    double t1 = ros::Time::now().toSec();
    current_distance = speed * (t1 - t0);
    ros::spinOnce();
    loop_rate.sleep();
    // cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
  } while ((current_distance < distance) && (ros::ok()));
  vel_msg.linear.x = 0;
  pubVel.publish(vel_msg);
}

void rotate(double angular_speed, double relative_angle, bool clockwise) {

  geometry_msgs::Twist vel_msg;
  // set a random linear velocity in the x-axis
  vel_msg.linear.x = 0;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  // set a random angular velocity in the y-axis
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  if (clockwise)
    vel_msg.angular.z = -abs(angular_speed);
  else
    vel_msg.angular.z = abs(angular_speed);

  double t0 = ros::Time::now().toSec();
  double current_angle = 0.0;
  ros::Rate loop_rate(1000);
  do {
    pubVel.publish(vel_msg);
    double t1 = ros::Time::now().toSec();
    current_angle = angular_speed * (t1 - t0);
    ros::spinOnce();
    loop_rate.sleep();
    // cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
  } while ((current_angle < relative_angle) && (ros::ok()));
  vel_msg.angular.z = 0;
  pubVel.publish(vel_msg);
}

/**
 *  converts angles from degree to radians
 */

double degrees2radians(double angle_in_degrees) {
  return angle_in_degrees * PI / 180.0;
}

/**
 *  turns the robot to a desried absolute angle
 */
void setDesiredOrientation(double desired_angle_radians) {
  double relative_angle_radians =
      desired_angle_radians - currentPose.orientation.z;
  // if we want to turn at a perticular orientation, we subtract the current
  // orientation from it
  bool clockwise = ((relative_angle_radians < 0) ? true : false);
  // cout<<desired_angle_radians
  // <<","<<turtlesim_pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;
  rotate(abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
}

void poseCallback(const nav_msgs::Odometry::ConstPtr &odomMsg) {
  currentPose.position.x = odomMsg->pose.pose.position.x;
  currentPose.position.y = odomMsg->pose.pose.position.y;

  tf::Quaternion q(
      odomMsg->pose.pose.orientation.x, odomMsg->pose.pose.orientation.y,
      odomMsg->pose.pose.orientation.z, odomMsg->pose.pose.orientation.w);

  currentPose.orientation.z = tf::getYaw(q);
}

double getLimitVelocity(double velocity, double minVelocity,
                        double maxVelocity) {
  if (velocity < minVelocity)
    return minVelocity;
  else if (velocity > maxVelocity)
    return maxVelocity;
  else
    return velocity;
}

void moveGoal(geometry_msgs::Pose goal_pose, double distance_tolerance) {
  geometry_msgs::Twist vel_msg;

  ros::Rate loop_rate(10);
  do {
    // linear velocity
    vel_msg.linear.x =
        0.1 * getDistance(currentPose.position.x, currentPose.position.y,
                          goal_pose.position.x, goal_pose.position.y);
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    // angular velocity
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    double errorAngle = atan2(goal_pose.position.y - currentPose.position.y,
                              goal_pose.position.x - currentPose.position.x) -
                        currentPose.orientation.z;
    errorAngle = (((PI > errorAngle) && (errorAngle > -PI))
                      ? errorAngle
                      : ((errorAngle < -PI) ? errorAngle + (2 * PI)
                                            : errorAngle - (2 * PI)));

    vel_msg.angular.z = -0.3 * errorAngle;
    // vel_msg.angular.z =
    //     -0.5 * (atan2(goal_pose.position.y - currentPose.position.y,
    //                   goal_pose.position.x - currentPose.position.x) -
    //             currentPose.orientation.z);

    vel_msg.linear.x += (vel_msg.linear.x > 0 ? 0.05 : -0.05);
    vel_msg.angular.z += (vel_msg.angular.z > 0 ? 0.02 : -0.02);

    vel_msg.linear.x = getLimitVelocity(vel_msg.linear.x, 0.2, 1);
    vel_msg.angular.z = getLimitVelocity(vel_msg.angular.z, -0.5, 0.5);
    pubVel.publish(vel_msg);

    cout << "\nError Angle : " << errorAngle
         << " | AVel : " << vel_msg.angular.z;

    ros::spinOnce();
    loop_rate.sleep();

  } while ((getDistance(currentPose.position.x, currentPose.position.y,
                        goal_pose.position.x,
                        goal_pose.position.y) > distance_tolerance) &&
           (ros::ok()));

  cout << "end move goal" << endl;
  vel_msg.linear.x = 0;
  vel_msg.angular.z = 0;
  pubVel.publish(vel_msg);
  loop_rate.sleep();
}

double getDistance(double x1, double y1, double x2, double y2) {
  return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}
