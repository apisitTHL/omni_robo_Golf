#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"
#include <math.h>

float x = 0;
float y = 0;
float th = 0;

float enc_wheel1, enc_wheel2, enc_wheel3;
float enc_wheel1_old = 0, enc_wheel2_old = 0, enc_wheel3_old = 0;

double wheel_radius = 0.05; // Radius of the wheels
double robot_radius = 0.185;  // Distance from the center to any wheel
double DistancePerCount = (2 * M_PI * wheel_radius) / 240;

void Wheel1_Callback(const std_msgs::Float32& enc1)
{
    enc_wheel1 = enc1.data;
}

void Wheel2_Callback(const std_msgs::Float32& enc2)
{
    enc_wheel2 = enc2.data;
}

void Wheel3_Callback(const std_msgs::Float32& enc3)
{
    enc_wheel3 = enc3.data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);

    ros::Subscriber sub_wheel1 = n.subscribe("Enc_C", 1000, Wheel1_Callback);
    ros::Subscriber sub_wheel2 = n.subscribe("Enc_B", 1000, Wheel2_Callback);
    ros::Subscriber sub_wheel3 = n.subscribe("Enc_A", 1000, Wheel3_Callback);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(20.0);
    while(n.ok()){

        ros::spinOnce(); // check for incoming messages
        current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();

        // Calculate the distance traveled by each wheel
        double dist_wheel1 = (enc_wheel1 - enc_wheel1_old) * DistancePerCount;
        double dist_wheel2 = (enc_wheel2 - enc_wheel2_old) * DistancePerCount;
        double dist_wheel3 = (enc_wheel3 - enc_wheel3_old) * DistancePerCount;



        // Calculate the robot's velocity using inverse kinematics
        //double vy = (2.0 / 3.0) * (dist_wheel1 - 0.5 * dist_wheel2 - 0.5 * dist_wheel3)*20;
        //double vx = (sqrt(3) / 3.0) * (dist_wheel2 - dist_wheel3)*20;
        //double vth = (1.0 / (3.0 * robot_radius)) * (dist_wheel1 + dist_wheel2 + dist_wheel3)*20;

    	double vy = ((wheel_radius / 3.0) * (dist_wheel1 + dist_wheel2 * cos(M_PI / 3.0) + dist_wheel3 * cos(M_PI / 3.0)))*-2461;
    	double vx = ((wheel_radius / 3.0) * (dist_wheel2 * sin(M_PI / 3.0) - dist_wheel3 * sin(M_PI / 3.0)))*-804;
    	double vth = ((wheel_radius / (3.0 * robot_radius)) * (dist_wheel1 + dist_wheel2 + dist_wheel3))*405;

        
        double delta_x = vx * dt;
        double delta_y = vy * dt;
        double delta_th = vth * dt;

    	//x += delta_x * cos(th) - delta_y * sin(th);
    	//y += delta_x * sin(th) + delta_y * cos(th);
    	//th += delta_th;

    	x += delta_x * cos(th) - delta_y * sin(th);
    	y += delta_x * sin(th) + delta_y * cos(th);
    	th += delta_th;

	last_time = current_time;
	// Update old encoder values
        enc_wheel1_old = enc_wheel1;
        enc_wheel2_old = enc_wheel2;
        enc_wheel3_old = enc_wheel3;


        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        odom_pub.publish(odom);

        
        r.sleep();
    }
}

