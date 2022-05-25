#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <iostream>

#define TURTLE_NUM  4
//#define MARKER0FRAME
#define WORLDFRAME

using namespace std;

//Global Variables
double* Turtle0Pos =new double[2];
double* Turtle1Pos =new double[2];
double* Turtle2Pos =new double[2];
double* Turtle3Pos =new double[2];
int flag = 0;
double offset_sum_x[100];
double offset_sum_y[100];
double whole_sum_x;
double whole_sum_y;
int i = 0;
string filePath = "x y.txt";
static double secs = 0;
static double default_secs = 0;
ofstream writeFile(filePath.data());

tf::Quaternion quarterion[TURTLE_NUM];
tf::Vector3 pre_pos0, pre_pos1, pre_pos2, pre_pos3;

void cb(ar_track_alvar_msgs::AlvarMarkers req) {
       if (!req.markers.empty()) {
         tf::Transform tC2W;
         tf::Transform tr[4];
         tf::Quaternion q0, q1, q2, q3;

         tf::Quaternion q_cali;

#ifdef WORLDFRAME
         // SAME VALUE WITH AR MARKER LAUNCH FILE
         tC2W.setOrigin(tf::Vector3(0,0,2.45));
         q_cali.setRPY(3.14,0,0); // at Launch file, 'yaw, pitch, roll' order
         tC2W.setRotation(q_cali);
         tC2W=tC2W.inverse();

         for(int i = 0; i < 4; i++){
           if(req.markers[i].id == 0){
              tr[0].setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q0.setX(req.markers[i].pose.pose.orientation.x);
              q0.setY(req.markers[i].pose.pose.orientation.y);
              q0.setZ(req.markers[i].pose.pose.orientation.z);
              q0.setW(req.markers[i].pose.pose.orientation.w);
              tr[0].setRotation(q0);
              tr[0] = tC2W*tr[0];
           }
           if(req.markers[i].id == 1){
              tr[1].setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q1.setX(req.markers[i].pose.pose.orientation.x);
              q1.setY(req.markers[i].pose.pose.orientation.y);
              q1.setZ(req.markers[i].pose.pose.orientation.z);
              q1.setW(req.markers[i].pose.pose.orientation.w);
              tr[1].setRotation(q1);
              tr[1] = tC2W*tr[1];
           }
           if(req.markers[i].id == 2){
              tr[2].setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q2.setX(req.markers[i].pose.pose.orientation.x);
              q2.setY(req.markers[i].pose.pose.orientation.y);
              q2.setZ(req.markers[i].pose.pose.orientation.z);
              q2.setW(req.markers[i].pose.pose.orientation.w);
              tr[2].setRotation(q2);
              tr[2] = tC2W*tr[2];
           }
           if(req.markers[i].id == 3){
              tr[3].setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q3.setX(req.markers[i].pose.pose.orientation.x);
              q3.setY(req.markers[i].pose.pose.orientation.y);
              q3.setZ(req.markers[i].pose.pose.orientation.z);
              q3.setW(req.markers[i].pose.pose.orientation.w);
              tr[3].setRotation(q3);
              tr[3] = tC2W*tr[3];
           }
         }
#endif

         tf::Vector3 pos0,pos1,pos2,pos3;
         pos0 = tr[0].getOrigin();
         if(pos0.x()<10 &&pos0.x()>-10) {

           if(pos0.x()==0.0 || pos0.y()==0.0){
               Turtle0Pos[0] = pre_pos0.x();
               Turtle0Pos[1] = pre_pos0.y();
           }
           else{
               pre_pos0.setX(pos0.x());
               pre_pos0.setY(pos0.y());
               Turtle0Pos[0] = pos0.x();
               Turtle0Pos[1] = pos0.y();
           }
           //ROS_INFO("0x : %1.2f  0y : %1.2f  0z : %1.2f", pos0.x(), pos0.y(), pos0.z());
         }
         pos1 = tr[1].getOrigin();
         if(pos1.x()<10 &&pos1.x()>-10) {

           if(pos1.x()==0.0 || pos1.y()==0.0){
               Turtle1Pos[0] = pre_pos1.x();
               Turtle1Pos[1] = pre_pos1.y();
           }
           else{
               pre_pos1.setX(pos1.x());
               pre_pos1.setY(pos1.y());
               Turtle1Pos[0] = pos1.x();
               Turtle1Pos[1] = pos1.y();
           }
           ROS_INFO("1x : %1.2f  1y : %1.2f  1z : %1.2f", pos1.x(), pos1.y(), pos1.z());
         }
         pos2 = tr[2].getOrigin();
         if(pos2.x()<10 &&pos2.x()>-10) { //10m  

           if(pos2.x()==0.0 || pos2.y()==0.0){
               Turtle2Pos[0] = pre_pos2.x();
               Turtle2Pos[1] = pre_pos2.y();
           }
           else{
               pre_pos2.setX(pos2.x());
               pre_pos2.setY(pos2.y());
               Turtle2Pos[0] = pos2.x();
               Turtle2Pos[1] = pos2.y();
           }
           ROS_INFO("2x : %1.2f  2y : %1.2f  2z : %1.2f", pos2.x(), pos2.y(), pos2.z());
         }
         pos3 = tr[3].getOrigin();
         if(pos3.x()<10 &&pos3.x()>-10) { //10m

           if(pos3.x()==0.0 || pos3.y()==0.0){
               Turtle3Pos[0] = pre_pos3.x();
               Turtle3Pos[1] = pre_pos3.y();
           }
           else{
               pre_pos3.setX(pos3.x());
               pre_pos3.setY(pos3.y());
               Turtle3Pos[0] = pos3.x();
               Turtle3Pos[1] = pos3.y();
           }
           ROS_INFO("3x : %1.2f  3y : %1.2f  3z : %1.2f", pos3.x(), pos3.y(), pos3.z());
         }
         quarterion[0] = tr[0].getRotation();
         quarterion[1] = tr[1].getRotation();
         quarterion[2] = tr[2].getRotation();
         quarterion[3] = tr[3].getRotation();
       }
}

int main(int argc, char **argv) {
     ros::init(argc, argv, "TF_Transform");
     ros::NodeHandle nh;
     ros::Subscriber sub = nh.subscribe("ar_pose_marker", 1, cb);

     ros::Publisher pub0 =nh.advertise<nav_msgs::Odometry>("/tb3_0/odom",10);
     ros::Publisher pub1 =nh.advertise<nav_msgs::Odometry>("/tb3_1/odom",10);
     ros::Publisher pub2 =nh.advertise<nav_msgs::Odometry>("/tb3_2/odom",10);
     ros::Publisher pub3 =nh.advertise<nav_msgs::Odometry>("/tb3_3/odom",10);

     nav_msgs::Odometry Odom0;
     nav_msgs::Odometry Odom1;
     nav_msgs::Odometry Odom2;
     nav_msgs::Odometry Odom3;

     ros::Rate rate(500);
     default_secs = (ros::Time::now().toSec()/1000)*1000;

     while(ros::ok()){
       Odom0.pose.pose.position.x=Turtle0Pos[0];
       Odom0.pose.pose.position.y=Turtle0Pos[1];
       Odom1.pose.pose.position.x=Turtle1Pos[0];
       Odom1.pose.pose.position.y=Turtle1Pos[1];
       Odom2.pose.pose.position.x=Turtle2Pos[0];
       Odom2.pose.pose.position.y=Turtle2Pos[1];
       Odom3.pose.pose.position.x=Turtle3Pos[0];
       Odom3.pose.pose.position.y=Turtle3Pos[1];

       Odom0.pose.pose.orientation.x = quarterion[0].getX();
       Odom0.pose.pose.orientation.y = quarterion[0].getY();
       Odom0.pose.pose.orientation.z = quarterion[0].getZ();
       Odom0.pose.pose.orientation.w = quarterion[0].getW();

       Odom1.pose.pose.orientation.x = quarterion[1].getX();
       Odom1.pose.pose.orientation.y = quarterion[1].getY();
       Odom1.pose.pose.orientation.z = quarterion[1].getZ();
       Odom1.pose.pose.orientation.w = quarterion[1].getW();

       Odom2.pose.pose.orientation.x = quarterion[2].getX();
       Odom2.pose.pose.orientation.y = quarterion[2].getY();
       Odom2.pose.pose.orientation.z = quarterion[2].getZ();
       Odom2.pose.pose.orientation.w = quarterion[2].getW();

       Odom3.pose.pose.orientation.x = quarterion[3].getX();
       Odom3.pose.pose.orientation.y = quarterion[3].getY();
       Odom3.pose.pose.orientation.z = quarterion[3].getZ();
       Odom3.pose.pose.orientation.w = quarterion[3].getW();

       pub0.publish(Odom0);
       pub1.publish(Odom1);
       pub2.publish(Odom2);
       pub3.publish(Odom3);

       ros::spinOnce();
       rate.sleep();
     }
     return 0;
}
