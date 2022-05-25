#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

#define TURTLE_NUM  6

using namespace std;

//Global Variables
double* Turtle1Pos =new double[3];
double* Turtle2Pos =new double[3];
double* Turtle3Pos =new double[3];
double* Turtle4Pos =new double[3];
double* Turtle5Pos =new double[3];
int flag = 0;
int i = 0;

tf::Quaternion quarterion[TURTLE_NUM];
tf::Vector3 pre_pos1, pre_pos2, pre_pos3, pre_pos4, pre_pos5;

void pose_estimation(ar_track_alvar_msgs::AlvarMarkers req) {
       if (!req.markers.empty()) {
         tf::Transform tC2W;
         tf::Transform tAR2CT[6];
         tf::Quaternion qt[6];
         tf::Transform tr[6];
         tf::Quaternion q1, q2, q3, q4, q5;
         tf::Quaternion q_cali;

         /* Define tAR2CT Matrix */
         tAR2CT[1].setOrigin(tf::Vector3(0,0,-0.105));
         qt[1].setRPY(-1.571,1.571,0);
         tAR2CT[1].setRotation(qt[1]);

         tAR2CT[2].setOrigin(tf::Vector3(0,0,-0.084));
         qt[2].setRPY(-1.571,0,0);
         tAR2CT[2].setRotation(qt[2]);

         tAR2CT[3].setOrigin(tf::Vector3(0,0,-0.073)); //7.8 -> 7.3
         qt[3].setRPY(0,0,0);
         tAR2CT[3].setRotation(qt[3]);

         tAR2CT[4].setOrigin(tf::Vector3(0,0,-0.093));
         qt[4].setRPY(-1.571,-1.571,0);
         tAR2CT[4].setRotation(qt[4]);

         tAR2CT[5].setOrigin(tf::Vector3(0,0,-0.085));
         qt[5].setRPY(-1.571,3.141,0);
         tAR2CT[5].setRotation(qt[5]);


         /*
         // SAME VALUE WITH AR MARKER LAUNCH FILE
         tC2W.setOrigin(tf::Vector3(0,0,2.45));
         q_cali.setRPY(3.14,0,0); // at Launch file, 'yaw, pitch, roll' order
         tC2W.setRotation(q_cali);
         tC2W=tC2W.inverse();
         */
         tC2W.setOrigin(tf::Vector3(0,0,2.566)); //2.45
         //tC2W.setBasis~~
         q_cali.setRPY(3.14,0,0); // at Launch file, 'yaw, pitch, roll' order
         tC2W.setRotation(q_cali);
         tC2W=tC2W.inverse();

         // Transform MTX from Upper AR Marker
         for(int i = 0; i < 6; i++){
           cout<<i;
           if(req.markers[i].id == 1){
//              ROS_INFO("1");
              tr[1].setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q1.setX(req.markers[i].pose.pose.orientation.x);
              q1.setY(req.markers[i].pose.pose.orientation.y);
              q1.setZ(req.markers[i].pose.pose.orientation.z);
              q1.setW(req.markers[i].pose.pose.orientation.w);
              tr[1].setRotation(q1);
              tr[1] = tC2W*tr[1]*tAR2CT[1];
           }
           if(req.markers[i].id == 2){
//               ROS_INFO("2");
              tr[2].setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q2.setX(req.markers[i].pose.pose.orientation.x);
              q2.setY(req.markers[i].pose.pose.orientation.y);
              q2.setZ(req.markers[i].pose.pose.orientation.z);
              q2.setW(req.markers[i].pose.pose.orientation.w);
              tr[2].setRotation(q2);
              tr[2] = tC2W*tr[2]*tAR2CT[2];
           }
           if(req.markers[i].id == 3){
//               ROS_INFO("3");
              tr[3].setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q3.setX(req.markers[i].pose.pose.orientation.x);
              q3.setY(req.markers[i].pose.pose.orientation.y);
              q3.setZ(req.markers[i].pose.pose.orientation.z);
              q3.setW(req.markers[i].pose.pose.orientation.w);
              tr[3].setRotation(q3);
              tr[3] = tC2W*tr[3]*tAR2CT[3];
           }
           if(req.markers[i].id == 4){
//               ROS_INFO("4");
              tr[4].setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q4.setX(req.markers[i].pose.pose.orientation.x);
              q4.setY(req.markers[i].pose.pose.orientation.y);
              q4.setZ(req.markers[i].pose.pose.orientation.z);
              q4.setW(req.markers[i].pose.pose.orientation.w);
              tr[4].setRotation(q4);
              tr[4] = tC2W*tr[4]*tAR2CT[4];
           }
           if(req.markers[i].id == 5){
//               ROS_INFO("5");
              tr[5].setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q5.setX(req.markers[i].pose.pose.orientation.x);
              q5.setY(req.markers[i].pose.pose.orientation.y);
              q5.setZ(req.markers[i].pose.pose.orientation.z);
              q5.setW(req.markers[i].pose.pose.orientation.w);
              tr[5].setRotation(q5);
              tr[5] = tC2W*tr[5]*tAR2CT[5];
           }
         }
//         ROS_INFO("gt\n");

         tf::Vector3 pos1,pos2,pos3,pos4,pos5;

         pos1 = tr[1].getOrigin();
         if(1) {//pos1.x()<10 &&pos1.x()>-10
           if(pos1.x()==0.0 || pos1.y()==0.0){
               Turtle1Pos[0] = pre_pos1.x();
               Turtle1Pos[1] = pre_pos1.y();
               Turtle1Pos[2] = pre_pos1.z();
           }
           else{
               pre_pos1.setX(pos1.x());
               pre_pos1.setY(pos1.y());
               pre_pos1.setZ(pos1.z());
               Turtle1Pos[0] = pos1.x();
               Turtle1Pos[1] = pos1.y();
               Turtle1Pos[2] = pos1.z();
           }
//           ROS_INFO("1 x:%1.2f y:%1.2f z:%1.2f", pos1.x(), pos1.y(), pos1.z());
         }

         pos2 = tr[2].getOrigin();
         if(1) { //pos2.x()<10 &&pos2.x()>-10

           if(pos2.x()==0.0 || pos2.y()==0.0){
               Turtle2Pos[0] = pre_pos2.x();
               Turtle2Pos[1] = pre_pos2.y();
               Turtle2Pos[2] = pre_pos2.z();
           }
           else{
               pre_pos2.setX(pos2.x());
               pre_pos2.setY(pos2.y());
               pre_pos2.setZ(pos2.z());
               Turtle2Pos[0] = pos2.x();
               Turtle2Pos[1] = pos2.y();
               Turtle2Pos[2] = pos2.z();
           }
//           ROS_INFO("2 x:%1.2f y:%1.2f z:%1.2f", pos2.x(), pos2.y(), pos2.z());
         }

         pos3 = tr[3].getOrigin();
         if(1) { //pos3.x()<10 &&pos3.x()>-10

           if(pos3.x()==0.0 || pos3.y()==0.0){
               Turtle3Pos[0] = pre_pos3.x();
               Turtle3Pos[1] = pre_pos3.y();
               Turtle3Pos[2] = pre_pos3.z();
           }
           else{
               pre_pos3.setX(pos3.x());
               pre_pos3.setY(pos3.y());
               pre_pos3.setZ(pos3.z());
               Turtle3Pos[0] = pos3.x();
               Turtle3Pos[1] = pos3.y();
               Turtle3Pos[2] = pos3.z();
           }
//           ROS_INFO("3 x:%1.2f y:%1.2f z:%1.2f", pos3.x(), pos3.y(), pos3.z());
         }

         pos4 = tr[4].getOrigin();
         if(1) {//pos4.x()<10 &&pos4.x()>-10

           if(pos4.x()==0.0 || pos4.y()==0.0){
               Turtle4Pos[0] = pre_pos4.x();
               Turtle4Pos[1] = pre_pos4.y();
               Turtle4Pos[2] = pre_pos4.z();
           }
           else{
               pre_pos4.setX(pos4.x());
               pre_pos4.setY(pos4.y());
               pre_pos4.setZ(pos4.z());
               Turtle4Pos[0] = pos4.x();
               Turtle4Pos[1] = pos4.y();
               Turtle4Pos[2] = pos4.z();
           }
//           ROS_INFO("4 x:%1.2f y:%1.2f z:%1.2f", pos4.x(), pos4.y(), pos4.z());
         }

         pos5 = tr[5].getOrigin();
         if(1) {//pos5.x()<10 &&pos5.x()>-10

            if(pos5.x()==0.0 || pos5.y()==0.0){
                Turtle5Pos[0] = pre_pos5.x();
                Turtle5Pos[1] = pre_pos5.y();
                Turtle5Pos[2] = pre_pos5.z();
            }
            else{
                pre_pos5.setX(pos5.x());
                pre_pos5.setY(pos5.y());
                pre_pos5.setZ(pos5.z());
                Turtle5Pos[0] = pos5.x();
                Turtle5Pos[1] = pos5.y();
                Turtle5Pos[2] = pos5.z();
            }
//            ROS_INFO("5 x:%1.2f y:%1.2f z:%1.2f", pos5.x(), pos5.y(), pos5.z());
         }

         quarterion[1] = tr[1].getRotation();
         quarterion[2] = tr[2].getRotation();
         quarterion[3] = tr[3].getRotation();
         quarterion[4] = tr[4].getRotation();
         quarterion[5] = tr[5].getRotation();

       }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "estimator_gt");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("camera1/ar_pose_marker", 1, pose_estimation);

    ros::Publisher pub1 =nh.advertise<nav_msgs::Odometry>("/gt_1/odom",10);
    ros::Publisher pub2 =nh.advertise<nav_msgs::Odometry>("/gt_2/odom",10);
    ros::Publisher pub3 =nh.advertise<nav_msgs::Odometry>("/gt_3/odom",10);
    ros::Publisher pub4 =nh.advertise<nav_msgs::Odometry>("/gt_4/odom",10);
    ros::Publisher pub5 =nh.advertise<nav_msgs::Odometry>("/gt_5/odom",10);

    nav_msgs::Odometry Odom1;
    nav_msgs::Odometry Odom2;
    nav_msgs::Odometry Odom3;
    nav_msgs::Odometry Odom4;
    nav_msgs::Odometry Odom5;

    ros::Rate rate(500);
    while(ros::ok()){
      Odom1.pose.pose.position.x=Turtle1Pos[0];//
      Odom1.pose.pose.position.y=Turtle1Pos[1];
      Odom1.pose.pose.position.z=Turtle1Pos[2];
      Odom2.pose.pose.position.x=Turtle2Pos[0];//
      Odom2.pose.pose.position.y=Turtle2Pos[1];
      Odom2.pose.pose.position.z=Turtle2Pos[2];
      Odom3.pose.pose.position.x=Turtle3Pos[0];//
      Odom3.pose.pose.position.y=Turtle3Pos[1];
      Odom3.pose.pose.position.z=Turtle3Pos[2];
      Odom4.pose.pose.position.x=Turtle4Pos[0];//
      Odom4.pose.pose.position.y=Turtle4Pos[1];
      Odom4.pose.pose.position.z=Turtle4Pos[2];
      Odom5.pose.pose.position.x=Turtle5Pos[0];//
      Odom5.pose.pose.position.y=Turtle5Pos[1];
      Odom5.pose.pose.position.z=Turtle5Pos[2];

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

      Odom4.pose.pose.orientation.x = quarterion[4].getX();
      Odom4.pose.pose.orientation.y = quarterion[4].getY();
      Odom4.pose.pose.orientation.z = quarterion[4].getZ();
      Odom4.pose.pose.orientation.w = quarterion[4].getW();

      Odom5.pose.pose.orientation.x = quarterion[5].getX();
      Odom5.pose.pose.orientation.y = quarterion[5].getY();
      Odom5.pose.pose.orientation.z = quarterion[5].getZ();
      Odom5.pose.pose.orientation.w = quarterion[5].getW();

//      pub1.publish(Odom1);
//      pub2.publish(Odom2);
      pub3.publish(Odom3);
//      pub4.publish(Odom4);
//      pub5.publish(Odom5);

      ros::spinOnce();
      rate.sleep();
    }
    return 0;

}
