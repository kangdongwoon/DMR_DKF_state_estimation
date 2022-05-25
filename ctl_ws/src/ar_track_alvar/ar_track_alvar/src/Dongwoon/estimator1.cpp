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
static double** TurtlePos = new double*[6];

int flag = 0;
int i = 0;

tf::Quaternion quarterion[TURTLE_NUM];
tf::Vector3 pre_pos1, pre_pos2, pre_pos3, pre_pos4, pre_pos5;
tf::Vector3 pre_pos[6];
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

         // TRUTH VALUE-7 //
         // at Launch file, 'yaw, pitch, roll' order
         tC2W.setOrigin(tf::Vector3(0.2519,-1.2687,0.1350));
         q_cali.setRPY(-1.4798,-0.1257,-0.0273); // FIX FRAME
         tC2W.setRotation(q_cali);

         // EMPTY VALUE //
//         tC2W.setOrigin(tf::Vector3(0,0,0));
//         q_cali.setRPY(0,0,0); // at Launch file, 'yaw, pitch, roll' order
//         tC2W.setRotation(q_cali);


         // Transform MTX from Upper AR Marker
         for(int i = 0; i < 6; i++){
           cout<<i;
           if(req.markers[i].id == 1){
              tr[1].setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q1.setX(req.markers[i].pose.pose.orientation.x);
              q1.setY(req.markers[i].pose.pose.orientation.y);
              q1.setZ(req.markers[i].pose.pose.orientation.z);
              q1.setW(req.markers[i].pose.pose.orientation.w);
              tr[1].setRotation(q1);
              tr[1] = tC2W*tr[1]*tAR2CT[1];
           }
           if(req.markers[i].id == 2){
              tr[2].setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q2.setX(req.markers[i].pose.pose.orientation.x);
              q2.setY(req.markers[i].pose.pose.orientation.y);
              q2.setZ(req.markers[i].pose.pose.orientation.z);
              q2.setW(req.markers[i].pose.pose.orientation.w);
              tr[2].setRotation(q2);
              tr[2] = tC2W*tr[2]*tAR2CT[2];
           }
           if(req.markers[i].id == 3){
              tr[3].setOrigin(tf::Vector3(req.markers[i].pose.pose.position.x,req.markers[i].pose.pose.position.y,req.markers[i].pose.pose.position.z));
              q3.setX(req.markers[i].pose.pose.orientation.x);
              q3.setY(req.markers[i].pose.pose.orientation.y);
              q3.setZ(req.markers[i].pose.pose.orientation.z);
              q3.setW(req.markers[i].pose.pose.orientation.w);
              tr[3].setRotation(q3);
              tr[3] = tC2W*tr[3]*tAR2CT[3];
           }
           if(req.markers[i].id == 4){
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
         tf::Vector3 pos1,pos2,pos3,pos4,pos5;
         tf::Vector3 pos[6];
         for (int i = 1;i<6;i++) {
              pos[i] = tr[i].getOrigin();
              if(pos[i].x()==0.0 || pos[i].y()==0.0){
                  TurtlePos[i][0] = pre_pos[i].x();
                  TurtlePos[i][1] = pre_pos[i].y();
                  TurtlePos[i][2] = pre_pos[i].z();
              }
              else{
                  pre_pos[i].setX(pos[i].x());
                  pre_pos[i].setY(pos[i].y());
                  pre_pos[i].setZ(pos[i].z());
                  TurtlePos[i][0] = pos[i].x();
                  TurtlePos[i][1] = pos[i].y();
                  TurtlePos[i][2] = pos[i].z();
              }
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
    ros::init(argc, argv, "estimator1");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("camera2/ar_pose_marker", 1, pose_estimation);

    ros::Publisher pub1 =nh.advertise<nav_msgs::Odometry>("/mk_1/odom",10);
    ros::Publisher pub2 =nh.advertise<nav_msgs::Odometry>("/mk_2/odom",10);
    ros::Publisher pub3 =nh.advertise<nav_msgs::Odometry>("/mk_3/odom",10);
    ros::Publisher pub4 =nh.advertise<nav_msgs::Odometry>("/mk_4/odom",10);
    ros::Publisher pub5 =nh.advertise<nav_msgs::Odometry>("/mk_5/odom",10);
    ros::Publisher robotpub =nh.advertise<nav_msgs::Odometry>("/robot/odom",10);

    nav_msgs::Odometry Odom1;
    nav_msgs::Odometry Odom2;
    nav_msgs::Odometry Odom3;
    nav_msgs::Odometry Odom4;
    nav_msgs::Odometry Odom5;
    nav_msgs::Odometry Odom[6];
    nav_msgs::Odometry robotOdom;
    int cnt = 0;


    for(int i =1; i<6; i++){
        TurtlePos[i] = new double[3];
    }
    ros::Rate rate(500);
    while(ros::ok()){

        for (int i =1; i<6; i++) {
            Odom[i].pose.pose.position.x=TurtlePos[i][0];//
            Odom[i].pose.pose.position.y=TurtlePos[i][1];
            Odom[i].pose.pose.position.z=TurtlePos[i][2];
            Odom[i].pose.pose.orientation.x = quarterion[i].getX();
            Odom[i].pose.pose.orientation.y = quarterion[i].getY();
            Odom[i].pose.pose.orientation.z = quarterion[i].getZ();
            Odom[i].pose.pose.orientation.w = quarterion[i].getW();
        }

        // Estimate Robot Pose <Mean> //
        robotOdom.pose.pose.position.x = 0;
        robotOdom.pose.pose.position.y = 0;
        robotOdom.pose.pose.position.z = 0;
        robotOdom.pose.pose.orientation.x = 0;
        robotOdom.pose.pose.orientation.y = 0;
        robotOdom.pose.pose.orientation.z = 0;
        robotOdom.pose.pose.orientation.w = 0;
        cnt = 0;
        for (int i = 1; i<6; i++) {
            if( (quarterion[i].getX()*quarterion[i].getX())>0.000001 && (quarterion[i].getY()*quarterion[i].getY())>0.000001){
                cnt++;
                robotOdom.pose.pose.position.x    += TurtlePos[i][0];
                robotOdom.pose.pose.position.y    += TurtlePos[i][1];
                robotOdom.pose.pose.position.z    += TurtlePos[i][2];
                robotOdom.pose.pose.orientation.x += quarterion[i].getX();;
                robotOdom.pose.pose.orientation.y += quarterion[i].getY();;
                robotOdom.pose.pose.orientation.z += quarterion[i].getZ();;
                robotOdom.pose.pose.orientation.w += quarterion[i].getW();;
            }
        }
        robotOdom.pose.pose.position.x    /= cnt;
        robotOdom.pose.pose.position.y    /= cnt;
        robotOdom.pose.pose.position.z    /= cnt;
        robotOdom.pose.pose.orientation.x /= cnt;
        robotOdom.pose.pose.orientation.y /= cnt;
        robotOdom.pose.pose.orientation.z /= cnt;
        robotOdom.pose.pose.orientation.w /= cnt;

        pub1.publish(Odom[1]);
        pub2.publish(Odom[2]);
        pub3.publish(Odom[3]);
        pub4.publish(Odom[4]);
        pub5.publish(Odom[5]);
        robotpub.publish(robotOdom);

      ros::spinOnce();
      rate.sleep();
    }
    return 0;

}
