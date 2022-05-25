#include<cstdlib>
#include<cstring>
#include<iostream>
#include<boost/asio.hpp>
#include<boost/bind.hpp>
#include "datatype.h"

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include <eigen3/Eigen/Eigen>

#include <time.h>
#include <sys/time.h>
#include <fstream>

#define SQR(x) ((x)*(x))
#define HZ 100

//#define ORIGINAL
#define ROT_SCAL
#define LEADER
using boost::asio::ip::tcp;
using namespace std;

Packet_t g_uRecv_odom;
geometry_msgs::Twist g_cmdvel;
float d_x = 0;
float d_y = 0;
float g_theta = 0;
static float g_leader_theta = 0;
static float g_gainK = 1;
unsigned char g_mode = 0;
Eigen::MatrixXf d(2,1);
Eigen::MatrixXf R(2,2);
Eigen::MatrixXf r(2,1);

//Time Check Variables//
clock_t start, end;
double result;
struct timeval start1, end1;
struct timeval default_clk;
long default_sec;
string filePath = "delay_joy.txt";
ofstream writeFile(filePath.data());

class client
{
public:
  client(boost::asio::io_service& io_context, const string& host, const int& port) : m_Socket(io_context)
  {
      // CONNECT //
      boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(host), port);
      m_Socket.async_connect(endpoint,
                          boost::bind(&client::connect_handler, this, boost::asio::placeholders::error));
  }
  Packet_t get_Packet(){
    return m_uRecvPacket;
  }
  void connect_handler(const boost::system::error_code& e){

    if(!e){
        cout << "Connected!" << endl;
        gettimeofday(&default_clk,0);
        default_sec = default_clk.tv_sec+(long)default_clk.tv_usec*1e-6;
        write_Packet();
    }
  }
  void write_Packet(){
      if (m_Socket.is_open() == false) {
          cout<<"Socket Closed!!"<<endl;
          return;
      }
      // SEND PACKET //
      m_uSendPacket.stData.header[0] = m_uSendPacket.stData.header[1] = m_uSendPacket.stData.header[2] = m_uSendPacket.stData.header[3] = 0xFF;
      m_uSendPacket.stData.mode = g_mode;
      m_uSendPacket.stData.check = 0;
      m_uSendPacket.stData.id = 1; // LEADER ID
      m_uSendPacket.stData.z_x = g_uRecv_odom.stData.z_x;
      m_uSendPacket.stData.z_y = g_uRecv_odom.stData.z_y;
      m_uSendPacket.stData.d_x = g_uRecv_odom.stData.d_x;
      m_uSendPacket.stData.d_y = g_uRecv_odom.stData.d_y;
      m_uSendPacket.stData.yaw = g_uRecv_odom.stData.yaw;
      m_uSendPacket.stData.gainK = g_gainK;

      for(int i = 8; i < sizeof(Packet_t); i++){
          m_uSendPacket.stData.check += m_uSendPacket.buffer[i];
      }
      boost::asio::async_write(m_Socket, boost::asio::buffer((char*)m_uSendPacket.buffer, sizeof(Packet_t)),
                               boost::bind(&client::write_handler,this,boost::asio::placeholders::error));
      // Time Check Start //
      gettimeofday(&start1,0);
      recv_Packet();
  }
  void write_handler(const boost::system::error_code& e){
    if(!e){
      cout << "Write Complete!" << endl;
    }
  }
  void recv_Packet(){
      memset(&m_RecvBuf, '\0', sizeof(m_RecvBuf));
      m_Socket.async_read_some(boost::asio::buffer(m_RecvBuf), boost::bind(&client::recv_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

  }
  void recv_handler(const boost::system::error_code& e, size_t m_readSize){
      if(!e){
        for(int i = 0; i<m_readSize; i++){
            switch (m_mode) {
            case 0: // HEADER CHECK
                if(m_RecvBuf[i] == 0xFE){
                    m_checkSize++;
                    if(m_checkSize ==4){
                        m_mode = 1;
                    }
                }
                else{
                    m_checkSize = 0;
                }
                break;

            case 1: // CHAR * 4
                m_uRecvPacket.buffer[m_checkSize++] = m_RecvBuf[i];
                if(m_checkSize == 8) {
                    m_mode = 2;
                }
                break;

            case 2: // DATA
                m_uRecvPacket.buffer[m_checkSize++] = m_RecvBuf[i];
                m_check += m_RecvBuf[i];
                if(m_checkSize == sizeof(Packet_t)){
                    if(m_check == m_uRecvPacket.stData.check){
                        // Packet DATA RECV CHECK //
//                        cout<<"x:"<<m_uRecvPacket.stData.z_x<<",\t"<<"y:"<<m_uRecvPacket.stData.z_y<<endl;
//                        cout<<"dx:"<<m_uRecvPacket.stData.d_x<<",\t"<<"dy:"<<m_uRecvPacket.stData.d_y<<",\t"<<"yaw:"<<m_uRecvPacket.stData.yaw<<endl;
                    }
                    m_check = 0;
                    m_mode = 0;
                    m_checkSize = 0;
                }
            }
        }
        // Time Check End //
        gettimeofday(&end1,0);
        long sec = end1.tv_sec - start1.tv_sec;
        long mic_sec = end1.tv_usec - start1.tv_usec;
        double time_delay = sec + mic_sec*1e-6;
        printf("time delay : %.6f\n",time_delay);

        // Log Time Delay //
//        sec = (ros::Time::now().toSec()/1000)*1000;
        if(writeFile.is_open()){
            // sec - defalut_sec : Time since the Program Start
            // time_delay : Time between send -> recv Packet at client
            writeFile << (long)(end1.tv_sec+(long)end1.tv_usec*1e-6) - default_sec<<" "<<time_delay<<endl;

            if(g_mode == 4){
                writeFile.close();
            }
        }
        write_Packet();
      }
      else{
          if (e == boost::asio::error::eof)
              std::cout << "서버와 연결이 끊어졌습니다" << std::endl;
          else
              std::cout << "error No: " << e.value() << " error Message: " << e.message() << std::endl;
      }
  }

private:
  tcp::socket m_Socket;

  unsigned char m_RecvBuf[36];
  Packet_t m_uSendPacket;
  Packet_t m_uRecvPacket;
  int m_mode=0, m_readSize = 0, m_checkSize=0;
  unsigned char m_check=0;
};
void tb0_trans(nav_msgs::Odometry odom) {
#ifdef ORIGINAL
    g_uRecv_odom.stData.z_x = odom.pose.pose.position.x - d_x;
    g_uRecv_odom.stData.z_y = odom.pose.pose.position.y - d_y;
    g_uRecv_odom.stData.d_x = d_x;
    g_uRecv_odom.stData.d_y = d_y;
    g_uRecv_odom.stData.yaw = g_theta = (double)(atan2f(2.0*(odom.pose.pose.orientation.w*odom.pose.pose.orientation.z - odom.pose.pose.orientation.x*odom.pose.pose.orientation.y),
                              -SQR(odom.pose.pose.orientation.x)+SQR(odom.pose.pose.orientation.y)-SQR(odom.pose.pose.orientation.z)+SQR(odom.pose.pose.orientation.w)));
#endif

#ifdef ROT_SCAL
    d << d_x, d_y;
    R << cos(g_leader_theta), -sin(g_leader_theta), sin(g_leader_theta), cos(g_leader_theta);

    r = g_gainK*R*d;
    cout<<"d:"<<r(0)<<","<<r(1)<<endl;
    g_uRecv_odom.stData.z_x = odom.pose.pose.position.x - r(0);
    g_uRecv_odom.stData.z_y = odom.pose.pose.position.y - r(1);
    g_uRecv_odom.stData.d_x = r(0);
    g_uRecv_odom.stData.d_y = r(1);
    g_uRecv_odom.stData.yaw = g_theta = (double)(atan2f(2.0*(odom.pose.pose.orientation.w*odom.pose.pose.orientation.z - odom.pose.pose.orientation.x*odom.pose.pose.orientation.y),
                              -SQR(odom.pose.pose.orientation.x)+SQR(odom.pose.pose.orientation.y)-SQR(odom.pose.pose.orientation.z)+SQR(odom.pose.pose.orientation.w)));
    cout<<"//Send2GZ-odom//"<<endl;
    cout<<"Rmtx_cos:"<<cos(g_leader_theta)<<"\t d:"<<d_x<<","<<d_y<<"\t r:"<<r(0)<<","<<r(1)<<endl;
    cout<<"g_gainK: "<<g_gainK<<"\tmode: "<<(int)g_mode<<"\tyaw: "<<g_theta<<endl;
#endif
}

void tb0_joy(const sensor_msgs::Joy::ConstPtr &msg) {
    g_cmdvel.linear.x = (double)msg->axes[1]*0.4;
    g_cmdvel.angular.z = (double)msg->axes[3]*0.6;
    if((double)msg->buttons[2]==1.){ // K decrease
        g_gainK -= 0.1;
    }
    if((double)msg->buttons[1]==1.){ // K increase
        g_gainK += 0.1;
    }
    if((double)msg->buttons[0]==1.){ // Mode Change
        if(g_mode < 3){
            g_mode += 1;
        }
        else{
            g_mode = 0;
        }
    }
    if((double)msg->buttons[3]==1.){ // K increase
        g_cmdvel.linear.x = 0.05;
        g_cmdvel.angular.z = 0.3;
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "async_client0");
    ros::NodeHandle nh;
    ros::Rate rate(HZ); // 10ms

    ros::Subscriber sub = nh.subscribe("/tb3_3/odom", 1, tb0_trans);
    ros::Subscriber joysub = nh.subscribe("/joy", 1, tb0_joy);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/tb3/cmd_vel",10);

    boost::asio::io_service io_context;
    client c(io_context, "52.69.115.24", 2222); //192.168.0.65
    double dt = 1 / 100;
    Packet_t pk;
    double lin_vel = 0.02;

    Eigen::MatrixXf xi_i(4,1);
    Eigen::MatrixXf xi_idot(4,1);
    Eigen::MatrixXf dt_11(1,1);
    xi_i<<0,0,0,0;
    xi_idot<<0,0,0,0;
    dt_11<<0.01;
    //Eps : 0.01 , pole : -10 -12 -12 -14 , 3 Robot //
//    Eigen::MatrixXf A(4,4);
//    A<<-22.0396,    1.0,        0.3958,         0,
//       -120.5324,   -0.2045,    4.7494,         0,
//       -0.3958,     0,          -25.9604,       1.0,
//        4.7494,     0,          -167.5831,      -0.2045;
//    Eigen::MatrixXf K(4, 2);
//    K<< -22.0396,    0.3958,
//        -120.4746,   4.7494,
//        -0.3958,    -25.9604,
//        4.7494,     -167.5254;
//    Eigen::MatrixXf P(4,4);
//    P<<0.0354,      0.0577,     0,          0,
//       0.0577,      0.2045,     0,          0,
//       0,           0,          0.0354,     0.0577,
//       0,           0,          0.0577,     0.2045;
    //Eps : 0.0001 , pole : -10 -12 -12 -14 , 4 Robot //
//    Eigen::MatrixXf A(4,4);
//    A<<-22.0396,    1.0,        0.3958,         0,
//       -120.4796,   -0.0502,    4.7494,         0,
//       -0.3958,     0,          -25.9604,       1.0,
//        4.7494,     0,          -167.5304,      -0.0502;
//    Eigen::MatrixXf K(4, 2);
//    K<< -22.0396,    0.3958,
//        -120.4746,   4.7494,
//        -0.3958,    -25.9604,
//        4.7494,     -167.5254;
//    Eigen::MatrixXf P(4,4);
//    P<<0.0010,      0.0050,     0,          0,
//       0.0050,      0.0502,     0,          0,
//       0,           0,          0.0010,     0.0050,
//       0,           0,          0.0050,     0.0502;
    //Eps : 0.1 , pole : -10 -12 -12 -14 , 4 Robot - JOY0 //
    Eigen::MatrixXf A(4,4);
    A<<-22.0396,    1.0,        0.3958,         0,
       -120.6327,   -0.3226,    4.7494,         0,
       -0.3958,     0,          -25.9604,       1.0,
        4.7494,     0,          -167.6835,      -0.3226;
    Eigen::MatrixXf K(4, 2);
    K<< -22.0396,    0.3958,
        -120.4746,   4.7494,
        -0.3958,    -25.9604,
        4.7494,     -167.5254;
    Eigen::MatrixXf P(4,4);
    P<<0.2040,      0.1581,     0,          0,
       0.1581,      0.3226,     0,          0,
       0,           0,          0.2040,     0.1581,
       0,           0,          0.1581,     0.3226;

    Eigen::MatrixXf Out(2,1);
    Eigen::MatrixXf U_inv(2,2);
    Eigen::MatrixXf B_T(2,4);
    B_T<<0,         1,          0,          0,
         0,         0,          0,          1;
    Eigen::MatrixXf Z_i(2,1);

    //TIME check
    clock_t start, end;
    double result;
    struct timeval start1, end1;
    g_cmdvel.linear.x = 0;
    g_cmdvel.angular.z = 0;
    while(ros::ok()){
        //Time Check Method//
        /*
        gettimeofday(&end1,0);
        long sec = end1.tv_sec - start1.tv_sec;
        long mic_sec = end1.tv_usec - start1.tv_usec;
        double measure = sec + mic_sec*1e-6;
        printf("measure : %.6f\n",measure);
        gettimeofday(&start1,0);
        */

        io_context.poll();

        // Make Turtlebot Input //
        U_inv << cos(g_theta), sin(g_theta), -sin(g_theta)/lin_vel, cos(g_theta)/lin_vel;
        pk = c.get_Packet();
#ifndef LEADER
        g_leader_theta = pk.stData.yaw;
        g_gainK = pk.stData.gainK;
        cout<<"leader:"<<g_leader_theta<<"\tK:"<<g_gainK<<"mode:"<<pk.stData.mode<<endl;
#endif
        Z_i << pk.stData.z_x, pk.stData.z_y;
        xi_idot = A*xi_i + K*Z_i;
//        xi_i = xi_i + xi_idot*dt_11;
        xi_i = xi_i + xi_idot*0.01;
        Out = U_inv*B_T*P*xi_i;
        lin_vel += Out(0)*0.01;
        //cout<<"dt:"<<dt<<end;z
//        cout<<"gth\t"<<g_theta<<"zi\t"<<Z_i(0)<<"cid\t"<<xi_idot(0)<<"out"<<Out(0)<<","<<Out(1)<<endl;
//        cout<<"u_inv:("<<U_inv(0)<<","<<U_inv(1)<<","<<U_inv(2)<<","<<U_inv(3)<<")"<<endl;
//        cout<<"xi:("<<xi_i(0)<<","<<xi_i(1)<<","<<xi_i(2)<<","<<xi_i(3)<<")"<<endl;
//        cout<<"out:("<<Out(0)<<","<<Out(1)<<")"<<endl;
        // Minimum Velocity Saturation //
        if(lin_vel>=0 && lin_vel<0.01)        lin_vel = 0.01;
        else if(lin_vel<=0 && lin_vel>-0.01)  lin_vel = -0.01;
        // Maximum Velocity Saturation // Turtlebot Velocity Limit at 0.22m/s -> Renewal Optimal 0.18
        if (lin_vel > 0.22)                  lin_vel = 0.22;
        else if (lin_vel < -0.22)            lin_vel = -0.22;
        // Maximum Omega Saturation // Turtlebot Omega Limit at 2.84rad/s
        if (Out(1)>1.8)                    Out(1) =1.8;
        else if (Out(1)<-1.8)              Out(1) =-1.8;

        //cout<<"lin_vel:"<<lin_vel<<"\t"<<"angular:"<<Out(1)<<"g_theta:"<<g_theta<<endl;
        //g_cmdvel.linear.x = lin_vel;
        //g_cmdvel.angular.z = Out(1);
        pub.publish(g_cmdvel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
