#include<cstdlib>
#include<cstring>
#include<iostream>
#include<boost/asio.hpp>
#include<boost/bind.hpp>
#include "datatype_estim.h"

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <eigen3/Eigen/Eigen>
#include <typeinfo>
#include <time.h>
#include <sys/time.h>
#include <fstream>

////// CONFIGURATION //////
/* TO FIND HOMOGENEOUS MTX OF CAMERA */
//#define FINDMTX
/* NUMBER OF ESTIMATORS */
#define NUM_ESTIMS 4
/* TIMECHECK */
//#define TIMECHECK
/* STEPCHECK */
#define STEPCHECK
/* MEASUREMENT */
#define YMEASURE
/* LOOP TIME */
#define HZ 200 //200 : 5ms

/* ROBOT MODEL */
//#define CIRCLE_MODEL
#define STOP_MODEL
///////////////////////////

#define SQR(x) ((x)*(x))

using boost::asio::ip::tcp;
using namespace std;

#ifdef TIMECHECK
clock_t start, end;
double result;
struct timeval start1, end1;
struct timeval default_clk;
long default_sec;
string filePath = "delay_1.txt";
ofstream writeFile(filePath.data());
#endif

#ifdef FINDMTX
string filePath = "client1.txt";
ofstream writeFile(filePath.data());
#endif

#ifdef STEPCHECK
string filePath2 = "stepcheck_1.txt";
ofstream writeFile2(filePath2.data());
#endif
////// GLOBAL VARIABLES //////
Packet_t g_uRecv_odom;
float g_theta = 0;
static unsigned char g_mode = 0;
static unsigned char g_svmode = 0;
static unsigned char pre_g_mode = 0;
static float g_leader_theta = 0;
float g_gainK = 1.0;
#ifdef FINDMTX
double g_pos[3] = {0,};
double g_qut[4] = {0,};
tf::Transform TF;
tf::Quaternion QT;
tf::Matrix3x3 g_mtx;
tf::Vector3 g_vec;
#endif

double g_rbpos[3] = {0,};
double g_rbqut[4] = {0,};
Eigen::MatrixXf g_y(1,1);
double g_yi = 0;
//////////////////////////////

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
        #ifdef TIMECHECK
        // Check Default Time //
        gettimeofday(&default_clk,0);
        default_sec = default_clk.tv_sec+(long)default_clk.tv_usec*1e-6;
        #endif
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
      m_uSendPacket.stData.mode   = g_mode;
      m_uSendPacket.stData.check  = 0;
      for (int i = 0; i<4;i++){
          m_uSendPacket.stData.xi[i]     = g_uRecv_odom.stData.xi[i];
          m_uSendPacket.stData.lambda[i] = g_uRecv_odom.stData.lambda[i];
      }
      for (int i = 0; i<10;i++){
          m_uSendPacket.stData.zeta[i]   = g_uRecv_odom.stData.zeta[i]  ; // zeta
          m_uSendPacket.stData.mu[i]     = g_uRecv_odom.stData.mu[i]    ; // mu
      }
      m_uSendPacket.stData.y_i           = g_uRecv_odom.stData.y_i;
      m_uSendPacket.stData.reserved      = g_uRecv_odom.stData.reserved;
      for(int i = 8; i < sizeof(Packet_t); i++){
          m_uSendPacket.stData.check += m_uSendPacket.buffer[i];
      }
      boost::asio::async_write(m_Socket, boost::asio::buffer((char*)m_uSendPacket.buffer, sizeof(Packet_t)),
                               boost::bind(&client::write_handler,this,boost::asio::placeholders::error));
      #ifdef TIMECHECK
      // Time Check Start //
      gettimeofday(&start1,0);
      #endif
      recv_Packet();
  }
  void write_handler(const boost::system::error_code& e){
    if(!e){
    }
    else{
        cout<<"write fail"<<endl;
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
                    }
                    m_check = 0;
                    m_mode = 0;
                    m_checkSize = 0;
                }
            }
        }
        #ifdef TIMECHECK
        // Time Check End //
        gettimeofday(&end1,0);
        long sec = end1.tv_sec - start1.tv_sec;
        long mic_sec = end1.tv_usec - start1.tv_usec;
        double time_delay = sec + mic_sec*1e-6;
        printf("time delay : %.6f\n",time_delay);
        if(writeFile.is_open()){
            // sec - defalut_sec : Time since the Program Start
            // time_delay : Time between send -> recv Packet at client
            writeFile << (long)(end1.tv_sec+(long)end1.tv_usec*1e-6) - default_sec<<" "<<time_delay<<endl;
        }
        #endif
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

  unsigned char m_RecvBuf[132];
  Packet_t m_uSendPacket;
  Packet_t m_uRecvPacket;
  int m_mode=0, m_readSize = 0, m_checkSize=0;
  unsigned char m_check=0;
};

#ifdef FINDMTX
void tb1_trans(nav_msgs::Odometry odom) {
    if((g_mode != pre_g_mode)&&(odom.pose.pose.position.x*odom.pose.pose.position.x+odom.pose.pose.position.y*odom.pose.pose.position.y+odom.pose.pose.position.z*odom.pose.pose.position.z>0.001)){ // Memorize Current Odometry of AR Marker
        pre_g_mode++;
        g_pos[0] = odom.pose.pose.position.x;
        g_pos[1] = odom.pose.pose.position.y;
        g_pos[2] = odom.pose.pose.position.z;

        QT.setX(odom.pose.pose.orientation.x);
        QT.setY(odom.pose.pose.orientation.y);
        QT.setZ(odom.pose.pose.orientation.z);
        QT.setW(odom.pose.pose.orientation.w);
        TF.setRotation(QT);
        g_mtx = TF.getBasis();

        if(writeFile.is_open()&&(g_mode == pre_g_mode)){
            writeFile<<(unsigned int)pre_g_mode<<" 1 ";
            for (int i = 0; i<3; i++) {
                g_vec = g_mtx[i]; // 3X3 MTX - ROW COLUMN RETURN
                writeFile<<g_vec.getX()<<" "<<g_vec.getY()<<" "<<g_vec.getZ()<<" ";
            }
            writeFile<<g_pos[0]<<" "<<g_pos[1]<<" "<<g_pos[2]<<" "<<(unsigned int)g_mode<<endl;

            if(g_mode > 200){
                writeFile.close();
            }
        }
    }
}
void tb2_trans(nav_msgs::Odometry odom) {
    if((g_mode != pre_g_mode)&&(odom.pose.pose.position.x*odom.pose.pose.position.x+odom.pose.pose.position.y*odom.pose.pose.position.y+odom.pose.pose.position.z*odom.pose.pose.position.z>0.001)){ // Memorize Current Odometry of AR Marker
        pre_g_mode++;
        g_pos[0] = odom.pose.pose.position.x;
        g_pos[1] = odom.pose.pose.position.y;
        g_pos[2] = odom.pose.pose.position.z;

        QT.setX(odom.pose.pose.orientation.x);
        QT.setY(odom.pose.pose.orientation.y);
        QT.setZ(odom.pose.pose.orientation.z);
        QT.setW(odom.pose.pose.orientation.w);
        TF.setRotation(QT);
        g_mtx = TF.getBasis();

        if(writeFile.is_open()&&(g_mode == pre_g_mode)){
            writeFile<<(unsigned int)pre_g_mode<<" 2 ";
            for (int i = 0; i<3; i++) {
                g_vec = g_mtx[i]; // 3X3 MTX - ROW COLUMN RETURN
                writeFile<<g_vec.getX()<<" "<<g_vec.getY()<<" "<<g_vec.getZ()<<" ";
            }
            writeFile<<g_pos[0]<<" "<<g_pos[1]<<" "<<g_pos[2]<<" "<<(unsigned int)g_mode<<endl;

            if(g_mode > 200){
                writeFile.close();
            }
        }
    }
}
void tb4_trans(nav_msgs::Odometry odom) {
    if((g_mode != pre_g_mode)&&(odom.pose.pose.position.x*odom.pose.pose.position.x+odom.pose.pose.position.y*odom.pose.pose.position.y+odom.pose.pose.position.z*odom.pose.pose.position.z>0.001)){ // Memorize Current Odometry of AR Marker
        pre_g_mode++;
        g_pos[0] = odom.pose.pose.position.x;
        g_pos[1] = odom.pose.pose.position.y;
        g_pos[2] = odom.pose.pose.position.z;

        QT.setX(odom.pose.pose.orientation.x);
        QT.setY(odom.pose.pose.orientation.y);
        QT.setZ(odom.pose.pose.orientation.z);
        QT.setW(odom.pose.pose.orientation.w);
        TF.setRotation(QT);
        g_mtx = TF.getBasis();

        if(writeFile.is_open()&&(g_mode == pre_g_mode)){
            writeFile<<(unsigned int)pre_g_mode<<" 4 ";
            for (int i = 0; i<3; i++) {
                g_vec = g_mtx[i]; // 3X3 MTX - ROW COLUMN RETURN
                writeFile<<g_vec.getX()<<" "<<g_vec.getY()<<" "<<g_vec.getZ()<<" ";
            }
            writeFile<<g_pos[0]<<" "<<g_pos[1]<<" "<<g_pos[2]<<" "<<(unsigned int)g_mode<<endl;

            if(g_mode > 200){
                writeFile.close();
            }
        }
    }
}
void tb5_trans(nav_msgs::Odometry odom) {
    if((g_mode != pre_g_mode)&&(odom.pose.pose.position.x*odom.pose.pose.position.x+odom.pose.pose.position.y*odom.pose.pose.position.y+odom.pose.pose.position.z*odom.pose.pose.position.z>0.001)){ // Memorize Current Odometry of AR Marker
        pre_g_mode++;
        g_pos[0] = odom.pose.pose.position.x;
        g_pos[1] = odom.pose.pose.position.y;
        g_pos[2] = odom.pose.pose.position.z;

        QT.setX(odom.pose.pose.orientation.x);
        QT.setY(odom.pose.pose.orientation.y);
        QT.setZ(odom.pose.pose.orientation.z);
        QT.setW(odom.pose.pose.orientation.w);
        TF.setRotation(QT);
        g_mtx = TF.getBasis();

        if(writeFile.is_open()&&(g_mode == pre_g_mode)){
            writeFile<<(unsigned int)pre_g_mode<<" 5 ";
            for (int i = 0; i<3; i++) {
                g_vec = g_mtx[i]; // 3X3 MTX - ROW COLUMN RETURN
                writeFile<<g_vec.getX()<<" "<<g_vec.getY()<<" "<<g_vec.getZ()<<" ";
            }
            writeFile<<g_pos[0]<<" "<<g_pos[1]<<" "<<g_pos[2]<<" "<<(unsigned int)g_mode<<endl;

            if(g_mode > 200){
                writeFile.close();
            }
        }
    }
}
#endif

void robot_trans(nav_msgs::Odometry odom) {
    if(odom.pose.pose.position.x*odom.pose.pose.position.x+odom.pose.pose.position.y*odom.pose.pose.position.y+odom.pose.pose.position.z*odom.pose.pose.position.z>0.001){ // Memorize Current Odometry of AR Marker

        // Robot Position Measurement //
        g_rbpos[0] = odom.pose.pose.position.x;
        g_rbpos[1] = odom.pose.pose.position.y;
        g_rbpos[2] = odom.pose.pose.position.z;
        g_theta = (double)(atan2f(2.0*(odom.pose.pose.orientation.w*odom.pose.pose.orientation.z - odom.pose.pose.orientation.x*odom.pose.pose.orientation.y),
                  -SQR(odom.pose.pose.orientation.x)+SQR(odom.pose.pose.orientation.y)-SQR(odom.pose.pose.orientation.z)+SQR(odom.pose.pose.orientation.w)));
        // Set Measurement Value at Packet & Loop - X position
#ifdef XMEASURE
        g_y << g_rbpos[0];
        g_uRecv_odom.stData.y_i = g_yi = g_rbpos[0];
#endif
#ifdef YMEASURE
        g_y << g_rbpos[1];
        g_uRecv_odom.stData.y_i = g_yi = g_rbpos[1];
#endif
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "estim_client1");
    ros::NodeHandle nh;
    ros::Rate rate(HZ); // 5ms
#ifdef FINDMTX
    ros::Subscriber sub1 = nh.subscribe("/mk_1/odom", 1, tb1_trans);
    ros::Subscriber sub2 = nh.subscribe("/mk_2/odom", 1, tb2_trans);
    ros::Subscriber sub4 = nh.subscribe("/mk_4/odom", 1, tb4_trans);
    ros::Subscriber sub5 = nh.subscribe("/mk_5/odom", 1, tb5_trans);
#endif
    ros::Subscriber rbsub = nh.subscribe("/robot/odom", 1, robot_trans);

    boost::asio::io_service io_context;
    client c(io_context, "192.168.0.189", 2222); //52.69.115.24

    double dt = 1 / 200;
    Packet_t pk;

    Eigen::MatrixXf A_d(4,4);
    // Setting (v : 0.05    w : 0.3     Ts : 0.05)
#ifdef CIRCLE_MODEL
    A_d <<  1,  0.05,   0,  -0.0004,
            0,  0.9999, 0,  -0.0150,
            0,  0.0004, 1,  0.05,
            0,  0.0150, 0,  0.9999;
#endif
#ifdef STOP_MODEL
    A_d = Eigen::MatrixXf::Identity(4,4);
#endif
    Eigen::MatrixXf H(1,4);
#ifdef XMEASURE
    H << 1, 0, 0, 0;
#endif
#ifdef YMEASURE
    H << 0, 0, 1, 0;
#endif
    Eigen::VectorXf xhat_cur(4,1);
    Eigen::VectorXf xhat_bef(4,1);
    xhat_bef << 3, 1, 3, 1;
    Eigen::MatrixXf P_cur(4,4);
    Eigen::MatrixXf P_bef(4,4);
    P_bef = 0.1 * Eigen::MatrixXf::Identity(4,4);
    Eigen::MatrixXf OMG(4,4);
    Eigen::VectorXf OMG_vec(10,1);
    Eigen::MatrixXf omg(4,4);
    Eigen::VectorXf omg_vec(10,1);


    Eigen::MatrixXf Q(4,4);
    Q = 0.1 * Eigen::MatrixXf::Identity(4,4);
    Eigen::MatrixXf R(1,1);
    R = 0.1 * Eigen::MatrixXf::Identity(1,1);

    Eigen::VectorXf lambda_cur(4,1);
    Eigen::VectorXf lambda_bef(4,1);
    Eigen::VectorXf lambda_trans(4,1);
    Eigen::MatrixXf K_dual(4,4);
    double eps = 0.1;

    Eigen::VectorXf xi(4,1);
    Eigen::VectorXf xi_trans(4,1);
    Eigen::VectorXf K_inno(4,1);
    Eigen::VectorXf psi(4,1);

    Eigen::VectorXf mu_cur(10,1);
    Eigen::VectorXf mu_bef(10,1);
    mu_bef.setZero();

    Eigen::VectorXf mu_trans(10,1);

    Eigen::VectorXf zeta(10,1);
    omg = H.transpose()*R.inverse()*H;
    zeta << omg(0),omg(1),omg(2) ,omg(3) ,
                   omg(5),omg(6) ,omg(7) ,
                          omg(10),omg(11),
                                  omg(15);
    Eigen::MatrixXf zeta_devec(4,4);
    Eigen::VectorXf zeta_trans(10,1);

    lambda_trans.setZero();
    xi_trans.setZero();
    mu_trans.setZero();
    zeta_trans.setZero();

    unsigned int N = NUM_ESTIMS; // Number of Estimators
    double alpha_lambda = 0.1;   // MAX(e.v) = 4
    double alpha_mu = 0.1;       // MAX(e.v) = 4
    double g_w = 0.3;
    unsigned int timer1 = 0;
    unsigned int timer2 = 0;
    unsigned int update = 1;
    unsigned int l = 0;
    unsigned int end = 0;
    unsigned int flag = 1;
    g_uRecv_odom.stData.reserved = 0;
    while(ros::ok()){ //5ms

        io_context.poll(); //return 2 every loop
        pk = c.get_Packet();

        // Update Global Variables recved from Master
        g_svmode = pk.stData.mode;
        lambda_trans = Eigen::Map<Eigen::Vector4f> (pk.stData.lambda,4,1);
        mu_trans = Eigen::Map<Eigen::VectorXf> (pk.stData.mu,10,1);
        xi_trans = Eigen::Map<Eigen::Vector4f> (pk.stData.xi,4,1);
        zeta_trans = Eigen::Map<Eigen::VectorXf> (pk.stData.zeta,10,1);

        if(timer2 % 10 == 0 && end ==1){ // 50ms
            // END LOOP Result , xhat_bef & P_bef //
            xhat_bef = xi;
            zeta_devec << zeta(0), zeta(1), zeta(2), zeta(3),
                          zeta(1), zeta(4), zeta(5), zeta(6),
                          zeta(2), zeta(5), zeta(7), zeta(8),
                          zeta(3), zeta(6), zeta(8), zeta(9);
            P_bef = (OMG + zeta_devec).inverse();
            timer2 = 0;
            update = 1;
            end = 0;
#ifdef STEPCHECK
        if(writeFile2.is_open()){
            writeFile2 <<"<end loop-gt>"<<endl;
            writeFile2 <<"<server time>: "<< pk.stData.reserved <<endl;
            writeFile2 <<"xi = "<< endl<<xi<<endl;
            writeFile2 <<"theta(estim):"<<atan2f(xi(3),xi(1))<<endl;
            writeFile2 <<"theta(measure):"<<g_theta<<endl;
            writeFile2 <<"v(estim): "<<xi(1)/cos(atan2f(xi(3),xi(1)))<<endl;
        }
#endif
        cout <<endl<<"xi = "<< endl<<xi<<endl;
        cout <<"theta(estim):"<<atan2f(xi(3),xi(1))<<endl;
        cout <<"theta(measure):"<<g_theta<<endl;
        cout <<"v(estim): "<<xi(1)/cos(atan2f(xi(3),xi(1)))<<endl;
        }
        timer2++;

        // LOCAL PREDICTION - OUTER LOOP//
        if(timer1 % 10 == 0){ // 50ms
            // START LOOP Result //
            xhat_cur = A_d*xhat_bef;
            P_cur = A_d*P_bef*A_d.transpose()+Q;// Noise : mean :0 variance^2=Q
            OMG = P_cur.inverse();
            // Initialization //
            lambda_bef.setZero();
            xi = xhat_cur;
            timer1 = 0;
#ifdef STEPCHECK
        if(writeFile2.is_open()){
//            writeFile2 <<"<t1>";
        }
#endif
        }
        timer1++;

        // DISTRIBUTED CORRECTION - INNER LOOP//
        if(g_svmode == 1 && update ==1 && flag ==1){
            K_dual = (1 / ( (N*P_cur).norm() + eps))*Eigen::MatrixXf::Identity(4,4);
            lambda_cur = lambda_bef + alpha_lambda * K_dual * xi_trans;             // 2)
            lambda_bef = lambda_cur;
#ifdef STEPCHECK
        if(writeFile2.is_open()){
//            writeFile2 <<"mu="<<endl<<mu_bef<<endl;
//            writeFile2 <<"zeta_trans="<<endl<<zeta_trans<<endl;
        }
#endif
            mu_cur = mu_bef + alpha_mu * zeta_trans;
            mu_bef = mu_cur;
            g_mode = 2;
            flag = 2;
#ifdef STEPCHECK
        if(writeFile2.is_open()){
//            writeFile2 <<"<sv1>";
        }
#endif
        }
        else if (g_svmode == 2 && update ==1 && flag ==2) {
            K_inno =  (H.transpose()*R.inverse()*H + OMG/N).inverse() * H.transpose()*R.inverse();
            psi =(H.transpose()*R.inverse()*H + OMG/N).inverse() * lambda_trans;
            xi = xhat_cur + K_inno*(g_y-H*xhat_cur) - psi;                                   // 1)

            // Vectorization of omg //
            omg = H.transpose()*R.inverse()*H;
            omg_vec << omg(0),omg(1),omg(2) ,omg(3) ,
                              omg(5),omg(6) ,omg(7) ,
                                     omg(10),omg(11),
                                             omg(15);
            zeta = N*omg_vec - mu_trans;

#ifdef STEPCHECK
        if(writeFile2.is_open()){
//            writeFile2 <<"<sv2>";
        }
#endif
            flag = 1;
            g_mode = 1;
            l++;
            g_uRecv_odom.stData.reserved = l;
            if(l > 0){ // l*: 2
                l = 0;
                update = 0;
                end = 1;
#ifdef STEPCHECK
        if(writeFile2.is_open()){
//            writeFile2 <<"<l*>"<<endl;
        }
#endif
            }
        }
        float* ft = lambda_cur.data();
        for (int i =0; i<4; i++) {
            g_uRecv_odom.stData.lambda[i] = ft[i];
        }
        ft = xi.data();
        for (int i =0; i<4; i++) {
            g_uRecv_odom.stData.xi[i] = ft[i];
        }
        ft = mu_cur.data();
        for (int i =0; i<10; i++) {
            g_uRecv_odom.stData.mu[i] = ft[i];
        }
        ft = zeta.data();
        for (int i =0; i<10; i++) {
            g_uRecv_odom.stData.zeta[i] = ft[i];
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
