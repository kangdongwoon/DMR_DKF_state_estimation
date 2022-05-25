#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include"datatype_estim.h"
#include <map>
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <eigen3/Eigen/Eigen>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <brl_msgs/brl_msgs.h>
#include <fstream>
#include <iostream>

////// CONFIGURATION //////
/* TO FIND HOMOGENEOUS MTX OF CAMERA */
//#define FINDMTX

/* TIMECHECK */
#define TIMECHECK

/* NUMBER OF ESTIMATORS */
#define NUM_ESTIMS  4
#define SESSION_NUM 4

/* DATA LOGGING */
#define DATALOG

/* CKF */
#define CKF

/* LOOP TIME */
#define HZ 500 // 2ms

/* ROBOT MODEL */
//#define CIRCLE_MODEL
#define STOP_MODEL
//////////////////////////////
/*
 * g_mode = 0 <Inital>
 * g_mode = 1 <Global Exchange-ALL Session Connected>
*/
using boost::asio::ip::tcp;
using namespace std;

#ifdef DATALOG
string filePath1 = "datalog.txt";
ofstream writeFile1(filePath1.data());
#endif

#ifdef CKF
string filePath2 = "CKF.txt";
ofstream writeFile2(filePath2.data());
#endif

class Session;
std::map<int, Session*> g_mpSessionIDs;
static int g_exchange = 0;
static float g_leader_theta = 0;
static float g_gainK = 1;
static unsigned char g_mode = 0;

class Session
{
public:
  Session(boost::asio::io_service& io_service) : m_Socket(io_service)
  {
    // HOLD HEADER //
    m_uSendPacket.stData.header[0]=m_uSendPacket.stData.header[1]=m_uSendPacket.stData.header[2]=m_uSendPacket.stData.header[3]=0x00;
  }
  tcp::socket& socket() {
    return m_Socket;
  }
  Packet_t get_Packet(){
    return m_uPacketBuffer;
  }
  void set_Packet(Packet_t pk){
      for (int i=0;i<4;i++) {
          m_uSendPacket.stData.xi[i] = pk.stData.xi[i];
          m_uSendPacket.stData.lambda[i] = pk.stData.lambda[i];
      }
      for (int i=0;i<10;i++) {
          m_uSendPacket.stData.zeta[i] = pk.stData.zeta[i];
          m_uSendPacket.stData.mu[i] = pk.stData.mu[i];
      }
  }
  void recv_Packet(){
    memset(&m_Buf, '\0', sizeof(m_Buf));
    m_Socket.async_read_some(boost::asio::buffer(m_Buf),
                             boost::bind(&Session::recv_handler, this,
                             boost::asio::placeholders::error,
                             boost::asio::placeholders::bytes_transferred));
  }
  void write_Packet(){
    if(g_exchange == 1){
      // Exchange Header 0xFE
      m_uSendPacket.stData.header[0]=m_uSendPacket.stData.header[1]=m_uSendPacket.stData.header[2]=m_uSendPacket.stData.header[3]=0xFE;
    }
    m_uSendPacket.stData.mode = g_mode;
    m_uSendPacket.stData.check = 0;

    for(int i=8; i<sizeof(Packet_t); i++){
      m_uSendPacket.stData.check += m_uSendPacket.buffer[i];
    }
    boost::asio::async_write(m_Socket,
                             boost::asio::buffer((char*)m_uSendPacket.buffer,sizeof(Packet_t)),
                             boost::bind(&Session::write_handler, this,
                             boost::asio::placeholders::error,
                             boost::asio::placeholders::bytes_transferred));
    recv_Packet();
  }
  void recv_handler(const boost::system::error_code& error, size_t m_readSize)
  {
    if (!error)
    {
      m_BufWriteCnt += m_readSize;
      for(int i = 0; i<m_readSize; i++){
          switch (m_PacketMode) {
          case 0: // HEADER CHECK
              if(m_Buf[i] == 0xFF){
                  m_checkSize++;
                  if(m_checkSize ==4){
                      m_PacketMode = 1;
                  }
              }
              else{
                  m_checkSize = 0;
              }
              break;

          case 1: // CHAR * 4
              m_uPacketBuffer.buffer[m_checkSize++] = m_Buf[i];
              if(m_checkSize == 8) {
                  m_PacketMode = 2;
              }
              break;

          case 2: // DATA
              m_uPacketBuffer.buffer[m_checkSize++] = m_Buf[i];
              m_check += m_Buf[i];
              if(m_checkSize == sizeof(Packet_t)){
                  if(m_check == m_uPacketBuffer.stData.check){
                      // Packet DATA RECV CHECK //
                  }
                  m_check = 0;
                  m_PacketMode = 0;
                  m_checkSize = 0;
              }
          }
      }
      write_Packet();
    }
    else
    {
      cout<<"Error :"<<error.message()<<endl;
      delete this;
    }
  }
  void write_handler(const boost::system::error_code& error,size_t readsize){
  }
private:
  tcp::socket m_Socket;
  Packet_t m_uSendPacket;
  Packet_t m_uPacketBuffer;
  unsigned char m_Buf[256], m_BufWriteCnt =0, m_BufReadCnt =0;
  unsigned char m_PacketMode = 0;
  int m_readSize =0, m_checkSize=0;
  unsigned char m_check=0;
};

class Server
{
public:
  Server(boost::asio::io_service& io_service, short port)
    : io_service_(io_service), acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
  {
    m_pSession = nullptr;
    accept_session();
  }
  ~Server(){
    if (m_pSession != nullptr)
          delete m_pSession;
  }
  int get_session_num(){
    return m_session_num;
  }
private:
  void accept_session()
  {
    m_pSession = new Session(io_service_);

    if(m_session_num == SESSION_NUM) {
        g_exchange = 1;
        g_mode = 1;
    }

    acceptor_.async_accept(m_pSession->socket(),
                           boost::bind(&Server::accept_handler, this, m_pSession,
                           boost::asio::placeholders::error));
    g_mpSessionIDs.insert(std::pair<int,Session*>(m_session_num++,m_pSession));
    cout<<m_session_num-1<<":"<<m_pSession<<"  "<<g_mpSessionIDs[m_session_num-1]<<endl;
  }
  void accept_handler(Session* m_pSession, const boost::system::error_code& error)
  {
    if (!error) {
      m_pSession->recv_Packet();
    }
    else {
      delete m_pSession;
    }
    accept_session();
  }
  int m_session_num = 0;
  boost::asio::io_service& io_service_;
  tcp::acceptor acceptor_;
  Session* m_pSession;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "async_server");
  ros::NodeHandle nh;

  ros::Rate rate(HZ);
  boost::asio::io_service io_service;
  Server s(io_service, 2222);

  Packet_t pk[SESSION_NUM];
  Eigen::MatrixXf L(SESSION_NUM, SESSION_NUM);
  Eigen::MatrixXf L_4cr(SESSION_NUM*4, SESSION_NUM*4);
  Eigen::MatrixXf L_10cr(SESSION_NUM*10, SESSION_NUM*10);
  if(SESSION_NUM == 4){
      #define SESS4
      L<< 3,    -1,     -1,     -1,
         -1,     3,     -1,     -1,
         -1,    -1,      3,     -1,
         -1,    -1,     -1,      3;
  }
  else if(SESSION_NUM == 3){
      #define SESS3
      L<< 2,   -1,   -1,
         -1,    2    -1,
         -1,   -1,    2;
  }
  else if(SESSION_NUM == 2){
      #define SESS2
      L<< 1,   -1,
         -1,    1;
  }
  L_4cr = Eigen::kroneckerProduct(L,Eigen::MatrixXf::Identity(4,4));
  L_10cr = Eigen::kroneckerProduct(L,Eigen::MatrixXf::Identity(10,10));
  Eigen::VectorXf lambda(4,1);
  Eigen::VectorXf lambda_ex(4,1);
  Eigen::VectorXf xi(4,1);
  Eigen::VectorXf xi_ex(4,1);
  Eigen::VectorXf mu(10,1);
  Eigen::VectorXf mu_ex(10,1);
  Eigen::VectorXf zeta(10,1);
  Eigen::VectorXf zeta_ex(10,1);

  Eigen::VectorXf Lambdas(4*SESSION_NUM,1);
  Eigen::VectorXf Lambda_exs(4*SESSION_NUM,1);
  Eigen::VectorXf Xis(4*SESSION_NUM,1);
  Eigen::VectorXf Xi_exs(4*SESSION_NUM,1);
  Eigen::VectorXf Mus(10*SESSION_NUM,1);
  Eigen::VectorXf Mu_exs(10*SESSION_NUM,1);
  Eigen::VectorXf Zetas(10*SESSION_NUM,1);
  Eigen::VectorXf Zeta_exs(10*SESSION_NUM,1);

  #ifdef CKF
  // Centralized Kalman Filtering
  Eigen::VectorXf Cxhat_cur(4,1);
  Eigen::VectorXf Cxhat_bef(4,1);
  Cxhat_bef << 0.1, 0.1, 0.1, 0.1;
  Eigen::VectorXf C_xi(4,1);
  Eigen::MatrixXf C_Ad(4,4);
  // Ts: 0.05s Mobilerobot DT-Model
  #ifdef CIRCLE_MODEL
    C_Ad <<  1,  0.05,   0,  -0.0004,
            0,  0.9999, 0,  -0.0150,
            0,  0.0004, 1,  0.05,
            0,  0.0150, 0,  0.9999;
  #endif
  #ifdef STOP_MODEL
    C_Ad = Eigen::MatrixXf::Identity(4,4);
  #endif

  Eigen::MatrixXf CQ(4,4);
  CQ = 0.1 * Eigen::MatrixXf::Identity(4,4);
  Eigen::MatrixXf CP_cur(4,4);
  Eigen::MatrixXf CP_bef(4,4);
  CP_bef = 0.1 * Eigen::MatrixXf::Identity(4,4);
  Eigen::MatrixXf CH(SESSION_NUM,4);
  if(SESSION_NUM == 2){
    CH << 1, 0, 0, 0,
          0, 0, 1, 0;
  }
  if(SESSION_NUM == 4){
    CH << 1, 0, 0, 0,
          0, 0, 1, 0,
          1, 0, 0, 0,
          0, 0, 1, 0;
  }
  Eigen::MatrixXf K_k(SESSION_NUM,SESSION_NUM);
  float Cy[SESSION_NUM];
  Eigen::VectorXf CYs(SESSION_NUM,1);
  Eigen::MatrixXf CR(SESSION_NUM,SESSION_NUM);
  CR = 0.1 * Eigen::MatrixXf::Identity(SESSION_NUM,SESSION_NUM);
  unsigned int timer = 0;
  #endif

  float default_secs = (ros::Time::now().toSec()/1000)*1000;
  float sec = 0;
  int cnt = 0;
  unsigned int k = 0;
  while(ros::ok()){
    io_service.poll();
    if(g_exchange == 1){
        // GET PACKET //
        for(int i = 0; i < s.get_session_num()-1; i++){
            pk[i] = g_mpSessionIDs[i]->get_Packet();
            #ifdef FINDMTX
                      if(pk[i].stData.id == 1){ // leader
                          g_mode = pk[i].stData.mode;
                          cout<<"gm::"<<g_mode<<endl;
                      }
            #endif
            #ifdef CKF
                      Cy[i] = pk[i].stData.y_i;
            #endif
            if(pk[i].stData.mode != g_mode){ // CHECK ALL THE CLIENT'S GMODE IS DIFF FROM SV
                cnt++;
            }
        }
        if(cnt == SESSION_NUM){
            sec = (ros::Time::now().toSec()/1000)*1000 - default_secs;
            /*
             * Clients Mode : 2
             * g_mode : 1
            */
            if(g_mode == 1){ // STEP1 EXCHANGE
                #ifdef DATALOG
                if(writeFile1.is_open()){
                    writeFile1<<sec<<endl;
                }
                #endif
                for(int i = 0; i < s.get_session_num()-1; i++){
                    // 1 : Lambdas block Generate
                    lambda = Eigen::Map<Eigen::Vector4f>(pk[i].stData.lambda,4,1);
                    Lambdas.block(4*i,0,4,1) = lambda;
                    // 2 : Mus block Generate
                    mu = Eigen::Map<Eigen::VectorXf>(pk[i].stData.mu,10,1);
                    Mus.block(10*i,0,10,1) = mu;
                    #ifdef DATALOG
                    if(writeFile1.is_open()){
                    //writeFile1<<"lambda("<<i<<"):"<<lambda<<endl;
                    //writeFile1<<"mu("<<i<<"):"<<mu.transpose()<<endl;
                    //writeFile1<<"y("<<i<<"):"<<pk[i].stData.y_i<<endl;
                    }
                    #endif
                }
                // Exchanges
                Lambda_exs = L_4cr*Lambdas;
                Mu_exs = L_10cr*Mus;

                for(int i = 0; i < s.get_session_num()-1; i++){
                    // 1 : Lambda Exchanged block Generate
                    lambda = Lambda_exs.block(4*i,0,4,1);
                    float* ft = lambda.data();
                    for (int j = 0; j < 4; j++) {
                        pk[i].stData.lambda[j] = ft[j];
                    }
                    // 2 : Mu Exchanged block Generate
                    mu = Mu_exs.block(10*i,0,10,1);
                    ft = mu.data();
                    for (int j = 0; j < 10; j++) {
                        pk[i].stData.mu[j] = ft[j];
                    }
                }
                g_mode = 2;
            }
            else if(g_mode == 2){ // STEP2 EXCHANGE
                for(int i = 0; i < s.get_session_num()-1; i++){
                    // 1 : Xis block Generate
                    xi = Eigen::Map<Eigen::Vector4f> (pk[i].stData.xi,4,1);
                    Xis.block(4*i,0,4,1) = xi;
                    // 2 : Zetas block Generate
                    zeta = Eigen::Map<Eigen::VectorXf> (pk[i].stData.zeta,10,1);
                    Zetas.block(10*i,0,10,1) = zeta;
                    cout<<"y("<<i<<"):"<<pk[i].stData.y_i<<endl;
                    cout<<"xi("<<i<<")"<<endl<<xi<<endl;
                    #ifdef DATALOG
                    if(writeFile1.is_open()){
//                        writeFile1<<"xi("<<i<<"):"<<endl<<xi<<endl;
//                        writeFile1<<"zeta("<<i<<"):"<<endl<<zeta.transpose()<<endl;
                        writeFile1<<i<<endl<<xi<<endl;
                    }
                    #endif
                }
                cout<<"CKF_Xhat: "<<endl<<Cxhat_bef<<endl;
                Xi_exs = L_4cr*Xis;
                Zeta_exs = L_10cr*Zetas;
                for(int i = 0; i < s.get_session_num()-1; i++){
                    xi = Xi_exs.block(4*i,0,4,1);
                    float* ft = xi.data();
                    for (int j = 0; j < 4; j++) {
                        pk[i].stData.xi[j] = ft[j];
                    }
                    zeta = Zeta_exs.block(10*i,0,10,1);
                    ft = zeta.data();
                    for (int j = 0; j < 10; j++) {
                        pk[i].stData.zeta[j] = ft[j];
                    }
                    #ifdef DATALOG
                    if(writeFile1.is_open()){
//                        writeFile1<<"xi("<<i<<"):"<<endl<<xi<<endl;
//                        writeFile1<<"zeta("<<i<<"):"<<endl<<zeta<<endl;
                    }
                    #endif
                }
                #ifdef DATALOG
                if(writeFile1.is_openjoy_tb0()){
                    writeFile1<<endl;
                }
                #endif
                g_mode = 1;
            }

            // SET PACKET //
            for(int i = 0; i < s.get_session_num()-1; i++){
              pk[i].stData.reserved = (float)sec;
              g_mpSessionIDs[i]->set_Packet(pk[i]);
            }
            cnt = 0;
        }
        else{
            cnt = 0;
        }

        // Centralized Kalman Filtering
        if(timer % 25 == 0){ // 2ms * 25 = 50ms
            CYs = Eigen::Map<Eigen::VectorXf>(Cy,SESSION_NUM,1);

            Cxhat_cur = C_Ad * Cxhat_bef;
            CP_cur = C_Ad*CP_bef*C_Ad.transpose()+CQ;

            K_k = (CH.transpose()*CR.inverse()*CH + CP_cur.inverse()).inverse()*CH.transpose()*CR.inverse();
            Cxhat_bef = Cxhat_cur + K_k*(CYs - CH*Cxhat_cur);
            CP_bef = CP_cur - CP_cur*CH.transpose()*(CH*CP_cur*CH.transpose() + CR).inverse()*CH*CP_cur;
            if(writeFile2.is_open()){
                // CKF Data Logging HERE!!! //
//                writeFile2<<"<<STEP1>>\t"<<sec<<"SEC"<<endl;
                writeFile2<<sec<<endl;
                for(int i = 0; i < s.get_session_num()-1; i++){
//                    writeFile2<<"y("<<i<<"):"<<pk[i].stData.y_i<<endl;
                }
//              writeFile2<<"Xhat: "<<endl<<Cxhat_bef<<endl;
                writeFile2<<Cxhat_bef<<endl;
//              writeFile2<<"PMTX: "<<endl<<CP_cur<<endl;
//              writeFile2<<"------------------------"<<endl;
            }
//          cout<<"Cxhat:"<<endl<<Cxhat_bef<<endl;
            timer++;
        }
        timer++;
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
