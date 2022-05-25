#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include"datatype.h"
#include <map>

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

#include <eigen3/Eigen/Eigen>

#include <brl_msgs/brl_msgs.h>
#include <fstream>
#include <iostream>

using boost::asio::ip::tcp;
using namespace std;

#define SESSION_NUM 4
class Session;
std::map<int, Session*> g_mpSessionIDs;
static int g_exchange = 0;
static float g_leader_theta = 0;
static float g_gainK = 1;
static unsigned char g_mode = 0;
string filePath = "xy.txt";
ofstream writeFile(filePath.data());

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
    m_uSendPacket.stData.z_x = pk.stData.z_x;
    m_uSendPacket.stData.z_y = pk.stData.z_y;
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
      m_uSendPacket.stData.header[0]=m_uSendPacket.stData.header[1]=m_uSendPacket.stData.header[2]=m_uSendPacket.stData.header[3]=0xFE;
    }
    m_uSendPacket.stData.d_x = m_uSendPacket.stData.d_y = 0;
    m_uSendPacket.stData.yaw = g_leader_theta;
    m_uSendPacket.stData.mode = g_mode;
    m_uSendPacket.stData.gainK = g_gainK;
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
//                        cout<<m_uPacketBuffer.stData.posx<<","<<m_uPacketBuffer.stData.posy<<endl;
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

    if(m_session_num == SESSION_NUM) g_exchange = 1;

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
  ros::Publisher Tb0_pub = nh.advertise<brl_msgs::brl_msgs>("/tb_0/odom",10);
  ros::Publisher Tb1_pub = nh.advertise<brl_msgs::brl_msgs>("/tb_1/odom",10);
  ros::Publisher Tb2_pub = nh.advertise<brl_msgs::brl_msgs>("/tb_2/odom",10);
  ros::Publisher Tb3_pub = nh.advertise<brl_msgs::brl_msgs>("/tb_3/odom",10);

  ros::Rate rate(50);
  boost::asio::io_service io_service;
  Server s(io_service, 2222);

  Packet_t pk[SESSION_NUM];
  Eigen::MatrixXf L(SESSION_NUM, SESSION_NUM);
  L<< 3,    -1,     -1,     -1,
     -1,    3,      -1,     -1,
     -1,    -1,     3,      -1,
     -1,    -1,     -1,     3;
  Eigen::MatrixXf z_ix(SESSION_NUM,1);
  Eigen::MatrixXf z_iy(SESSION_NUM,1);
  Eigen::MatrixXf zdot_ix(SESSION_NUM,1);
  Eigen::MatrixXf zdot_iy(SESSION_NUM,1);

  brl_msgs::brl_msgs tb0_odom;
  brl_msgs::brl_msgs tb1_odom;
  brl_msgs::brl_msgs tb2_odom;
  brl_msgs::brl_msgs tb3_odom;
  double default_secs = (ros::Time::now().toSec()/1000)*1000;
  double sec = 0;
  int cnt = 0;
  while(ros::ok()){
    io_service.poll();

    if(g_exchange == 1){
        // GET PACKET //
        for(int i = 0; i < s.get_session_num()-1; i++){
          pk[i] = g_mpSessionIDs[i]->get_Packet();
          cout<<i<<"zx: "<<pk[i].stData.z_x<<",zy"<<pk[i].stData.z_y<<endl;
          cout<<"dx:"<<pk[i].stData.d_x<<",\t"<<"dy:"<<pk[i].stData.d_y<<",\t yaw"<<pk[i].stData.yaw<<endl;

          if(pk[i].stData.id == 1){ // leader
              g_mode = pk[i].stData.mode;
              g_gainK = pk[i].stData.gainK;
              g_leader_theta = pk[i].stData.yaw;
          }
        }
        cout<<"---------------------------------"<<endl;
        // Draw Turtlebot IN Gazebo //
        tb0_odom.posx = pk[0].stData.z_x + pk[0].stData.d_x;;
        tb0_odom.posy = pk[0].stData.z_y + pk[0].stData.d_y;;
        tb0_odom.yaw  = pk[0].stData.yaw;

        tb1_odom.posx = pk[1].stData.z_x + pk[1].stData.d_x;;
        tb1_odom.posy = pk[1].stData.z_y + pk[1].stData.d_y;;
        tb1_odom.yaw  = pk[1].stData.yaw;

        tb2_odom.posx = pk[2].stData.z_x + pk[2].stData.d_x;
        tb2_odom.posy = pk[2].stData.z_y + pk[2].stData.d_y;
        tb2_odom.yaw  = pk[2].stData.yaw;

        tb3_odom.posx = pk[3].stData.z_x + pk[3].stData.d_x;;
        tb3_odom.posy = pk[3].stData.z_y + pk[3].stData.d_y;;
        tb3_odom.yaw  = pk[3].stData.yaw;

        Tb0_pub.publish(tb0_odom);
        Tb1_pub.publish(tb1_odom);
        Tb2_pub.publish(tb2_odom);
        Tb3_pub.publish(tb3_odom);

        // Log turtlebot's coordination //
        sec = (ros::Time::now().toSec()/1000)*1000;
        cnt++;
        if(writeFile.is_open() && cnt / 5 ==1){ //Every 0.1sec Log coordinate
            cnt = 0;
            writeFile << sec - default_secs<<endl;
            writeFile << tb0_odom.posx << " " << tb0_odom.posy<<"\t"<<tb0_odom.yaw<<endl;
            writeFile << tb1_odom.posx << " " << tb1_odom.posy<<"\t"<<tb1_odom.yaw<<endl;
            writeFile << tb2_odom.posx << " " << tb2_odom.posy<<"\t"<<tb2_odom.yaw<<endl;
            writeFile << tb3_odom.posx << " " << tb3_odom.posy<<"\t"<<tb3_odom.yaw<<endl;
            if(g_mode == 4){
                writeFile.close();
            }
        }

        // Exchange with Laplacian //
        z_ix << pk[0].stData.z_x,pk[1].stData.z_x,pk[2].stData.z_x,pk[3].stData.z_x;
        z_iy << pk[0].stData.z_y,pk[1].stData.z_y,pk[2].stData.z_y,pk[3].stData.z_y;
        zdot_ix = L*z_ix;
        zdot_iy = L*z_iy;

        // SET PACKET //
        for(int i = 0; i < s.get_session_num()-1; i++){
          pk[i].stData.z_x = zdot_ix(i);
          pk[i].stData.z_y = zdot_iy(i);
          g_mpSessionIDs[i]->set_Packet(pk[i]);
        }
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
