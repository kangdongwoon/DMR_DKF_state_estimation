//#include<cstdlib>
//#include<iostream>
//#include<boost/asio.hpp>
//#include"datatype.h"
//#include <map>

//#include <ros/ros.h>
//#include "nav_msgs/Odometry.h"

//using namespace std;
//using boost::asio::ip::tcp;
//class session;

//std::map<int, session*> g_mpSessionIDs;
//class session
//{
//public:
//    session(tcp::socket& sock)
//      :m_socket(sock){
////      cout<<g_mpSessionIDs.find(sock)<<endl;

//      cout<<"session start"<<endl;
////      start();
//    }

//    void start(){
//        m_uSendPacket.stData.header[0]=m_uSendPacket.stData.header[1]=m_uSendPacket.stData.header[2]=m_uSendPacket.stData.header[3]=0xFE;
//        m_uSendPacket.stData.orix =m_uSendPacket.stData.oriy = m_uSendPacket.stData.oriz = m_uSendPacket.stData.oriw = 0;
//        while(1){
//          boost::system::error_code error;
//          // RECV PACKET //
//          m_readSize = m_socket.read_some(boost::asio::buffer(m_Buf), error);
//          for(int i = 0; i<m_readSize; i++){
//              switch (m_PacketMode) {
//              case 0: // HEADER CHECK
//                  if(m_Buf[i] == 0xFF){
//                      m_checkSize++;
//                      if(m_checkSize ==4){
//                          m_PacketMode = 1;
//                      }
//                  }
//                  else{
//                      m_checkSize = 0;
//                  }
//                  break;

//              case 1: // CHAR * 4
//                  m_uPacketBuffer.buffer[m_checkSize++] = m_Buf[i];
//                  if(m_checkSize == 8) {
//                      m_PacketMode = 2;
//                  }
//                  break;

//              case 2: // DATA
//                  m_uPacketBuffer.buffer[m_checkSize++] = m_Buf[i];
//                  m_check += m_Buf[i];
//                  if(m_checkSize == sizeof(Packet_t)){
//                      if(m_check == m_uPacketBuffer.stData.check){
//                          // Packet DATA RECV CHECK //
//                          cout<<m_uPacketBuffer.stData.posx<<","<<m_uPacketBuffer.stData.posy<<endl;
//                      }
//                      m_check = 0;
//                      m_PacketMode = 0;
//                      m_checkSize = 0;
//                  }
//              }
//          }
//          // SEND PACKET //
//          m_uSendPacket.stData.mode = 3;
//          m_uSendPacket.stData.check = 0;
//          m_uSendPacket.stData.posx = 2; // cmd_vel.linear.x
//          m_uSendPacket.stData.posy = 4; // cmd_vel.angular.y
//          for(int i=8; i<sizeof(Packet_t); i++){
//            m_uSendPacket.stData.check += m_uSendPacket.buffer[i];
//          }
//            boost::asio::write(m_socket, boost::asio::buffer((char*)m_uSendPacket.buffer, sizeof(Packet_t)));
//        }
//    }

//    void SendPacket(){
//        m_uSendPacket.stData.header[0]=m_uSendPacket.stData.header[1]=m_uSendPacket.stData.header[2]=m_uSendPacket.stData.header[3]=0xFE;
//        m_uSendPacket.stData.orix =m_uSendPacket.stData.oriy = m_uSendPacket.stData.oriz = m_uSendPacket.stData.oriw = 0;
//        m_uSendPacket.stData.mode = 3;
//        m_uSendPacket.stData.check = 0;
//        m_uSendPacket.stData.posx += 1; // cmd_vel.linear.x
//        m_uSendPacket.stData.posy += 2; // cmd_vel.angular.y
//        for(int i=8; i<sizeof(Packet_t); i++){
//          m_uSendPacket.stData.check += m_uSendPacket.buffer[i];
//        }
//        boost::asio::write(m_socket, boost::asio::buffer((char*)m_uSendPacket.buffer, sizeof(Packet_t)));
//    }
//    void RecvPacket(){
//        boost::system::error_code error;
//        // RECV PACKET //
//        m_readSize = m_socket.read_some(boost::asio::buffer(m_Buf), error);
//        m_BufWriteCnt += m_readSize;


//        for(int i = 0; i<m_readSize; i++){
//            switch (m_PacketMode) {
//            case 0: // HEADER CHECK
//                if(m_Buf[i] == 0xFF){
//                    m_checkSize++;
//                    if(m_checkSize ==4){
//                        m_PacketMode = 1;
//                    }
//                }
//                else{
//                    m_checkSize = 0;
//                }
//                break;

//            case 1: // CHAR * 4
//                m_uPacketBuffer.buffer[m_checkSize++] = m_Buf[i];
//                if(m_checkSize == 8) {
//                    m_PacketMode = 2;
//                }
//                break;

//            case 2: // DATA
//                m_uPacketBuffer.buffer[m_checkSize++] = m_Buf[i];
//                m_check += m_Buf[i];
//                if(m_checkSize == sizeof(Packet_t)){
//                    if(m_check == m_uPacketBuffer.stData.check){
//                        // Packet DATA RECV CHECK //
//                        cout<<m_uPacketBuffer.stData.posx<<","<<m_uPacketBuffer.stData.posy<<endl;
//                    }
//                    m_check = 0;
//                    m_PacketMode = 0;
//                    m_checkSize = 0;
//                }
//            }
//        }
//    }

//    tcp::socket& socket(){return m_socket;}

//private:
//    tcp::socket& m_socket;
//    Packet_t m_uSendPacket;
//    Packet_t m_uPacketBuffer;
//    unsigned char m_Buf[256],m_BufWriteCnt =0, m_BufReadCnt =0;
//    unsigned char m_PacketMode = 0;
//    int m_readSize =0, m_checkSize=0;
//    unsigned char m_check=0;
//    int m_id = 0;
//    nav_msgs::Odometry m_odom;
//};


//class server
//{
//public:
//    server(boost::asio::io_service& io_service, short port)
//    : io_service_(io_service), acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
//    {
//        cout<<"server start"<<endl;


//        while(1){
//            //sever의 소켓 생성
//            cout<<"socket start"<<endl;
//            tcp::socket sock(io_service);
//            //client로부터 접속 대기
//            cout<<"acceptor start"<<endl;
//            acceptor_.accept(sock);
//            cout<<"session start"<<endl;
//            pt_session = new session(sock); // session No Return Data

//            cout<<"new session -> "<<session_num<<endl;
//            g_mpSessionIDs.insert(std::pair<int,session*>(session_num++,pt_session));
//            cout<<session_num-1<<" "<<(int64_t*)pt_session<<endl;
//            while(session_num == 3) {
//              for(int i = 1; i<session_num;i++){
//                cout<<"i"<<i<<"ses num"<<session_num<<endl;
//                DataCollect(i);
//              }
//            }
//        }
//    }
//    void DataCollect(int ses_num){
//        session* pt_ses;
//        pt_ses = g_mpSessionIDs[ses_num];
//        cout<<ses_num<<" "<<(int64_t*)pt_ses<<endl;
//        pt_ses->SendPacket();
//        pt_ses->RecvPacket();
//    }
//private:
//    boost::asio::io_service& io_service_;
//    tcp::acceptor acceptor_;
//    int session_num = 1;
//    session* pt_session;

//public:
////    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/odom",10);
//};

int main(int argc, char* argv[]){
//    ros::init(argc, argv, "boost_server");
//    ros::NodeHandle nh;
//    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/odom",10);
//    ros::Rate rate(50); //HZ

//    boost::asio::io_service io_service;
//    server s(io_service, 2222);
    return 0;
}
