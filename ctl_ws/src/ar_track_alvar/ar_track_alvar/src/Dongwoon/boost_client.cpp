//#include<cstdlib>
//#include<iostream>
//#include<boost/asio.hpp>

//#include <ros/ros.h>
//#include "nav_msgs/Odometry.h"
//#include "datatype.h"

//using boost::asio::ip::tcp;
//using namespace std;
//class client;

//Packet_t g_uRecv_odom;
//geometry_msgs::Twist g_cmdvel;

//class client
//{
//public:
//    client(boost::asio::io_service& io_service, const string& host, const string& port)
//        : m_socket(io_service)
//    {
//        tcp::resolver resolver(io_service);
//        cout<<"connect start"<<endl;
//        boost::asio::connect(m_socket, resolver.resolve({host, port}));
//    }
//    void SendPacket(){
//        boost::system::error_code error;
//        // SEND PACKET //
//        m_uSendPacket.stData.header[0] = m_uSendPacket.stData.header[1] = m_uSendPacket.stData.header[2] = m_uSendPacket.stData.header[3] = 0xFF;
//        m_uSendPacket.stData.mode = 2;
//        m_uSendPacket.stData.check = 0;
//        m_uSendPacket.stData.posx = g_uRecv_odom.stData.posx;
//        m_uSendPacket.stData.posy = g_uRecv_odom.stData.posy;
//        m_uSendPacket.stData.orix = g_uRecv_odom.stData.orix;
//        m_uSendPacket.stData.oriy = g_uRecv_odom.stData.oriy;
//        m_uSendPacket.stData.oriz = g_uRecv_odom.stData.oriz;
//        m_uSendPacket.stData.oriw = g_uRecv_odom.stData.oriw;
//        for(int i = 8; i < sizeof(Packet_t); i++){
//            m_uSendPacket.stData.check += m_uSendPacket.buffer[i];
//        }
//        try {
//            boost::asio::write(m_socket, boost::asio::buffer((char*)m_uSendPacket.buffer, sizeof(Packet_t)),error); //OR send()
//        } catch (boost::system::error_code& error) {
//            cout<<"hello error!"<<error.message()<<endl;
//        }
//    }
//    void RecvPacket(){
//        // RECV PACKET //
//        boost::system::error_code error;

//        try {
//            m_readSize = boost::asio::read(m_socket, boost::asio::buffer(m_RecvBuf, sizeof(m_RecvBuf)),error);
//            for(int i = 0; i<m_readSize; i++){
//                switch (m_mode) {
//                case 0: // HEADER CHECK
//                    if(m_RecvBuf[i] == 0xFE){
//                        m_checkSize++;
//                        if(m_checkSize ==4){
//                            m_mode = 1;
//                        }
//                    }
//                    else{
//                        m_checkSize = 0;
//                    }
//                    break;

//                case 1: // CHAR * 4
//                    m_uRecvPacket.buffer[m_checkSize++] = m_RecvBuf[i];
//                    if(m_checkSize == 8) {
//                        m_mode = 2;
//                    }
//                    break;

//                case 2: // DATA
//                    m_uRecvPacket.buffer[m_checkSize++] = m_RecvBuf[i];
//                    m_check += m_RecvBuf[i];
//                    if(m_checkSize == sizeof(Packet_t)){
//                        if(m_check == m_uRecvPacket.stData.check){
//                            // Packet DATA RECV CHECK //
//                            cout<<m_uRecvPacket.stData.posx<<","<<m_uRecvPacket.stData.posy<<endl;
//                            g_cmdvel.linear.x = m_uRecvPacket.stData.posx; // velocity
//                            g_cmdvel.angular.z = m_uRecvPacket.stData.posy; // angular velocity
//                        }
//                        m_check = 0;
//                        m_mode = 0;
//                        m_checkSize = 0;
//                    }
//                }
//            }
//        } catch (boost::system::error_code& error) {
//            cout<<"hello error!"<<error.message()<<endl;
//        }
//    }

//private:
//    unsigned char m_RecvBuf[32];
//    tcp::socket m_socket;
//    Packet_t m_uSendPacket;
//    Packet_t m_uRecvPacket;
//    int m_mode=0, m_readSize = 0, m_checkSize=0;
//    unsigned char m_check=0;
//};

//void tb2_trans(nav_msgs::Odometry odom) {
//    static boost::asio::io_service io_service;
//    static client c(io_service, "192.168.0.174", "2222");

//    g_uRecv_odom.stData.posx = odom.pose.pose.position.x;
//    g_uRecv_odom.stData.posy = odom.pose.pose.position.y;
//    g_uRecv_odom.stData.orix = odom.pose.pose.orientation.x;
//    g_uRecv_odom.stData.oriy = odom.pose.pose.orientation.y;
//    g_uRecv_odom.stData.oriz = odom.pose.pose.orientation.z;
//    g_uRecv_odom.stData.oriw = odom.pose.pose.orientation.w;

//    c.RecvPacket();
//    c.SendPacket();
//}

int main(int argc, char* argv[]){
//    ros::init(argc, argv, "boost_client");
//    ros::NodeHandle nh;
//    ros::Rate rate(50); //HZ

//    ros::Subscriber sub = nh.subscribe("/tb3_2/odom", 1, tb2_trans);
//    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/tb3_2/cmd_vel",10);

//    while(ros::ok()){
//      pub.publish(g_cmdvel);
//      ros::spinOnce();
//      rate.sleep();
//    }
    return 0;
}
