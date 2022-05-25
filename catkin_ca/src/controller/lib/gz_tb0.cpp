#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <brl_msgs/brl_msgs.h>
using namespace std;

//Global Variables
brl_msgs::brl_msgs Turtle_odom0;
geometry_msgs::Twist Turtle_twist[3];
static double des_position = 0.2;
static double err_position = 0;
static double preerr_position = 0;

void Robot0PoseSub(const brl_msgs::brl_msgs::ConstPtr &odom){
    Turtle_odom0.posx = odom->posx;
    Turtle_odom0.posy = odom->posy;
    Turtle_odom0.yaw = odom->yaw;
}

namespace gazebo
{
  class ModelPush0 : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      RobotPose = nh.subscribe("/tb_0/odom",100,Robot0PoseSub);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ModelPush0::OnUpdate, this));
    }


    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      math::Pose cur_pose= this->model->GetWorldPose();

      Turtle_pose.Set(Turtle_odom0.posx,Turtle_odom0.posy,0,0,0,Turtle_odom0.yaw);

      math::Vector3 linVel, angVel;
      linVel.x = 0;
      linVel.y = 0;
      linVel.z = 0;
      angVel.x = 0;
      angVel.y = 0;
      angVel.z = 0;
      this->model->SetWorldPose(Turtle_pose);
      this->model->SetWorldTwist(linVel, angVel);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private:
      math::Pose Turtle_pose;

    public:
      ros::NodeHandle nh;
      ros::Subscriber RobotPose;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush0)
}
