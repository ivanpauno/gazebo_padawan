#include <cmath>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>


namespace gazebo
{

class ModelVirtualWall : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->model = _parent;
    world = _parent->GetWorld();
    create_2 = world->ModelByName("irobot_create2");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelVirtualWall::OnUpdate, this));
    this->rosnode_ = new ros::NodeHandle();
    this->pub_ = this->rosnode_->advertise<std_msgs::Bool>(
      "virtual_wall", 1);
  }

  // Called by the world update start event
  void OnUpdate()
  {
    ignition::math::Vector3d laser_dir(1., 0., 0.);
    ignition::math::Pose3d laser_pose(0., 0., 0.014, 0., 0., 0.);
    ignition::math::Pose3d sensor_pose(0.137, 0., 0.045, 0., 0., 0.);

    const ignition::math::Pose3d & my_pose = model->WorldPose();
    laser_dir = my_pose.Rot().RotateVector(laser_dir);
    laser_pose = my_pose + laser_pose;

    const ignition::math::Pose3d & robot_pose = create_2->WorldPose();
    sensor_pose = robot_pose + sensor_pose;

    // distance from P to straight line X=X0+lambda*V is:
    // sin(phi)*abs(P-X0) where cos(phi)=(P-X0)*V/abs(P-X0)
    const auto & error = (sensor_pose.Pos() - laser_pose.Pos());
    double phi = std::acos(error.Dot(laser_dir) / error.Length());
    double d = error.Length() * std::sin(phi);
    double lambda = (sensor_pose.Pos().X() - laser_pose.Pos().X()) / laser_dir.X();

    std_msgs::Bool x;
    x.data = d < 0.1 && lambda > 0;
    this->pub_.publish(x);

    printf("laser_dir: %f %f %f\n", laser_dir.X(), laser_dir.Y(), laser_dir.Z());
    printf("laser_pose: %f %f %f\n", laser_pose.Pos().X(), laser_pose.Pos().Y(), laser_pose.Pos().Z());
    printf("sensor_pose: %f %f %f\n", sensor_pose.Pos().X(), sensor_pose.Pos().Y(), sensor_pose.Pos().Z());
    printf("%s\n", (lambda > 0) ? "front" : "back");
    printf("d = %f\n", d);

    // double lambda_2 = (robot_pose.Pos().Y() - sensor_pose.Pos().Y()) / laser_dir.Y();
    // double lambda_3 = (robot_pose.Pos().Z() - sensor_pose.Pos().Z()) / laser_dir.Z();

    // if (error_in_range(lambda_1, lambda_2) && error_in_range(lambda_3, lambda_2) && error_in_range(lambda_1, lambda_3)) {
    //   // publish mesage
    // }
  }

private:

  // bool
  // error_in_range(double x, double y)
  // {
  //   return std::abs(x - y) < 0.1;
  // }

  physics::ModelPtr model;
  physics::ModelPtr create_2;
  physics::WorldPtr world;
  event::ConnectionPtr updateConnection;

  ros::NodeHandle* rosnode_;
  ros::Publisher pub_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelVirtualWall)

}
