// File: fish_dvl_plugin/src/dvl_plugin.cpp

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/PhysicsIface.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo {

  class CustomDVLPlugin : public SensorPlugin
  {
  public:
    CustomDVLPlugin() : SensorPlugin(), previous_range_(0.0) {}
    virtual ~CustomDVLPlugin() {}

    /// \brief Load the sensor plugin.
    /// \param _sensor Pointer to the sensor that loaded this plugin.
    /// \param _sdf SDF element containing plugin parameters.
    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override
    {
      // Ensure ROS is running
      if (!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }
      // Create a ROS 2 node
      node_ = rclcpp::Node::make_shared("custom_dvl_plugin");
      publisher_ = node_->create_publisher<std_msgs::msg::Float64>("dvl/velocity", 10);

      // Cast sensor pointer to RaySensor (we assume our sensor element is of type "ray")
      this->raySensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
      if (!this->raySensor_)
      {
        gzerr << "CustomDVLPlugin requires a RaySensor.\n";
        return;
      }

      // Store the initial update time and initial range reading
      last_update_time_ = this->raySensor_->LastUpdateTime();
      previous_range_ = this->raySensor_->Range(0);

      // Connect the update event
      updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CustomDVLPlugin::OnUpdate, this));
    }

    /// \brief Called on every simulation update.
    void OnUpdate()
    {
      // Get current time and compute elapsed time
      common::Time current_time = this->raySensor_->LastUpdateTime();
      double dt = (current_time - last_update_time_).Double();
      if (dt <= 0.0)
      {
        return;
      }
      last_update_time_ = current_time;

      // For demonstration, use the first ray sensor's reading
      double current_range = this->raySensor_->Range(0);

      // Compute a simple velocity estimate (difference in range over time)
      double velocity = (previous_range_ - current_range) / dt;
      previous_range_ = current_range;

      // Publish the velocity
      std_msgs::msg::Float64 msg;
      msg.data = velocity;
      publisher_->publish(msg);

      // Process ROS callbacks (non-blocking)
      rclcpp::spin_some(node_);
    }

  private:
    sensors::RaySensorPtr raySensor_;
    event::ConnectionPtr updateConnection_;
    common::Time last_update_time_;
    double previous_range_;

    // ROS 2 components
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  };

  // Register this plugin with Gazebo
  GZ_REGISTER_SENSOR_PLUGIN(CustomDVLPlugin)
}
