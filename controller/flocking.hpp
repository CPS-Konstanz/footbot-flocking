#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "argos3_ros2_bridge/msg/light_list.hpp"
#include "argos3_ros2_bridge/msg/blob_list.hpp"

class Flocking : public rclcpp::Node {
public:
    Flocking() : Node("flocking")
    {
        // Declare and load parameters with default values
        this->declare_parameter<float>("MaxSpeed", 1.0);
        this->declare_parameter<float>("FlockingTargetDistance", 5.0);
        this->declare_parameter<float>("TargetDistance", 75.0);
        this->declare_parameter<float>("Exponent", 2.0);
        this->declare_parameter<float>("Gain", 1000.0);
        this->declare_parameter<double>("HARD_TURN", 90.0);
        this->declare_parameter<double>("SOFT_TURN", 0.0);
        this->declare_parameter<double>("NO_TURN", 10.0);

        // Retrieve parameters
        this->get_parameter("MaxSpeed", MaxSpeed);
        this->get_parameter("FlockingTargetDistance", FlockingTargetDistance);
        this->get_parameter("TargetDistance", TargetDistance);
        this->get_parameter("Exponent", Exponent);
        this->get_parameter("Gain", Gain);
        this->get_parameter("HARD_TURN", HARD_TURN);
        this->get_parameter("SOFT_TURN", SOFT_TURN);
        this->get_parameter("NO_TURN", NO_TURN);

        // subscriber
        lightSubscription_ = this->create_subscription<argos3_ros2_bridge::msg::LightList>(
            "lightList", 10, std::bind(&Flocking::light_callback, this, std::placeholders::_1));

        blobSubscription_ = this->create_subscription<argos3_ros2_bridge::msg::BlobList>(
            "blobList", 10, std::bind(&Flocking::blob_callback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Flocking::ControlStep, this));
    }

private:
    argos3_ros2_bridge::msg::LightList LightList;
    argos3_ros2_bridge::msg::BlobList OmnidirectionalCameraBlobList;
    

    float MaxSpeed = 1;
    float FlockingTargetDistance = 5;
    float TargetDistance = 75;
    float Exponent = 2;
    float Gain = 1000;
    double HARD_TURN = 90;
    double SOFT_TURN = 0;
    double NO_TURN = 10;

    void light_callback(const argos3_ros2_bridge::msg::LightList::SharedPtr msg)
    {
        LightList = *msg;
    }
    void blob_callback(const argos3_ros2_bridge::msg::BlobList::SharedPtr msg)
    {
        OmnidirectionalCameraBlobList = *msg;
    }

    std::pair<float, float> VectorToLight();

    std::pair<float, float> FlockingVector();

    void SetWheelSpeedsFromVector(std::pair<float, float> c_heading);

    float GeneralizedLennardJones(float f_distance);

    void ControlStep();

    rclcpp::Subscription<argos3_ros2_bridge::msg::LightList>::SharedPtr lightSubscription_;
    rclcpp::Subscription<argos3_ros2_bridge::msg::BlobList>::SharedPtr blobSubscription_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


