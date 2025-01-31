#include "flocking.hpp"

std::pair<float, float> Flocking::VectorToLight()
{
    float total_x = 0.0f;
    float total_y = 0.0f;

    for (const auto &light : LightList.lights)
    {
        float value = light.value;
        float angle = light.angle;

        total_x += value * std::cos(angle);
        total_y += value * std::sin(angle);
    }

    float magnitude = std::sqrt(total_x * total_x + total_y * total_y);

    if (magnitude > 0.0f)
    {
        total_x /= magnitude;
        total_y /= magnitude;
    }
    else
    {
        total_x = 0.0f;
        total_y = 0.0f;
    }

    return {total_x, total_y};
}

float Flocking::GeneralizedLennardJones(float f_distance)
{
    if (f_distance < 1e-6f)
    {
        return 0.0f; 
    }

    float fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
    return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

std::pair<float, float> Flocking::FlockingVector()
{
    float total_x = 0.0f;
    float total_y = 0.0f;
    int num = 0;

    for (const auto &blob : OmnidirectionalCameraBlobList.blobs)
    {

        if (blob.color == "RED" && blob.distance < 1.8 * TargetDistance)
        {
            float value = blob.distance;
            float angle = blob.angle;

             total_x += value * std::cos(angle);
             total_y += value * std::sin(angle);

            num++;
        }
    }

    if (!total_x == 0 && !total_y == 0)
    {
        total_x = GeneralizedLennardJones(total_x) / num;
        total_y = GeneralizedLennardJones(total_y) / num;
    }
    return {total_x, total_y};
}

void Flocking::SetWheelSpeedsFromVector(std::pair<float, float> c_heading)
{
    geometry_msgs::msg::Twist cmd_vel;

    float total_x = c_heading.first;
    float total_y = c_heading.second;

    float target_angle = atan2(total_y, total_x);

    float linear_velocity = sqrt(total_x * total_x + total_y * total_y);

    float angular_velocity = target_angle * 1.0; 

    if (angular_velocity > SOFT_TURN)
    {
        linear_velocity = 0.0;
    }

    cmd_vel.linear.x = linear_velocity;   
    cmd_vel.angular.z = angular_velocity; 

    cmd_vel_publisher_->publish(cmd_vel);
}

void Flocking::ControlStep()
{
    std::pair<float, float> light = VectorToLight();
    std::pair<float, float> flocking = FlockingVector();
    std::pair<float, float> vector;
    vector.first = light.first + flocking.first;
    vector.second = light.second + flocking.second;

    SetWheelSpeedsFromVector(vector);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Flocking>());
    rclcpp::shutdown();
    return 0;
}