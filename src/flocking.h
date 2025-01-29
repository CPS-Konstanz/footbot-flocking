/*
 * flocking.h
 *
 * Created on: 16 Jul 2024
 * Author: Sindiso Mkhatshwa
 * Email: sindiso.mkhatshwa@uni-konstanz.de
 */
#ifndef FOOTBOT_FLOCKING_H
#define FOOTBOT_FLOCKING_H

/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

/**
 * ROS2 Imports
 */
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "argos3_ros2_bridge/msg/led.hpp"
#include "argos3_ros2_bridge/msg/light.hpp"
#include "argos3_ros2_bridge/msg/light_list.hpp"
#include "argos3_ros2_bridge/msg/blob.hpp"
#include "argos3_ros2_bridge/msg/blob_list.hpp"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;
using namespace argos3_ros2_bridge::msg;
using namespace geometry_msgs::msg;

class Flocking{

public:

   /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><wheel_turning>
    * section.
    */
   struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

   /*
    * The following variables are used as parameters for
    * flocking interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><flocking>
    * section.
    */
   struct SFlockingInteractionParams {
      /* Target robot-robot distance in cm */
      Real TargetDistance;
      /* Gain of the Lennard-Jones potential */
      Real Gain;
      /* Exponent of the Lennard-Jones potential */
      Real Exponent;

      void Init(TConfigurationNode& t_node);
      Real GeneralizedLennardJones(Real f_distance);
   };

public:

   /* Class constructor. */
   Flocking(std::shared_ptr<rclcpp::Node> node);

   /* Class destructor. */
   virtual ~Flocking() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML file
    * in the <controllers><footbot_flocking_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Destroy() {}

protected:

   /*
    * Calculates the vector to the closest light.
    */
   virtual CVector2 VectorToLight();

   /*
    * Calculates the flocking interaction vector.
    */
   virtual CVector2 FlockingVector();

   /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
   void SetWheelSpeedsFromVector(const CVector2& c_heading);

private:

    /**************************************
	 * Create topic subscribers
	 *************************************/
	// Light list subscriber
	rclcpp::Subscription<LightList>::SharedPtr lightListSubscriber_;
    // colored-blob-omnidirectional-camera sensor subscriber
	rclcpp::Subscription<BlobList>::SharedPtr blobSubscriber_;

    /**************************************
	 * Create topic publishers
	 **************************************/
    // We use ROS2's Twist to publish velocity commands
	rclcpp::Publisher<Twist>::SharedPtr cmdVelPublisher_;
	// We use a Led interface to publish the desired LED color
	rclcpp::Publisher<Led>::SharedPtr cmdLedPublisher_;


    /* The turning parameters. */
    SWheelTurningParams m_sWheelTurningParams;
    /* The flocking interaction parameters. */
    SFlockingInteractionParams m_sFlockingParams;
};

#endif
