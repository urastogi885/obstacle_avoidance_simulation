/**
 * BSD 3-Clause License
 *
 * @copyright (c) 2019, Umang Rastogi
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file main.cpp
 * @author Umang Rastogi
 * @brief Library source file to implement obstacle avoidance on a turtlebot
 */

#include "obstacle_avoidance.h"

ObstacleAvoidance::ObstacleAvoidance() {
	ROS_INFO_STREAM("Setting up the robot config for obstacle avoidance...");
	/// Initialize previous with the current value of velocities
	prevLinearVelocity = linearVelocity;
	prevAnguarVelocity = anguarVelocity;
	/// Initialize obstacle detected value with false
	obstacleDetected = false;
	/// Publish the velocities to the bot on the navigation topic
  	publishVelocity = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
  	/// Subscribe for data from the laser sensor on the scan topic
  	subscibeSensor = nh.subscribe<sensor_msgs::LaserScan>("/scan", 500, \
  				&ObstacleAvoidance::sensorCallback, this);
  	ROS_INFO_STREAM("Set up complete");
}

ObstacleAvoidance::~ObstacleAvoidance() {
	/// Reset the bot before deconstructing
	resetBot();
}

bool ObstacleAvoidance::checkObstacle() {
	/// Check if obstacle is ahead
	if(getObstacleDetected()) {
		ROS_WARN_STREAM("Obstacle ahead!");
		return true;
	}

	return false;
}

void ObstacleAvoidance::startBot() {
  // Set the publishing rate
  ros::Rate loop_rate(rate);
  while(ros::ok()) {
    if (checkObstacle()) {
	   	/// Start turning the bot to avoid obstackes
	   	velocities.linear.x = 0.0;
      	velocities.angular.z = anguarVelocity;
      	/// Check if velocities have changed
      	checkVelocityChanged();
    } else {
      	/// Start moving the bot once obstacle is avoided
      	velocities.angular.z = 0.0;
      	velocities.linear.x = linearVelocity;
      	/// Check if velocities have changed
      	checkVelocityChanged();
    }

    /// Publish the velocities
    publishVelocity.publish(velocities);
    //. Handle callbacks
    ros::spinOnce();
    /// Make the system sleep to maintain loop rate
    loop_rate.sleep();
  }
}

void ObstacleAvoidance::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorData) {
  /// Read sensor data to get obstacle distances with respect to the bot
  for (const float &range : sensorData->ranges) {
   	if (range < distanceThreshold) {
  		setObstacleDetected(true);
    	return;
    }
  }

  setObstacleDetected(false);
}

void ObstacleAvoidance::resetBot() {
	ROS_INFO_STREAM("Resetting the robot config...");
	/// Reset linear velocities of the both bot
	velocities.linear.x = 0.0;
  	velocities.linear.y = 0.0;
  	velocities.linear.z = 0.0;
  	/// Reset angular velocities of the both bot
  	velocities.angular.x = 0.0;
  	velocities.angular.y = 0.0;
  	velocities.angular.z = 0.0;
  	/// Publish reset velocities
  	publishVelocity.publish(velocities);
  	ROS_INFO_STREAM("Reset complete");
}

bool ObstacleAvoidance::checkVelocityChanged() {
	/// Linear and angular change simulataneously
	/// Check if both the velocities have changed
	if (velocities.linear.x != prevLinearVelocity and \
      		velocities.angular.z != prevAnguarVelocity) {
      		ROS_DEBUG_STREAM("Velocity of the bot changed");
      	 	/// Update previous velocities
      		velocities.linear.x = prevLinearVelocity;
      		velocities.angular.z = prevAnguarVelocity;
      		return true;
     	}

    return false;
}
