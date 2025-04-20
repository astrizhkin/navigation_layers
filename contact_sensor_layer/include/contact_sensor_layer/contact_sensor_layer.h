/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025 Andrey Strizhkin
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CONTACT_SENSOR_LAYER_CONTACT_SENSOR_LAYER_H_
#define CONTACT_SENSOR_LAYER_CONTACT_SENSOR_LAYER_H_
#include <list>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <mutex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <contact_sensor_layer/Contact.h>
#include <contact_sensor_layer/ContactSensorLayerConfig.h>
#include <dynamic_reconfigure/server.h>

namespace contact_sensor_layer
{

/**
 * @class ContactSensorLayer
 * @brief Takes contatac signal populates in costmap
 */
class ContactSensorLayer : public costmap_2d::CostmapLayer
{
public:

  /**
   * @brief A constructor
   */
  ContactSensorLayer();
  ~ContactSensorLayer();

  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(
    costmap_2d::Costmap2D& master_grid, int min_i, 
    int min_j, int max_i, int max_j);

  /**
   * @brief Reset this costmap
   */
  virtual void reset();

  /**
   * @brief Deactivate the layer
   */
  virtual void deactivate();

  /**
   * @brief Activate the layer
   */
  virtual void activate();

private:
  void reconfigureCB(contact_sensor_layer::ContactSensorLayerConfig &config, uint32_t level);

  /**
   * @brief Handle an incoming Range message to populate into costmap
   */
  void bufferIncomingContactMsg(const contact_sensor_layer::ContactConstPtr& contact_message);

  /**
   * @brief Processes all sensors into the costmap buffered from callbacks
   */
  void updateCostmap();

  /**
   * @brief Update the actual costmap with the values processed
   */
  void updateCostmap(contact_sensor_layer::Contact& contact_message);

  /**
   * @brief Process general incoming range sensor data. If min=max ranges,
   * fixed processor callback is used, else uses variable callback
   */
  void processContactMsg(contact_sensor_layer::Contact& contact_message);

  /**
   * @brief Reset the angle min/max x, and min/max y values
   */
  void resetRange();

  /**
   * @brief Apply the sensor model of the layer for range sensors
   */
  inline double sensor_model(double r, double phi, double theta);

  /**
   * @brief Get angles
   */
  inline void get_deltas(double angle, double * dx, double * dy);

  /**
   * @brief Update the cost in a cell with information
   */
  inline void update_cell(
    double ox, double oy, double ot,
    double r, double nx, double ny, bool clear);

  std::function<void(contact_sensor_layer::Contact & contact_message)> processContactMessageFunc_;
  std::mutex contact_message_mutex_;
  std::list<contact_sensor_layer::Contact> contact_msgs_buffer_;

  double obstacle_radius_,obstacle_forward_offset_;
  std::string global_frame_;

  bool was_reset_;

  double transform_tolerance_;
  double no_readings_timeout_;
  ros::Time last_reading_time_;
  unsigned int buffered_readings_;
  std::vector<ros::Subscriber> contact_subs_;
  double min_x_, min_y_, max_x_, max_y_;

  bool debug_publisher_;
  
  dynamic_reconfigure::Server<contact_sensor_layer::ContactSensorLayerConfig> *dsrv_;
  costmap_2d::Costmap2DPublisher* publisher_;

  /**
   * @brief Find the area of 3 points of a triangle
   */
  float area(int x1, int y1, int x2, int y2, int x3, int y3)
  {
    return fabs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
  }

  /**
   * @brief Find the cross product of 3 vectors, A,B,C
   */
  int orient2d(int Ax, int Ay, int Bx, int By, int Cx, int Cy)
  {
    return (Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax);
  }
};
}  // namespace contact_sensor_layer
#endif  // CONTACT_SENSOR_LAYER_CONTACT_SENSOR_LAYER_H
