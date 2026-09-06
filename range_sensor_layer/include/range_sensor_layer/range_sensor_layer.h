/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 David V. Lu!!
 *  Copyright (c) 2020, Bytes Robotics
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

#ifndef RANGE_SENSOR_LAYER_RANGE_SENSOR_LAYER_H_
#define RANGE_SENSOR_LAYER_RANGE_SENSOR_LAYER_H_
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
#include <sensor_msgs/Range.h>
#include <range_sensor_layer/RangeSensorLayerConfig.h>
#include <dynamic_reconfigure/server.h>

namespace range_sensor_layer
{

/**
 * @class RangeSensorLayer
 * @brief Takes in IR/Sonar/similar point measurement sensors and populates in costmap
 */
class RangeSensorLayer : public costmap_2d::CostmapLayer
{
public:
  enum InputSensorType
  {
    VARIABLE,
    FIXED,
    ALL
  };

  /**
   * @brief A constructor
   */
  RangeSensorLayer();
  ~RangeSensorLayer();

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
  void removeOutdatedReadings();

  void reconfigureCB(range_sensor_layer::RangeSensorLayerConfig &config, uint32_t level);

  /**
   * @brief Handle an incoming Range message to populate into costmap
   */
  void bufferIncomingRangeMsg(const sensor_msgs::RangeConstPtr& range_message);

  /**
   * @brief Processes all sensors into the costmap buffered from callbacks
   */
  void updateCostmap();

  /**
   * @brief Update the actual costmap with the values processed
   */
  void updateCostmap(sensor_msgs::Range& range_message, bool clear_sensor_cone);

  /**
   * @brief Process general incoming range sensor data. If min=max ranges,
   * fixed processor callback is used, else uses variable callback
   */
  void processRangeMsg(sensor_msgs::Range& range_message);

  /**
    * @brief Process fixed range incoming range sensor data
    */
  void processFixedRangeMsg(sensor_msgs::Range& range_message);

  /**
    * @brief Process variable range incoming range sensor data
    */
  void processVariableRangeMsg(sensor_msgs::Range& range_message);

  /**
   * @brief Reset the angle min/max x, and min/max y values
   */
  void resetRange();

  /**
   * @brief Get the gamma value for an angle, theta
   * @param exp gamma exponent (mark or clear profile, see gamma_exp /
   *        gamma_exp_clear)
   */
  inline double gamma(double theta, double exp);

  /**
   * @brief Get the delta value for an angle, phi
   */
  inline double delta(double phi);

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

  /**
   * @brief Find probability value of a cost
   */
  inline double to_prob(unsigned char c)
  {
    return static_cast<double>(c) / costmap_2d::LETHAL_OBSTACLE;
  }

  /**
   * @brief Find cost value of a probability
   */
  inline unsigned char to_cost(double p)
  {
    return static_cast<unsigned char>(p * costmap_2d::LETHAL_OBSTACLE);
  }

  /**
   * @brief Fixed world-lattice key for a costmap cell, for marked_point_history_
   *
   * The key is the cell's index in the fixed world lattice. Costmap2D keeps
   * the rolling origin on that lattice (updateOrigin snaps it to a multiple
   * of resolution_), so the lattice does not move with the robot and the key
   * survives every origin update. Plain cell indices break there (the
   * renumbering after an origin move points at the wrong world cell), and
   * quantizing the cell's *center* world position to the lattice does not
   * round-trip through worldToMap: the center is a half-cell past the cell
   * edge, so the double rounding (llround at mark time, int(v+0.5) at clear
   * time) resolves some keys to the neighboring cell — depending on the
   * float value of the current origin — and the marked cell itself is never
   * freed. Integer-only arithmetic makes key -> cell exact for any origin.
   */
  inline std::pair<int, int> worldCellKey(unsigned int x, unsigned int y) const
  {
    return std::make_pair(static_cast<int>(x) + static_cast<int>(std::llround(origin_x_ / resolution_)),
                          static_cast<int>(y) + static_cast<int>(std::llround(origin_y_ / resolution_)));
  }

  std::function<void(sensor_msgs::Range & range_message)> processRangeMessageFunc_;
  std::mutex range_message_mutex_;
  std::list<sensor_msgs::Range> range_msgs_buffer_;
  // Fixed world-lattice index of a marked cell (see worldCellKey) -> time of
  // its last mark. Keyed by world lattice index rather than cell index: in a
  // rolling costmap the origin moves with the robot and renumbers every
  // cell, so cell-index keys would point at the wrong world cells after a
  // move and the decay could never clear what it marked.
  std::map<std::pair<int, int>, double> marked_point_history_;

  double max_angle_, phi_v_;
  // Exponent in gamma(theta) = 1 - (theta/max_angle_)^exp. Two profiles:
  // gamma_exp for MARKING (obstacle band + ramp, center-peaked trust) and
  // gamma_exp_clear for CLEARING (free space ahead of the reading, flatter
  // so blind/edge cells get a real clear instead of lingering).
  double gamma_exp_;
  double gamma_exp_clear_;
  double delta_slope_;  // tanh slope in delta(phi) = 1 - (1+tanh(delta_slope_*(phi-phi_v_)))/2
  double inflate_cone_;
  std::string global_frame_;

  double clear_threshold_, mark_threshold_;
  // sensor-model fill tuning: absolute min full obstacle thickness (m), and the
  // band-center offset as a fraction of the thickness (0 = at reading,
  // +/-0.5 = shift by half the thickness toward/away from the sensor).
  double min_obstacle_thickness_;
  double obstacle_center_offset_;
  bool clear_on_max_reading_;
  bool was_reset_;

  double transform_tolerance_;
  double no_readings_timeout_;
  ros::Time last_reading_time_;
  unsigned int buffered_readings_;
  std::vector<ros::Subscriber> range_subs_;
  double min_x_, min_y_, max_x_, max_y_;

  bool use_decay_;
  bool debug_publisher_;
  double pixel_decay_;
  // May updateCosts() lower master cells (strong clear -> FREE)?
  // false = legacy one-way merge (fill unknown + raise only) — safe when the
  // layer shares its costmap with static/contact layers; true = authoritative
  // merge, required when this layer is the SOLE layer of its costmap so that
  // decay can actually clear cells nothing else rewrites (uss_costmap).
  bool allow_clearing_;
  
  dynamic_reconfigure::Server<range_sensor_layer::RangeSensorLayerConfig> *dsrv_;
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
}  // namespace range_sensor_layer
#endif  // RANGE_SENSOR_LAYER_RANGE_SENSOR_LAYER_H
