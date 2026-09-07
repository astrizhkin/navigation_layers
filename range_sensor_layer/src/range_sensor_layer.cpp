// Copyright 2018 David V. Lu!!
#include <range_sensor_layer/range_sensor_layer.h>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <algorithm>
#include <cmath>
#include <list>
#include <limits>
#include <map>
#include <string>
#include <utility>

PLUGINLIB_EXPORT_CLASS(range_sensor_layer::RangeSensorLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;

namespace range_sensor_layer
{

RangeSensorLayer::RangeSensorLayer() {}

RangeSensorLayer::~RangeSensorLayer() {
  if (publisher_ != NULL) {
    delete publisher_;
  }

  delete dsrv_;
}

void RangeSensorLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  was_reset_ = false;
  buffered_readings_ = 0;
  last_reading_time_ = ros::Time::now();
  default_value_ = to_cost(0.5);

  matchSize();
  resetRange();

  // Default topic names list contains a single topic: /sonar
  // We use the XmlRpcValue constructor that takes a XML string and reading start offset
  const char* xml = "<value><array><data><value>/sonar</value></data></array></value>";
  int zero_offset = 0;
  std::string topics_ns;
  XmlRpc::XmlRpcValue topic_names(xml, &zero_offset);

  nh.param("ns", topics_ns, std::string());
  nh.param("topics", topic_names, topic_names);

  InputSensorType input_sensor_type = ALL;
  std::string sensor_type_name;
  nh.param("input_sensor_type", sensor_type_name, std::string("ALL"));

  nh.param("debug_publisher", debug_publisher_, false);
  nh.param("allow_clearing", allow_clearing_, false);
  nh.param("transform_tolerance_", transform_tolerance_, 0.3);

  boost::to_upper(sensor_type_name);
  ROS_INFO("[range_sensor_layer] %s: %s as input_sensor_type given", name_.c_str(), sensor_type_name.c_str());

  if (sensor_type_name == "VARIABLE")
    input_sensor_type = VARIABLE;
  else if (sensor_type_name == "FIXED")
    input_sensor_type = FIXED;
  else if (sensor_type_name == "ALL")
    input_sensor_type = ALL;
  else
  {
    ROS_ERROR("[range_sensor_layer] %s: Invalid input sensor type: %s", name_.c_str(), sensor_type_name.c_str());
  }

  // Validate topic names list: it must be a (normally non-empty) list of strings
  if ((topic_names.valid() == false) || (topic_names.getType() != XmlRpc::XmlRpcValue::TypeArray))
  {
    ROS_ERROR("[range_sensor_layer] Invalid topic names list: it must be a non-empty list of strings");
    return;
  }

  if (topic_names.size() < 1)
  {
    // This could be an error, but I keep it as it can be useful for debug
    ROS_WARN("[range_sensor_layer] Empty topic names list: range sensor layer will have no effect on costmap");
  }

  // Traverse the topic names list subscribing to all of them with the same callback method
  for (int i = 0; i < topic_names.size(); i++)
  {
    if (topic_names[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_WARN("[range_sensor_layer] Invalid topic names list: element %d is not a string, so it will be ignored", i);
    }
    else
    {
      std::string topic_name(topics_ns);
      if ((topic_name.size() > 0) && (topic_name.at(topic_name.size() - 1) != '/'))
        topic_name += "/";
      topic_name += static_cast<std::string>(topic_names[i]);

      if (input_sensor_type == VARIABLE)
        processRangeMessageFunc_ = boost::bind(&RangeSensorLayer::processVariableRangeMsg, this, _1);
      else if (input_sensor_type == FIXED)
        processRangeMessageFunc_ = boost::bind(&RangeSensorLayer::processFixedRangeMsg, this, _1);
      else if (input_sensor_type == ALL)
        processRangeMessageFunc_ = boost::bind(&RangeSensorLayer::processRangeMsg, this, _1);
      else
      {
        ROS_ERROR(
          "[range_sensor_layer] %s: Invalid input sensor type: %s. Did you make a new type and forgot to choose the subscriber for it?",
          name_.c_str(), sensor_type_name.c_str());
      }

      range_subs_.push_back(nh.subscribe(topic_name, 100, &RangeSensorLayer::bufferIncomingRangeMsg, this));

      ROS_INFO("[range_sensor_layer] subscribed to topic %s", range_subs_.back().getTopic().c_str());
    }
  }

  dsrv_ = new dynamic_reconfigure::Server<range_sensor_layer::RangeSensorLayerConfig>(nh);
  dynamic_reconfigure::Server<range_sensor_layer::RangeSensorLayerConfig>::CallbackType cb =
    boost::bind(&RangeSensorLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  global_frame_ = layered_costmap_->getGlobalFrameID();

  if (debug_publisher_) {
    publisher_ = new costmap_2d::Costmap2DPublisher(&nh, this, global_frame_, "costmap",true);
  }
}


double RangeSensorLayer::gamma(double theta, double exp)
{
  if (fabs(theta) > max_angle_) {
    return 0.0;
  } else {
    // exp=2 (stock) is zero at the cone edge; a lower exponent keeps the
    // weight meaningful near the edge. fabs matters: theta is signed here,
    // and pow(negative, non-integer) is NaN (odd integer exponents return
    // a negative base -> gamma > 1 -> lambda_clear > 1 -> NEGATIVE "clear"
    // sensor values -> the to_cost unsigned-char cast wraps to ~250 and
    // "clears" the cell to LETHAL — the 2026-09-07 clear-gamma-3.0 phantom
    // wall).
    return 1 - pow(fabs(theta) / max_angle_, exp);
  }
}

double RangeSensorLayer::delta(double phi)
{
  return 1 - (1 + tanh(delta_slope_ * (phi - phi_v_))) / 2;
}

void RangeSensorLayer::get_deltas(double angle, double * dx, double * dy)
{
  double ta = tan(angle);
  if (ta == 0) {
    *dx = 0;
  } else {
    *dx = resolution_ / ta;
  }

  *dx = copysign(*dx, cos(angle));
  *dy = copysign(resolution_, sin(angle));
}

double RangeSensorLayer::sensor_model(double r, double phi, double theta)
{
  // Two trust profiles: the MARK (obstacle band + ramp) uses gamma_exp
  // (center-peaked — trust the boresight, less on the angles), the CLEAR
  // (free space ahead of the reading) uses gamma_exp_clear (flat by
  // default, so blind/edge cells that the mark profile cannot see through
  // are still cleared instead of lingering until pixel_decay).
  double dphi = delta(phi);
  double lbda_mark = dphi * gamma(theta, gamma_exp_);
  double lbda_clear = dphi * gamma(theta, gamma_exp_clear_);

  // Obstacle band thickness: scales with the reading, floored at
  // min_obstacle_thickness (m) — the absolute minimum thickness.
  double full_thickness = 2.0 * resolution_ * r;
  full_thickness = std::max(full_thickness, min_obstacle_thickness_);
  double half_thickness = 0.5 * full_thickness;

  // Band center. obstacle_center_offset is a fraction of the full thickness:
  // 0 = at the reading (model unchanged), -0.5 = shift half the thickness
  // toward the sensor, +0.5 = shift half the thickness away from the sensor.
  double center = r + obstacle_center_offset_ * full_thickness;

  if (phi >= 0.0 && phi < center - 2 * half_thickness) {
    // Free space ahead of the reading: clearing only, clear profile.
    return (1 - lbda_clear) * (0.5);
  } else if (phi < center - half_thickness) {
    // Ramp from the free-space value up to the band edge (0.5). Still ahead
    // of the reading, so it uses the clear profile (continuous at both ends).
    return lbda_clear * 0.5 *
           pow((phi - (center - 2 * half_thickness)) / half_thickness, 2) +
           (1 - lbda_clear) * .5;
  } else if (phi < center + half_thickness) {
    // Obstacle band: marking only, mark profile.
    double J = (center - phi) / half_thickness;
    return lbda_mark * ((1 - (0.5) * pow(J, 2)) - 0.5) + 0.5;
  } else {
    return 0.5;
  }
}


void RangeSensorLayer::reconfigureCB(range_sensor_layer::RangeSensorLayerConfig &config, uint32_t level)
{
  phi_v_ = config.phi;
  gamma_exp_ = config.gamma_exp;
  gamma_exp_clear_ = config.gamma_exp_clear;
  delta_slope_ = config.delta_slope;
  inflate_cone_ = config.inflate_cone;
  no_readings_timeout_ = config.no_readings_timeout;
  clear_threshold_ = config.clear_threshold;
  mark_threshold_ = config.mark_threshold;
  min_obstacle_thickness_ = std::max(config.min_obstacle_thickness, resolution_);
  obstacle_center_offset_ = config.obstacle_center_offset;
  clear_on_max_reading_ = config.clear_on_max_reading;
  use_decay_ = config.use_decay;
  pixel_decay_ = config.pixel_decay;

  if (enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    current_ = false;
  }
}

void RangeSensorLayer::bufferIncomingRangeMsg(const sensor_msgs::RangeConstPtr& range_message)
{
  range_message_mutex_.lock();
  range_msgs_buffer_.push_back(*range_message);
  range_message_mutex_.unlock();
}

bool compare_range(const sensor_msgs::Range& first, const sensor_msgs::Range& second)
{
  return first.range < second.range;
}

void RangeSensorLayer::updateCostmap()
{
  std::list<sensor_msgs::Range> range_msgs_buffer_copy;

  range_message_mutex_.lock();
  range_msgs_buffer_copy = std::list<sensor_msgs::Range>(range_msgs_buffer_);
  range_msgs_buffer_.clear();
  range_message_mutex_.unlock();

  //range_msgs_buffer_copy.sort(compare_range);
  //if(!range_msgs_buffer_copy.empty()){
  //  ROS_INFO("[range_sensor_layer] %s processing %d messages with ranges %.2f-%.2f",name_.c_str(),(int)range_msgs_buffer_copy.size(),range_msgs_buffer_copy.front().range,range_msgs_buffer_copy.back().range);
  //}
  for (auto & range_msgs_it : range_msgs_buffer_copy) {
    processRangeMessageFunc_(range_msgs_it);
  }
}

void RangeSensorLayer::processRangeMsg(sensor_msgs::Range& range_message)
{
  if (range_message.min_range == range_message.max_range) {
    processFixedRangeMsg(range_message);
  } else {
    processVariableRangeMsg(range_message);
  }
}

void RangeSensorLayer::processFixedRangeMsg(sensor_msgs::Range& range_message)
{
  if (!std::isinf(range_message.range)) {
    ROS_ERROR_THROTTLE(1.0,
                       "[range_sensor_layer] Fixed distance ranger (min_range == max_range) in frame %s sent invalid value. "
                       "Only -Inf (== object detected) and Inf (== no object detected) are valid.",
                       range_message.header.frame_id.c_str());
    return;
  }

  bool clear_sensor_cone = false;

  if (range_message.range > 0) {  // +inf
    if (!clear_on_max_reading_) {
      return;  // no clearing at all
    }
    clear_sensor_cone = true;
  }

  range_message.range = range_message.min_range;

  updateCostmap(range_message, clear_sensor_cone);
}

void RangeSensorLayer::processVariableRangeMsg(sensor_msgs::Range& range_message)
{
  if (range_message.range < range_message.min_range || range_message.range >
    range_message.max_range)
  {
    return;
  }

  bool clear_sensor_cone = false;

  if (range_message.range >= range_message.max_range && clear_on_max_reading_) {
    clear_sensor_cone = true;
  }

  updateCostmap(range_message, clear_sensor_cone);
}

void RangeSensorLayer::updateCostmap(sensor_msgs::Range& range_message, bool clear_sensor_cone)
{
  max_angle_ = range_message.field_of_view / 2;

  geometry_msgs::PointStamped in, out;
  in.header.stamp = range_message.header.stamp;
  in.header.frame_id = range_message.header.frame_id;

  ros::Duration transform_tolerance_duration(transform_tolerance_);
  if (!tf_->canTransform(
      in.header.frame_id, global_frame_,
      in.header.stamp,
      transform_tolerance_duration))
  {
    ros::Time now = ros::Time::now();
    ROS_ERROR_THROTTLE(1.0, "[range_sensor_layer] can't transform from %s to %s at %f. Age is %fs",
                       global_frame_.c_str(), in.header.frame_id.c_str(),
                       in.header.stamp.toSec(),(now-in.header.stamp).toSec());
    return;
  }

  tf_->transform(in, out, global_frame_, transform_tolerance_duration);

  double ox = out.point.x, oy = out.point.y;

  in.point.x = range_message.range;

  tf_->transform(in, out, global_frame_, transform_tolerance_duration);

  double tx = out.point.x, ty = out.point.y;

  // calculate target props
  double dx = tx - ox, dy = ty - oy, theta = atan2(dy, dx), d = sqrt(dx * dx + dy * dy);

  // Integer Bounds of Update
  int bx0, by0, bx1, by1;

  // Triangle that will be really updated; the other cells within bounds are ignored
  // This triangle is formed by the origin and left and right sides of sonar cone
  int Ox, Oy, Ax, Ay, Bx, By;

  // Bounds includes the origin
  worldToMapNoBounds(ox, oy, Ox, Oy);
  bx1 = bx0 = Ox;
  by1 = by0 = Oy;
  touch(ox, oy, &min_x_, &min_y_, &max_x_, &max_y_);

  // Update Map with Target Point
  // unsigned int aa, ab;
  // if (range_message.range < range_message.max_range && worldToMap(tx, ty, aa, ab)) {
  //   const char targetCost = 233;
  //   setCost(aa, ab, targetCost);
  //   touch(tx, ty, &min_x_, &min_y_, &max_x_, &max_y_);
  //   if(use_decay_ && targetCost > to_cost(mark_threshold_)) {
  //     marked_point_history_[worldCellKey(aa, ab)] = last_reading_time_.toSec();
  //   }
  // }

  double mx, my;

  // Update left side of sonar cone
  mx = ox + cos(theta - max_angle_) * d * 1.2;
  my = oy + sin(theta - max_angle_) * d * 1.2;
  worldToMapNoBounds(mx, my, Ax, Ay);
  bx0 = std::min(bx0, Ax);
  bx1 = std::max(bx1, Ax);
  by0 = std::min(by0, Ay);
  by1 = std::max(by1, Ay);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // Update right side of sonar cone
  mx = ox + cos(theta + max_angle_) * d * 1.2;
  my = oy + sin(theta + max_angle_) * d * 1.2;

  worldToMapNoBounds(mx, my, Bx, By);
  bx0 = std::min(bx0, Bx);
  bx1 = std::max(bx1, Bx);
  by0 = std::min(by0, By);
  by1 = std::max(by1, By);
  touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

  // Limit Bounds to Grid
  bx0 = std::max(0, bx0);
  by0 = std::max(0, by0);
  bx1 = std::max(0,std::min(static_cast<int>(size_x_), bx1));
  by1 = std::max(0,std::min(static_cast<int>(size_y_), by1));

  for (unsigned int x = bx0; x <= (unsigned int)bx1; x++) {
    for (unsigned int y = by0; y <= (unsigned int)by1; y++) {
      bool update_xy_cell = true;

      // Unless inflate_cone_ is set to 100 %, we update cells only within the
      // (partially inflated) sensor cone, projected on the costmap as a triangle.
      // 0 % corresponds to just the triangle, but if your sensor fov is very
      // narrow, the covered area can become zero due to cell discretization.
      // See wiki description for more details
      if (inflate_cone_ < 1.0) {
        // Determine barycentric coordinates
        int w0 = orient2d(Ax, Ay, Bx, By, x, y);
        int w1 = orient2d(Bx, By, Ox, Oy, x, y);
        int w2 = orient2d(Ox, Oy, Ax, Ay, x, y);

        // Barycentric coordinates inside area threshold; this is not mathematically
        // sound at all, but it works!
        float bcciath = -static_cast<float>(inflate_cone_) * area(Ax, Ay, Bx, By, Ox, Oy);
        update_xy_cell = w0 >= bcciath && w1 >= bcciath && w2 >= bcciath;
      }

      if (update_xy_cell) {
        double wx, wy;
        mapToWorld(x, y, wx, wy);
        update_cell(ox, oy, theta, range_message.range, wx, wy, clear_sensor_cone);
      }
    }
  }

  buffered_readings_++;
  last_reading_time_ = ros::Time::now();
  if(use_decay_)
    removeOutdatedReadings();
}

void RangeSensorLayer::removeOutdatedReadings()
{
  std::map<std::pair<int, int>, double>::iterator it_map;
  double removal_time = last_reading_time_.toSec() - pixel_decay_;
  const int kx = static_cast<int>(std::llround(origin_x_ / resolution_));
  const int ky = static_cast<int>(std::llround(origin_y_ / resolution_));
  for (it_map = marked_point_history_.begin() ; it_map != marked_point_history_.end() ; ) {
    if(it_map->second < removal_time) {
      // Key is the cell's fixed world-lattice index; subtract the current
      // origin's lattice offset to get the current cell index. If the cell
      // scrolled out of the rolling window it is already gone from the map,
      // so just drop the bookkeeping entry.
      const int x = it_map->first.first - kx;
      const int y = it_map->first.second - ky;
      if (x >= 0 && y >= 0 && x < static_cast<int>(size_x_) && y < static_cast<int>(size_y_)) {
        double wx, wy;
        mapToWorld(static_cast<unsigned int>(x), static_cast<unsigned int>(y), wx, wy);
        touch(wx, wy, &min_x_, &min_y_, &max_x_, &max_y_);
        setCost(x, y, costmap_2d::FREE_SPACE);
      }
      it_map = marked_point_history_.erase(it_map);
    } else {
      it_map++;
    }
  }
}

void RangeSensorLayer::update_cell(
  double ox, double oy, double ot, double r,
  double nx, double ny, bool clear)
{
  unsigned int x, y;
  if (worldToMap(nx, ny, x, y)) {
    double dx = nx - ox, dy = ny - oy;
    double theta = atan2(dy, dx) - ot;
    theta = angles::normalize_angle(theta);
    double phi = sqrt(dx * dx + dy * dy);
    double sensor = 0.0;
    if (!clear) {
      sensor = sensor_model(r, phi, theta);
    }
    double prior = to_prob(getCost(x, y));
    if(prior<0.1){
      prior = 0.1;
    }
    double prob_occ = sensor * prior;
    double prob_not = (1 - sensor) * (1 - prior);
    if((prob_occ + prob_not) == 0){
      ROS_ERROR("[range_sensor_layer] Divide by zero???");
    }
    double new_prob = prob_occ / (prob_occ + prob_not);

    ROS_DEBUG("%f %f | %f %f = %f", dx, dy, theta, phi, sensor);
    ROS_DEBUG("%f | %f %f | %f", prior, prob_occ, prob_not, new_prob);
    unsigned char c = to_cost(new_prob);

    setCost(x, y, c);
    if(use_decay_) {
      // Key by the cell's fixed world-lattice index so the history survives
      // the renumbering that a rolling-window origin update performs.
      std::pair<int, int> coordinate_pair = worldCellKey(x, y);
      // If the point has a score high enough to be marked in the costmap, we add it's time to the marked_point_history
      if(c > to_cost(mark_threshold_)) {
        marked_point_history_[coordinate_pair] = last_reading_time_.toSec();
      }
      // If the point score is not high enough, we try to find it in the mark history point.
      // In the case we find it in the marked_point_history we clear it from the map so we won't checked already cleared point
      else if(c < to_cost(clear_threshold_)) {
        std::map<std::pair<int, int>, double>::iterator it_clear;
        it_clear = marked_point_history_.find(coordinate_pair);
        if(it_clear != marked_point_history_.end()) {
          marked_point_history_.erase(it_clear);
        }
      }
    }
  }
}

void RangeSensorLayer::resetRange()
{
  min_x_ = min_y_ = std::numeric_limits<double>::max();
  max_x_ = max_y_ = -std::numeric_limits<double>::max();
}

void RangeSensorLayer::updateBounds(
  double robot_x, double robot_y,
  double robot_yaw, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  robot_yaw = 0 + robot_yaw;  // Avoid error if variable not in use
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  updateCostmap();

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);

  resetRange();

  if (!enabled_) {
    current_ = true;
    return;
  }

  if (buffered_readings_ == 0) {
    if (no_readings_timeout_ > 0.0 &&
        (ros::Time::now() - last_reading_time_).toSec() > no_readings_timeout_)
    {
      //TODO: we should check for all expected frames
      ROS_WARN_THROTTLE(2.0, "[range_sensor_layer] No readings received for %.2f seconds, " \
                        "while expected at least every %.2f seconds.",
                        (ros::Time::now() - last_reading_time_).toSec(), no_readings_timeout_);
      current_ = false;
    }
  } else {
    current_ = true;
  }
}

void RangeSensorLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  unsigned char clear = to_cost(clear_threshold_), mark = to_cost(mark_threshold_);

  // Merge direction depends on allow_clearing_:
  //  false (default, legacy): one-way — only fill NO_INFORMATION cells and
  //        raise costs. Never lowers a master cell, so when this layer shares
  //        its costmap with other layers (static map, contact) its clear cones
  //        cannot erase their obstacles. Stale US cells are still cleared in
  //        multi-layer costmaps because the static layer rewrites the master
  //        region every cycle.
  //  true: authoritative — a strong "occupied" signal raises the cell to
  //        LETHAL and a strong "clear" signal lowers it to FREE. The lowering
  //        is REQUIRED when this layer is the sole layer of its costmap
  //        (uss_costmap): nothing else ever rewrites the master grid, so a
  //        raise-only merge would keep every marked cell lethal forever and
  //        decay could not clear the published map.
  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      unsigned char prob = costmap_[it];
      unsigned char old_cost = master_array[it];
      if (prob == costmap_2d::NO_INFORMATION) {
        // layer has no info here: leave the master untouched
      } else if (prob > mark) {
        if (allow_clearing_ || old_cost == costmap_2d::NO_INFORMATION ||
            old_cost < costmap_2d::LETHAL_OBSTACLE) {
          master_array[it] = costmap_2d::LETHAL_OBSTACLE;
        }
      } else if (prob < clear) {
        if (allow_clearing_ || old_cost == costmap_2d::NO_INFORMATION) {
          master_array[it] = costmap_2d::FREE_SPACE;
        }
      }
      // band [clear, mark]: no strong signal, leave the master untouched
      it++;
    }
  }

  buffered_readings_ = 0;

  // if not current due to reset, set current now after clearing
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
  if (debug_publisher_) {
    publisher_->publishCostmap();
  }
}

void RangeSensorLayer::reset()
{
  ROS_DEBUG("[range_sensor_layer] reseting...");
  deactivate();
  resetMaps();
  marked_point_history_.clear();
  was_reset_ = true;
  activate();
}

void RangeSensorLayer::deactivate()
{
  range_msgs_buffer_.clear();
}

void RangeSensorLayer::activate()
{
  range_msgs_buffer_.clear();
}

}  // namespace range_sensor_layer
