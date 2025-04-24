// Copyright 2025 Andrey Styrizhkin
#include <contact_sensor_layer/contact_sensor_layer.h>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <algorithm>
#include <list>
#include <limits>
#include <map>
#include <string>
#include <utility>

PLUGINLIB_EXPORT_CLASS(contact_sensor_layer::ContactSensorLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;

namespace contact_sensor_layer
{

ContactSensorLayer::ContactSensorLayer() {}

ContactSensorLayer::~ContactSensorLayer() {
  if (publisher_ != NULL) {
    delete publisher_;
  }

  delete dsrv_;
}

void ContactSensorLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  was_reset_ = false;
  buffered_readings_ = 0;
  last_reading_time_ = ros::Time::now();
  default_value_ = costmap_2d::FREE_SPACE;

  matchSize();
  resetRange();

  // Default topic names list contains a single topic: /bumper
  // We use the XmlRpcValue constructor that takes a XML string and reading start offset
  const char* xml = "<value><array><data><value>/bumper</value></data></array></value>";
  int zero_offset = 0;
  std::string topics_ns;
  XmlRpc::XmlRpcValue topic_names(xml, &zero_offset);

  nh.param("ns", topics_ns, std::string());
  nh.param("topics", topic_names, topic_names);

  nh.param("debug_publisher", debug_publisher_, false);
  nh.param("transform_tolerance_", transform_tolerance_, 0.3);

  // Validate topic names list: it must be a (normally non-empty) list of strings
  if ((topic_names.valid() == false) || (topic_names.getType() != XmlRpc::XmlRpcValue::TypeArray))
  {
    ROS_ERROR("[contact_sensor_layer] Invalid topic names list: it must be a non-empty list of strings");
    return;
  }

  if (topic_names.size() < 1)
  {
    // This could be an error, but I keep it as it can be useful for debug
    ROS_WARN("[contact_sensor_layer] Empty topic names list: range sensor layer will have no effect on costmap");
  }

  // Traverse the topic names list subscribing to all of them with the same callback method
  for (int i = 0; i < topic_names.size(); i++)
  {
    if (topic_names[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_WARN("[contact_sensor_layer] Invalid topic names list: element %d is not a string, so it will be ignored", i);
    }
    else
    {
      std::string topic_name(topics_ns);
      if ((topic_name.size() > 0) && (topic_name.at(topic_name.size() - 1) != '/'))
        topic_name += "/";
      topic_name += static_cast<std::string>(topic_names[i]);

      processContactMessageFunc_ = boost::bind(&ContactSensorLayer::processContactMsg, this, _1);

      contact_subs_.push_back(nh.subscribe(topic_name, 100, &ContactSensorLayer::bufferIncomingContactMsg, this));

      ROS_INFO("[contact_sensor_layer] subscribed to topic %s", contact_subs_.back().getTopic().c_str());
    }
  }

  dsrv_ = new dynamic_reconfigure::Server<contact_sensor_layer::ContactSensorLayerConfig>(nh);
  dynamic_reconfigure::Server<contact_sensor_layer::ContactSensorLayerConfig>::CallbackType cb =
    boost::bind(&ContactSensorLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  global_frame_ = layered_costmap_->getGlobalFrameID();

  if (debug_publisher_) {
    publisher_ = new costmap_2d::Costmap2DPublisher(&nh, this, global_frame_, "costmap",true);
  }
}

double ContactSensorLayer::sensor_model(double r, double phi, double theta)
{
  if( phi <= r ){
    return 1.0;
  }else{
    return 0.0;
  }
}

void ContactSensorLayer::reconfigureCB(contact_sensor_layer::ContactSensorLayerConfig &config, uint32_t level)
{
  obstacle_radius_ = config.obstacle_radius;
  obstacle_forward_offset_ = config.obstacle_forward_offset;
  no_readings_timeout_ = 1.0;

  if (enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    current_ = false;
    ROS_INFO_STREAM_THROTTLE(2,"[contact_sensor_layer] reconfigure current = false");
  }
}

void ContactSensorLayer::bufferIncomingContactMsg(const contact_sensor_layer::ContactConstPtr& contact_message)
{
  contact_message_mutex_.lock();
  contact_msgs_buffer_.push_back(*contact_message);
  contact_message_mutex_.unlock();
}

void ContactSensorLayer::updateCostmap()
{
  std::list<contact_sensor_layer::Contact> contact_msgs_buffer_copy;

  contact_message_mutex_.lock();
  contact_msgs_buffer_copy = std::list<contact_sensor_layer::Contact>(contact_msgs_buffer_);
  contact_msgs_buffer_.clear();
  contact_message_mutex_.unlock();

  //range_msgs_buffer_copy.sort(compare_range);
  //if(!range_msgs_buffer_copy.empty()){
  //  ROS_INFO("[contact_sensor_layer] %s processing %d messages with ranges %.2f-%.2f",name_.c_str(),(int)range_msgs_buffer_copy.size(),range_msgs_buffer_copy.front().range,range_msgs_buffer_copy.back().range);
  //}
  for (auto & range_msgs_it : contact_msgs_buffer_copy) {
    processContactMessageFunc_(range_msgs_it);
  }
}

void ContactSensorLayer::processContactMsg(contact_sensor_layer::Contact& contact_message)
{
  if(contact_message.is_active) {
    updateCostmap(contact_message);
  } else {
    buffered_readings_++;
    last_reading_time_ = ros::Time::now();
  }
}

void ContactSensorLayer::updateCostmap(contact_sensor_layer::Contact& contact_message)
{
  geometry_msgs::PointStamped in, out;
  in.header.stamp = contact_message.header.stamp;
  in.header.frame_id = contact_message.header.frame_id;

  ros::Duration transform_tolerance_duration(transform_tolerance_);
  if (!tf_->canTransform(
      in.header.frame_id, global_frame_,
      in.header.stamp,
      transform_tolerance_duration))
  {
    ros::Time now = ros::Time::now();
    ROS_ERROR_THROTTLE(1.0, "[contact_sensor_layer] can't transform from %s to %s at %f. Age is %fs",
                       global_frame_.c_str(), in.header.frame_id.c_str(),
                       in.header.stamp.toSec(),(now-in.header.stamp).toSec());
    return;
  }

  tf_->transform(in, out, global_frame_, transform_tolerance_duration);

  double ox = out.point.x, oy = out.point.y;

  in.point.x = obstacle_forward_offset_;

  tf_->transform(in, out, global_frame_, transform_tolerance_duration);

  double tx = out.point.x, ty = out.point.y;

  // calculate target props
  double dx = tx - ox, dy = ty - oy, theta = atan2(dy, dx)/*, d = sqrt(dx * dx + dy * dy)*/;

  // Integer Bounds of Update
  int bx0, by0, bx1, by1;

  // Bounds includes the origin
  worldToMapNoBounds(tx-obstacle_radius_, ty-obstacle_radius_, bx0, by0);
  touch(tx-obstacle_radius_, ty-obstacle_radius_, &min_x_, &min_y_, &max_x_, &max_y_);

  worldToMapNoBounds(tx+obstacle_radius_, ty+obstacle_radius_, bx1, by1);
  touch(tx+obstacle_radius_, ty+obstacle_radius_, &min_x_, &min_y_, &max_x_, &max_y_);

  // Limit Bounds to Grid
  bx0 = std::max(0, bx0);
  by0 = std::max(0, by0);
  bx1 = std::max(0,std::min(static_cast<int>(size_x_), bx1));
  by1 = std::max(0,std::min(static_cast<int>(size_y_), by1));

  for (unsigned int x = bx0; x <= (unsigned int)bx1; x++) {
    for (unsigned int y = by0; y <= (unsigned int)by1; y++) {
      double wx, wy;
      mapToWorld(x, y, wx, wy);
      update_cell(tx, ty, theta, obstacle_radius_, wx, wy, false);
    }
  }

  buffered_readings_++;
  last_reading_time_ = ros::Time::now();
}

void ContactSensorLayer::update_cell(
  double tx, double ty, double ot, double r,
  double nx, double ny, bool clear)
{
  unsigned int x, y;
  if (worldToMap(nx, ny, x, y)) {
    double dx = nx - tx, dy = ny - ty;
    double theta = atan2(dy, dx) - ot;
    theta = angles::normalize_angle(theta);
    double phi = sqrt(dx * dx + dy * dy);
    double sensor = 0.0;
    if (!clear) {
      sensor = sensor_model(r, phi, theta);
    }

    unsigned char c = sensor > 0.5 ? costmap_2d::LETHAL_OBSTACLE : getCost(x, y);
    setCost(x, y, c);
  }
}

void ContactSensorLayer::resetRange()
{
  min_x_ = min_y_ = std::numeric_limits<double>::max();
  max_x_ = max_y_ = -std::numeric_limits<double>::max();
}

void ContactSensorLayer::updateBounds(
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
      ROS_WARN_THROTTLE(2.0, "[contact_sensor_layer] No readings received for %.2f seconds, " \
                        "while expected at least every %.2f seconds.",
                        (ros::Time::now() - last_reading_time_).toSec(), no_readings_timeout_);
      current_ = false;
    }
  }
}

void ContactSensorLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  unsigned char clear = costmap_2d::FREE_SPACE, mark = costmap_2d::LETHAL_OBSTACLE;

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      unsigned char prob = costmap_[it];
      unsigned char current;
      if (prob == costmap_2d::NO_INFORMATION) {
        it++;
        continue;
      } else if (prob >= mark) {
        current = costmap_2d::LETHAL_OBSTACLE;
      } else if (prob <= clear) {
        current = costmap_2d::FREE_SPACE;
      } else {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];

      if (old_cost == NO_INFORMATION || old_cost < current) {
        master_array[it] = current;
      }
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
  ROS_INFO_STREAM_THROTTLE(2,"[contact_sensor_layer] current " << current_);

}

void ContactSensorLayer::reset()
{
  ROS_DEBUG("[contact_sensor_layer] reseting...");
  deactivate();
  resetMaps();
  was_reset_ = true;
  activate();
}

void ContactSensorLayer::deactivate()
{
  contact_msgs_buffer_.clear();
}

void ContactSensorLayer::activate()
{
  contact_msgs_buffer_.clear();
}

}  // namespace contact_sensor_layer
