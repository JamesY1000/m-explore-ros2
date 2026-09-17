/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
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
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>
#include <algorithm>
#include <exception>
#include <mutex>
#include <nav2_costmap_2d/cost_values.hpp>

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , last_markers_count_(0)
{
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<double>("roi_weight", 1.0);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("roi_weight", roi_weight_);

  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // ExploreToPose action server
  explore_to_pose_server_ = rclcpp_action::create_server<rs1_interfaces::action::ExploreToPose>(
      this,
      "explore_to_pose",
      std::bind(&Explore::exploreToPoseRequestCb, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&Explore::exploreToPoseCancelRequestCb, this,
                std::placeholders::_1),
      std::bind(&Explore::exploreToPoseAcceptedCb, this,
                std::placeholders::_1));


  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });

  // Cancel the timer so it starts off action call. Reenable it when we starting exploration
  exploring_timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Waiting for an ExploreToPose goal");

  // // Start exploration right away
  // auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  // status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  // status_pub_->publish(status_msg);
  // makePlan();
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  // Make sure it only starts when it receives an active goal handle
  if (!explore_to_pose_goal_handle_ || !explore_to_pose_goal_handle_->is_active() ||
      explore_to_pose_goal_handle_->is_canceling() || exploring_timer_->is_canceled())
  {
    return;
  }

  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  auto feedback =
      std::make_shared<rs1_interfaces::action::ExploreToPose::Feedback>();
  feedback->stage = final_navigation_ ? "navigating_to_roi" : "exploring";
  feedback->distance_to_target = static_cast<float>(std::hypot(
      pose.position.x - target_pose_.pose.position.x,
      pose.position.y - target_pose_.pose.position.y));
  explore_to_pose_goal_handle_->publish_feedback(feedback);

  // Finish one navigation leg before choosing another destination.
  // goal_active_ also covers a request awaiting acceptance by Nav2.
  if (goal_active_) {
    return;
  }
  if (!roi_attempted_since_frontier_ && roiIsKnownFree()) {
    RCLCPP_INFO(logger_, "ROI is known free; attempting direct navigation");
    sendNavigationGoal(target_pose_, true);
    return;
  }

  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);

  const auto &roi = target_pose_.pose.position;

  // Add penalty for frontiers away from the ROI - just take euclidean distance for now
  for (auto &frontier : frontiers)
  {
    const double distance_to_roi = std::hypot(
      frontier.centroid.x - roi.x,
      frontier.centroid.y - roi.y
    );

    frontier.cost += roi_weight_ * distance_to_roi;
  }

  // Resort now with updated costs
  // Re-sort because the costs have changed.
  std::sort(frontiers.begin(), frontiers.end(),
    [](const frontier_exploration::Frontier &a,
      const frontier_exploration::Frontier &b)
      {
        return a.cost < b.cost;
      }
  );

  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    finishExploration(false, "No frontiers remain and the ROI was not reached");
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    finishExploration(false, "No usable frontiers remain and the ROI was not reached");
    return;
  }
  geometry_msgs::msg::Point target_position = frontier->centroid;

  geometry_msgs::msg::PoseStamped frontier_pose;
  frontier_pose.pose.position = target_position;
  frontier_pose.pose.orientation.w = 1.0;
  frontier_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  sendNavigationGoal(frontier_pose, false);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal) {
  goal_active_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      roi_attempted_since_frontier_ = false;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_DEBUG(logger_, "Goal aborted; blacklisting frontier");
      frontier_blacklist_.push_back(frontier_goal);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // This node uses the default mutually exclusive callback group.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");

  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }

  exploring_timer_->cancel();
  if (navigation_goal_handle_) {
    move_base_client_->async_cancel_goal(navigation_goal_handle_);
  }

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  // Make sure it only starts when it receives an active goal handle
  if (!explore_to_pose_goal_handle_ || !explore_to_pose_goal_handle_->is_active() ||
      explore_to_pose_goal_handle_->is_canceling())
  {
    return;
  }

  roi_attempted_since_frontier_ = false;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

rclcpp_action::GoalResponse Explore::exploreToPoseRequestCb(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const rs1_interfaces::action::ExploreToPose::Goal> goal)
{
  (void)uuid;
  // Reject if another exploration task is still active
  if (explore_to_pose_goal_handle_ && explore_to_pose_goal_handle_->is_active())
  {
    RCLCPP_WARN(logger_, "Rejecting goal: exploration is already active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Reject an invalid target pose
  if (!validPose(goal->target_pose))
  {
    RCLCPP_WARN(logger_, "Rejecting goal: target pose is invalid");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Initially require the target frame to equal costmap_client_.getGlobalFrameID()
  const auto map_frame = costmap_client_.getGlobalFrameID();
  if (goal->target_pose.header.frame_id.empty() || goal->target_pose.header.frame_id != map_frame)
  {
    RCLCPP_WARN(logger_, "Rejecting goal: target frame must be '%s'", map_frame.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Check move_base_client_->action_server_is_ready()
  if (!move_base_client_->action_server_is_ready()) {
    RCLCPP_WARN(logger_, "Rejecting goal: Nav2 server unavailable");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Otherwise if all checks complete, return ACCEPT_AND_EXECUTE
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Explore::exploreToPoseCancelRequestCb(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::ExploreToPose>> goal_handle)
{
  // Keep cancellation unsupported until navigation cleanup is implemented.
  (void)goal_handle;
  RCLCPP_WARN(logger_, "Exploration cancellation is not implemented yet");
  return rclcpp_action::CancelResponse::REJECT;
}

void Explore::exploreToPoseAcceptedCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<rs1_interfaces::action::ExploreToPose>> goal_handle)
{
  // Start exploration with target pose and enable timer
  explore_to_pose_goal_handle_ = goal_handle;
  target_pose_ = goal_handle->get_goal()->target_pose;

  final_navigation_ = false;
  roi_attempted_since_frontier_ = false;
  goal_active_ = false;
  navigation_goal_handle_.reset();
  frontier_blacklist_.clear();

  start(); // Currently this publises EXPLORATION_STARTED
  exploring_timer_->reset(); // Reenables the makePlan() calls
}


bool Explore::roiIsKnownFree()
{
  auto *map = costmap_client_.getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
  unsigned int mx, my;
  const auto &position = target_pose_.pose.position;
  return map->worldToMap(position.x, position.y, mx, my) &&
         map->getCost(mx, my) == nav2_costmap_2d::FREE_SPACE;
}

void Explore::sendNavigationGoal(
    const geometry_msgs::msg::PoseStamped &pose, bool to_roi)
{
  final_navigation_ = to_roi;
  if (to_roi) {
    roi_attempted_since_frontier_ = true;
  }
  goal_active_ = true;

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose = pose;
  goal.pose.header.stamp = this->now();

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
      [this, pose, to_roi](NavigationGoalHandle::SharedPtr handle) {
        if (!handle) {
          goal_active_ = false;
          final_navigation_ = false;
          navigation_goal_handle_.reset();
          if (!to_roi) {
            frontier_blacklist_.push_back(pose.pose.position);
          }
          RCLCPP_WARN(logger_, "Nav2 rejected %s goal",
                      to_roi ? "ROI" : "frontier");
          return;
        }
        navigation_goal_handle_ = handle;

        // A pause may have arrived while Nav2 was accepting the goal.
        if (exploring_timer_->is_canceled()) {
          move_base_client_->async_cancel_goal(handle);
        }
      };

  options.result_callback =
      [this, pose, to_roi](
          const NavigationGoalHandle::WrappedResult &result) {
        if (!explore_to_pose_goal_handle_) {
          return;
        }
        navigation_goal_handle_.reset();

        if (!to_roi) {
          reachedGoal(result, pose.pose.position);
          return;
        }

        goal_active_ = false;
        final_navigation_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          finishExploration(true, "Reached the ROI");
        } else {
          RCLCPP_WARN(logger_, "ROI navigation did not succeed; resuming exploration");
          makePlan();
        }
      };

  try {
    move_base_client_->async_send_goal(goal, options);
  } catch (const std::exception &error) {
    goal_active_ = false;
    final_navigation_ = false;
    navigation_goal_handle_.reset();
    if (!to_roi) {
      frontier_blacklist_.push_back(pose.pose.position);
    }
    RCLCPP_ERROR(logger_, "Could not send navigation goal: %s", error.what());
  }
}

void Explore::finishExploration(bool success, const std::string &message)
{
  if (!explore_to_pose_goal_handle_) {
    return;
  }

  exploring_timer_->cancel();
  goal_active_ = false;
  final_navigation_ = false;
  navigation_goal_handle_.reset();

  auto result = std::make_shared<rs1_interfaces::action::ExploreToPose::Result>();
  result->message = message;
  if (success) {
    explore_to_pose_goal_handle_->succeed(result);
    auto status = explore_lite_msgs::msg::ExploreStatus();
    status.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status);
    RCLCPP_INFO(logger_, "%s", message.c_str());
  } else {
    explore_to_pose_goal_handle_->abort(result);
    RCLCPP_WARN(logger_, "%s", message.c_str());
  }
  explore_to_pose_goal_handle_.reset();
}

bool Explore::validPose(const geometry_msgs::msg::PoseStamped &pose)
{
  const auto q = pose.pose.orientation;
  const double quaternion_norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;

  // Check position and orientation
  if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) || !std::isfinite(pose.pose.position.z) ||
      !std::isfinite(quaternion_norm_squared) || std::abs(quaternion_norm_squared - 1.0) > 1e-3)
  {
    return false;
  }

  return true;
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
