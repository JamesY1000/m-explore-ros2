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
#include <limits>
#include <mutex>
#include <stdexcept>
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
  this->declare_parameter<double>("minimum_frontier_distance", 2.0);
  this->declare_parameter<double>("preferred_frontier_distance", 20.0);
  this->declare_parameter<double>("roi_min_progress", 0.5);
  this->declare_parameter<bool>("roi_allow_backtracking", true);
  this->declare_parameter<double>("frontier_gain_cap", 5.0);
  this->declare_parameter<double>("frontier_endpoint_tolerance", 0.5);
  this->declare_parameter<int>("frontier_validation_timeout", 90);
  this->declare_parameter<double>("frontier_revisit_radius", 2.5);
  this->declare_parameter<int>("recent_frontier_history_size", 20);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("roi_weight", roi_weight_);
  this->get_parameter("minimum_frontier_distance", frontier_options_.minimum_distance);
  this->get_parameter("preferred_frontier_distance", frontier_options_.preferred_distance);
  this->get_parameter("roi_min_progress", frontier_options_.min_roi_progress);
  this->get_parameter("roi_allow_backtracking", frontier_options_.allow_backtracking);
  this->get_parameter("frontier_gain_cap", frontier_options_.max_frontier_gain);
  this->get_parameter("frontier_endpoint_tolerance", frontier_endpoint_tolerance_);
  this->get_parameter("frontier_validation_timeout", frontier_validation_timeout_);
  this->get_parameter("frontier_revisit_radius", frontier_options_.revisit_radius);
  this->get_parameter("recent_frontier_history_size", recent_frontier_history_size_);

  if (!std::isfinite(frontier_options_.revisit_radius) ||
      frontier_options_.revisit_radius < 0.0 || recent_frontier_history_size_ <= 0) {
    throw std::invalid_argument(
        "frontier_revisit_radius must be finite and nonnegative; "
        "recent_frontier_history_size must be positive");
  }

  if (!std::isfinite(frontier_endpoint_tolerance_) ||
      frontier_endpoint_tolerance_ < 0.0)
  {
      throw std::invalid_argument(
          "frontier_endpoint_tolerance must be finite and nonnegative");
  }

  if (frontier_validation_timeout_ <= 0)
  {
      throw std::invalid_argument(
          "frontier_validation_timeout must be positive");
  }

  frontier_options_.potential_scale = potential_scale_;
  frontier_options_.gain_scale = gain_scale_;
  frontier_options_.roi_weight = roi_weight_;
  if (!std::isfinite(frontier_options_.minimum_distance) ||
      frontier_options_.minimum_distance < 0.0 ||
      !std::isfinite(frontier_options_.preferred_distance) ||
      frontier_options_.preferred_distance <= 0.0 ||
      !std::isfinite(frontier_options_.min_roi_progress) ||
      frontier_options_.min_roi_progress < 0.0 ||
      !std::isfinite(frontier_options_.max_frontier_gain) ||
      frontier_options_.max_frontier_gain < 0.0 ||
      !std::isfinite(potential_scale_) || potential_scale_ < 0.0 ||
      !std::isfinite(gain_scale_) || gain_scale_ < 0.0 ||
      !std::isfinite(roi_weight_) || roi_weight_ < 0.0) {
    throw std::invalid_argument("Frontier distances and weights must be finite and nonnegative; "
        "preferred_frontier_distance must be positive");
  }
  RCLCPP_INFO(logger_,
      "ROI-directed selection: minimum distance %.2f m, preferred distance %.2f m, "
      "minimum progress %.2f m, backtracking %s, ROI weight %.2f",
      frontier_options_.minimum_distance, frontier_options_.preferred_distance,
      frontier_options_.min_roi_progress,
      frontier_options_.allow_backtracking ? "allowed as fallback" : "disabled", roi_weight_);

  RCLCPP_INFO(logger_,
      "Frontier validation: endpoint tolerance %.3f m, batch timeout %d s",
      frontier_endpoint_tolerance_, frontier_validation_timeout_);
  RCLCPP_INFO(logger_,
      "Frontier revisit filter: radius %.2f m, history %d successful destinations",
      frontier_options_.revisit_radius, recent_frontier_history_size_);

  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  planner_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
      this, "compute_path_to_pose");
  navigation_costmap_client_ = create_client<nav2_msgs::srv::GetCostmap>(
      "global_costmap/get_costmap");
  validation_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
    if (validation_active_ && std::chrono::steady_clock::now() > validation_deadline_) {
      cancelFrontierValidation();
      finishExploration(false, "Frontier validation batch exceeded its time budget");
    }
  });

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
  geometry_msgs::msg::Pose pose;
  if (!costmap_client_.getRobotPose(pose)) {
    RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 5000,
        "Waiting for robot pose before selecting a frontier");
    return;
  }
  auto feedback =
      std::make_shared<rs1_interfaces::action::ExploreToPose::Feedback>();
  feedback->stage = validation_active_ ? "validating_frontier" :
      (final_navigation_ ? "navigating_to_roi" : "exploring");
  feedback->distance_to_target = static_cast<float>(std::hypot(
      pose.position.x - target_pose_.pose.position.x,
      pose.position.y - target_pose_.pose.position.y));
  explore_to_pose_goal_handle_->publish_feedback(feedback);

  // Finish one navigation leg before choosing another destination.
  // goal_active_ also covers a request awaiting acceptance by Nav2.
  if (goal_active_ || validation_active_ ||
      std::chrono::steady_clock::now() < validation_retry_after_) {
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

  // Centroid scores order visualization. Actual navigation candidates are
  // ranked separately by directedFrontierTargets using their approach positions.
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
    RCLCPP_DEBUG(logger_, "frontier %zd centroid visualization score: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    finishExploration(false, "No frontiers remain and the ROI was not reached");
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  frontiers.erase(std::remove_if(frontiers.begin(), frontiers.end(),
      [this](const frontier_exploration::Frontier &frontier) {
        return goalOnBlacklist(frontier.centroid);
      }), frontiers.end());
  if (frontiers.empty()) {
    finishExploration(false, "No usable frontiers remain and the ROI was not reached");
    return;
  }
  validateFrontiers(frontiers);
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
                          const geometry_msgs::msg::Point& frontier_goal,
                          const geometry_msgs::msg::Point& destination) {
  goal_active_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      recent_frontier_goals_.push_back(destination);
      if (recent_frontier_goals_.size() >
          static_cast<std::size_t>(recent_frontier_history_size_)) {
        recent_frontier_goals_.erase(recent_frontier_goals_.begin());
      }
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
  cancelFrontierValidation();
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
  empty_validation_attempts_ = 0;
  validation_retry_after_ = {};
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
  cancelFrontierValidation();
  explore_to_pose_goal_handle_ = goal_handle;
  target_pose_ = goal_handle->get_goal()->target_pose;
  empty_validation_attempts_ = 0;
  validation_retry_after_ = {};

  final_navigation_ = false;
  roi_attempted_since_frontier_ = false;
  goal_active_ = false;
  navigation_goal_handle_.reset();
  frontier_blacklist_.clear();
  recent_frontier_goals_.clear();

  start(); // Currently this publises EXPLORATION_STARTED
  exploring_timer_->reset(); // Reenables the makePlan() calls
}


bool Explore::validationIsCurrent(uint64_t generation) const
{
  return validation_active_ && generation == validation_generation_ &&
         explore_to_pose_goal_handle_ && explore_to_pose_goal_handle_->is_active() &&
         !explore_to_pose_goal_handle_->is_canceling() && !exploring_timer_->is_canceled();
}

void Explore::cancelFrontierValidation()
{
  ++validation_generation_;
  validation_active_ = false;
  validation_targets_.clear();
  validation_detour_announced_ = false;
  validation_costmap_.reset();
  if (costmap_request_id_ >= 0) {
    navigation_costmap_client_->remove_pending_request(costmap_request_id_);
    costmap_request_id_ = -1;
  }
  if (planning_goal_handle_ && rclcpp::ok()) {
    try {
      planner_client_->async_cancel_goal(planning_goal_handle_);
    } catch (const std::exception &error) {
      RCLCPP_WARN(logger_, "Could not cancel frontier planning: %s", error.what());
    }
  }
  planning_goal_handle_.reset();
}

void Explore::validateFrontiers(
    const std::vector<frontier_exploration::Frontier> &frontiers)
{
  if (!navigation_costmap_client_->service_is_ready() ||
      !planner_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 5000,
        "Waiting for Nav2 costmap and planner before validating frontiers");
    return;
  }
  validation_active_ = true;
  const auto generation = ++validation_generation_;
  validation_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::seconds(frontier_validation_timeout_);
  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  try {
    auto pending = navigation_costmap_client_->async_send_request(request,
        [this, generation, frontiers](
            rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture future) {
          if (!validationIsCurrent(generation)) {
            return;
          }
          costmap_request_id_ = -1;
          try {
            auto response = future.get();
            if (response->map.header.frame_id != costmap_client_.getGlobalFrameID()) {
              finishExploration(false, "Frontier map and Nav2 costmap frames differ");
              return;
            }
            validation_costmap_ = std::make_shared<nav2_msgs::msg::Costmap>(
                std::move(response->map));
            geometry_msgs::msg::Pose robot_pose;
            if (!costmap_client_.getRobotPose(robot_pose)) {
              RCLCPP_WARN(logger_, "Deferring frontier selection until robot pose is available");
              cancelFrontierValidation();
              validation_retry_after_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
              return;
            }
            RCLCPP_INFO(logger_, "%s", describeNavigationStart(
                *validation_costmap_, robot_pose.position).c_str());
            std::size_t revisit_excluded = 0;
            validation_targets_ = directedFrontierTargets(
                *costmap_client_.getCostmap(), *validation_costmap_, frontiers,
                robot_pose.position, target_pose_.pose.position, frontier_options_,
                recent_frontier_goals_, &revisit_excluded);
            RCLCPP_INFO(logger_,
                "Frontier revisit filter excluded %zu approach cells near %zu recent "
                "destinations (radius %.2f m)",
                revisit_excluded, recent_frontier_goals_.size(), frontier_options_.revisit_radius);
            validation_detour_announced_ = false;
            validation_target_index_ = 0;
            if (validation_targets_.empty()) {
              if (++empty_validation_attempts_ < 3) {
                RCLCPP_WARN(logger_,
                    "No frontier approach cell passed clearance checks and distance/ROI/revisit policy; refreshing maps (attempt %u/3)",
                    empty_validation_attempts_);
                cancelFrontierValidation();
                validation_retry_after_ = std::chrono::steady_clock::now() +
                    std::chrono::seconds(2);
              } else {
                std::string message = frontier_options_.allow_backtracking ?
                    "No known-free frontier approach cell passed Nav2 costmap clearance checks and distance/ROI/revisit policy after 3 snapshots" :
                    "No known-free ROI-progress approach cell passed Nav2 clearance checks and distance/revisit policy after 3 snapshots; backtracking is disabled";
                if (revisit_excluded > 0) {
                  message += "; recently visited approach regions remain excluded to prevent cycling";
                }
                finishExploration(false, message);
              }
              return;
            }
            empty_validation_attempts_ = 0;
            size_t tier_counts[4] = {0, 0, 0, 0};
            for (const auto &target : validation_targets_) {
              ++tier_counts[target.tier];
            }
            RCLCPP_INFO(logger_,
                "Checking %zu frontier approach candidates with Nav2 "
                "(local progress: %zu, distant progress: %zu, local detour: %zu, distant detour: %zu)",
                validation_targets_.size(), tier_counts[0], tier_counts[1],
                tier_counts[2], tier_counts[3]);
            validateNextTarget(generation);
          } catch (const std::exception &error) {
            finishExploration(false, std::string("Frontier costmap validation failed: ") + error.what());
          }
        });
    costmap_request_id_ = pending.request_id;
  } catch (const std::exception &error) {
    finishExploration(false, std::string("Could not request Nav2 costmap: ") + error.what());
  }
}

void Explore::validateNextTarget(uint64_t generation)
{
  if (!validationIsCurrent(generation)) {
    return;
  }
  if (std::chrono::steady_clock::now() > validation_deadline_) {
    finishExploration(false, "Frontier validation batch exceeded its time budget");
    return;
  }
  if (validation_target_index_ >= validation_targets_.size()) {
    finishExploration(false, frontier_options_.allow_backtracking ?
        "None of the sampled frontier targets passed Nav2 path validation; see rejection reasons above and Nav2 planner logs" :
        "No sampled ROI-progress target passed Nav2 path validation; backtracking is disabled");
    return;
  }
  const auto target = validation_targets_[validation_target_index_++];
  if (target.tier >= 2 && !validation_detour_announced_) {
    RCLCPP_WARN(logger_,
        "No sampled ROI-progress candidate passed validation; trying detour candidates");
    validation_detour_announced_ = true;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = costmap_client_.getGlobalFrameID();
  pose.header.stamp = now();
  pose.pose.position = target.position;
  pose.pose.orientation.w = 1.0;
  nav2_msgs::action::ComputePathToPose::Goal goal;
  goal.goal = pose;
  goal.use_start = false;

  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions options;
  options.goal_response_callback = [this, generation](PlanningGoalHandle::SharedPtr handle) {
    if (!validationIsCurrent(generation)) {
      if (handle && rclcpp::ok()) {
        try {
          planner_client_->async_cancel_goal(handle);
        } catch (const std::exception &error) {
          RCLCPP_WARN(logger_, "Could not cancel stale frontier planning: %s", error.what());
        }
      }
      return;
    }
    if (!handle) {
      RCLCPP_WARN(logger_, "Nav2 rejected frontier path validation request");
      validateNextTarget(generation);
      return;
    }
    planning_goal_handle_ = handle;
  };
  options.result_callback = [this, generation, target, pose](
      const PlanningGoalHandle::WrappedResult &result) {
    if (!validationIsCurrent(generation)) {
      return;
    }
    planning_goal_handle_.reset();
    // Preserve the reason: a planner abort is different from rejecting a
    // successful path whose endpoint only satisfies NavFn's tolerance fallback.
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      const char *status = result.code == rclcpp_action::ResultCode::ABORTED ?
          "ABORTED" : result.code == rclcpp_action::ResultCode::CANCELED ?
          "CANCELED" : "UNKNOWN";
      RCLCPP_WARN(logger_,
          "Frontier target (%.2f, %.2f): Nav2 planner returned %s; inspect planner logs for cause",
          target.position.x, target.position.y, status);
      validateNextTarget(generation);
      return;
    }
    if (!result.result) {
      RCLCPP_WARN(logger_,
          "Frontier target (%.2f, %.2f): Nav2 planner reported success without a result",
          target.position.x, target.position.y);
      validateNextTarget(generation);
      return;
    }
    const auto &path = result.result->path;
    if (!pathReachesTarget(path, target.position, pose.header.frame_id,
            frontier_endpoint_tolerance_)) {
      const double endpoint_error = path.poses.empty() ?
          std::numeric_limits<double>::infinity() :
          std::hypot(path.poses.back().pose.position.x - target.position.x,
              path.poses.back().pose.position.y - target.position.y);
      RCLCPP_WARN(logger_,
          "Frontier target (%.2f, %.2f): Nav2 succeeded but path/endpoint validation failed "
          "(poses=%zu, frame='%s', expected='%s', endpoint error=%.3f m); "
          "require nonempty finite path in expected frame and endpoint within %.3f m",
          target.position.x, target.position.y, path.poses.size(),
          path.header.frame_id.c_str(), pose.header.frame_id.c_str(), endpoint_error,
          frontier_endpoint_tolerance_);
      validateNextTarget(generation);
      return;
    }

    // NavFn may return a nearby reachable endpoint. Validate and navigate to
    // that point, retaining the original frontier identity for blacklisting.
    auto navigation_pose = pose;
    const auto &endpoint = path.poses.back().pose.position;
    navigation_pose.pose.position.x = endpoint.x;
    navigation_pose.pose.position.y = endpoint.y;
    navigation_pose.pose.position.z = 0.0;
    const auto &destination = navigation_pose.pose.position;

    // A planner-adjusted endpoint must not slip back into a recently visited region.
    if (isNearRecentGoal(destination, recent_frontier_goals_, frontier_options_.revisit_radius)) {
      RCLCPP_WARN(logger_,
          "Planned frontier endpoint (%.2f, %.2f) is within the revisit radius "
          "of a recent destination; trying next candidate",
          destination.x, destination.y);
      validateNextTarget(generation);
      return;
    }

    // The raw exploration map may have changed while Nav2 was planning.
    // Nav2 clearance still uses the saved selection snapshot; navigation
    // replans against the live costmap after receiving this destination.
    if (!isFrontierTargetValid(*costmap_client_.getCostmap(),
            *validation_costmap_, destination)) {
      RCLCPP_WARN(logger_,
          "Planned frontier endpoint (%.2f, %.2f) failed known-free or costmap clearance "
          "checks against current exploration map and selection costmap snapshot",
          destination.x, destination.y);
      validateNextTarget(generation);
      return;
    }
    geometry_msgs::msg::Pose robot_pose;
    if (!costmap_client_.getRobotPose(robot_pose)) {
      RCLCPP_WARN(logger_, "Deferring frontier dispatch until robot pose is available");
      cancelFrontierValidation();
      validation_retry_after_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      return;
    }
    if (std::chrono::steady_clock::now() > validation_deadline_) {
      finishExploration(false, "Frontier validation batch exceeded its time budget");
      return;
    }
    const auto &robot = robot_pose.position;
    const double target_distance = std::hypot(
        destination.x - robot.x, destination.y - robot.y);
    // Both the endpoint and robot position may have changed during planning.
    // Apply this to progress targets and detours before sending navigation.
    if (!std::isfinite(target_distance) ||
        target_distance < frontier_options_.minimum_distance) {
      RCLCPP_WARN(logger_,
          "Planned frontier endpoint (%.2f, %.2f) failed minimum distance check "
          "(distance=%.3f m, minimum=%.3f m); trying next candidate",
          destination.x, destination.y, target_distance, frontier_options_.minimum_distance);
      validateNextTarget(generation);
      return;
    }
    const auto &roi = target_pose_.pose.position;
    const double current_progress = std::hypot(robot.x - roi.x, robot.y - roi.y) -
        std::hypot(destination.x - roi.x, destination.y - roi.y);
    if (target.tier < 2 && current_progress + 1e-6 < frontier_options_.min_roi_progress) {
      RCLCPP_WARN(logger_,
          "Planned frontier endpoint (%.2f, %.2f) no longer satisfies ROI progress "
          "after planning; trying next candidate",
          destination.x, destination.y);
      validateNextTarget(generation);
      return;
    }
    double path_length = 0.0;
    const auto &poses = path.poses;
    for (size_t i = 1; i < poses.size(); ++i) {
      const auto &a = poses[i - 1].pose.position;
      const auto &b = poses[i].pose.position;
      path_length += std::hypot(b.x - a.x, b.y - a.y);
    }
    const double endpoint_shift = std::hypot(
        destination.x - target.position.x, destination.y - target.position.y);
    const char *tier_names[] = {"local_progress", "distant_progress", "local_detour", "distant_detour"};
    RCLCPP_INFO(logger_,
        "Validated frontier goal (%.2f, %.2f), requested (%.2f, %.2f), shift=%.3f m, "
        "centroid (%.2f, %.2f), path has %zu poses; "
        "tier=%s, ROI progress=%.2f m, target distance=%.2f m, path length=%.2f m, "
        "candidate score=%.2f",
        destination.x, destination.y, target.position.x, target.position.y, endpoint_shift,
        target.frontier_centroid.x, target.frontier_centroid.y, poses.size(),
        tier_names[target.tier], current_progress, target_distance,
        path_length, target.selection_cost);
    cancelFrontierValidation();
    sendNavigationGoal(navigation_pose, false, target.frontier_centroid);
  };
  try {
    planner_client_->async_send_goal(goal, options);
  } catch (const std::exception &error) {
    finishExploration(false, std::string("Could not validate frontier path: ") + error.what());
  }
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
    const geometry_msgs::msg::PoseStamped &pose, bool to_roi,
    const geometry_msgs::msg::Point &frontier_identity)
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
      [this, pose, to_roi, frontier_identity](NavigationGoalHandle::SharedPtr handle) {
        if (!handle) {
          goal_active_ = false;
          final_navigation_ = false;
          navigation_goal_handle_.reset();
          if (!to_roi) {
            frontier_blacklist_.push_back(frontier_identity);
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
      [this, pose, to_roi, frontier_identity](
          const NavigationGoalHandle::WrappedResult &result) {
        if (!explore_to_pose_goal_handle_) {
          return;
        }
        navigation_goal_handle_.reset();

        if (!to_roi) {
          reachedGoal(result, frontier_identity, pose.pose.position);
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
      frontier_blacklist_.push_back(frontier_identity);
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
  cancelFrontierValidation();
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
