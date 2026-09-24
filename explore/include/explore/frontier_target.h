#ifndef EXPLORE_FRONTIER_TARGET_H_
#define EXPLORE_FRONTIER_TARGET_H_

#include <cstddef>
#include <string>
#include <vector>

#include <explore/frontier_search.h>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/path.hpp>

namespace explore
{

struct FrontierTarget
{
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Point frontier_centroid;
  double robot_distance{0.0};
  double roi_distance{0.0};
  double roi_progress{0.0};
  double selection_cost{0.0};
  unsigned int tier{0};
};

struct DirectedFrontierOptions
{
  double minimum_distance{2.0};
  double revisit_radius{2.5};
  double preferred_distance{20.0};
  double min_roi_progress{0.5};
  double potential_scale{3.0};
  double gain_scale{1.0};
  double roi_weight{10.0};
  double max_frontier_gain{5.0};
  bool allow_backtracking{true};
  std::size_t candidates_per_frontier{3};
  std::size_t candidates_per_tier{6};
  std::size_t max_candidates{24};
};

// Select known-free frontier neighbours cleared by Nav2's inflated costmap.
// The input order preserves frontier ranking; candidates are closest to the
// centroid within each frontier and distinct at navigation-map resolution.
std::vector<FrontierTarget> frontierTargets(
  nav2_costmap_2d::Costmap2D & raw,
  const nav2_msgs::msg::Costmap & navigation_map,
  const std::vector<frontier_exploration::Frontier> & ranked_frontiers,
  std::size_t candidates_per_frontier = 3,
  std::size_t max_candidates = 24);

// Rank actual approach cells in four tiers: local progress, distant progress,
// local detour, distant detour. Consider every frontier before imposing limits.
// Exclude points closer than minimum_distance and points near recent goals before
// deduplication and candidate limits. A zero revisit_radius disables that filter.
// revisit_excluded counts distinct eligible raw approach cells rejected by it.
// Invalid maps, points or policy values produce an empty candidate list.
std::vector<FrontierTarget> directedFrontierTargets(
  nav2_costmap_2d::Costmap2D & raw,
  const nav2_msgs::msg::Costmap & navigation_map,
  const std::vector<frontier_exploration::Frontier> & ranked_frontiers,
  const geometry_msgs::msg::Point & robot_point,
  const geometry_msgs::msg::Point & roi_point,
  const DirectedFrontierOptions & options = DirectedFrontierOptions(),
  const std::vector<geometry_msgs::msg::Point> & recent_goals = {},
  std::size_t * revisit_excluded = nullptr);

// Inclusive XY-radius check, also used to recheck a planner-adjusted endpoint.
// Zero or invalid radii disable the check; nonfinite XY points are ignored.
bool isNearRecentGoal(
  const geometry_msgs::msg::Point & point,
  const std::vector<geometry_msgs::msg::Point> & recent_goals,
  double radius);

bool isFrontierTargetValid(
  nav2_costmap_2d::Costmap2D & raw,
  const nav2_msgs::msg::Costmap & navigation_map,
  const geometry_msgs::msg::Point & point);

// Describe only the robot's nearby cells in this global costmap snapshot.
// Includes NavFn Humble's rounded cell alongside the containing (floor) cell.
// This is diagnostic information, not a route reachability determination.
std::string describeNavigationStart(
  const nav2_msgs::msg::Costmap & navigation_map,
  const geometry_msgs::msg::Point & robot_point);

// Evidence of an obstructed start in a saved Nav2 costmap, not a reachability
// test. Check both containing and Humble NavFn-rounded cells and the rounded
// cell's four immediate neighbours. Unknown or out-of-map start cells and
// malformed inputs are inconclusive and return false. Callers must still let
// Nav2 check whether a recovery movement is collision free.
bool navigationStartAppearsBlocked(
  const nav2_msgs::msg::Costmap & navigation_map,
  const geometry_msgs::msg::Point & robot_point);

// Check path validity and its endpoint's distance from the requested target.
bool pathReachesTarget(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & target,
  const std::string & frame,
  double endpoint_tolerance = 1e-3);

}  // namespace explore

#endif  // EXPLORE_FRONTIER_TARGET_H_
