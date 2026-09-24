#include <explore/frontier_target.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include <nav2_costmap_2d/cost_values.hpp>

namespace explore
{
namespace
{

bool finitePoint(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool validRawMap(const nav2_costmap_2d::Costmap2D & map)
{
  return map.getSizeInCellsX() > 0 && map.getSizeInCellsY() > 0 &&
         std::isfinite(map.getResolution()) && map.getResolution() > 0.0 &&
         std::isfinite(map.getOriginX()) && std::isfinite(map.getOriginY());
}

bool validNavigationMap(const nav2_msgs::msg::Costmap & map)
{
  const auto & metadata = map.metadata;
  const auto & origin = metadata.origin;
  const auto & q = origin.orientation;
  // Costmap2D grids are axis aligned. Reject malformed or rotated messages
  // rather than silently checking a different cell from the planner.
  constexpr double epsilon = 1e-6;
  if (metadata.size_x == 0 || metadata.size_y == 0 ||
    !std::isfinite(metadata.resolution) || metadata.resolution <= 0.0 ||
    !finitePoint(origin.position) || !std::isfinite(origin.position.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
    !std::isfinite(q.w) || std::abs(q.x) > epsilon ||
    std::abs(q.y) > epsilon || std::abs(q.z) > epsilon ||
    std::abs(std::abs(q.w) - 1.0) > epsilon)
  {
    return false;
  }
  // Check division before multiplication to avoid overflow on malformed sizes.
  const auto width = static_cast<std::size_t>(metadata.size_x);
  const auto height = static_cast<std::size_t>(metadata.size_y);
  return width <= map.data.size() / height && width * height == map.data.size();
}

bool worldToCell(
  const geometry_msgs::msg::Point & point, double origin_x, double origin_y,
  double resolution, unsigned int width, unsigned int height,
  unsigned int & x, unsigned int & y)
{
  if (!finitePoint(point)) {
    return false;
  }
  const double grid_x = (point.x - origin_x) / resolution;
  const double grid_y = (point.y - origin_y) / resolution;
  // Validate before converting to integers (NaN and huge values are unsafe).
  if (!std::isfinite(grid_x) || !std::isfinite(grid_y) ||
    grid_x < 0.0 || grid_y < 0.0 || grid_x >= width || grid_y >= height)
  {
    return false;
  }
  x = static_cast<unsigned int>(std::floor(grid_x));
  y = static_cast<unsigned int>(std::floor(grid_y));
  return true;
}

bool validTargetUnlocked(
  const nav2_costmap_2d::Costmap2D & raw,
  const nav2_msgs::msg::Costmap & navigation_map,
  const geometry_msgs::msg::Point & point, std::size_t & navigation_index)
{
  unsigned int raw_x, raw_y, navigation_x, navigation_y;
  const auto & metadata = navigation_map.metadata;
  if (!worldToCell(
      point, raw.getOriginX(), raw.getOriginY(), raw.getResolution(),
      raw.getSizeInCellsX(), raw.getSizeInCellsY(), raw_x, raw_y) ||
    raw.getCost(raw_x, raw_y) != nav2_costmap_2d::FREE_SPACE ||
    !worldToCell(
      point, metadata.origin.position.x, metadata.origin.position.y,
      metadata.resolution, metadata.size_x, metadata.size_y,
      navigation_x, navigation_y))
  {
    return false;
  }
  navigation_index = static_cast<std::size_t>(navigation_y) * metadata.size_x +
    navigation_x;
  return navigation_map.data[navigation_index] <
         nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

struct Candidate
{
  geometry_msgs::msg::Point point;
  double distance;
  std::size_t navigation_index;
};

bool candidateLess(const Candidate & a, const Candidate & b)
{
  if (a.distance != b.distance) {
    return a.distance < b.distance;
  }
  if (a.point.x != b.point.x) {
    return a.point.x < b.point.x;
  }
  return a.point.y < b.point.y;
}

bool validDirectedOptions(const DirectedFrontierOptions & options)
{
  const std::array<double, 8> nonnegative_values{{
    options.minimum_distance, options.revisit_radius, options.preferred_distance,
    options.min_roi_progress, options.potential_scale,
    options.gain_scale, options.roi_weight, options.max_frontier_gain}};
  return std::all_of(
    nonnegative_values.begin(), nonnegative_values.end(),
    [](double value) {return std::isfinite(value) && value >= 0.0;}) &&
         options.candidates_per_frontier > 0 && options.candidates_per_tier > 0 &&
         options.max_candidates > 0;
}

struct DirectedCandidate
{
  FrontierTarget target;
  std::size_t frontier_index;
};

bool directedCandidateLess(const DirectedCandidate & a, const DirectedCandidate & b)
{
  if (a.target.tier != b.target.tier) {
    return a.target.tier < b.target.tier;
  }
  if (a.target.selection_cost != b.target.selection_cost) {
    return a.target.selection_cost < b.target.selection_cost;
  }
  if (a.target.position.x != b.target.position.x) {
    return a.target.position.x < b.target.position.x;
  }
  if (a.target.position.y != b.target.position.y) {
    return a.target.position.y < b.target.position.y;
  }
  if (a.target.frontier_centroid.x != b.target.frontier_centroid.x) {
    return a.target.frontier_centroid.x < b.target.frontier_centroid.x;
  }
  if (a.target.frontier_centroid.y != b.target.frontier_centroid.y) {
    return a.target.frontier_centroid.y < b.target.frontier_centroid.y;
  }
  return a.frontier_index < b.frontier_index;
}

}  // namespace

std::string describeNavigationStart(
  const nav2_msgs::msg::Costmap & navigation_map,
  const geometry_msgs::msg::Point & robot_point)
{
  const auto & metadata = navigation_map.metadata;
  const auto & origin = metadata.origin.position;
  std::ostringstream description;
  description << std::setprecision(9) << "Global costmap snapshot: frame=" <<
    navigation_map.header.frame_id << ", size=" << metadata.size_x << "x" <<
    metadata.size_y << ", resolution=" << metadata.resolution << ", origin=(" <<
    origin.x << "," << origin.y << "), robot=(" << robot_point.x << "," <<
    robot_point.y << "); ";
  if (!validNavigationMap(navigation_map)) {
    description << "invalid costmap metadata or data size; no cells inspected";
    return description.str();
  }
  if (!finitePoint(robot_point)) {
    description << "invalid robot XY position; no cells inspected";
    return description.str();
  }
  unsigned int x, y;
  if (!worldToCell(
      robot_point, origin.x, origin.y, metadata.resolution,
      metadata.size_x, metadata.size_y, x, y))
  {
    description << "robot position outside costmap bounds; no cells inspected";
    return description.str();
  }
  const auto describe_cost = [&](unsigned int cell_x, unsigned int cell_y) {
      const auto cost = navigation_map.data[
        static_cast<std::size_t>(cell_y) * metadata.size_x + cell_x];
      description << static_cast<unsigned int>(cost);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        description << " (unknown)";
      } else if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        description << " (lethal)";
      } else if (cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        description << " (inscribed)";
      } else if (cost == nav2_costmap_2d::FREE_SPACE) {
        description << " (free)";
      }
    };
  description << "containing cell=(" << x << "," << y << ") cost=";
  describe_cost(x, y);

  // NavFn Humble rounds world coordinates, unlike Costmap2D's floor conversion.
  // Validate the rounded doubles before converting to unsigned cell indices.
  const double rounded_x = std::round((robot_point.x - origin.x) / metadata.resolution);
  const double rounded_y = std::round((robot_point.y - origin.y) / metadata.resolution);
  description << "; NavFn-rounded cell=(" << rounded_x << "," << rounded_y << ")";
  if (rounded_x >= metadata.size_x || rounded_y >= metadata.size_y) {
    description << " outside costmap bounds";
  } else {
    description << " cost=";
    describe_cost(static_cast<unsigned int>(rounded_x), static_cast<unsigned int>(rounded_y));
  }

  description << "; containing-cell 3x3 [y+1 / y / y-1, x-1..x+1]=";
  for (int dy = 1; dy >= -1; --dy) {
    description << (dy == 1 ? "[" : " / ");
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx != -1) {
        description << ",";
      }
      const auto cell_x = static_cast<int64_t>(x) + dx;
      const auto cell_y = static_cast<int64_t>(y) + dy;
      if (cell_x < 0 || cell_y < 0 || cell_x >= metadata.size_x || cell_y >= metadata.size_y) {
        description << "OOB";
      } else {
        description << static_cast<unsigned int>(navigation_map.data[
          static_cast<std::size_t>(cell_y) * metadata.size_x + static_cast<std::size_t>(cell_x)]);
      }
    }
  }
  description << "]; 0=free, 253=inscribed, 254=lethal, 255=unknown; " <<
    "local snapshot only, not a reachability test";
  return description.str();
}

bool navigationStartAppearsBlocked(
  const nav2_msgs::msg::Costmap & navigation_map,
  const geometry_msgs::msg::Point & robot_point)
{
  const auto & metadata = navigation_map.metadata;
  const auto & origin = metadata.origin.position;
  unsigned int containing_x, containing_y;
  if (!validNavigationMap(navigation_map) || !std::isfinite(robot_point.z) ||
    !worldToCell(
      robot_point, origin.x, origin.y, metadata.resolution,
      metadata.size_x, metadata.size_y, containing_x, containing_y))
  {
    return false;
  }

  // Humble NavFn rounds coordinates instead of selecting the containing cell.
  const double rounded_x = std::round((robot_point.x - origin.x) / metadata.resolution);
  const double rounded_y = std::round((robot_point.y - origin.y) / metadata.resolution);
  if (rounded_x < 0.0 || rounded_y < 0.0 ||
    rounded_x >= metadata.size_x || rounded_y >= metadata.size_y)
  {
    return false;
  }
  const auto x = static_cast<unsigned int>(rounded_x);
  const auto y = static_cast<unsigned int>(rounded_y);
  const auto cost = [&](unsigned int cell_x, unsigned int cell_y) {
      return navigation_map.data[
        static_cast<std::size_t>(cell_y) * metadata.size_x + cell_x];
    };
  const auto blocked = [](unsigned char value) {
      return value == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
             value == nav2_costmap_2d::LETHAL_OBSTACLE;
    };
  const auto containing_cost = cost(containing_x, containing_y);
  const auto rounded_cost = cost(x, y);
  // Missing start information is not evidence that an automatic retreat helps.
  if (containing_cost == nav2_costmap_2d::NO_INFORMATION ||
    rounded_cost == nav2_costmap_2d::NO_INFORMATION)
  {
    return false;
  }
  if (blocked(containing_cost) || blocked(rounded_cost)) {
    return true;
  }

  // NavFn can clear its start cell yet remain unable to propagate out of it.
  // Map boundaries or unknown neighbours alone must not trigger recovery.
  if (x == 0 || y == 0 || x + 1 >= metadata.size_x || y + 1 >= metadata.size_y) {
    return false;
  }
  return blocked(cost(x - 1, y)) && blocked(cost(x + 1, y)) &&
         blocked(cost(x, y - 1)) && blocked(cost(x, y + 1));
}

std::vector<FrontierTarget> frontierTargets(
  nav2_costmap_2d::Costmap2D & raw,
  const nav2_msgs::msg::Costmap & navigation_map,
  const std::vector<frontier_exploration::Frontier> & ranked_frontiers,
  std::size_t candidates_per_frontier, std::size_t max_candidates)
{
  std::vector<FrontierTarget> result;
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*raw.getMutex());
  if (!validRawMap(raw) || !validNavigationMap(navigation_map) ||
    candidates_per_frontier == 0 || max_candidates == 0)
  {
    return result;
  }

  std::unordered_set<std::size_t> emitted_cells;
  const auto width = raw.getSizeInCellsX();
  const auto height = raw.getSizeInCellsY();
  for (const auto & frontier : ranked_frontiers) {
    if (!finitePoint(frontier.centroid)) {
      continue;
    }
    std::unordered_map<std::size_t, Candidate> unique_candidates;
    const auto add_neighbour = [&](unsigned int x, unsigned int y) {
        geometry_msgs::msg::Point point;
        raw.mapToWorld(x, y, point.x, point.y);
        std::size_t navigation_index;
        if (!validTargetUnlocked(raw, navigation_map, point, navigation_index) ||
          emitted_cells.count(navigation_index) != 0)
        {
          return;
        }
        Candidate candidate{
          point, std::hypot(point.x - frontier.centroid.x, point.y - frontier.centroid.y),
          navigation_index};
        if (!std::isfinite(candidate.distance)) {
          return;
        }
        const auto existing = unique_candidates.find(navigation_index);
        if (existing == unique_candidates.end() || candidateLess(candidate, existing->second)) {
          unique_candidates[navigation_index] = candidate;
        }
      };

    for (const auto & point : frontier.points) {
      unsigned int x, y;
      if (!worldToCell(
          point, raw.getOriginX(), raw.getOriginY(), raw.getResolution(),
          width, height, x, y) || raw.getCost(x, y) != nav2_costmap_2d::NO_INFORMATION)
      {
        continue;
      }
      if (x > 0) {
        add_neighbour(x - 1, y);
      }
      if (x < width - 1) {
        add_neighbour(x + 1, y);
      }
      if (y > 0) {
        add_neighbour(x, y - 1);
      }
      if (y < height - 1) {
        add_neighbour(x, y + 1);
      }
    }

    std::vector<Candidate> candidates;
    candidates.reserve(unique_candidates.size());
    for (const auto & entry : unique_candidates) {
      candidates.push_back(entry.second);
    }
    std::sort(candidates.begin(), candidates.end(), candidateLess);
    const auto count = std::min(candidates_per_frontier, candidates.size());
    for (std::size_t i = 0; i < count && result.size() < max_candidates; ++i) {
      result.push_back(FrontierTarget{candidates[i].point, frontier.centroid});
      emitted_cells.insert(candidates[i].navigation_index);
    }
    if (result.size() == max_candidates) {
      break;
    }
  }
  return result;
}

std::vector<FrontierTarget> directedFrontierTargets(
  nav2_costmap_2d::Costmap2D & raw,
  const nav2_msgs::msg::Costmap & navigation_map,
  const std::vector<frontier_exploration::Frontier> & ranked_frontiers,
  const geometry_msgs::msg::Point & robot_point,
  const geometry_msgs::msg::Point & roi_point,
  const DirectedFrontierOptions & options,
  const std::vector<geometry_msgs::msg::Point> & recent_goals,
  std::size_t * revisit_excluded)
{
  std::vector<FrontierTarget> result;
  if (revisit_excluded) {
    *revisit_excluded = 0;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*raw.getMutex());
  if (!validRawMap(raw) || !validNavigationMap(navigation_map) ||
    !validDirectedOptions(options) || !finitePoint(robot_point) ||
    !std::isfinite(robot_point.z) || !finitePoint(roi_point) || !std::isfinite(roi_point.z))
  {
    return result;
  }
  const double initial_roi_distance =
    std::hypot(robot_point.x - roi_point.x, robot_point.y - roi_point.y);
  if (!std::isfinite(initial_roi_distance)) {
    return result;
  }

  // Deduplicate only after computing tier and score. Neither legacy frontier
  // order nor a limit applied to that order may hide a forward approach cell.
  std::unordered_map<std::size_t, DirectedCandidate> unique_candidates;
  std::unordered_set<std::size_t> excluded_raw_cells;
  const auto width = raw.getSizeInCellsX();
  const auto height = raw.getSizeInCellsY();
  for (std::size_t frontier_index = 0; frontier_index < ranked_frontiers.size();
    ++frontier_index)
  {
    const auto & frontier = ranked_frontiers[frontier_index];
    if (!finitePoint(frontier.centroid) || !std::isfinite(frontier.centroid.z)) {
      continue;
    }
    const double gain = std::min(
      static_cast<double>(frontier.size) * raw.getResolution(), options.max_frontier_gain);
    const auto add_neighbour = [&](unsigned int x, unsigned int y) {
        FrontierTarget target;
        raw.mapToWorld(x, y, target.position.x, target.position.y);
        std::size_t navigation_index;
        if (!validTargetUnlocked(raw, navigation_map, target.position, navigation_index))
        {
          return;
        }
        target.frontier_centroid = frontier.centroid;
        target.robot_distance = std::hypot(
          target.position.x - robot_point.x, target.position.y - robot_point.y);
        // Skip individual nearby cells before ranking or consuming any slots.
        // Other approach points on this same frontier may still be useful.
        if (!std::isfinite(target.robot_distance) ||
          target.robot_distance < options.minimum_distance)
        {
          return;
        }
        target.roi_distance = std::hypot(
          target.position.x - roi_point.x, target.position.y - roi_point.y);
        target.roi_progress = initial_roi_distance - target.roi_distance;
        const bool progresses = target.roi_progress >= options.min_roi_progress;
        if (!progresses && !options.allow_backtracking) {
          return;
        }
        target.tier = (progresses ? 0u : 2u) +
          (target.robot_distance <= options.preferred_distance ? 0u : 1u);
        target.selection_cost = options.potential_scale * target.robot_distance +
          options.roi_weight * target.roi_distance - options.gain_scale * gain;
        if (!std::isfinite(target.robot_distance) || !std::isfinite(target.roi_distance) ||
          !std::isfinite(target.roi_progress) || !std::isfinite(target.selection_cost))
        {
          return;
        }
        // A successful approach remains in the map until sensing changes the
        // frontier. Do not let its nearby cells consume another sampling slot.
        if (isNearRecentGoal(target.position, recent_goals, options.revisit_radius)) {
          if (revisit_excluded) {
            excluded_raw_cells.insert(raw.getIndex(x, y));
          }
          return;
        }
        const DirectedCandidate candidate{target, frontier_index};
        const auto existing = unique_candidates.find(navigation_index);
        if (existing == unique_candidates.end() ||
          directedCandidateLess(candidate, existing->second))
        {
          unique_candidates[navigation_index] = candidate;
        }
      };
    for (const auto & cell : frontier.points) {
      unsigned int x, y;
      if (!std::isfinite(cell.z) || !worldToCell(
          cell, raw.getOriginX(), raw.getOriginY(), raw.getResolution(),
          width, height, x, y) || raw.getCost(x, y) != nav2_costmap_2d::NO_INFORMATION)
      {
        continue;
      }
      if (x > 0) {
        add_neighbour(x - 1, y);
      }
      if (x < width - 1) {
        add_neighbour(x + 1, y);
      }
      if (y > 0) {
        add_neighbour(x, y - 1);
      }
      if (y < height - 1) {
        add_neighbour(x, y + 1);
      }
    }
  }

  if (revisit_excluded) {
    *revisit_excluded = excluded_raw_cells.size();
  }
  std::vector<DirectedCandidate> candidates;
  candidates.reserve(unique_candidates.size());
  for (const auto & entry : unique_candidates) {
    candidates.push_back(entry.second);
  }
  std::sort(candidates.begin(), candidates.end(), directedCandidateLess);
  std::array<std::vector<FrontierTarget>, 4> tiers;
  std::vector<std::array<std::size_t, 4>> frontier_counts(ranked_frontiers.size());
  for (const auto & candidate : candidates) {
    const auto tier = candidate.target.tier;
    if (frontier_counts[candidate.frontier_index][tier] >= options.candidates_per_frontier) {
      continue;
    }
    tiers[tier].push_back(candidate.target);
    ++frontier_counts[candidate.frontier_index][tier];
  }

  // Reserve a sampling allowance for each tier, then give unused capacity to
  // earlier tiers. A map with only forward local approaches can use the entire
  // budget instead of failing after the first per-tier allowance.
  std::array<std::size_t, 4> tier_counts{};
  std::size_t remaining = options.max_candidates;
  for (std::size_t tier = 0; tier < tiers.size(); ++tier) {
    tier_counts[tier] = std::min({options.candidates_per_tier, tiers[tier].size(), remaining});
    remaining -= tier_counts[tier];
  }
  for (std::size_t tier = 0; tier < tiers.size(); ++tier) {
    const auto extra = std::min(tiers[tier].size() - tier_counts[tier], remaining);
    tier_counts[tier] += extra;
    remaining -= extra;
    result.insert(result.end(), tiers[tier].begin(), tiers[tier].begin() + tier_counts[tier]);
  }
  return result;
}

bool isNearRecentGoal(
  const geometry_msgs::msg::Point & point,
  const std::vector<geometry_msgs::msg::Point> & recent_goals,
  double radius)
{
  if (!std::isfinite(radius) || radius <= 0.0 || !finitePoint(point)) {
    return false;
  }
  return std::any_of(recent_goals.begin(), recent_goals.end(),
    [&](const geometry_msgs::msg::Point & recent) {
      return finitePoint(recent) &&
             std::hypot(point.x - recent.x, point.y - recent.y) <= radius;
    });
}

bool isFrontierTargetValid(
  nav2_costmap_2d::Costmap2D & raw,
  const nav2_msgs::msg::Costmap & navigation_map,
  const geometry_msgs::msg::Point & point)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*raw.getMutex());
  std::size_t navigation_index;
  return validRawMap(raw) && validNavigationMap(navigation_map) &&
         validTargetUnlocked(raw, navigation_map, point, navigation_index);
}

bool pathReachesTarget(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & target,
  const std::string & frame,
  double endpoint_tolerance)
{
  if (!std::isfinite(endpoint_tolerance) || endpoint_tolerance < 0.0 ||
      frame.empty() || path.header.frame_id != frame ||
      path.poses.empty() || !finitePoint(target))
  {
    return false;
  }

  for (const auto & pose : path.poses) {
    if ((!pose.header.frame_id.empty() &&
         pose.header.frame_id != frame) ||
        !finitePoint(pose.pose.position))
    {
      return false;
    }
  }

  const auto & endpoint = path.poses.back().pose.position;

  return std::hypot(
      endpoint.x - target.x,
      endpoint.y - target.y) <= endpoint_tolerance;
}

}  // namespace explore
