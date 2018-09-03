#pragma once

#include <Eigen/Dense>
#include <limits.h>

#include "lib/rrt_avoidance/Tree.hpp"

namespace lib {
namespace rrt_avoidance {

// Bi-directional RRT
// This uses two RRTs when searching the state space, with one rooted at the
// source and one rooted at the goal. When the two trees intersect, a solution
// has been found.
class BiRRT {
 public:
  BiRRT(std::shared_ptr<StateSpace<::Eigen::Vector2d>> state_space,
        std::function<size_t(::Eigen::Vector2d)> hash, int dimensions,
        std::function<::Eigen::Vector2d(double *)> array_to_t = NULL,
        std::function<void(::Eigen::Vector2d, double *)> t_to_array = NULL);

  // Get the shortest path from the start to the goal
  std::vector<::Eigen::Vector2d> GetPath();

  // Attempts to add a new node to each of the two trees.  If a new solution is
  // found that is shorter than any previous solution, we store it instead.
  void Grow();

  // Grows the trees until we find a solution or run out of iterations.
  int Run();

  // Reset everything for another calculation.
  void Reset();

  int max_iterations() const { return start_tree_.maxIterations(); }
  void set_max_iterations(int itr) {
    start_tree_.setMaxIterations(itr);
    goal_tree_.setMaxIterations(itr);
  }

  // The minimum number of iterations to run.
  // At the default value of zero, the rrt will return the first path it finds.
  // Setting this to a higher value can allow the tree to search for longer in
  // order to find a better path.
  int min_iterations() const { return min_iterations_; }
  void set_min_iterations(int itr) { min_iterations_ = itr; }

  const std::vector<::Eigen::Vector2d> &waypoints() {
    return start_tree_.waypoints();
  }
  void set_waypoints(const std::vector<::Eigen::Vector2d> &waypoints) {
    start_tree_.setWaypoints(waypoints);
    goal_tree_.setWaypoints(waypoints);
  }

  double step_size() const { return start_tree_.stepSize(); }
  void set_step_size(double step_size) {
    start_tree_.setStepSize(step_size);
    goal_tree_.setStepSize(step_size);
  }

  double max_step_size() const { return start_tree_.maxStepSize(); }
  void set_max_step_size(double stepSize) {
    start_tree_.setMaxStepSize(stepSize);
    goal_tree_.setMaxStepSize(stepSize);
  }

  double goal_max_dist() const { return start_tree_.goalMaxDist(); }
  void set_goal_max_dist(double maxDist) {
    start_tree_.setGoalMaxDist(maxDist);
    goal_tree_.setGoalMaxDist(maxDist);
  }

  const ::Eigen::Vector2d &start_state() const {
    return start_tree_.startState();
  }
  const ::Eigen::Vector2d &goal_state() const {
    return start_tree_.goalState();
  }

  void set_start_state(const ::Eigen::Vector2d &start) {
    start_tree_.setStartState(start);
    goal_tree_.setGoalState(start);
  }

  void set_goal_state(const ::Eigen::Vector2d &goal) {
    start_tree_.setGoalState(goal);
    goal_tree_.setStartState(goal);
  }

  const Node<::Eigen::Vector2d> *start_solution_node() {
    return start_solution_node_;
  }
  const Node<::Eigen::Vector2d> *goal_solution_node() {
    return goal_solution_node_;
  }

  int iteration_count() const { return iteration_count_; }

  bool is_asc_enabled() const { return start_tree_.isASCEnabled(); }
  void set_asc_enabled(bool checked) {
    start_tree_.setASCEnabled(checked);
    goal_tree_.setASCEnabled(checked);
  }

  double goal_bias() const { return start_tree_.goalBias(); }
  void set_goal_bias(double goalBias) {
    start_tree_.setGoalBias(goalBias);
    goal_tree_.setGoalBias(goalBias);
  }

 private:
  const Node<::Eigen::Vector2d> *
  FindBestPath(const ::Eigen::Vector2d &target_state,
               Tree<::Eigen::Vector2d> &tree_to_search, int *depth_out) const;

  Tree<::Eigen::Vector2d> start_tree_, goal_tree_;
  const Node<::Eigen::Vector2d> *start_solution_node_, *goal_solution_node_;

  int iteration_count_;
  int min_iterations_;

  int solution_length_;
};

} // namespace rrt_avoidance
} // namespace lib
