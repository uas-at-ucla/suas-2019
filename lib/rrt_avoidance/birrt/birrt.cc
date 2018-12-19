#include "birrt.h"

namespace lib {
namespace rrt_avoidance {

BiRRT::BiRRT(::std::shared_ptr<StateSpace<::Eigen::Vector2d>> state_space,
             ::std::function<size_t(::Eigen::Vector2d)> hash, int dimensions,
             ::std::function<::Eigen::Vector2d(double *)> array_to_t,
             ::std::function<void(::Eigen::Vector2d, double *)> t_to_array) :
    start_tree_(state_space, hash, dimensions, array_to_t, t_to_array),
    goal_tree_(state_space, hash, dimensions, array_to_t, t_to_array),
    min_iterations_(0) {
  Reset();
}

void BiRRT::Reset() {
  start_tree_.reset();
  goal_tree_.reset();

  iteration_count_ = 0;

  start_solution_node_ = nullptr;
  goal_solution_node_ = nullptr;
  solution_length_ = INT_MAX;
}

::std::vector<::Eigen::Vector2d> BiRRT::GetPath() {
  ::std::vector<::Eigen::Vector2d> path;
  start_tree_.getPath(&path, start_solution_node_);
  start_tree_.getPath(&path, goal_solution_node_, true);

  return path;
}

void BiRRT::Grow() {
  int depth;
  const Node<::Eigen::Vector2d> *other_node;

  Node<::Eigen::Vector2d> *new_start_node = start_tree_.grow();
  if (new_start_node) {
    other_node = FindBestPath(new_start_node->state(), goal_tree_, &depth);
    if (other_node && depth + new_start_node->depth() < solution_length_ &&
        goal_tree_.stateSpace().transitionValid(new_start_node->state(),
                                                other_node->state())) {
      start_solution_node_ = new_start_node;
      goal_solution_node_ = other_node;
      solution_length_ = new_start_node->depth() + depth;
    }
  }

  Node<::Eigen::Vector2d> *new_goal_node = goal_tree_.grow();
  if (new_goal_node) {
    other_node = FindBestPath(new_goal_node->state(), start_tree_, &depth);
    if (other_node && depth + new_goal_node->depth() < solution_length_ &&
        goal_tree_.stateSpace().transitionValid(new_goal_node->state(),
                                                other_node->state())) {
      start_solution_node_ = other_node;
      goal_solution_node_ = new_goal_node;
      solution_length_ = new_goal_node->depth() + depth;
    }
  }

  ++iteration_count_;
}

int BiRRT::Run() {
  for (int i = 0; i < start_tree_.maxIterations(); i++) {
    Grow();
    if (start_solution_node_ != nullptr && i >= min_iterations_) {
      return i;
    }
  }

  return -1;
}

const Node<::Eigen::Vector2d> *
BiRRT::FindBestPath(const ::Eigen::Vector2d &target_state,
                    Tree<::Eigen::Vector2d> &tree_to_search,
                    int *depth_out) const {
  const Node<::Eigen::Vector2d> *best_node = nullptr;
  int depth = INT_MAX;

  for (const Node<::Eigen::Vector2d> &other : tree_to_search.allNodes()) {
    double dist =
        start_tree_.stateSpace().distance(other.state(), target_state);
    if (dist < goal_max_dist() && other.depth() < depth) {
      best_node = &other;
      depth = other.depth();
    }
  }

  if (depth_out)
    *depth_out = depth;

  return best_node;
}

} // namespace rrt_avoidance
} // namespace lib
