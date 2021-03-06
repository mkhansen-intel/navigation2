// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <chrono>
#include "nav2_astar_planner/astar_planner.hpp"

using namespace std::chrono_literals;
using nav2_tasks::TaskStatus;

namespace nav2_astar_planner
{

AStarPlanner::AStarPlanner()
: nav2_tasks::ComputePathToPoseTaskServer("ComputePathToPoseNode")
{
  RCLCPP_INFO(get_logger(), "Initializing AStarPlanner");
}

AStarPlanner::~AStarPlanner()
{
  RCLCPP_INFO(get_logger(), "Shutting down AStarPlanner");
}

TaskStatus
AStarPlanner::execute(const nav2_tasks::ComputePathToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "AStarPlanner: Attempting to a find path from (%.2f, %.2f) to "
    "(%.2f, %.2f).",command->start.position.x, command->start.position.y,
    command->goal.position.x, command->goal.position.y);

  // Spin here for a bit to fake out some processing time
  for (int i = 0; i < 10; i++) {
    // Do a bit of the task
    RCLCPP_DEBUG(get_logger(), "AStarPlanner::execute: doing work: %d", i);
    std::this_thread::sleep_for(100ms);

    // if (failed_to_plan)
    // {
    //   RCLCPP_WARN(get_logger(), "AStarPlanner: Failed to generate path.");
    // }

    // Before we loop again to do more work, check if we've been canceled
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "AStarPlanner: Cancelled global planning task.");
      setCanceled();
      return TaskStatus::CANCELED;
    }
  }

  // We've successfully completed the task, so return the result
  RCLCPP_INFO(get_logger(), "AStarPlanner: Successfully navigated to (%.2f, %.2f) with tolerance %.2f",
    command->goal.position.x, command->goal.position.y, command->tolerance);

  nav2_tasks::ComputePathToPoseResult result;
  setResult(result);

  return TaskStatus::SUCCEEDED;
}

}  // namespace nav2_astar_planner
