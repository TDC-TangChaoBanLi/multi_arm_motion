#include <rclcpp/rclcpp.hpp>
#include <moveit/global_planner/global_planner_component.hpp>
#include <moveit/local_planner/local_planner_component.hpp>
#include <moveit/hybrid_planning_manager/hybrid_planning_manager.hpp>

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  try
  {
    // Avoid process-wide __node remaps: each upstream wrapper owns its own node.
    const auto options=rclcpp::NodeOptions();
    auto global=std::make_shared<moveit::hybrid_planning::GlobalPlannerComponent>(options);
    auto local=std::make_shared<moveit::hybrid_planning::LocalPlannerComponent>(options);
    auto manager=std::make_shared<moveit::hybrid_planning::HybridPlanningManager>(options);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),4);
    executor.add_node(global->get_node_base_interface());
    executor.add_node(local->get_node_base_interface());
    executor.add_node(manager->get_node_base_interface());
    RCLCPP_INFO(rclcpp::get_logger("hybrid_runtime"),"Global, local and manager nodes initialized; spinning executor");
    executor.spin();
  }
  catch(const std::exception& e)
  {RCLCPP_FATAL(rclcpp::get_logger("hybrid_runtime"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
