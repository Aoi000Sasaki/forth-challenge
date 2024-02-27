#include "global_path_planner/obstacle_expander.hpp"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); // ノードの初期化
    rclcpp::spin(std::make_shared<ObstacleExpander>()); // ノードの実行
    rclcpp::shutdown(); // ノードの終了
    return 0;
}
