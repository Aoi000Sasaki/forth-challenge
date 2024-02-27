#ifndef OBSTACLE_EXPANDER_HPP
#define OBSTACLE_EXPANDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>


// ===== クラス =====
class ObstacleExpander : public rclcpp::Node
{
public:
    ObstacleExpander(); // デフォルトコンストラクタ

private:
    // ----- 関数（引数あり）------
    // コールバック関数
    void map_callback(const nav_msgs::msg::OccupancyGrid& msg);

    // その他の関数
    void   change_surrounding_grid_color(const int occupied_grid_index); // 周囲のグリッドの色の変更(円形状に膨張)
    void   search_rect_grid_index_list(const int center_grid_index);     // 探索対象のグリッドのインデックスを追加
    bool   calc_max_grid_index_in_same_line(const int index);            // 行のインデックスの最大値の算出
    bool   calc_mix_grid_index_in_same_line(const int index);            // 行のインデックスの最小値の算出
    int    find_upper_left_grid_index(const int center_grid_index);      // rect_grid_index_listの左上のインデックスを検索
    int    find_lower_right_grid_index(const int center_grid_index);     // rect_grid_index_listの右下のインデックスを検索
    int    calc_grid_index_x(const int index);                           // グリッドのインデックスからxのインデックスを計算
    int    calc_grid_index_y(const int index);                           // グリッドのインデックスからyのインデックスを計算
    double calc_dist_cell(const int index1, const int index2);           // グリッド同士の距離[cell]を計算


    // ----- 関数（引数なし）------
    void expand_obstacle(); // 障害物の膨張
    void show_exe_time();   // 実行時間を表示（スタート時間beginを予め設定する）


    // ----- 変数 -----
    int    hz_;            // ループ周波数 [Hz]
    double sleep_time_;    // 待機時間（他のノードの起動を待つ）[s]
    double target_margin_; // 車両マージン [m]

    // 占有グリッド周辺のインデックス格納用
    std::vector<int> rect_grid_index_list_;

    // msg受け取りフラグ
    bool flag_map_ = false;

    // 実行時間表示用
    rclcpp::Time begin_;


    // ----- その他のオブジェクト -----

    // Subscriber
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_raw_map_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_updated_map_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // 各種オブジェクト
    nav_msgs::msg::OccupancyGrid raw_map_;     // map_serverから受け取るマップ
    nav_msgs::msg::OccupancyGrid updated_map_; // 障害物を車両マージン分膨張させたマップ
    rclcpp::Clock clock_;
};

#endif
