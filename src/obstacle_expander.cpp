#include "global_path_planner/obstacle_expander.hpp"

// コンストラクタ
ObstacleExpander::ObstacleExpander() : Node("obstacle_expander")
{
    // パラメータの取得
    hz_ = declare_parameter("hz", 10);
    sleep_time_ = declare_parameter("sleep_time", 1.5);
    target_margin_ = declare_parameter("target_margin", 0.25);

    // Subscriber
    sub_raw_map_  = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable(), std::bind(&ObstacleExpander::map_callback, this, std::placeholders::_1));

    // Publisher
    pub_updated_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map/updated_map", 1);

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0/hz_), std::bind(&ObstacleExpander::expand_obstacle, this));
}

// mapのコールバック関数
void ObstacleExpander::map_callback(const nav_msgs::msg::OccupancyGrid& msg)
{
    raw_map_  = msg;
    flag_map_ = true;
    std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time_));
}


// 障害物を膨張
void ObstacleExpander::expand_obstacle()
{
    if (!flag_map_) return; // mapの取得が完了していない場合は終了

    RCLCPP_INFO(get_logger(), "----- Obstacle Expander will begin ------");
    begin_ = clock_.now(); // 実行時間のスタートを設定
    updated_map_ = raw_map_;

    const int size = raw_map_.data.size();
    for(int index=0; index<size; index++)
        if(raw_map_.data[index] == 100) // 「占有」のとき
            change_surrounding_grid_color(index); // 周囲のグリッドの色の変更

    pub_updated_map_->publish(updated_map_);
    show_exe_time(); // 実行時間を表示
    exit(0); // ノード終了
}

// 周囲のグリッドの色を変更（円形状に膨張）
void ObstacleExpander::change_surrounding_grid_color(const int occupied_grid_index)
{
    search_rect_grid_index_list(occupied_grid_index); // 探索対象のグリッドのインデックスを追加
    const int size = rect_grid_index_list_.size();
    const int target_margin_cell = int(round(target_margin_/updated_map_.info.resolution)); // 車両マージン [cell]

    for(int i=0; i<size; i++)
    {
        const int    index = rect_grid_index_list_[i];
        const double dist  = calc_dist_cell(occupied_grid_index, index);
        if(dist <= target_margin_cell) // マージン内のセルの場合
            updated_map_.data[index] = 100;
    }
}

// 特定の占有グリッドを中心とした一辺の長さが車両マージンの2倍の長方形を構成するインデックスのリストを検索
void ObstacleExpander::search_rect_grid_index_list(const int center_grid_index)
{
    rect_grid_index_list_.clear();

    const int upper_left_grid_index    = find_upper_left_grid_index(center_grid_index);
    const int upper_left_grid_index_x  = calc_grid_index_x(upper_left_grid_index);
    const int upper_left_grid_index_y  = calc_grid_index_y(upper_left_grid_index);

    const int lower_right_grid_index   = find_lower_right_grid_index(center_grid_index);
    const int lower_right_grid_index_x = calc_grid_index_x(lower_right_grid_index);
    const int lower_right_grid_index_y = calc_grid_index_y(lower_right_grid_index);

    const int width  = lower_right_grid_index_x - upper_left_grid_index_x + 1;
    const int height = lower_right_grid_index_y - upper_left_grid_index_y + 1;

    int index = upper_left_grid_index;
    for(int i=0; i<height; i++)
    {
        for(int j=0; j<width; j++)
        {
            index++;
            rect_grid_index_list_.push_back(index);
        }
        index = upper_left_grid_index + updated_map_.info.width*(i+1);
    }
}

// rect_grid_index_listの左上のインデックスを検索
int ObstacleExpander::find_upper_left_grid_index(const int center_grid_index)
{
    int index = center_grid_index;
    const int target_margin_cell = int(round(target_margin_/updated_map_.info.resolution)); // 車両マージン [cell]

    // 上方向に検索位置を移動
    for(int i=0; i<target_margin_cell; i++)
    {
        index -= updated_map_.info.width;
        if(index < 0)
        {
            index += updated_map_.info.width;
            break;
        }
    }

    // 左方向に検索位置を移動
    const int min_index = calc_mix_grid_index_in_same_line(index);
    for(int i=0; i<target_margin_cell; i++)
    {
        index--;
        if(index < min_index)
        {
            index++;
            break;
        }
    }

    return index;
}

// 占有グリッドを含む行のインデックスの最小値の算出
bool ObstacleExpander::calc_mix_grid_index_in_same_line(const int index)
{
    int min_index = 0;
    int max_index = updated_map_.info.width-1;
    while(max_index < index)
    {
        min_index += updated_map_.info.width;
        max_index += updated_map_.info.width;
    }

    return min_index;
}

// グリッドのインデックスからxのインデックスを計算
int ObstacleExpander::calc_grid_index_x(const int index)
{
    return index % updated_map_.info.width;
}

// グリッドのインデックスからyのインデックスを計算
int ObstacleExpander::calc_grid_index_y(const int index)
{
    return index / updated_map_.info.width;
}

// rect_grid_index_listの右下のインデックスを検索
int ObstacleExpander::find_lower_right_grid_index(const int center_grid_index)
{
    int index = center_grid_index;
    const int target_margin_cell = int(round(target_margin_/updated_map_.info.resolution)); // 車両マージン [cell]

    // 下方向に検索位置を移動
    for(int i=0; i<target_margin_cell; i++)
    {
        index += updated_map_.info.width;
        if(updated_map_.data.size() <= index)
        {
            index -= updated_map_.info.width;
            break;
        }
    }

    // 右方向に検索位置を移動
    const int man_index = calc_max_grid_index_in_same_line(index);
    for(int i=0; i<target_margin_cell; i++)
    {
        index++;
        if(man_index < index)
        {
            index--;
            break;
        }
    }

    return index;
}

// 占有グリッドを含む行のインデックスの最大値の算出
bool ObstacleExpander::calc_max_grid_index_in_same_line(const int index)
{
    int min_index = 0;
    int max_index = updated_map_.info.width-1;
    while(max_index < index)
    {
        min_index += updated_map_.info.width;
        max_index += updated_map_.info.width;
    }

    return max_index;
}

// グリッド同士の距離[cell]を計算
double ObstacleExpander::calc_dist_cell(const int index1, const int index2)
{
    const int index1_x = calc_grid_index_x(index1);
    const int index1_y = calc_grid_index_y(index1);
    const int index2_x = calc_grid_index_x(index2);
    const int index2_y = calc_grid_index_y(index2);

    const double dx = double(index1_x - index2_x);
    const double dy = double(index1_y - index2_y);

    return hypot(dx, dy);
}

// 実行時間を表示（スタート時間beginを予め設定する）
void ObstacleExpander::show_exe_time()
{
    RCLCPP_INFO(get_logger(), "Duration = %.2fs", (clock_.now() - begin_).seconds());
}
