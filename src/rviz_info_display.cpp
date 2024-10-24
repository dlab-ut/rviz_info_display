#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <cmath>
#include <sstream>
#include <iomanip>

class DistanceCalculator : public rclcpp::Node
{
public:
    DistanceCalculator() : Node("distance_calculator")
    {
        // /whill/odomトピックをサブスクライブ
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/whill/odom", 10, std::bind(&DistanceCalculator::OnOdomReceived, this, std::placeholders::_1));

        // /joyトピックをサブスクライブ（ジョイスティック入力の監視）
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&DistanceCalculator::OnJoyReceived, this, std::placeholders::_1));

        // /distanceトピックに進んだ距離をパブリッシュ
        distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("/distance", 10);

        // 初期化
        reset_distance();

        // 定期的に進んだ距離をパブリッシュするためのタイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&DistanceCalculator::PublishDistance, this));
    }

    // 進んだ距離を取得する関数
    float get_total_distance() const
    {
        return total_distance_;
    }

private:
    // オドメトリコールバック
    void OnOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // 前回の位置が初期化されていない場合（最初の位置）
        if (!initialized_)
        {
            last_x_ = x;
            last_y_ = y;
            initialized_ = true;
        }

        // 現在位置と前回位置の距離を計算
        double dx = x - last_x_;
        double dy = y - last_y_;
        double distance_increment = std::sqrt(dx * dx + dy * dy);

        // 進んだ距離に加算
        total_distance_ += distance_increment;

        // 前回位置の更新
        last_x_ = x;
        last_y_ = y;
    }

    // ジョイスティックコールバック（リセット用）
    void OnJoyReceived(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // ボタン0が押されたら距離をリセット
        if (msg->buttons[8] == 1)  // ボタン0が押されたら
        {
            reset_distance();
            RCLCPP_INFO(this->get_logger(), "Distance reset to 0");
        }
    }

    // 距離をパブリッシュする関数
    void PublishDistance()
    {
        auto distance_msg = std_msgs::msg::Float32();
        distance_msg.data = total_distance_;

        // /distance トピックにパブリッシュ
        distance_pub_->publish(distance_msg);
    }

    // 距離をリセットする関数
    void reset_distance()
    {
        total_distance_ = 0.0;
        initialized_ = false; // オドメトリ初期化フラグをリセット
    }

    // サブスクライバー
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // パブリッシャー
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr timer_;

    // 進んだ距離とオドメトリの前回位置を保持する変数
    double total_distance_;
    double last_x_;
    double last_y_;
    bool initialized_;  // 最初のオドメトリ位置が設定されたかを追跡
};

class WhillInfoPublisher : public rclcpp::Node
{
public:
    WhillInfoPublisher() : Node("whill_info_publisher")
    {
        // サブスクライバーの設定
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/whill/controller/cmd_vel", 10, std::bind(&WhillInfoPublisher::OnCmdVelReceived, this, std::placeholders::_1));
        battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/for_rviz", 10, std::bind(&WhillInfoPublisher::OnBatteryReceived, this, std::placeholders::_1));
        distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/distance", 10, std::bind(&WhillInfoPublisher::OnDistanceReceived, this, std::placeholders::_1));
        state_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/state", 10, std::bind(&WhillInfoPublisher::OnStateReceived, this, std::placeholders::_1));

        // /whill_info を rviz_2d_overlay_msgs::msg::OverlayText 型でパブリッシュ
        info_pub_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("/whill_info", 10);

        // 初期化
        cmd_vel_speed_ = 0.0f;
        battery_ = 0.0f;
        distance_ = 0.0f;
        state_ = 0.0f;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&WhillInfoPublisher::PublishWhillInfo, this));
    }

private:
    // トピックのコールバック関数
    void OnCmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cmd_vel_speed_ = static_cast<float>(msg->linear.x); // linear.x を float32 型に変換
    }

    void OnBatteryReceived(const std_msgs::msg::Float32::SharedPtr msg)
    {
        battery_ = msg->data;
    }

    void OnDistanceReceived(const std_msgs::msg::Float32::SharedPtr msg)
    {
        distance_ = msg->data;
    }

    void OnStateReceived(const std_msgs::msg::Float32::SharedPtr msg)
    {
        state_ = msg->data;
    }

    void PublishWhillInfo()
    {
        // OverlayText メッセージを生成
        auto overlay_msg = rviz_2d_overlay_msgs::msg::OverlayText();

        // 小数点以下2桁まで表示するために stringstream を使用
        std::stringstream ss;
        ss << std::fixed;
        
        // speed: 小数第2位まで表示
        ss << std::setprecision(2) << "speed:    " << cmd_vel_speed_;
        
        // battery: 整数値で表示
        ss << "   battery:  " << static_cast<int>(battery_);
        
        // distance: 小数第2位まで表示
        ss << std::setprecision(2) << "   distance: " << distance_;
        
        // state: 整数値で表示
        ss << "   state:    " << static_cast<int>(state_);

        overlay_msg.text = ss.str();

        // レイアウトや見た目に関する設定
        overlay_msg.width = 400;
        overlay_msg.height = 100;
        overlay_msg.text_size = 12.0;
        overlay_msg.line_width = 2;
        overlay_msg.font = "Arial";
        overlay_msg.fg_color.r = 1.0;
        overlay_msg.fg_color.g = 1.0;
        overlay_msg.fg_color.b = 1.0;
        overlay_msg.fg_color.a = 1.0;  // 白
        overlay_msg.bg_color.r = 0.0;
        overlay_msg.bg_color.g = 0.0;
        overlay_msg.bg_color.b = 0.0;
        overlay_msg.bg_color.a = 0.5;  // 半透明の黒背景
        overlay_msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;

        // /whill_info トピックにパブリッシュ
        info_pub_->publish(overlay_msg);
    }

    // サブスクライバー
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr state_sub_;

    // パブリッシャー
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr info_pub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr timer_;

    // データ保持用変数
    float cmd_vel_speed_;
    float battery_;
    float distance_;
    float state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // ノードをまとめてスピン
    auto distance_calculator_node = std::make_shared<DistanceCalculator>();
    auto whill_info_publisher_node = std::make_shared<WhillInfoPublisher>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(distance_calculator_node);
    executor.add_node(whill_info_publisher_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
