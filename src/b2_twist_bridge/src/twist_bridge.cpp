/*
    需求：订阅twist消息，并将之转换为go2所需的request消息格式，以控制机器狗运动。
    实现：
        1.创建一个Request的发布对象
        2.创建一个Twist的订阅对象
        3.在回调函数中，将Twist消息转换为Request消息，并发布出去
*/ 
// 包含头文件
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_api/msg/request.hpp"
#include "nlohmann/json.hpp"
#include "sport_model.hpp"

using namespace std::placeholders;

// 自定义节点类
class TwistBridge : public rclcpp::Node{
public:
    TwistBridge() : Node("twist_bridge"){
        RCLCPP_INFO(this->get_logger(), "TwistBridge node has been started. Subscribing to /cmd_vel and publishing to /api/sport/request ...");
        // 创建Request的发布对象
        request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        // 创建Twist的订阅对象
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&TwistBridge::twist_callback, this, _1));
    }
private:
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    // 回调函数，将Twist消息转换为Request消息，并发布出去
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist){
        // 创建Request消息对象
        unitree_api::msg::Request request;
        // 转换逻辑
        // 获取twist消息的线速度和角速度
        double x = twist->linear.x;
        double y = twist->linear.y; 
        double z = twist->angular.z;
        // 默认api_id为平衡站立
        auto api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
        if(x != 0 || y != 0 || z != 0){
            // 如果有非零的线速度或角速度，则切换到运动控制模式
            api_id = ROBOT_SPORT_API_ID_MOVE;
            // 设置参数，组织一个字符串样式的速度指令
            nlohmann::json js;
            js["x"] = x;
            js["y"] = y;    
            js["z"] = z;
            request.parameter = js.dump();
        }
        request.header.identity.api_id = api_id;
        request_pub_->publish(request);
    }
};


int main(int argc, char *argv[]){
    // 初始化ROS2
    rclcpp::init(argc, argv);
    // 创建节点对象
    auto node = std::make_shared<TwistBridge>();
    // 进入循环，等待回调函数被调用
    rclcpp::spin(node);
    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}