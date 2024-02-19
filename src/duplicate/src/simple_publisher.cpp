#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>
#include<chrono>
using namespace std::chrono_literals;
class Mynode : public rclcpp::Node {
public:
    Mynode(): Node("OOPS_based_class"),Counter{0}
    {
        pub = create_publisher<std_msgs::msg::String>("Publisher",10);
        RCLCPP_INFO(get_logger(),"NODE CREATED");
        timer_=create_wall_timer(1s,std::bind(&Mynode::timer_callback,this));

    }

private:
        unsigned int Counter ;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
        rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback(){
        Counter++;

        RCLCPP_INFO(get_logger(),"Counter incremented",Counter);
        pub->publish(Counter);
        
    }
};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Mynode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}