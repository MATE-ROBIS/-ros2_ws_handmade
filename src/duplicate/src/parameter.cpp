#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>
#include<vector>
#include<string>
#include<rcl_interfaces/msg/set_parameters_result.hpp>
#include<rclcpp/parameter.hpp>

using std::placeholders::_1;

class Parameters : public rclcpp::Node {

public:
    Parameters():Node("Parameter_Cpp"){
        declare_parameter<std::string>("Device_name","Moons_driver");

        shtr_=add_on_set_parameters_callback(std::bind(&Parameters::parameter_callback,this,_1));

    }




private:
    OnSetParametersCallbackHandle::SharedPtr shtr_;
    
    rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter>& parameters){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto parameter:parameters){
            if (parameter.get_name()=="Device_name" and parameter.get_type()==rclcpp::ParameterType::PARAMETER_STRING){
                RCLCPP_INFO(get_logger(),"Changed",parameter.as_string());
                result.successful=true;
            }
        }

        return result;

    }



};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Parameters>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}