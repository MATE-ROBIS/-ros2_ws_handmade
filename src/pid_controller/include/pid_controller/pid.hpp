#ifndef PID_CONTROLLER_PID_HPP
#define PID_CONTROLLER_PID_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>


class PID :public rclcpp::Node
{
	public:
		explicit PID(double Kp,double Ki ,double Kd);

		void sumation();

		double control_effort_ =0;

private:

	void printParameters();

	double ki_=0,kp_=0,kd_=0;

	

	//limit the amplitute for clamp controller
	double upperLimit_ {1000},lowerLimit_ {1000};

	double antiwindup {1000};

	double set_point_ {0};

	double measured_value_ {0};


	
	double	error_ {0};	

	double preverror_ {0};

	double  setpoint_ {0};

	double	Propotional_ {0};

	double	integral_ {0};

	double previntegral {0};

	double	derivative_ {0};

	double prevderivative_ {0};

	double thou_ {0};

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_effort_pub_;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr set_point_sub_;

	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measured_valuesub_;



};




#endif 