import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Parameter



class simple_parameter(Node):
    def __init__(self):
        super().__init__("Simple_param")

        self.declare_parameter("simple_init_param",20)
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self,param):
        result=SetParametersResult()

        for para in param:
            if para.name =="simple_init_param" and para.type_== Parameter.Type.INTEGER:
                self.get_logger("parameter value get updated"%para.value)
                result.successful=True
        return result

def main():
    rclpy.init()
    simple=simple_parameter()
    rclpy.spin(simple)
    simple_parameter.destroy_node()
    rclpy.shutdown()


if __name__ =="__main__":
    main()


        
