import rclpy
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor
import time

class XycarMotorTestNode(Node):
    def __init__(self):
        super().__init__('XycarMotor_test_node')
        self.publisher_ = self.create_publisher(XycarMotor, '/xycar_motor', 10)
        self.get_logger().info('XycarMotor node started')
        time.sleep(2)

    def publish_xycar_motor_command(self, speed, steering_angle=0.0):
        msg = XycarMotor()
        msg.speed = speed
        msg.angle = steering_angle
        self.publisher_.publish(msg)

    def main_loop(self):
    
        while rclpy.ok():
 
            # 정지
            self.publish_xycar_motor_command(0.0)
            time.sleep(4)
        
            # 점점 빠르게 전진            
            for i in range(100):
                speed = i * 1.0  # 0.0 -> 99.0 (100단계)
                self.publish_xycar_motor_command(speed)
                self.get_logger().info(f"Speed {i:.0f}: {speed:.0f}")
                time.sleep(0.1)
            
            # 정지
            self.publish_xycar_motor_command(0.0)
            time.sleep(4)
        
            # 점점 빠르게 후진
            for i in range(100):
                speed = -i * 1.0  # 0.0 -> -99.0 (100단계)
                self.publish_xycar_motor_command(speed)
                self.get_logger().info(f"Speed {i:.0f}: {speed:.0f}")
                time.sleep(0.1)
            
            # 정지
            self.publish_xycar_motor_command(0.0)
            time.sleep(4)
            
            # 좌 조향 테스트 
            for i in range(100):
                angle = i * -1.0  # 0.0 -> -99.0 (100단계)
                self.publish_xycar_motor_command(20.0, angle)  # 속도 20.0으로 고정
                self.get_logger().info(f"Speed 10.0 Angle {i:.0f}: {angle:.0f}")
                time.sleep(0.1)
                
            for i in range(100):
                angle = -99.0 + i * 1.0  # -99.0 -> 0.0
                self.publish_xycar_motor_command(20.0, angle)  # 속도 20.0으로 고정
                self.get_logger().info(f"Speed 10.0 Angle {i:.0f}: {angle:.0f}")
                time.sleep(0.1)
           
            # 우 조향 테스트 
            for i in range(100):
                angle = i * 1.0  # 0.0 -> 99.0
                self.publish_xycar_motor_command(20.0, angle)  # 속도 20.0으로 고정
                self.get_logger().info(f"Speed 10.0 Angle {i:.0f}: {angle:.0f}")
                time.sleep(0.1)
           
            for i in range(100):
                angle = 99.0 - i * 1.0  # 99.0 -> 0.0
                self.publish_xycar_motor_command(20.0, angle)  # 속도 20.0으로 고정
                self.get_logger().info(f"Speed 10.0 Angle {i:.0f}: {angle:.0f}")
                time.sleep(0.1)
           
            # 정지
            self.publish_xycar_motor_command(0.0)
            time.sleep(3)            
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
            self.get_logger().info('XycarMotor test completed')

def main(args=None):
    rclpy.init(args=args)
    node = XycarMotorTestNode()
    
    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()


