import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import time

class VESCTestNode(Node):
    def __init__(self):
        super().__init__('vesc_test_node')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.get_logger().info('VESC test node started')
        time.sleep(2)
        
    def publish_ackermann_command(self, speed, steering_angle=0.0):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        self.publisher_.publish(msg)

    def main_loop(self):
    
        while rclpy.ok():
 
            # 정지
            self.publish_ackermann_command(0.0)
            self.get_logger().info(f"Speed 0.0")
            time.sleep(4)
            
                                  
            # 점점 빠르게 전진
            
            for j in range(250):
                for i in range(1):
                    speed = 0.02 * -j  # 0.0 ~ 5.0
                    self.publish_ackermann_command(speed)
                    self.get_logger().info(f"Speed {i:.0f}: {speed:.2f}")
                    time.sleep(0.1)
                    
            # 정지
            self.publish_ackermann_command(0.0)
            self.get_logger().info(f"Speed 0.0")
            time.sleep(4)
            
            for j in range(250):
                for i in range(1):
                    speed = 0.02 * j  # 0.0 -> 5.0
                    self.publish_ackermann_command(speed)
                    self.get_logger().info(f"Speed {i:.0f}: {speed:.2f}")
                    time.sleep(0.1)
            
            # 정지
            self.publish_ackermann_command(0.0)
            time.sleep(4)           
            
                       
            # 좌 조향 테스트 (최대 ±0.3 제한) 
            angle = -0.01           
            for i in range(350):
                angle = angle + 0.001
                self.publish_ackermann_command(1.5, angle)  # 속도 고정
                self.get_logger().info(f"Angle {i:.0f}: {angle:.3f}")
                time.sleep(0.1)
                
            time.sleep(4)    
            
            for i in range(350):
                angle = angle - 0.001
                self.publish_ackermann_command(1.5, angle)  # 속도 고정
                self.get_logger().info(f"Angle {i:.0f}: {angle:.3f}")
                time.sleep(0.1)
           
            # 우 조향 테스트 (최대 ±0.3 제한)
            for i in range(350):
                angle = angle - 0.001
                self.publish_ackermann_command(1.5, angle)  # 속도 고정
                self.get_logger().info(f"Angle {i:.0f}: {angle:.3f}")
                time.sleep(0.1)
           
            for i in range(350):
                angle = angle + 0.001 
                self.publish_ackermann_command(1.5, angle)  # 속도 고정
                self.get_logger().info(f"Angle {i:.0f}: {angle:.3f}")
                time.sleep(0.1)
           
            time.sleep(4)    
                       
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.5)
        
            self.get_logger().info('VESC test completed')

def main(args=None):
    rclpy.init(args=args)
    node = VESCTestNode()
    
    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()

