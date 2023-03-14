# storing way point into excel file
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
co = {1:([],[]),2:[[],[]]}

class Waypoint(Node):
    def __init__(self) -> None:
        super().__init__('waypoint')
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.subscription

    def odom_callback(self,msg): 
        
        # self.get_logger().info('I heard: "%s"' % pos)
            cmd_char = str(input("Press s (to store point)/ Press e to store to csv file: "))
            
            if cmd_char == "s":
                pos = msg.pose.pose.position 
                orien = msg.pose.pose.orientation
                tb_int = int(input("Enter table number: "))
                co[tb_int][0].extend((pos.x,pos.y,pos.z,'\n'))
                co[tb_int][1].extend((orien.x,orien.y,orien.z,orien.z,orien.w,'\n'))
                print(co)
            if cmd_char == 'e':
                 with open("waypoint.csv","w",newline='') as f:
                    writer =csv.DictWriter(f, fieldnames= co.keys())
                    writer.writeheader()
                    writer.writerow(co)
                 
            # print(pos.x,pos.y,pos.z)
            # print(orien.x,orien.y,orien.z,orien.w)       

def main(args=None):
    rclpy.init(args=args)
    waypoint = Waypoint()
    cmd_char = str(input("Press q to get point: "))
    if cmd_char == "q":
        rclpy.spin(waypoint)

    waypoint.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
