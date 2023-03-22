# storing way point into excel file
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pathlib import Path
import pandas as pd
import pickle

waypoints = {1:([],[]),2:([],[]),3:([],[]),4:([],[]),5:([],[]),6:([],[])}
# header = ['name','co','orientation']
# co = [[1,[],[]],
#       [2,[],[]]]

# waypoints = {'table': table, "co":co ,"Orien": ori}
# [3,[],[]],[4,[],[]],[5,[],[]],[6,[],[]]]
# output_file = 'Waypoint.csv'

# def saving_file():
#     print("in it")
#     df = pd.DataFrame(dict)
#     print(df)
#     #with open("way.csv","w+") as f:
#     #    write = f.write
#     # df.to_csv('Waypoint.csv', index= False)
#     # do = pd.read_csv(r'/home/colcon_ws/build\Waypoint.csv')
#     print(do)
#     # with open('waypoint.csv', 'w',encoding = 'UTF8', newline= '') as handle:
#     #                 writer = csv.writer(handle)
#     #                 writer.writerow(header)
#     #                 writer.writerows(co)
class Waypoint(Node):
    def __init__(self) -> None:
        super().__init__('waypoint')
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.subscription

    def odom_callback(self,msg): 
        
        # self.get_logger().info('I heard: "%s"' % pos)
            cmd_char = str(input(" Press s to store to csv file/ n for new point: "))
            pos = msg.pose.pose.position 
            orien = msg.pose.pose.orientation
            if cmd_char =="n":
                tb_int = int(input("Enter table number: "))
                waypoints[tb_int][0].extend((pos.x,pos.y,pos.z))
                waypoints[tb_int][1].extend((orien.x,orien.y,orien.z,orien.z,orien.w))
                # co[tb_int-1][1].extend((pos.x,pos.y,pos.z))
                # co[tb_int-1][2].extend((orien.x,orien.y,orien.z,orien.z,orien.w))
                print(waypoints)
            if cmd_char == 's': 
                print("saving...")
                with open('waypoints.pickle','wb') as handle:
                    pickle.dump(waypoints, handle, protocol=pickle.HIGHEST_PROTOCOL)
                # saving_file()
                

                 
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
