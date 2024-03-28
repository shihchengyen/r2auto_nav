import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import math
import scipy.stats

import argparse

from collections import deque

from std_msgs.msg import Int16MultiArray

# constants
occ_bins = [-1, 0, 50, 100]
map_bg_color = 1

FRONTIER_THRESHOLD = 5

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class FrontierSearchNode(Node):

    def __init__(self, show_plot):
        super().__init__('frontierSearchNode')
        self.show_plot = (show_plot == 'y')
        
        ''' ================================================ Occupancy Grid ================================================ '''
        # Create a subscriber to the topic "map"
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        ''' ================================================ Frontier Coords ================================================ '''
        # Create a publisher to the topic "frontierCoords", an array of coordinates relative to the map frame at where the frontiers are
        # send array of tuples (x, y) as a one dimensional array
        self.frontierCoords_publisher = self.create_publisher(Int16MultiArray, 'frontierCoords', 10)

    def listener_callback(self, msg):
        ''' ================================================ Occupancy Grid ================================================ '''
        # create numpy array
        occdata = np.array(msg.data)
        # compute histogram to identify bins with -1, values between 0 and below 50, 
        # and values between 50 and 100. The binned_statistic function will also
        # return the bin numbers so we can use that easily to create the image 
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        # get width and height of map
        iwidth = msg.info.width
        iheight = msg.info.height
        resolution = msg.info.resolution
        # calculate total number of bins
        total_bins = iwidth * iheight
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))
        self.get_logger().info('resolution %f m/cell' % resolution)

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
            
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        # self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        # get map resolution
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        # get map grid positions for x, y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round(((cur_pos.y - map_origin.y) / map_res))
        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        # set current robot location to 0
        odata[grid_y][grid_x] = 0
        # create image from 2D array using PIL
        img = Image.fromarray(odata)
        # find center of image
        i_centerx = iwidth/2
        i_centery = iheight/2
        # find how much to shift the image to move grid_x and grid_y to center of image
        shift_x = round(grid_x - i_centerx)
        shift_y = round(grid_y - i_centery)
        # self.get_logger().info('Shift Y: %i Shift X: %i' % (shift_y, shift_x))

        # pad image to move robot position to the center
        # adapted from https://note.nkmk.me/en/python-pillow-add-margin-expand-canvas/ 
        left = 0
        right = 0
        top = 0
        bottom = 0
        if shift_x > 0:
            # pad right margin
            right = 2 * shift_x
        else:
            # pad left margin
            left = 2 * (-shift_x)
            
        if shift_y > 0:
            # pad bottom margin
            bottom = 2 * shift_y
        else:
            # pad top margin
            top = 2 * (-shift_y)
            
        # create new image
        new_width = iwidth + right + left
        new_height = iheight + top + bottom
        img_transformed = Image.new(img.mode, (new_width, new_height), map_bg_color)
        img_transformed.paste(img, (left, top))

        # rotate by 90 degrees so that the forward direction is at the top of the image
        rotated = img_transformed.rotate(np.degrees(yaw)-90, expand=True, fillcolor=map_bg_color)

        # show the image using grayscale map
        # plt.imshow(img, cmap='gray', origin='lower')
        # plt.imshow(img_transformed, cmap='gray', origin='lower')
        
        # if self.show_plot:
        #     plt.imshow(rotated, cmap='gray', origin='lower')
        
        ''' ================================================ Frontier Search ================================================ '''
        # since expand=true, we need to re get the dimensions
        img_array = np.array(rotated)

        # Get the shape of the array
        height, width = img_array.shape
        
        # pixel 0 = black = robot
        # pixel 1 = dark grey = unmapped
        # pixel 2 = light grey = mapped and open
        # pixel 3 = white = mapped and obstacle
        # frontier is between 1 = dark grey = unmapped and 2 = light grey = mapped and open
        
        frontier = []

        # Iterate over the array
        for i in range(height):
            for j in range(width):
                # Check if the current pixel is 1
                if img_array[i, j] == 1:
                    # check for diagonals also so BFS with UP, DOWN, LEFT, RIGHT can colect all frontier pixels
                    for di in [-1, 0, 1]:
                        for dj in [-1, 0, 1]:
                            # Skip the current pixel
                            if di == 0 and dj == 0:
                                continue
                            # Check if the neighboring pixel is inside the image
                            if 0 <= i + di < height and 0 <= j + dj < width:
                                # Check if the neighboring pixel is 2
                                if img_array[i + di, j + dj] == 2:
                                    frontier.append((i, j))
                                    # self.get_logger().info(str("Pixel 1 at (" + str(i) + ", " + str(j) + ") is next to pixel 2 at (" + str(i + di) + ", " + str(j + dj) + ")" ))
        
        # BFS to find all frontier groups
        # Initialize the queue with the first pixel
        queue = deque([frontier[0]])

        # Initialize the set of visited pixels
        visited = set([frontier[0]])

        # Initialize the list of groups
        groups = []

        # Perform the BFS
        while queue:
            # Start a new group
            group = []

            # Process all pixels in the current group
            while queue:
                i, j = queue.popleft()
                group.append((i, j))

                # Check the neighboring pixels
                for di in [-1, 0, 1]:
                    for dj in [-1, 0, 1]:
                        # Skip the current pixel and diagonal pixels
                        if (di == 0 and dj == 0) or (di != 0 and dj != 0):
                            continue

                        # Check if the neighboring pixel is inside the image and in the frontier
                        if 0 <= i + di < height and 0 <= j + dj < width and (i + di, j + dj) in frontier:
                            # Check if the neighboring pixel has not been visited yet
                            if (i + di, j + dj) not in visited:
                                # Add the neighboring pixel to the queue and the set of visited pixels
                                queue.append((i + di, j + dj))
                                visited.add((i + di, j + dj))

            # Add the group to the list of groups
            groups.append(group)

            # Find the next unvisited pixel in the frontier
            for pixel in frontier:
                if pixel not in visited:
                    queue.append(pixel)
                    visited.add(pixel)
                    break
               
        # find frontier points if the frontier group has more than FRONTIER_THRESHOLD points        
        # Initialize the list of frontier points
        frontierPoints = []

        # Iterate over the groups
        for group in groups:
            if len(group) < FRONTIER_THRESHOLD:
                continue
            
            # Extract the x and y coordinates
            x_coords = [x for x, y in group]
            y_coords = [y for x, y in group]

            # Calculate the middle x and y coordinates
            middle_x = sorted(x_coords)[len(x_coords) // 2]
            middle_y = sorted(y_coords)[len(y_coords) // 2]

            # Add the median coordinates to the list of frontier points
            frontierPoints.append((middle_x, middle_y))
            
        self.get_logger().info("frontierPoints:" + str(frontierPoints))
        
        # publish frontier points
        # flatten the list of tuples into a single list
        flattenedFrontierPoint = [item for tup in frontierPoints for item in tup]
        
        frontierPoint_msg = Int16MultiArray()
        frontierPoint_msg.data = flattenedFrontierPoint
        
        self.frontierCoords_publisher.publish(frontierPoint_msg)        
        
        # only do all this if plotting else its useless
        if self.show_plot:
            # Create a copy of the image array
            frontier_img_array = np.zeros((height, width, 3), dtype=np.uint8)
            
            # Set the color of the frontier pixels
            # Iterate over the grayscale image to convert to colour
            for i in range(height):
                for j in range(width):
                    # if its a frontier point give it pink
                    # if its a frontier give it cyan 
                    # else Map the 0, 1, 2, 3 value to a grey scale color value
                    if (i, j) in frontierPoints:
                        frontier_img_array[i, j] = [255, 0, 255] # pink
                    elif (i, j) in frontier:
                        frontier_img_array[i, j] = [0, 255, 255]  # cyan
                    elif img_array[i, j] == 0:
                        frontier_img_array[i, j] = [0, 0, 0]  # Black
                    elif img_array[i, j] == 1:
                        frontier_img_array[i, j] = [85, 85, 85]  # Dark Gray
                    elif img_array[i, j] == 2:
                        frontier_img_array[i, j] = [170, 170, 170]  # Light gray
                    elif img_array[i, j] == 3:
                        frontier_img_array[i, j] = [255, 255, 255]  # White
                        
            # Create a PIL image
            frontier_img = Image.fromarray(frontier_img_array)
                        
            plt.imshow(frontier_img, origin='lower')
        
            plt.draw_all()
            # pause to make sure the plot gets created
            plt.pause(0.00000000001)

def main(args=None): 
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Start the frontierSearchNode.')
    parser.add_argument('-s', type=str, default='n', help='Show plot (y/n)')
    args = parser.parse_args()
    
    frontierSearchNode = FrontierSearchNode(args.s)
    
    if args.s == 'y':
        # create matplotlib figure
        plt.ion()
        plt.figure()    
    
    try:
        rclpy.spin(frontierSearchNode)
    except KeyboardInterrupt:
        pass
    finally:
        frontierSearchNode.destroy_node()


if __name__ == '__main__':
    main()