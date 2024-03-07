import numpy as np
import matplotlib.pyplot as plt

# laser_range = np.loadtxt('/home/yttrium/colcon_ws/src/auto_nav/auto_nav/lidar.txt')

# for i in range(len(laser_range)):
    
#     if str(laser_range[i]) == "inf":
#         # print(laser_range[i])
#         laser_range[i] = 0
#         # print(laser_range[i])

# laser_range[laser_range==0] = np.nan
# lr2i = np.nanargmax(laser_range)
# print(lr2i)
# print(laser_range[lr2i])

# plt.figure()
# plt.polar(np.arange(0,360)/180*float(np.pi),laser_range)
# plt.show()

# print("done lidar plot")

# quat = np.loadtxt('/home/yttrium/colcon_ws/src/auto_nav/auto_nav/odom.txt', skiprows=13, delimiter=':', usecols=1, max_rows=4) 
# print(quat)

omap = np.loadtxt('/home/yttrium/colcon_ws/src/auto_nav/auto_nav/map.txt')
plt.imshow(omap, origin='lower')
plt.show()

print("show omap")





