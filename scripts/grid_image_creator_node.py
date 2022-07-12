#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import os
from custom_msgs.msg import gridMap, gridRow, gridPoint
from custom_msgs.msg import pointData, pointDataArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class grid_image_creator:

    # Description of the Grid- {1--> not occupied} {0--> occupied} 

    def __init__(self):
        self.image_pub = rospy.Publisher("gridMap",Image, queue_size=1, latch=True)
        self.gridM_sub = rospy.Subscriber("grid", gridMap, self.callbackGrid)
        self.bridge = CvBridge()
        self.index = 1
        self.dir_image_save = '/home/kalana/ROS/MSC/src/grid'

    def callbackGrid(self, msg):
        rowCount = msg.rowCount
        colCount = msg.colCount
        gridImage = np.ones([rowCount, colCount, 3], np.uint8)*255     #initalized as a all while traversable
        gridPath = []

        for i in range(0, rowCount):
            array = msg.grid[i]
            for j in range (0, colCount):
                point = array.row[j]
                #processed grid (with padding) is saved as GREEN channel
                if (point.proc==0):
                    gridImage[i][j][0] = 0
                    gridImage[i][j][1] = 255
                    gridImage[i][j][2] = 0

                #obstacles saved as Black channel
                if (point.init==0):
                    gridImage[i][j][0] = 0
                    gridImage[i][j][1] = 0
                    gridImage[i][j][2] = 0

        length = msg.pathLength

        for i in range(0, length):
            x = int(msg.path[i].x)
            y = int(msg.path[i].y)
            gridPath.append([x,y])

        for k in range(0, (len(gridPath)-1)):
            cv2.line(gridImage, (gridPath[k][0], gridPath[k][1]), (gridPath[k+1][0], gridPath[k+1][1]), (0,0,255), thickness=1)

        for i in range(0, rowCount):
            array = msg.grid[i]
            for j in range (0, colCount):
                point = array.row[j]
                #unexplored region saved as gray channel
                if (point.disc==1):
                    gridImage[i][j][0] = 128
                    gridImage[i][j][1] = 128
                    gridImage[i][j][2] = 128

        try:
            gridImageRot = cv2.rotate(gridImage, cv2.ROTATE_90_COUNTERCLOCKWISE)
            gridImageFlip = cv2.flip(gridImageRot, 1)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(gridImageFlip, "bgr8"))
            
            name = str(self.index) +".jpeg"

            cv2.imwrite(os.path.join(self.dir_image_save , name), gridImageFlip)
            self.index += 1

        except CvBridgeError as e:
            print(e)

        
def main():
    creator = grid_image_creator()
    rospy.init_node('Grid_Image_Creator')
    rospy.spin()

if __name__ == '__main__':
    main()