import cv2
import numpy as np

class Obstacle_Detector:

    def __init__(self):
        self.obstacle_colors = ['black']
        self.H = np.load("homography_matrix.npy")
        self.obstacles = []


    def image_to_world(self,pixel_x, pixel_y):
        H = np.load("homography_matrix.npy")
        pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        world_point = cv2.perspectiveTransform(pixel_point,H)
        return world_point[0][0]   #returns an array of world points

    def apply_homography(self,points):
        world_points = []
        for i in points:
            world_point = self.image_to_world(i[0],i[1])
            # print(world_point)
            world_points.append(world_point)

        med_x,med_y = np.median(world_points,axis=0)
        return np.array([med_x,med_y])

    def find_ground_contact_points(self,object_mask, niter=1):
        corners =  cv2.goodFeaturesToTrack(
            np.float32(object_mask),
            100,
            0.01,
            10
        )

        corners = np.squeeze(np.intp(corners))

        isort = np.flip(np.argsort(corners[:,1]))[:5]
        lowest_pt = corners[isort,:]

        return lowest_pt

    def get__single_obstacle(self,mask):


        numLabels,labels,stats,centroids = cv2.connectedComponentsWithStats(mask)

        for i in range(1,numLabels):
            if stats[i,cv2.CC_STAT_AREA] > 300:
            

                object_mask = (labels==i).astype(np.uint8)*255

                # time.sleep(5)

                bottompoints = self.find_ground_contact_points(object_mask)

                obstacle_posn = self.apply_homography(bottompoints)

                if(obstacle_posn[0] > 0):
                    self.obstacles.append(obstacle_posn)


    def get_all_obstacles(self,img):
        # img = cv2.imread('black_flask.png')

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # cv2.imwrite("captured_image_4_hsv.png", hsv)

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])

        mask = cv2.inRange(hsv, lower_black, upper_black)

        self.get__single_obstacle(mask)



