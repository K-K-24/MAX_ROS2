import cv2
import numpy as np

class Obstacle_Detector:

    def __init__(self):
        self.obstacle_colors = ['black']
    
    def detect_all_obstacles(self,image):
        obstacles = []

        for color in self.obstacle_colors:
            obstacle = self.detect_color_obstacles(image,color)
            obstacles.append(obstacle)

        return obstacles
    
    def get_color_range(self,color):
        colors = {
            'black':[(np.array([0, 0, 0]), np.array([180, 255, 50]))]
        }

        return colors.get(color,[])

    def get_color_mask(self,image,color):
        hsv = cv2.cvtColor(image,cv2.BGR2HSV)
        ranges = cv2.get_color_range(color)

        combined_mask = None
        for lower,upper in ranges:
            mask = cv2.inRange(hsv,lower,upper)
            
            if(combined_mask==None):
                combined_mask = mask
            else:
                combined_mask = cv2.bitwise_or(combined_mask,mask)

        return combined_mask
    






def main(args=None):
    a = 2




if __name__ == "__main__":
    main()