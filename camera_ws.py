import cv2
import numpy as np
import math


image = cv2.imread("captured_image_4.png")

[height,width,_] = image.shape

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# cv2.imwrite("captured_image_4_hsv.png", hsv)

lower_yellow = np.array([25, 150, 150])
upper_yellow = np.array([35, 255, 255])

mask = cv2.inRange(hsv, lower_yellow, upper_yellow)



cv2.imwrite("mask.png", mask)

#contours

contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


object_geo_center = 0
pixel_object_width = 0


for cnt in contours:
    area = cv2.contourArea(cnt)
    if area > 300:  # Filter out small contours
        x, y, w, h = cv2.boundingRect(cnt)
        object_geo_center = x + w /2
        pixel_object_width = w
        # cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)

        cv2.drawContours(image, [cnt], 0, (0,0,255), 5)

        #Centroid
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        
        else:
            cx, cy = 0, 0

        # cv2.circle(image, (cx, cy), 5, (255, 0, 0), -1)

        sides = len(approx)

        if sides == 3:
            label = 'Triangle'
        elif sides == 4:
            label = 'Quadrilateral'
        elif sides == 5:
            label = 'Pentagon'
        elif sides == 6:
            label = 'Hexagon'
        else:
            label = 'Circle'

        cv2.putText(image, label, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)



cv2.imwrite("centroid.png", image)

#Angle Calculation

offset = object_geo_center - width/2 # Assuming image width is 640 pixels ( negative means left, positive means right)

field_of_view = 60  # Camera field of view in degrees

theta = (offset / (width/2)) * (field_of_view/2)  # Convert pixel offset to angle
theta_in_rad = math.radians(theta)  # Convert degrees to radians

#Distance Calculation
real_object_width = 5

d = (real_object_width * width) / pixel_object_width * 2 * math.tan(math.radians(field_of_view/2))

#robot coordinates
robot_x = d * math.cos(theta_in_rad)
robot_y = d * math.sin(theta_in_rad)

print(f"Object Coordinates: x = {robot_x}, y = {robot_y}, Label = {label}")


















