import cv2
import numpy as np
import math

img = cv2.imread("A_letter.png")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

cv2.imwrite("gray.png", gray)

_, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

cv2.imwrite("thresh.png", thresh)

contours, _ = cv2.findContours(thresh,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

print(len(contours))

for i, cnt in enumerate(contours):

    if i==4:
        continue


    approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)

    cv2.drawContours(img, [cnt], 0, (0,0,255), 5)

    M = cv2.moments(cnt)
    if M["m00"] != 0:
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])

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

    cv2.putText(img, label, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

cv2.imwrite("shape_with_label.png",img)



