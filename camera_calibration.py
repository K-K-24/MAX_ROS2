import cv2
import numpy as np

#Step - 1
# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
# if cap.isOpened():
#     ret, frame = cap.read() 
#     if ret:
#         filename = f"calibration_image.jpg"
#         frame = cv2.rotate(frame, cv2.ROTATE_180)  
#         cv2.imwrite(filename, frame)
#         print(f"✅ Success on /dev/video — saved as {filename}")
#     else:
#         print(f"❌ /dev/video opened but no frame.")
#     cap.release()
# else:
#     print(f"❌ Could not open /dev/video")


#Step-2


# Click callback function
# clicks = []
# def mouse_click(event, x, y, flags, param):
#     if event == cv2.EVENT_LBUTTONDOWN:
#         clicks.append((x, y))
#         print(f"Clicked: ({x}, {y})")

# # Load image and set up clicking
# img = cv2.imread("calibration_image.jpg")
# cv2.namedWindow("Click Points")
# cv2.setMouseCallback("Click Points", mouse_click)

# print("Click on your 4 reference points in order: A, B, C, D")
# print("Press 'q' when done")

# while True:
#     cv2.imshow("Click Points", img)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cv2.destroyAllWindows()
# print(f"Pixel coordinates: {clicks}")

#Step-3( Calculate Homography )
# Your measured real-world coordinates (in cm)
# clicks = [(20, 302),
# (465, 319),
# (218, 265),
# (462, 266)]
# world_points = np.array([
#     [60, 60],   # Point A
#     [60, 0],   # Point B  
#     [120, 60],  # Point C
#     [120, 0]   # Point D
# ], dtype=np.float32)

# # Your clicked pixel coordinates
# pixel_points = np.array(clicks, dtype=np.float32)

# # Calculate homography
# H, mask = cv2.findHomography(pixel_points, world_points)
# print("Homography Matrix:")
# print(H)

# # Save for later use
# np.save("homography_matrix.npy", H)

#Step-3 ( Test the matrix)
clicks =[(296, 278)]
# Test function
def pixel_to_world(pixel_x, pixel_y, H):
    pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
    world_point = cv2.perspectiveTransform(pixel_point, H)
    return world_point[0][0]

# Test on known points
H = np.load("homography_matrix.npy")
test_pixel = clicks[0]  # First clicked point
result = pixel_to_world(test_pixel[0], test_pixel[1], H)
print(f"Pixel {test_pixel} → World {result}")
