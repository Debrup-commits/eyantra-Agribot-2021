import cv2
import numpy as np

def nothing(x):
    pass

frame = cv2.imread('image.png')
img_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

cv2.namedWindow("image")


# create trackbars for color change
cv2.createTrackbar('lowH',"image",0,255,nothing)
cv2.createTrackbar('highH',"image",0,255,nothing)

cv2.createTrackbar('lowS',"image",0,255,nothing)
cv2.createTrackbar('highS',"image",0,255,nothing)

cv2.createTrackbar('lowV',"image",0,255,nothing)
cv2.createTrackbar('highV',"image",0,255,nothing)

while(True):
    h = cv2.getTrackbarPos("lowH", "image")
    s = cv2.getTrackbarPos("lowS", "image")
    v = cv2.getTrackbarPos("lowV", "image")

    H = cv2.getTrackbarPos("highH", "image")
    S = cv2.getTrackbarPos("highS", "image")
    V = cv2.getTrackbarPos("highV", "image")

    mask = cv2.inRange(img_HSV, np.array([h, s, v]), np.array([H, S, V]))
    output = cv2.bitwise_and(frame, frame, mask = mask)
    output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

    ret, threshold = cv2.threshold(output, 25, 255, cv2.THRESH_BINARY_INV)

    cv2.imshow("image", frame)
    cv2.imshow("threshold", output)

    if cv2.waitKey(25) and 0xFF == ord('q'):
            break

cv2.destroyAllWindows()