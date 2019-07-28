import cv2

image = cv2.imread("./imgs/baboon.jpg")
cv2.imshow('frog', image)
cv2.waitKey(0)
cv2.destroyAllWindows()