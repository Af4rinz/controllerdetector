import numpy as np
import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise IOError("Webcam can't be accessed")

while 1:

    _, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # range for bgr
    lower_blue = np.array([90, 150, 50])
    upper_blue = np.array([105, 255, 255])
    lower_green = np.array([50, 90, 50])
    upper_green = np.array([80, 255, 255])
    lower_red = np.array([175, 150, 50])
    upper_red = np.array([190, 255, 255])
    # bgr masks
    bMask = cv2.inRange(hsv, lower_blue, upper_blue)
    gMask = cv2.inRange(hsv, lower_green, upper_green)
    rMask = cv2.inRange(hsv, lower_red, upper_red)
    # setting bgr outputs to colour
    gOut = cv2.bitwise_and(frame, frame, mask=gMask)
    bOut = cv2.bitwise_and(frame, frame, mask=bMask)
    rOut = cv2.bitwise_and(frame, frame, mask=rMask)
    # finding contours
    bContours, _ = cv2.findContours(bMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    approx = tuple()
    # takes approximates of all contours and makes a new tuple
    for cnt in bContours:
        epsilon = 0.1 * cv2.arcLength(cnt, True)
        approxElement = cv2.approxPolyDP(cnt, epsilon, True)
        approx = approx + (approxElement,)

    if not (len(bContours) == 0):
        c = max(approx, key=cv2.contourArea)
        if len(c) == 4:
            rect = cv2.minAreaRect(c)
            temp = rect.points()[0]
            max = 0.0
            for p in rect.points():
                if cv2.norm(p - temp) >= max:
                    max = cv2.norm(p-temp)
                    p1 = p
                    p2 = temp
                temp = p

            contSlope = (p1.y - p2.y)/(p1.x - p2.x)
            print(contSlope)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(frame, [box], 0, (0, 255, 25), 2)
        else:
            print("No large rectangle found.")
    else:
        print("No shape found.")

    # showing frames
    cv2.imshow('hsv', hsv)
    cv2.imshow('frame', frame)
    # cv2.imshow('gOut', gOut)
    # cv2.imshow('rOut', rOut)
    cv2.imshow('bMask', bMask)
    cv2.imshow('bOut', bOut)
    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
