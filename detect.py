import numpy as np
import cv2
import pyvjoy


# range for bgr
lower_blue = np.array([90, 150, 50])
upper_blue = np.array([105, 255, 255])
lower_green = np.array([50, 90, 50])
upper_green = np.array([80, 255, 255])
lower_red = np.array([0, 150, 150])
upper_red = np.array([20, 255, 255])

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise IOError("Web-cam can't be accessed")

j = pyvjoy.VJoyDevice(1)

# initial position of rButton and gButton
rButton = False
gButton = False
# indicates the first detection of rButton and gButton,
# initial detection is needed to enable these buttons
gButtonDet = False
rButtonDet = False

rTimer = 0
gTimer = 0


while 1:

    _, frame = cap.read()

    cv2.medianBlur(frame, 7, frame)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # bgr masks
    bMask = cv2.inRange(hsv, lower_blue, upper_blue)
    gMask = cv2.inRange(hsv, lower_green, upper_green)
    rMask = cv2.inRange(hsv, lower_red, upper_red)
    # setting bgr outputs to colour
    gOut = cv2.bitwise_and(frame, frame, mask=gMask)
    bOut = cv2.bitwise_and(frame, frame, mask=bMask)
    rOut = cv2.bitwise_and(frame, frame, mask=rMask)
    # finding contours in bgr channels separately
    bContours, _ = cv2.findContours(bMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    gContours, _ = cv2.findContours(gMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rContours, _ = cv2.findContours(rMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    bApprox = tuple()
    gApprox = tuple()
    rApprox = tuple()
    # takes approximates of all contours and makes a new tuple
    for cnt in rContours:
        epsilon = 0.03 * cv2.arcLength(cnt, True)
        approxElement = cv2.approxPolyDP(cnt, epsilon, True)
        rApprox = rApprox + (approxElement, )
    for cnt in bContours:
        epsilon = 0.1 * cv2.arcLength(cnt, True)
        approxElement = cv2.approxPolyDP(cnt, epsilon, True)
        bApprox = bApprox + (approxElement, )
    for cnt in gContours:
        epsilon = 0.03 * cv2.arcLength(cnt, True)
        approxElement = cv2.approxPolyDP(cnt, epsilon, True)
        gApprox = gApprox + (approxElement, )

    if not (len(bContours) == 0):
        c = max(bApprox, key=cv2.contourArea)
        if len(c) == 4 and cv2.contourArea(c) > 2000:
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            temp = box[0]
            maxLen = 0.0
            for p in box:
                # find the width of the rectangle
                if cv2.norm(p - temp) >= maxLen:
                    maxLen = cv2.norm(p-temp)
                    p1 = p
                    p2 = temp
                temp = p
            if p1[0] == p2[0] or abs(p1[0] - p2[0] <= 0.0001):
                contSlope = float("inf")
            else:
                contSlope = (p1[1] - p2[1])/(p1[0] - p2[0])
                if -1 < contSlope < -0.1:
                    j.set_axis(pyvjoy.HID_USAGE_X, 0x6000)
                elif contSlope <= -1:
                    j.set_axis(pyvjoy.HID_USAGE_X, 0x8000)
                elif 0.1 < contSlope < 1:
                    j.set_axis(pyvjoy.HID_USAGE_X, 0x2000)
                elif contSlope > 1:
                    j.set_axis(pyvjoy.HID_USAGE_X, 0x1)
                else:
                    # set the axis to centre position
                    j.set_axis(pyvjoy.HID_USAGE_X, 0x4000)

            # print(contSlope)
            box = np.int0(box)
            cv2.drawContours(frame, [box], 0, (0, 255, 25), 2)

    # else:
        # print("No shape found in blue channel.")

    if not(len(gContours) == 0):
        c = max(gApprox, key=cv2.contourArea)
        # print(len(c))
        if (7 <= len(c)) and 5000 > cv2.contourArea(c) > 1000:
            gButton = False
            gButtonDet = True
            gTimer = 0
            j.set_button(6, 0)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            center = (int(x), int(y))
            cv2.circle(frame, center, int(radius), (0, 255, 255), 2)
        else:
            gButton = True
            j.set_button(6, 1)
            gTimer += 1
    elif gButtonDet:
        gButton = True
        j.set_button(6, 1)
        gTimer += 1
    # else:
        # print("No shape found in green channel.")
    if gTimer >= 50:
        gButtonDet = False
        j.set_button(6, 0)
        print("gButton reset.")

    if not(len(rContours) == 0):
        c = max(rApprox, key=cv2.contourArea)
        if (7 <= len(c)) and 5000 > cv2.contourArea(c) > 1000:
            rButton = False
            rButtonDet = True
            rTimer = 0
            j.set_button(1, 0)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            center = (int(x), int(y))
            cv2.circle(frame, center, int(radius), (0, 255, 255), 2)
        else:
            rButton = True
            j.set_button(1, 1)
            rTimer += 1
    elif rButtonDet:
        rButton = True
        j.set_button(1, 1)
        rTimer += 1
    # else:
        # print("No shape found in red channel.")
    if rTimer >= 50:
        rButtonDet = False
        j.set_button(1, 0)
        print("rButton reset.")

    # showing frames
    # cv2.imshow('hsv', hsv)
    cv2.imshow('frame', frame)
    # cv2.imshow('gOut', gOut)
    cv2.imshow('rOut', rOut)
    # cv2.imshow('bMask', bMask)
    cv2.imshow('bOut', bOut)
    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
