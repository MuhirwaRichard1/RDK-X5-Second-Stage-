import cv2

cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

print("Opened:", cap.isOpened())

if cap.isOpened():
    ret, frame = cap.read()
    print("Frame received:", ret)

    if ret:
        print("Frame size:", frame.shape)

cap.release()