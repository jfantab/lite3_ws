import apriltag
import cv2
import numpy as np
import pyrealsense2 as rs

def undistort_image(raw_image):
    return cv2.undistort(raw_image, K, D)

# From your /camera_info:
width = 1280  # Image width
height = 720   # Image height
K = np.array([[641.42, 0, width - 650.5],
             [0, 640.79, height - 354.92],
             [0, 0, 1]])
D = np.array([-0.0575, 0.0696, -0.00026, 0.000758, -0.0232])

# Configure pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

print("[INFO] configuring AprilTag parameters...")
options = apriltag.DetectorOptions(families='tag36h11')
detector = apriltag.Detector()

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        
        # Convert to OpenCV format
        image = np.asanyarray(color_frame.get_data())
        image = cv2.rotate(img, cv2.ROTATE_180)
        # image = cv2.flip(img, 0)
        # image = undistort_image(image)

        # Convert to gray
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect Apriltags
        results = detector.detect(gray)
        print("[INFO] {} total AprilTags detected".format(len(results)))

        # detection results
        for r in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            # draw the bounding box of the AprilTag detection
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)
            
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

            # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("[INFO] tag family: {}".format(tagFamily))

        # Display stream
        cv2.imshow('Color Feed', image)
        if cv2.waitKey(1) == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
