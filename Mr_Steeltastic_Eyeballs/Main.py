from cscore import CameraServer

# Import OpenCV and NumPy
import cv2
import numpy as np

def main():
    cs = CameraServer
    cs.enableLogging()

    # Capture  camera
    camera = cs.startAutomaticCapture()
    camera.setResolution(320, 240)


   
    cvSink = cs.getVideo() # Get a CvSink. This will capture images from the camera

    
    outputStream = cs.putVideo("Name", 320, 240) # (optional) Setup a CvSource. This will send images back to the Dashboard

  
    img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)  # Allocating new images is very expensive, always try to preallocate

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, img = cvSink.grabFrame(img)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError());
            # skip the rest of the current iteration
            continue

        cv2.cvtColor(mat, cv2.COLOR_RGB2GRAY, dst=grayMat)

        detections = detector.detect(grayMat)

        tags.clear()

        for detection in detections:
            # Remember the tag we saw
            tags.append(detection.getId())

            # Draw lines around the tag
            for i in range(4):
                j = (i + 1) % 4
                point1 = (int(detection.getCorner(i).x), int(detection.getCorner(i).y))
                point2 = (int(detection.getCorner(j).x), int(detection.getCorner(j).y))
                mat = cv2.line(mat, point1, point2, outlineColor, 2)

            # Mark the center of the tag
            cx = int(detection.getCenter().x)
            cy = int(detection.getCenter().y)
            ll = 10
            mat = cv2.line(
                mat,
                (cx - ll, cy),
                (cx + ll, cy),
                crossColor,
                2,
            )
            mat = cv2.line(
                mat,
                (cx, cy - ll),
                (cx, cy + ll),
                crossColor,
                2,
            )

            # Identify the tag
            mat = cv2.putText(
                mat,
                str(detection.getId()),
                (cx + ll, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                crossColor,
                3,
            )

            # Determine Tag Pose
            pose = estimator.estimate(detection)

            # put pose into dashboard
            rot = pose.rotation()
            tagsTable.getEntry(f"pose_{detection.getId()}").setDoubleArray(
                [pose.X(), pose.Y(), pose.Z(), rot.X(), rot.Y(), rot.Z()]
            )

        # Put List of Tags onto Dashboard
        pubTags.set(tags)

        # (optional) send some image back to the dashboard
        outputStream.putFrame(img)