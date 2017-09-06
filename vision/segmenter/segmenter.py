import numpy as np
#from matplotlib import pyplot as plt
import cv
import cv2
import sys
import time
import os
import platform

TESTING = platform.platform().find("armv7") == -1
if TESTING:
    print "NOT ON PI, TESTING MODE ENABLED"

class Segmenter:
    def cut_from_image(self, frame, padding, x, y, w, h):
        x_min = max(0, x - padding)
        x_max = min(np.size(frame, 1), x + w + padding * 2)
        y_min = max(0, y - padding)
        y_max = min(np.size(frame, 1), y + h + padding * 2)

        return frame[y_min:y_max, x_min:x_max]

    def process_frame(self, original_frame):
        bounding_box_highlight_frame = original_frame.copy()
        frame = original_frame.copy()

        # Apply various filters to the image to make the targets stand out.
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.bitwise_not(frame)
        blur = cv2.GaussianBlur(frame, (15, 15), 5)
        thresh = cv2.adaptiveThreshold(blur, 255,
                 cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 1)
        if TESTING:
            cv2.imshow("Threshold", thresh)
            cv2.moveWindow("Threshold", 400, 20)

        # Generate a list of contours that stand out from the image after the
        # above filters were applied.
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)

        possible_shapes = list()
        i = 0

        frame_height, frame_width  = frame.shape[:2]
        for cnt in contours:
            if len(cnt) < 5:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            area = (w / float(frame_width)) * (h / float(frame_height)) * 100.0
            aspect_ratio = w / float(h)

            reject = False
            if area < 0.01 or area > 1:
                reject = True

            aspect_ratio_tolerance = 0.5
            if abs(aspect_ratio - 1) > aspect_ratio_tolerance:
                reject = True

            extracted_contour = self.cut_from_image(original_frame, 10, x, y, \
                    w, h)

            hist_image = np.zeros((300, 256, 3))
            bins = np.arange(256).reshape(256, 1)
            color = [(255,0,0), (0,255,0), (0,0,255)]

            if not reject:
                histogram_accept = False
                for ch, col in enumerate(color):
                    hist_item = cv2.calcHist([extracted_contour], [ch], None, \
                            [256], [0,255])
                    hist = np.int32(np.around(hist_item))
                    if hist_item.max() > 80:
                        histogram_accept = True

                    pts = np.column_stack((bins, hist))
                    cv2.polylines(hist_image, [pts], False, col)

                if not histogram_accept:
                    reject = True

            if TESTING:
                r = 0
                g = 0
                if reject is False:
                    g = 255  # Good contour :)
                    i = i + 1
                else:
                    r = 255  # Bad contour :(

                cv2.rectangle(bounding_box_highlight_frame, (x, y), \
                        (x + w, y + h), (0, g, r), 1)

                hist_image = np.flipud(hist_image)

            if not reject:
                possible_shapes.append(extracted_contour)

        print str(len(possible_shapes)) \
                + " good contours were found! (" \
                + str(len(contours)) \
                + " were filtered out)"
        if TESTING:
            cv2.imshow("Contour Locations", bounding_box_highlight_frame)
            cv2.moveWindow("Contour Locations", 400, 20)

        return possible_shapes

    def use_webcam_stream(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 720)

        while True:
            ret, frame = cap.read()
            process_frame(frame)

            if TESTING:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cap.release()
        if TESTING:
            cv2.destroyAllWindows()

    def use_given_image(self):
        frame = cv2.imread(sys.argv[1], cv2.CV_LOAD_IMAGE_COLOR)
        if frame is None:
            print "Image given has no data to analyze."
            sys.exit(1)
        grey_scale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        possible_shapes = self.process_frame(frame);
        if TESTING:
            i = 0
            for shape in possible_shapes:
                cv2.imshow("contour" + str(i), shape)
                cv2.moveWindow("contour" + str(i), (i % 3) * 90, \
                        (i / 3) * 90)
                i = i + 1
                if i > 36:
                    print "CLIPPING IMAGE SEGMENT PREVIEWS!!!"
                    break

            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def use_deque_stream(self, save_directory, photos):
        while True:
            if len(photos) == 0:
                time.sleep(1 / 100.0)
                continue

            raw_image_path = photos.pop() + ".jpg"
            print "Segmenting " + raw_image_path
            frame = cv2.imread(raw_image_path, cv2.CV_LOAD_IMAGE_COLOR)
            grey_scale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            possible_shapes = self.process_frame(frame);

            # Make a new folder just for this image's segments.
            image_folder_name = os.path.basename(raw_image_path)
            image_folder_name = image_folder_name.replace(".jpg", "")
            image_folder_name = save_directory + "/" + image_folder_name
            print image_folder_name
            if os.path.isdir(image_folder_name) is True:
                print "Segments directory out of sync."
                sys.exit(1)
            os.makedirs(image_folder_name)

            # Output the images.
            i = 0
            for shape in possible_shapes:
                image_path = image_folder_name + "/" + str(i).zfill(5) + ".jpg"
                cv2.imwrite(image_path, shape)
                i = i + 1

def main():
    if len(sys.argv) > 1:
        segmenter = Segmenter()
        segmenter.use_given_image()

if __name__ == "__main__":
    main()
