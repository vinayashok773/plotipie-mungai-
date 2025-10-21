import cv2
import numpy as npls


def extract_lines_from_image(image_path, show=True, use_lsd=True):
    # Read and preprocess image
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Adaptive Thresholding + Gaussian Blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    adaptive = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2
    )

    # Edge detection
    edges = cv2.Canny(adaptive, 50, 150, apertureSize=3)

    # HoughLinesP for line detection
    hough_lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=5,        # lower = more sensitive
        minLineLength=7,     # lower = detect short lines
        maxLineGap=5       # higher = connect broken lines
    )

    line_segments = []
    output_img = img.copy()

    if hough_lines is not None:
        for line in hough_lines:
            x1, y1, x2, y2 = line[0]
            line_segments.append([x1, y1, x2, y2])
            if show:
                cv2.line(output_img, (x1, y1), (x2, y2), (0, 255, 0), 1)

    # Optional: Use Line Segment Detector (LSD) to find more subtle lines
    if use_lsd:
        lsd = cv2.createLineSegmentDetector(0)
        detected_lines = lsd.detect(gray)[0]
        if detected_lines is not None:
            for dline in detected_lines:
                x1, y1, x2, y2 = map(int, dline[0])
                line_segments.append([x1, y1, x2, y2])
                if show:
                    cv2.line(output_img, (x1, y1), (x2, y2), (255, 0, 0), 1)  # LSD lines in blue

    if show:
        cv2.imshow("Detected Lines", output_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return line_segments

# Example usage
if __name__ == "__main__":
    lines = extract_lines_from_image("resource/img_3.png", show=True, use_lsd=True)
    print("Detected line segments:")
    print(lines)  # 2D array of [x1, y1, x2, y2]
