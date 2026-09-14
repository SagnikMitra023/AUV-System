import cv2
import numpy as np
import requests

# Function to fetch image from ESP cam
def fetch_image(url):
    try:
        response = requests.get(url)
        if response.status_code == 200:
            img_array = np.array(bytearray(response.content), dtype=np.uint8)
            img = cv2.imdecode(img_array, -1)
            return img
        else:
            print("Error: Failed to fetch image from", url)
            return None
    except Exception as e:
        print("Error:", e)
        return None

# URLs for ESP cams
esp_cam1_url = "http://esp_cam1_ip_address:port/image"
esp_cam2_url = "http://esp_cam2_ip_address:port/image"

def trace_inner_moore_boundary(contour):
    B = []
    start_point = tuple(contour[0][0])
    B_x = start_point[0]
    B_y = start_point[1]
    B.append((B_x, B_y))

    b_x = start_point[0]
    b_y = start_point[1] - 1
    d_x = b_x - B_x
    d_y = b_y - B_y

    OffsetTable = [(0, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1)]

    count = 1
    prev_x = B_x
    prev_y = B_y
    while count <= 2:
        for i in range(9):
            if d_x == OffsetTable[i][0] and d_y == OffsetTable[i][1]:
                id = i
                break
        while True:
            if id == 8:
                id = 0
            c_x = B_x + OffsetTable[id + 1][0]
            c_y = B_y + OffsetTable[id + 1][1]
            if (c_x, c_y) == start_point:
                count += 1
            if count > 2:
                break
            temp = (c_x, c_y)
            if temp in B:
                count += 1
            B.append((c_x, c_y))
            d_x = prev_x - c_x
            d_y = prev_y - c_y
            prev_x = c_x
            prev_y = c_y
            id += 1

    return B

# Function to capture screenshots with inner Moore boundaries
def capture_screenshots():
    while True:
        # Fetch images from ESP cams
        img1 = fetch_image(esp_cam1_url)
        img2 = fetch_image(esp_cam2_url)

        if img1 is None or img2 is None:
            print("Error: Unable to fetch images from ESP cams.")
            break

        # Your image processing code for both images
        # For example, you can apply Canny edge detection on one image and inner Moore boundaries on the other
        
        # Convert images to grayscale
        gray_img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray_img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        
        # Apply Canny edge detection on one image
        edges_img1 = cv2.Canny(gray_img1, 50, 150)

        # Apply inner Moore boundaries on the other image
        _, thresh_img2 = cv2.threshold(gray_img2, 135, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh_img2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_image = np.zeros_like(gray_img2)
        cv2.drawContours(contour_image, contours, -1, (255), 2)

        # Display images on separate screens
        cv2.imshow('ESP Cam 1 - Canny Edge Detection', edges_img1)
        cv2.imshow('ESP Cam 2 - Inner Moore Boundaries', contour_image)
        # Trace and draw inner Moore boundaries for each contour
        for contour in contours:
            inner_moore_boundary = trace_inner_moore_boundary(contour)
            for point in inner_moore_boundary:
                row_index, col_index = point
                if 0 <= row_index < contour_image.shape[0] and 0 <= col_index < contour_image.shape[1]:
                    contour_image[row_index, col_index] = 0

        # Break the loop when 'q' key is pressed
        if cv2.waitKey(30) == ord('q'):
            break

    cv2.destroyAllWindows()

# Call the function to capture and display screenshots
capture_screenshots()
