
import cv2
import numpy as np


Directory = '2022Fheldout'



resize_h = int(320 * .1)
resize_w = int(240 * .1)
shape = resize_w*resize_h*3

def findObjects(image):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  
    cont = sorted(contours, key=cv2.contourArea, reverse=True)[:1]  
    x = y = w = h = 1
    # Find the center of mass of the blob if there are any
    if len(cont) > 0:
        M = cv2.moments(cont[0])
        if M['m00'] > 250:
            x, y, w, h = cv2.boundingRect(cont[0])
    return x, y, w, h


def filter_colors(image):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define color ranges for red, green, and blue in HSV
    lower_red = np.array([0, 100, 45])
    upper_red = np.array([225, 250, 255])  # Increased red range
    lower_green = np.array([36, 0, 0])
    upper_green = np.array([86, 255, 255])
    lower_blue = np.array([110,50,50])  # Decreased lower limit for blue
    upper_blue = np.array([130, 255, 255])  # Increased upper limit for blue

    # Create masks for red, green, and blue colors
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Combine the masks
    mask = cv2.bitwise_or(mask_red, mask_green)
    mask = cv2.bitwise_or(mask, mask_blue)

    # Apply the mask to the image
    filtered_image = cv2.bitwise_and(image, image, mask=mask)
 # Perform morphological operations to reduce noise
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Convert non-zero pixels to white in the final filtered image
    filtered_image = np.where(mask == 0, 255, 0).astype(np.uint8)

    return filtered_image


def preprocess(img):
    # downsample to speed up
    img = cv2.resize(img, (int(0.3*320), int(0.3*240)))
    img_rgb = img
    # filter the background and noise
    img = filter_colors(img)
    img = cv2.Canny(img, 100, 150)
    element = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
    fix = cv2.dilate(img, element, iterations=1)
    # Fill in holes
    img = cv2.morphologyEx(fix, cv2.MORPH_CLOSE, np.ones((8, 8), np.uint8))
    x, y, w, h = findObjects(img)
    # crop the original image to the object area
    img = img_rgb[y:y + h, x:x + w]
    img = cv2.resize(img, (resize_w, resize_h))

    return img



#test the preprocess function for all png in one folder, store the result img in another folder
# if __name__ == '__main__':
#     import os
#     # read all the png in the folder
#     path = './'+Directory+'/'
#     files = os.listdir(path)
#     for file in files:
#         if file.endswith('.png'):
#             img = cv2.imread(os.path.join(path, file))
#             img = preprocess(img)
#             #store the img in data folder
#             cv2.imwrite(os.path.join('./preprocess/', file), img)
#             print('finish', file)
#     print('all finished')

# test the preprocess function for one image
# if __name__ == '__main__':
#     img = cv2.imread('./2023Fimgs/2023Fimgs/37.jpg')
#     img = preprocess(img)
#     cv2.imshow('img', img)
#     cv2.waitKey(0) 
#     cv2.destroyAllWindows()
    