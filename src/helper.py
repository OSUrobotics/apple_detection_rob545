
from sklearn.cluster import KMeans
import os
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv2
import struct

# Bit hack to bitwise convert float to three 8 bit ints
def float_to_rgb(f):
    s = struct.pack('>f', f)
    bits = struct.unpack('>l', s)[0]
    r = bits & 0xFF
    g = bits >> 8 & 0xFF
    b = bits >> 16 & 0xFF
    return (r,g,b)

################################################################################*
#* Image Segemntation Helper funcs
################################################################################*
def np_read_csv(depth_path):
    depth_np = np.loadtxt(depth_path, delimiter=',')
    return np.array(depth_np, dtype=np.float32)

def check_ids(mask, save_folder):
    """  Separates a clustered mask into images with only one of the IDs so 
    the user can determine which ID is the stem and which is the apple
    @param mask - mask with multiple different ids"""
    num_ids = np.max(mask)
    for ID in range(num_ids+1):
        mask_copy = np.zeros(mask.shape)
        mask_copy[mask == ID] = 1
        plt.imsave(os.path.join(save_folder, f'id_num{ID}.png'), mask_copy)
        #print("id out",ID)
        
def blur_im(target, img, save_fl=0):
    """  Dialates the image to reduce the effect of noise for higher efficiency
    in cluster matching
    @param target - folder name where images are saved
    @param img - image to be blurred
    @param save_fl - Bool to save images or not"""
    img_blur = cv2.blur(img, (7, 7))
    if save_fl == 1:
        plt.imsave(os.path.join(target,"img_dila.png"), img_blur)

    #print("blur_im_out")
    return img_blur


def RGB2YUV(target, image, save_fl=0):
    """  Changes image coordinates from RGB to YUV to improve accuracy
    @param target - folder name where images are saved
    @param image - image from real world
    @param save_fl - Bool to save images or not"""
    img_out = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
    if save_fl == 1:
        plt.imsave(os.path.join(target, "image_yuv.png"), img_out)
    return img_out

def Kmeanclus(target, image, parK=5, save_fl=0):
    """  Divides the pixels into groups based on their value and saves masks
    @param target - folder name where images are saved
    @param image - image from real world
    @param parK - number of clusters to generate
    @param save_fl - Bool to save images or not"""

    # Reshaping the image into a 2D array of pixels and 3 color values (RGB)
    pixel_vals = np.float32(image.reshape((-1, 3)))
    h, w, _ = image.shape

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    retval, labels, centers = cv2.kmeans(pixel_vals, parK, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
    
    labels_out = labels.reshape(h, w)
    centers_out = np.uint8(centers) # convert data back into 8-bit values 
    
    if save_fl == 1:
        plt.imsave(os.path.join(target, f"real_K{parK}_mod.png"), labels_out)
    return labels_out, centers_out

def kmeans_apple_and_stem(image, K, save_fl=1, save_folder='./'):
    """  Perform kmeans clustering and automatically assign the apple and stem colors
    @param real_image - real image in rgb"""
    image_dia = blur_im(save_folder, image, save_fl=1)
    image_yuv = RGB2YUV(save_folder, image_dia, save_fl=1)


    labels, centers = Kmeanclus(save_folder, image_yuv, K, save_fl=save_fl)
    #print(centers)

    check_ids(labels, save_folder)

    # find closest center to red
    red = np.array([36, 177, 114]) # the red apple in yuv
    dis = []
    for i in range(centers.shape[0]):
        dis.append(np.linalg.norm(centers[i] - red))
    #print(dis)
    apple_label = dis.index(min(dis))

    # find closest center to the blue tape
    blue = np.array([105,93,159]) # the blue stem in yuv
    dis = []
    for i in range(centers.shape[0]):
        dis.append(np.linalg.norm(centers[i] - blue))
    #print(dis)
    stem_label = dis.index(min(dis))

    # # extract just the stem and apple mask
    apple_indxs = np.where(labels == apple_label)
    stem_indxs = np.where(labels == stem_label)

    apple_stem_segmented = np.zeros(labels.shape)
    apple_stem_segmented[apple_indxs] = 2
    apple_stem_segmented[stem_indxs] = 1

    if save_fl:
        plt.imsave(os.path.join(save_folder,'segmented.png'), apple_stem_segmented)

    return apple_stem_segmented, apple_indxs, stem_indxs

################################################################################*
#* Apple - Stem extraction function
################################################################################*
def get_apple_and_stem_points(rgb_image, cloud_rectangle, K, save_folder='./'):
    """Return the (x,y,z,color) point cloud points that correspond
    to the apple and stem from the rgb_image

    Args:
        rgb_image (_type_): RGB Image from the realsense
        cloud_rectangle (_type_): Point cloud from the realsense that corresonds to the RGB
        load_model (bool, optional): Whether to train new kmeans model or load last one. Defaults to True.

    Returns:
        _type_: Returns two numpy arrays, one each for apple and stem points
    """

    ###* 1. kmeans
    apple_stem_segmented, apple_indxs, stem_indxs = kmeans_apple_and_stem(rgb_image, K=K, save_fl=1, save_folder=save_folder)

    ###* 2. Do blob detection on apple and stem masks
    #! not using anymore
    # apple_mask = np.copy(labels)
    # apple_mask[apple_mask == 1] = 0 # blank out the stem
    # apple_mask[apple_mask == 2] = 255
    # cv2.imwrite("apple_mask.png", apple_mask)
    # # blob detection params
    # params = cv2.SimpleBlobDetector_Params()
    # params.filterByColor = True
    # params.blobColor = 255
    # params.filterByConvexity = True
    # params.minConvexity = 0.8
    # detector = cv2.SimpleBlobDetector_create(params)
    # keypoints = detector.detect(apple_mask)
    # im_with_keypoints = cv2.drawKeypoints(apple_mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv2.imwrite("keypoints.png", im_with_keypoints)
    # cv2.waitKey(0)


    ###* 4. Get the apple and stem point cloud points
    apple_points = cloud_rectangle[apple_indxs[0], apple_indxs[1], :]
    #print(apple_points.shape)

    stem_points = cloud_rectangle[stem_indxs[0], stem_indxs[1], :]
    #print(stem_points.shape)

    return apple_points, stem_points