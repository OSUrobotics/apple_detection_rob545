import matplotlib
#import imageio
from sklearn.cluster import KMeans
import os
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv2
from plotly.subplots import make_subplots
import plotly.express as px
import pandas as pd
import open3d as o3d


################################################################################*
#* Image Segemntation Helper funcs
################################################################################*
def np_read_csv(depth_path):
    depth_np = np.loadtxt(depth_path, delimiter=',')
    return np.array(depth_np, dtype=np.float32)

def check_ids(mask):
    """  Separates a clustered mask into images with only one of the IDs so 
    the user can determine which ID is the stem and which is the apple
    @param mask - mask with multiple different ids"""
    num_ids = np.max(mask)
    for ID in range(num_ids+1):
        mask_copy = np.zeros(mask.shape)
        mask_copy[mask == ID] = 1
        plt.imsave(f'{OUTPUT}/id_num{ID}.png', mask_copy)
        #print("id out",ID)
        
def blur_im(target, img, save_fl=0):
    """  Dialates the image to reduce the effect of noise for higher efficiency
    in cluster matching
    @param target - folder name where images are saved
    @param real_img - real image to be blurred
    @param save_fl - Bool to save images or not"""
    img_blur = cv2.blur(img, (7, 7))
    if save_fl == 1:
        plt.imsave(os.path.join(target,"img_dila.png"), img_blur)

    #print("blur_im_out")
    return img_blur


def RGB2YUV(target, real, save_fl=0):
    """  Changes image coordinates from RGB to YUV to improve accuracy
    @param target - folder name where images are saved
    @param real - image from real world
    @param save_fl - Bool to save images or not"""
    real_out = cv2.cvtColor(real, cv2.COLOR_BGR2YUV)
    if save_fl == 1:
        plt.imsave(target + "/image_yuv.png", real_out)
    return real_out

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
        plt.imsave(f"{target}/real_K{parK}_mod.png", labels_out)
    return labels_out, centers_out

def useKmeans(real_image, params=None, save_fl = 0):
    """  Divides the pixels into groups based on their value and saves masks
    @param image_yuv - real image in yuv
    @param params - center and std_dev from kmeans clustering for real im
    @param save_fl - Bool to save images or not"""

    real_dia = blur_im(OUTPUT, real_image, save_fl=1)
    image_yuv = RGB2YUV(OUTPUT, real_dia, save_fl=1)

    if params is None:
        params = np.load(OUTPUT + 'params.npy')

    #print(params[0][0])
    
    Z = 2
    stem_mins      = params[0][0] - Z*np.sqrt(params[0][1])
    stem_maxes     = params[0][0] + Z*np.sqrt(params[0][1])
    apple_mins     = params[1][0] - Z*np.sqrt(params[1][1])
    apple_maxes    = params[1][0] + Z*np.sqrt(params[1][1])

    # extract just the stem and apple mask
    stem_mask = cv2.inRange(image_yuv, stem_mins, stem_maxes)
    stem_mask[stem_mask > 0] = 1

    apple_mask = cv2.inRange(image_yuv, apple_mins, apple_maxes)
    apple_mask[apple_mask > 0] = 2

    real_mask = apple_mask + stem_mask

    if save_fl:
        plt.imsave(OUTPUT + 'real_segmented.png', real_mask)
    return real_mask
    
def find_kmeans_params(real_image, K=None):
    """  Finds the mean and std dev from kmeans clustering for stem and apple 
    in both real and rviz images. Need user input to find correct ID
    @param real_image - real image in rgb"""
    real_dia = blur_im(OUTPUT, real_image, save_fl=1)
    image_yuv = RGB2YUV(OUTPUT, real_dia, save_fl=1)
    real_masks = []
    params = []
    if K is None:
        for j in range(6):
            labels, centers = Kmeanclus(OUTPUT, image_yuv, j+4, save_fl=1)
            real_masks.append(labels)
            params.append(centers)
            print('kmean', j+4)
            #print(params)
        a = input('which of the 6 is best for the real images?')
        a = int(a)
        labels = real_masks[a-4]
    else:
        labels, centers = Kmeanclus(OUTPUT, image_yuv, K, save_fl=1)
        real_masks.append(labels)
        params.append(centers)
        print('kmean', K)
        a = K

    check_ids(labels)
    stem_label = input('which id is the stem?')
    stem_label = int(stem_label)
    apple_label = input('which id is the apple?')
    apple_label = int(apple_label)
    apple_colors = []
    stem_colors = []
    for i in range(labels.shape[0]):
        for j in range(labels.shape[1]):
            if labels[i, j] == apple_label:
                apple_colors.append(image_yuv[i][j])

            elif labels[i, j] == stem_label:
                stem_colors.append(image_yuv[i][j])
    apple_var = np.var(apple_colors, axis = 0)
    stem_var = np.var(stem_colors, axis = 0)
    if len(params) > 1:
        params = np.array([[params[a-4][stem_label], stem_var], [params[a-4][apple_label], apple_var]])
    else:
        params = np.array([[params[0][stem_label], stem_var], [params[0][apple_label], apple_var]])
    np.save(OUTPUT + 'params.npy', params)
   
    return params

################################################################################*
#* Apple - Stem extraction function
################################################################################*
def get_apple_and_stem_points(rgb_image, cloud_rectangle, load_model=True):
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
    if not load_model:
        ###* A. Option 1 - fit new model
        # there is an interactive component where you assign a kmeans group to the apple/stem
        params = find_kmeans_params(rgb_image, K=9)
        labels = useKmeans(rgb_image, params, save_fl=save_flag) 
    else:
        ###* B. Option 2 - Use the previously saved kmeans model
        labels = useKmeans(rgb_image, save_fl=save_flag)

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

    ###* 3. Grab the image indexes for the apples and stems
    apple_indxs = np.where(labels == 2)
    stem_indxs = np.where(labels == 1)

    ###* 4. Get the apple and stem point cloud points
    apple_points = cloud_rectangle[apple_indxs[0], apple_indxs[1], :]
    #print(apple_points.shape)

    stem_points = cloud_rectangle[stem_indxs[0], stem_indxs[1], :]
    #print(stem_points.shape)

    return apple_points, stem_points


##################################
# Main
#################################
DATA_FOLDER = "/mnt/apple_picking/collected_5_12/"
OUTPUT = "/mnt/apple_picking/output/"
VISUALIZE = True
save_flag = 1
POSES = [2]

# apple model for surface matching
# TODO: invesigate why ppf_match_3d is not found in the cv2 module
# print('Loading apple model...')
apple_model_pc = np.asarray(o3d.io.read_point_cloud("/mnt/apple_picking/apple.ply").points)
# detector = cv2.ppf_match_3d_PPF3DDetector(0.025, 0.05)
# detector.trainModel(pc)
# print('Apple model loaded.')

# where to store al poitns after being shifted to base frame
all_apple_points_base_frame = np.zeros((0,4))
all_stem_points_base_frame = np.zeros((0,4))

# loop through each pose
for POSE_NUM in POSES:
    filepath    = os.path.join(DATA_FOLDER, f'pose{POSE_NUM}', f'rgb_pose{POSE_NUM}.bmp')
    depth_path  = os.path.join(DATA_FOLDER, f'pose{POSE_NUM}', f'depth_pose{POSE_NUM}.csv')
    cloud_path  = os.path.join(DATA_FOLDER, f'pose{POSE_NUM}', f'PointCloud_pose{POSE_NUM}.csv')
    transform_path = os.path.join(DATA_FOLDER, f'pose{POSE_NUM}', f'transform_pose{POSE_NUM}.csv')

    # read stuff from files
    tranform_np = np_read_csv(transform_path)
    rgb_image = matplotlib.image.imread(filepath)
    cloud = np_read_csv(cloud_path)
    cloud_rectangle = np.reshape(cloud, (rgb_image.shape[0], rgb_image.shape[1], cloud.shape[-1]))

    # extract the apple and stemp point cloud points
    apple_points, stem_points = get_apple_and_stem_points(rgb_image, cloud_rectangle, load_model=True)

    ###*  Tranform apple points to base frame
    def apply_transform(row):
        pt_base_frame = tranform_np @ np.transpose(row)
        return pt_base_frame

    apple_points_base = np.copy(apple_points)
    apple_points_base[:,3] = 1 # 4th column must be 1 for the transforms
    apple_points_base = np.apply_along_axis(apply_transform, axis=1, arr=apple_points_base)
    apple_points_base[:,3] = apple_points[:,3] # copy the colors back in
    #print(apple_points_base.shape)
    
    ###*  Tranform stem points to base frame
    stem_points_base = np.copy(stem_points)
    stem_points_base[:,3] = 1 # 4th column must be 1 for the transforms
    stem_points_base = np.apply_along_axis(apply_transform, axis=1, arr=stem_points_base)
    stem_points_base[:,3] = stem_points[:,3] # copy the colors back in
    #print(stem_points_base.shape)

    ###* Surface matching for apple
    #TODO: figure out why this surface matching module isnt found (maybe it isnt in python 3.6?)
    # icp = cv2.ppf_match_3d_ICP(100)
    # I = np.eye(4)
    # retval, residual, pose = icp.registerModelToScene(apple_model_pc, apple_points)
    # print(pose)

    ###* Visualize the points
    if VISUALIZE:
        fig = make_subplots(
            rows=2, cols=2,
            subplot_titles=("Apple - End Frame", "Apple - Base Frame","Stem - End Frame", "Stem - Base Frame"),
            specs=[[{'type': 'surface'}, {'type': 'surface'}],[{'type': 'surface'}, {'type': 'surface'}]]
        )
        fig.add_trace(px.scatter_3d(x=apple_points[:,0], y=apple_points[:, 1], z=apple_points[:, 2], color=apple_points[:, 2]).data[0], row=1, col=1)
        fig.add_trace(px.scatter_3d(x=apple_points_base[:,0], y=apple_points_base[:, 1], z=apple_points_base[:, 2], color=apple_points_base[:, 2]).data[0], row=1, col=2)
        fig.add_trace(px.scatter_3d(x=stem_points[:,0], y=stem_points[:, 1], z=stem_points[:, 2], color=stem_points[:, 2]).data[0], row=2, col=1)
        fig.add_trace(px.scatter_3d(x=stem_points_base[:,0], y=stem_points_base[:, 1], z=stem_points_base[:, 2], color=stem_points_base[:, 2]).data[0], row=2, col=2)
        fig.show()

    # append the points from this pose to the combined array
    all_apple_points_base_frame = np.append(all_apple_points_base_frame, apple_points_base, axis=0)
    all_stem_points_base_frame = np.append(all_stem_points_base_frame, stem_points_base, axis=0)

    print(f"Pose {POSE_NUM}:")
    print(f"\t{apple_points_base.shape[0]} apple points")
    print(f"\t{stem_points_base.shape[0]} stem points")
    print("")


print(f"Total:")
print(f"\t{all_apple_points_base_frame.shape[0]} apple points")
print(f"\t{all_stem_points_base_frame.shape[0]} stem points")
fig = make_subplots(
    rows=1, cols=2,
    subplot_titles=("All Apple - Base Frame", "All Stem - Base Frame"),
    specs=[[{'type': 'surface'},{'type': 'surface'}]]
)
fig.add_trace(
    px.scatter_3d(
        x=all_apple_points_base_frame[:,0], 
        y=all_apple_points_base_frame[:, 1], 
        z=all_apple_points_base_frame[:, 2]).data[0], 
    row=1, col=1)
fig.add_trace(
    px.scatter_3d(
        x=all_stem_points_base_frame[:,0], 
        y=all_stem_points_base_frame[:, 1], 
        z=all_stem_points_base_frame[:, 2]).data[0], 
    row=1, col=2)
fig.show()
