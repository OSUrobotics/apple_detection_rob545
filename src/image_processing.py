import matplotlib
import os
import numpy as np
import plotly.express as px
import open3d as o3d
import pickle

from helper import *

def do_icp(scene_cloud, object):
    pass

################################################################################
# Main
################################################################################

ROOT_FOLDER = "/mnt/c/Users/rbbro/OneDrive/Documents/Volumes/apple_picking/"
DATA_FOLDER =  os.path.join(ROOT_FOLDER, "jun4/data")
OUTPUT =  os.path.join(ROOT_FOLDER, "output/")

RADI = ["Radius_35cm","Radius_45cm_1st_trial","Radius_50cm","Radius_60cm","Radius_40cm","Radius_45cm_2nd_trial","Radius_55cm","Radius_65cm"]
RADI = ["Radius_50cm"]
MAX_POSES = 20 

wrist_to_cam_tf = np.array([
    [0.70710678, -0.70710678, 0.0, 0.0221],
    [0.70710678, 0.70710678, 0.0, -0.05038],
    [0.0, 0.0, 1.0, 0.05223],
    [0.0, 0.0, 0.0, 1.0]
])

VISUALIZE = False
SAVE_FLAG = True

K = 14

# apple model for surface matching
print('Loading apple model...')
apple_file = os.path.join(ROOT_FOLDER, "solid_apple.ply")
apple_model_pc = o3d.io.read_point_cloud(apple_file)
print('Apple model loaded.')

# where to store al poitns after being shifted to base frame

radi_results = {}
for rad in RADI:
    radii_apple_points_base_frame = np.zeros((0,4))
    radii_stem_points_base_frame = np.zeros((0,4))
    radi_results[rad] = {}

    for POSE_NUM in range(MAX_POSES):
        pose_folder = os.path.join(DATA_FOLDER, rad, f'pose{POSE_NUM}')
        if not os.path.isdir(pose_folder):
            continue

        filepath    = os.path.join(pose_folder, f'rgb_pose{POSE_NUM}.bmp')
        depth_path  = os.path.join(pose_folder, f'depth_pose{POSE_NUM}.csv')
        cloud_path  = os.path.join(pose_folder, f'PointCloud_pose{POSE_NUM}.pickle')
        transform_path = os.path.join(pose_folder, f'transform_pose{POSE_NUM}.csv')

        # read stuff from files
        tranform_np = np_read_csv(transform_path)
        rgb_image = matplotlib.image.imread(filepath)
        with (open(cloud_path, "rb")) as openfile:
            u = pickle._Unpickler(openfile)
            u.encoding = 'latin1'
            cloud = u.load()
        cloud_rectangle = np.reshape(cloud, (rgb_image.shape[0], rgb_image.shape[1], cloud.shape[-1]))

        # extract the apple and stemp point cloud points
        apple_points, stem_points = get_apple_and_stem_points(rgb_image, cloud_rectangle, K, save_folder=OUTPUT)

        ###* filter out far away points
        min_thresh = .25
        max_thresh = .75
        apple_points = np.array([apple_points[a] for a in range(apple_points.shape[0]) if np.linalg.norm(apple_points[a]) < max_thresh and  np.linalg.norm(apple_points[a]) > min_thresh])
        stem_points = np.array([stem_points[a] for a in range(stem_points.shape[0]) if np.linalg.norm(stem_points[a]) < max_thresh and  np.linalg.norm(stem_points[a]) > min_thresh])

        def apply_transform(row):
            pt_base_frame = tranform_np @ np.transpose(row)
            return pt_base_frame

        ###*  Tranform apple points to base frame
        apple_points_base = np.copy(apple_points)
        apple_points_base[:,3] = 1 # 4th column must be 1 for the transforms
        apple_points_base = np.apply_along_axis(apply_transform, axis=1, arr=apple_points_base)
        apple_points_base[:,3] = apple_points[:,3] # copy the colors back in
        
        ###*  Tranform stem points to base frame
        stem_points_base = np.copy(stem_points)
        stem_points_base[:,3] = 1 # 4th column must be 1 for the transforms
        stem_points_base = np.apply_along_axis(apply_transform, axis=1, arr=stem_points_base)
        stem_points_base[:,3] = stem_points[:,3] # copy the colors back in

        # append the points from this pose to the combined array
        radii_apple_points_base_frame = np.append(radii_apple_points_base_frame, apple_points_base, axis=0)
        radii_stem_points_base_frame = np.append(radii_stem_points_base_frame, stem_points_base, axis=0)

        ###* Surface matching o3d
        trans_init = np.asarray([[-1, 0, 0, 1],
                                [0, 1, 0, 1],
                                [0, 0, 1, 1],
                                [0, 0, 0, 1]])
        apple_points_base_pc = o3d.geometry.PointCloud()
        apple_points_base_pc.points = o3d.utility.Vector3dVector(apple_points_base[:,:3])
        #evaluation = o3d.pipelines.registration.evaluate_registration(apple_points_base_pc, apple_model_pc,0.02, trans_init)
        #print(evaluation)

        reg_p2p = o3d.pipelines.registration.registration_icp(
            apple_points_base_pc, apple_model_pc, 3, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = 10000))

        
        # print(f"Radi {rad} | Pose {POSE_NUM}:")
        # print(f"\t{reg_p2p}")
        # print("\tTransformation is:")
        # print(f"{reg_p2p.transformation}")
        # print(f"\t{apple_points_base.shape[0]} apple points")
        # print(f"\t{stem_points_base.shape[0]} stem points")

        radi_results[rad][f"pose_{POSE_NUM}"] = reg_p2p

        if VISUALIZE:
            draw_registration_result(apple_points_base_pc, apple_model_pc, reg_p2p.transformation)

        ###* Visualize the points
        # if VISUALIZE:
        #     sub_fig = px.scatter_3d(x=apple_points_base[:,0], y=apple_points_base[:, 1], z=apple_points_base[:, 2], color=apple_points_base[:, 3])
        #     sub_fig.update_layout(scene = dict(aspectmode='data'))
        #     #fig.add_trace(sub_fig.data[0], row=1, col=2)
        #     sub_fig.show()

        #     sub_fig = px.scatter_3d(x=stem_points_base[:,0], y=stem_points_base[:, 1], z=stem_points_base[:, 2], color=stem_points_base[:, 3])
        #     sub_fig.update_layout(scene = dict(aspectmode='data'))
        #     #fig.add_trace(sub_fig.data[0], row=2, col=2)
        #     sub_fig.show()

        #     # combined plot
        #     # comb = np.append(stem_points, apple_points, axis=0)
        #     # fig = px.scatter_3d(x=comb[:,0], y=comb[:, 1], z=comb[:, 2], color=comb[:, 3])
        #     # fig.update_layout(scene = dict(aspectmode='data'))
        #     # fig.show()

    # fig = px.scatter_3d(x=radii_apple_points_base_frame[:,0], y=radii_apple_points_base_frame[:, 1], z=radii_apple_points_base_frame[:, 2], color=radii_apple_points_base_frame[:, 3])
    # fig.update_layout(scene = dict(aspectmode='data'))
    # fig.show()

    ###* Surface matching o3d for all the combined clouds at each radii
    trans_init = np.asarray([[-1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 0]])
    apple_points_base_pc = o3d.geometry.PointCloud()
    apple_points_base_pc.points = o3d.utility.Vector3dVector(radii_apple_points_base_frame[:,:3])
    #evaluation = o3d.pipelines.registration.evaluate_registration(apple_points_base_pc, apple_model_pc,0.02, trans_init)
    reg_p2p = o3d.pipelines.registration.registration_icp(
        apple_points_base_pc, apple_model_pc, 3, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = 10000))

    #draw_registration_result(apple_points_base_pc, apple_model_pc, reg_p2p.transformation)


# TODO:: loop across radi_results and average the error from the apple pos

for radi, poses in radi_results.items():
    for pose, icp_res in poses.items():
        print(f"Pose {pose}:")
        print(np.linalg.inv(icp_res.transformation))
        print("")
