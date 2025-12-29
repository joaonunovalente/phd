#!/usr/bin/env python3

import os
import argparse
import pickle
import numpy as np
import open3d as o3d

def extract_transformation(matrix):
    translation = np.asarray(matrix[:3, 3])  # Ensure numpy array
    R = np.asarray(matrix[:3, :3])  # Convert rotation matrix to numpy array
    
    # Compute Euler angles (ZYX convention: yaw-pitch-roll)
    yaw = np.arctan2(R[1, 0], R[0, 0]).item()
    pitch = np.arcsin(-R[2, 0]).item()
    roll = np.arctan2(R[2, 1], R[2, 2]).item()
    
    # Convert to degrees
    yaw_deg = float(np.degrees(yaw))
    pitch_deg = float(np.degrees(pitch))
    roll_deg = float(np.degrees(roll))
    
    return translation.tolist(), (roll_deg, pitch_deg, yaw_deg)

def load_transformations(pickle_path):
    print(f"Looking for file at: {pickle_path}")
    with open(pickle_path, 'rb') as f:
        data = pickle.load(f)
    
    est_matrix = np.asarray(data.get("estimated-transformation"))
    ref_matrix = np.asarray(data.get("refined-transformation"))
    
    est_translation, est_angles = extract_transformation(est_matrix)
    ref_translation, ref_angles = extract_transformation(ref_matrix)
    
    return est_matrix, est_translation, est_angles, ref_matrix, ref_translation, ref_angles

def apply_transformation(source_ply, transformation_matrix):
    source = o3d.io.read_point_cloud(source_ply)
    source.transform(transformation_matrix)
    return source

def downsample_point_cloud(point_cloud, voxel_size=10):
    return point_cloud.voxel_down_sample(voxel_size)

def visualize_point_clouds(source_ply, transformed_ply, target_ply=None, voxel_size=10):
    source = o3d.io.read_point_cloud(source_ply)
    transformed = o3d.io.read_point_cloud(transformed_ply)
    
    source = downsample_point_cloud(source, voxel_size)
    transformed = downsample_point_cloud(transformed, voxel_size)
    
    point_clouds = [transformed]
    
    if target_ply:
        target = o3d.io.read_point_cloud(target_ply)
        target = downsample_point_cloud(target, voxel_size)
        target.paint_uniform_color([0, 0, 1])  # Red
        point_clouds.append(target)
        
    o3d.visualization.draw_geometries(point_clouds)

def main(test_number, point_cloud_1, point_cloud_2):
    # Construct paths
    pickle_path = f'../exhaustive-grid-search/results/test{test_number}/depth_{point_cloud_1}_cropped_depth_{point_cloud_2}_cropped.pickle'
    source_ply = f'../data/test{test_number}/depth_{point_cloud_1}_cropped.ply'
    target_ply = f'../data/test{test_number}/depth_{point_cloud_2}_cropped.ply'
    
    # Create results folder structure
    results_folder = f"../results/test{test_number}/"
    os.makedirs(results_folder, exist_ok=True)  # Create the results folder if it doesn't exist

    merged_ply = os.path.join(results_folder, f"pointcloud_{point_cloud_1}_{point_cloud_2}.ply")  # File to save merged point cloud
    transformed_ply = os.path.join(results_folder, "transformed.ply")  # Save transformed point cloud
    
    # Load transformations
    est_matrix, est_translation, est_angles, ref_matrix, ref_translation, ref_angles = load_transformations(pickle_path)
    
    print("--------- Estimated Transformation -----------")
    print(f"Translation: {est_translation}")
    print(f"Rotation (Roll, Pitch, Yaw): ({est_angles[0]:.2f}, {est_angles[1]:.2f}, {est_angles[2]:.2f})\n")
    
    print("--------- Refined Transformation -----------")
    print(f"Translation: {ref_translation}")
    print(f"Rotation (Roll, Pitch, Yaw): ({ref_angles[0]:.2f}, {ref_angles[1]:.2f}, {ref_angles[2]:.2f})")
    
    # Apply transformation to source
    transformed = apply_transformation(source_ply, ref_matrix)
    transformed.paint_uniform_color([1, 0, 0])  # Red
    
    # Save transformed point cloud
    o3d.io.write_point_cloud(transformed_ply, transformed)
    
    # Load target point cloud
    target = o3d.io.read_point_cloud(target_ply)
    target.paint_uniform_color([0, 0, 1])  # Blue

    # Merge point clouds
    merged_cloud = transformed + target

    # Save merged point cloud with the specified name
    o3d.io.write_point_cloud(merged_ply, merged_cloud)
    print(f"Merged point cloud saved to: {merged_ply}")
    
    # Visualize the merged point cloud
    visualize_point_clouds(source_ply, transformed_ply, target_ply, voxel_size=10)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Visualize point clouds based on test and point cloud numbers.')
    parser.add_argument('test_number', type=int, help='Test number (e.g., 1)')
    parser.add_argument('point_cloud_1', type=int, help='Point cloud number 1 (e.g., 1)')
    parser.add_argument('point_cloud_2', type=int, help='Point cloud number 2 (e.g., 2)')

    args = parser.parse_args()
    main(args.test_number, args.point_cloud_1, args.point_cloud_2)

