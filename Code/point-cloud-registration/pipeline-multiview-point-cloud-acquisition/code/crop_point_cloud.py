#!/usr/bin/env python3
import open3d as o3d

# Load original point cloud (never modified)
pcd = o3d.io.read_point_cloud("../data/test7/depth_6.ply")

# MANUALLY define the bounding box limits
min_bound = [-1000, -1000, 0]   # mm
max_bound = [1000, 500, 2500]   # mm

bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
bbox.color = (1, 0, 0)

# Crop → creates a NEW point cloud
cropped_pcd = pcd.crop(bbox)

# Save / visualize
o3d.io.write_point_cloud("../data/test7/depth_6_cropped.ply", cropped_pcd)
o3d.visualization.draw_geometries([pcd, cropped_pcd, bbox])


