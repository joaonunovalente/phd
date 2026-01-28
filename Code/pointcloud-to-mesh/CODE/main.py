#!/usr/bin/env python3
import open3d as o3d
import copy
import numpy as np
from pathlib import Path


# ============================================================
# Configuration
# ============================================================

INPUT_PLY = Path("../DATA/point-cloud.ply")
OUTPUT_DIR = Path("../RESULTS")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

VOXEL_SIZE = 0.01
OUTLIER_NB_NEIGHBORS = 30
OUTLIER_STD_RATIO = 2.0

NORMAL_RADIUS_MULT = 15
NORMAL_MAX_NN = 50
NORMAL_ORIENT_K = 50

POISSON_DEPTH = 9
POISSON_SCALE = 1.1
POISSON_DENSITY_QUANTILE = 0.05

LOD_TARGETS = [100_000, 50_000, 10_000]


# ============================================================
# Utilities
# ============================================================

def clean_mesh(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    return mesh


def export_lods(mesh, targets, name):
    for t in targets:
        lod = mesh.simplify_quadric_decimation(t)
        o3d.io.write_triangle_mesh(
            OUTPUT_DIR / f"{name}_lod_{t}.ply",
            lod
        )


def generate_lods(mesh, targets):
    lods = []
    for t in targets:
        lod = mesh.simplify_quadric_decimation(t)
        lod.compute_vertex_normals()
        lod.paint_uniform_color([0.9, 0.9, 0.9])
        lods.append((t, lod))
    return lods


# ============================================================
# Load & Preprocess Point Cloud
# ============================================================

pcd = o3d.io.read_point_cloud(str(INPUT_PLY))
assert len(pcd.points) > 0, "Point cloud is empty!"

pcd.remove_non_finite_points()
pcd = pcd.voxel_down_sample(VOXEL_SIZE)

pcd, _ = pcd.remove_statistical_outlier(
    nb_neighbors=OUTLIER_NB_NEIGHBORS,
    std_ratio=OUTLIER_STD_RATIO
)

pcd.remove_non_finite_points()

pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=VOXEL_SIZE * NORMAL_RADIUS_MULT,
        max_nn=NORMAL_MAX_NN
    )
)

pcd.normalize_normals()
pcd.orient_normals_consistent_tangent_plane(NORMAL_ORIENT_K)


# ============================================================
# Strategy 1: Ball Pivoting
# ============================================================

distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)

radii = [avg_dist * 1.5, avg_dist * 3.0, avg_dist * 6.0]

bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd,
    o3d.utility.DoubleVector(radii)
)

bpa_mesh = clean_mesh(bpa_mesh)
bpa_mesh.compute_vertex_normals()
bpa_mesh.paint_uniform_color([0.7, 0.8, 1.0])

o3d.io.write_triangle_mesh(
    OUTPUT_DIR / "mesh_bpa.ply",
    bpa_mesh
)


# ============================================================
# Strategy 2: Poisson Reconstruction
# ============================================================

poisson_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    pcd,
    depth=POISSON_DEPTH,
    scale=POISSON_SCALE,
    linear_fit=False
)

# Crop to input bounds
bbox = pcd.get_axis_aligned_bounding_box()
poisson_mesh = poisson_mesh.crop(bbox)

poisson_mesh = clean_mesh(poisson_mesh)
poisson_mesh.compute_vertex_normals()
poisson_mesh.paint_uniform_color([0.9, 0.9, 0.9])

o3d.io.write_triangle_mesh(
    OUTPUT_DIR / "mesh_poisson.ply",
    poisson_mesh
)


# ============================================================
# Visualization: BPA vs Poisson
# ============================================================

bpa_vis = bpa_mesh.translate([-1.2, 0, 0], relative=False)
poisson_vis = poisson_mesh.translate([1.2, 0, 0], relative=False)

o3d.visualization.draw_geometries(
    [bpa_vis, poisson_vis],
    window_name="BPA (Left) vs Poisson (Right)"
)


# ============================================================
# LOD Visualization (Poisson)
# ============================================================

poisson_lods = generate_lods(poisson_mesh, LOD_TARGETS)

translated = []
offset = 0.0

for _, lod in poisson_lods:
    lod.translate([offset, 0, 0], relative=False)
    translated.append(lod)
    offset += lod.get_axis_aligned_bounding_box().get_extent()[0] * 1.2

o3d.visualization.draw_geometries(
    translated,
    window_name="Poisson LODs"
)


# ============================================================
# Export LODs
# ============================================================

export_lods(bpa_mesh, LOD_TARGETS, "bpa")
export_lods(poisson_mesh, LOD_TARGETS, "poisson")


# ============================================================
# Export BPA vs Poisson visualization meshes
# ============================================================

scene_mesh = copy.deepcopy(bpa_mesh)
scene_mesh.translate([-1.2, 0, 0], relative=False)

poisson_scene = copy.deepcopy(poisson_mesh)
poisson_scene.translate([1.2, 0, 0], relative=False)

scene_mesh += poisson_scene
scene_mesh = clean_mesh(scene_mesh)

o3d.io.write_triangle_mesh(
    OUTPUT_DIR / "scene_bpa_vs_poisson.ply",
    scene_mesh
)

print("Meshing complete")
