"""Module providingFunction PLY file viewing and editing"""
import numpy as np
import open3d as o3d

def main():
    """Main function"""
    # Look at a PLY file
    # Read in the PLY file
    pcd = o3d.io.read_point_cloud("out.ply")
    # Print the amount of points in the point cloud
    print(pcd)
    # Print the first 20 points of the point cloud
    for cords_20_points in pcd.points[0:20]:
        print(cords_20_points)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])



    # Look at a PLY file and show a section (sphere shape) of the point cloud
    # Read in the PLY file
    pcd = o3d.io.read_point_cloud("out.ply")
    # Print the amount of points in the point cloud
    print(pcd)
    # # Print the points of the point cloud
    for cords_all in pcd.points:
        print(cords_all)

    # Safe the points of the old point cloud
    pointss = np.asarray(pcd.points)

    # Centre point and the radius of the sphere
    center = np.array([0, 0, 0])
    radius = 21

    # Radius of the sphere
    distances = np.linalg.norm(pointss - center, axis=1)
    # Overwrite the old point cloud with the new point cloud
    pcd.points = o3d.utility.Vector3dVector(pointss[distances <= radius])

    # Write the new point cloud to a PLY file
    o3d.io.write_point_cloud("outt.ply", pcd)

    # Read the new PLY file
    pcd2 = o3d.io.read_point_cloud("outt.ply")
    # Print the amount of points in the point cloud
    print(pcd2)
    # Print the points of the point cloud
    for cords_all_section in pcd2.points:
        print(cords_all_section)

    # Visualize the section of the point cloud
    o3d.visualization.draw_geometries([pcd2],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])

if __name__ == "__main__":
    main()