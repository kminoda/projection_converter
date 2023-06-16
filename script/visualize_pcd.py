import argparse
import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np

def main(pcd_path):
    # Load a .pcd file
    pcd = o3d.io.read_point_cloud(pcd_path)

    # Downsample the point cloud
    voxel_size = 10.0  # Set this to a value that works well for your specific point cloud
    pcd_downsampled = pcd.voxel_down_sample(voxel_size)

    # Convert Open3D.o3d.geometry.PointCloud to numpy array
    points = np.asarray(pcd_downsampled.points)

    # Create a scatter plot
    fig, ax = plt.subplots()
    ax.scatter(points[:,0], points[:,1], s=1) # You may want to adjust the size parameter depending on your point cloud
    ax.set_aspect('equal', 'box')

    # Show the plot
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Point Cloud (X, Y) Scatter Plot')
    plt.savefig('point_cloud_scatter_plot.png') # save the figure to file
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot point cloud data from a .pcd file.")
    parser.add_argument('pcd_path', type=str, help="Path to the .pcd file.")

    args = parser.parse_args()

    main(args.pcd_path)