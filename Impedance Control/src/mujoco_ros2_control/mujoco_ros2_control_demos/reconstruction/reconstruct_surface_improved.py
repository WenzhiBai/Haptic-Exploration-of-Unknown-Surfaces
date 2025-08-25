import os
import glob
import pandas as pd
import open3d as o3d
import numpy as np

def find_latest_file(pattern):
    """
    Finds the file with the latest timestamp that matches the given pattern.
    """
    list_of_files = glob.glob(pattern)
    if not list_of_files:
        print(f"No files found matching the pattern: {pattern}")
        return None
    latest_file = max(list_of_files, key=os.path.getctime)
    return latest_file

def main():
    """
    Main function to load point cloud, reconstruct the surface with improved quality, and save it.
    """

    file_pattern = "scan_pointcloud_*.csv"
    latest_file = find_latest_file(file_pattern)

    if not latest_file:
        return

    print(f"Using the latest point cloud file: {latest_file}")


    try:
        df = pd.read_csv(latest_file)
        if 'x' in df.columns and 'y' in df.columns and 'z' in df.columns:
            points = df[['x', 'y', 'z']].values
        elif df.shape[1] >= 5: 
            points = df.iloc[:, 2:5].values
        else:
            print("CSV file does not contain the required 'x,y,z' columns.")
            return

    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    if points.size == 0:
        print("Loaded point cloud is empty.")
        return


    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    print(f"Loaded {len(pcd.points)} points.")
    print("Preprocessing point cloud...")
    

    print("  - Removing outliers...")
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    

    print("  - Downsampling...")
    pcd = pcd.voxel_down_sample(voxel_size=0.01)
    print(f"After preprocessing: {len(pcd.points)} points.")


    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(k=30)


    print("Poisson reconstruction...")
    try:
        mesh_poisson, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
        if len(mesh_poisson.vertices) > 0:
            meshes = [("poisson", mesh_poisson)]
            print(f"  - Poisson mesh: {len(mesh_poisson.vertices)} vertices, {len(mesh_poisson.triangles)} triangles")
        else:
            print("Poisson reconstruction failed: no vertices generated!")
            return
    except Exception as e:
        print(f"Poisson reconstruction failed: {e}")
        return

    print("Post-processing meshes...")
    processed_meshes = []
    
    for method_name, mesh in meshes:
        print(f"Processing {method_name} mesh...")
        

        mesh.remove_duplicated_vertices()
        mesh.remove_duplicated_triangles()
        mesh.remove_degenerate_triangles()
        mesh.compute_vertex_normals()
        mesh.compute_triangle_normals()
        

        color = [0.76, 0.74, 0.87]  
        mesh.paint_uniform_color(color)
        

        try:
            mesh = mesh.fill_holes()
            print(f"  - Filled holes in {method_name} mesh")
        except Exception as e:
            print(f"  - Hole filling failed for {method_name}: {e}")
        
        processed_meshes.append((method_name, mesh))
    

    print("Saving reconstructed surfaces...")
    
    for method_name, mesh in processed_meshes:
        if len(mesh.vertices) > 0:

            ply_file = f"reconstructed_surface_{method_name}.ply"
            o3d.io.write_triangle_mesh(ply_file, mesh)
            print(f"  - {method_name} mesh saved to {ply_file}")
            

            obj_file = f"reconstructed_surface_{method_name}.obj"
            o3d.io.write_triangle_mesh(obj_file, mesh)
            print(f"  - {method_name} mesh saved to {obj_file}")
            

            print(f"  - {method_name} statistics:")
            print(f"    Vertices: {len(mesh.vertices)}")
            print(f"    Triangles: {len(mesh.triangles)}")
            print(f"    Surface area: {mesh.get_surface_area():.6f}")
            

            try:
                if mesh.is_watertight():
                    volume = mesh.get_volume()
                    print(f"    Volume: {volume:.6f}")
                    print(f"    Watertight: True")
                else:
                    print(f"    Volume: Cannot compute (not watertight)")
                    print(f"    Watertight: False")
            except Exception as e:
                print(f"    Volume: Error - {e}")
    
    
    print("Reconstruction completed successfully!")

if __name__ == "__main__":
    main()
