import numpy as np
import matplotlib.pyplot as plt
from pypcd4 import PointCloud
import open3d as o3d
import cv2

class CloudProcessing:
    def __init__(self):
        pass

    def detect_good_features(self, image_array, max_corners=100, quality_level=0.01, min_distance=10):
        """
        Detect corners in a 2D numpy array using cv2.goodFeaturesToTrack().
        
        Parameters:
        - image_array: 2D numpy array.
        - max_corners: Maximum number of corners to return (default: 100).
        - quality_level: Minimum accepted quality of image corners (default: 0.01).
        - min_distance: Minimum possible Euclidean distance between the returned corners (default: 10).
        
        Returns:
        - corners: Detected corner points as a list of keypoints.
        """
        # Ensure the input is a 2D numpy array
        if not isinstance(image_array, np.ndarray) or image_array.ndim != 2:
            raise ValueError("Input must be a 2D numpy array")
        
        # Handle NaN and infinite values
        image_array = np.nan_to_num(image_array, nan=0.0, posinf=255.0, neginf=0.0)
        
        # Normalize and convert to uint8
        image_array = cv2.normalize(image_array, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        
        # Detect corners using goodFeaturesToTrack
        corners = cv2.goodFeaturesToTrack(image_array, maxCorners=max_corners, qualityLevel=quality_level, minDistance=min_distance)
        
        # Convert corners to keypoints
        keypoints = [cv2.KeyPoint(x=float(pt[0][0]), y=float(pt[0][1]), size=1) for pt in corners]
        
        return keypoints

    def visualize_good_features(self, image_array, keypoints):
        """
        Visualize corners on the image detected by goodFeaturesToTrack.
        
        Parameters:
        - image_array: 2D numpy array.
        - keypoints: Detected corner points as a list of keypoints.
        """
        # Ensure the input is a 2D numpy array
        if not isinstance(image_array, np.ndarray) or image_array.ndim != 2:
            raise ValueError("Input must be a 2D numpy array")
        
        # Handle NaN and infinite values
        image_array = np.nan_to_num(image_array, nan=0.0, posinf=255.0, neginf=0.0)
        
        # Normalize and convert to uint8
        image_array = cv2.normalize(image_array, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        
        # Convert to a 3-channel BGR image
        image_bgr = cv2.merge([image_array, image_array, image_array])
        
        # Draw keypoints on the image
        image_with_keypoints = cv2.drawKeypoints(image_bgr, keypoints, None, color=(0, 255, 0))
        
        # Display the image with keypoints
        plt.figure(figsize=(10, 8))
        plt.imshow(cv2.cvtColor(image_with_keypoints, cv2.COLOR_BGR2RGB))
        plt.title('Good Features to Track')
        plt.axis('off')
        plt.show()

    def compute_2d_density_map_fixed(self, pcd, cell_size=1.0):
        """
        Computes a 2D density map from a 3D point cloud by projecting it onto the XY plane.
        The grid cell size is fixed, and the grid dimensions adjust based on the point cloud bounds.
        
        Parameters:
            pcd (np.array): A Nx3 numpy array of 3D points.
            cell_size (float): The size of each cell in the grid (defines resolution).
        
        Returns:
            None: Displays the density map as a heatmap.
        """
        # Project the point cloud onto the XY plane
        points_xy = pcd[:, :2]

        # Calculate the bounds needed to cover all points with the fixed cell size
        x_min, x_max = np.floor(np.min(points_xy[:, 0])), np.ceil(np.max(points_xy[:, 0]))
        y_min, y_max = np.floor(np.min(points_xy[:, 1])), np.ceil(np.max(points_xy[:, 1]))

        # Calculate the number of bins along each axis based on the cell size
        x_bins = int((x_max - x_min) / cell_size)
        y_bins = int((y_max - y_min) / cell_size)

        # Create a 2D histogram with the fixed cell size
        density, xedges, yedges = np.histogram2d(points_xy[:, 0], points_xy[:, 1], bins=[x_bins, y_bins], range=[[x_min, x_max], [y_min, y_max]])

        return density

    def create_height_map(self, pcd, cell_size=1.0):# # #{
        """
        Creates a 2D map from a 3D point cloud by projecting it onto the XY plane.
        Each cell in the resulting map stores the highest Z value of the points that fall into that cell.
        
        Parameters:
            pcd (np.array): A Nx3 numpy array of 3D points.
            cell_size (float): The size of each cell in the grid (defines resolution).
        
        Returns:
            None: Displays the height map as an image.
        """
        # Project the point cloud onto the XY plane and separate the Z coordinates
        points_xy = pcd[:, :2]
        z_values = pcd[:, 2]

        # Calculate the bounds needed to cover all points with the fixed cell size
        x_min, x_max = np.floor(np.min(points_xy[:, 0])), np.ceil(np.max(points_xy[:, 0]))
        y_min, y_max = np.floor(np.min(points_xy[:, 1])), np.ceil(np.max(points_xy[:, 1]))

        # Calculate the number of bins along each axis based on the cell size
        x_bins = int((x_max - x_min) / cell_size)
        y_bins = int((y_max - y_min) / cell_size)

        # Initialize the height map array with negative infinity (to store maximum values)
        height_map = np.full((y_bins, x_bins), -np.inf)

        # Assign each point to the appropriate cell and update the max Z value
        for x, y, z in zip(points_xy[:, 0], points_xy[:, 1], z_values):
            x_idx = int((x - x_min) / cell_size)
            y_idx = int((y - y_min) / cell_size)

            if(x_idx < 0 or x_idx >= x_bins or y_idx<0 or y_idx>=y_bins):
                continue
            if height_map[y_idx, x_idx] <= z:
                height_map[y_idx, x_idx] = z

        print("created heightmap", height_map.shape)
        return height_map# # #}
   
    def visualize_2d_array(self, array, title='2D Array Visualization', cmap='viridis'):# # #{
        """
        Visualizes a 2D numpy array using Matplotlib.
        
        Parameters:
        - array: 2D numpy array to be visualized.
        - title: Title of the plot (default: '2D Array Visualization').
        - cmap: Color map to be used (default: 'viridis').
        """
        if not isinstance(array, np.ndarray):
            raise TypeError("Input must be a numpy array")
        if len(array.shape) != 2:
            raise ValueError("Input array must be 2-dimensional")
        
        plt.figure(figsize=(8, 6))
        plt.imshow(array, cmap=cmap, aspect='auto')
        plt.colorbar()
        plt.title(title)
        plt.xlabel('X axis')
        plt.ylabel('Y axis')
        plt.gca().set_aspect('equal', adjustable='box')

        plt.show()# # #}
    
    def visualize_multiple_2d_arrays(self, arrays, titles=None, cmap='viridis'):# # #{
        """
        Visualizes multiple 2D numpy arrays in one plot using subplots.
        
        Parameters:
        - arrays: List of 2D numpy arrays to be visualized.
        - titles: List of titles for each subplot (default: None).
        - cmap: Color map to be used (default: 'viridis').
        """
        if not all(isinstance(array, np.ndarray) and len(array.shape) == 2 for array in arrays):
            raise ValueError("All elements in arrays must be 2-dimensional numpy arrays")
        
        num_arrays = len(arrays)
        
        # Determine the grid size for subplots
        cols = int(np.ceil(np.sqrt(num_arrays)))
        rows = int(np.ceil(num_arrays / cols))
        
        fig, axes = plt.subplots(rows, cols, figsize=(15, 15))
        axes = axes.flatten()  # Flatten in case of grid having more than 1 row

        for i, array in enumerate(arrays):
            im = axes[i].imshow(array, cmap=cmap)
            axes[i].set_title(titles[i] if titles else f'Array {i+1}')
            axes[i].set_aspect('equal', adjustable='box')
        
        # Remove any empty subplots
        for j in range(i + 1, len(axes)):
            fig.delaxes(axes[j])

        fig.colorbar(im, ax=axes, orientation='vertical', fraction=0.02, pad=0.04)
        plt.tight_layout()
        plt.show()# # #}

    def detect_orb_features(self, image_array, nfeatures=500):# # #{
   
        """
        Detect ORB features in a 2D numpy array.
        
        Parameters:
        - image_array: 2D numpy array.
        - nfeatures: Number of keypoints to retain (default: 500).
        
        Returns:
        - keypoints: Detected ORB keypoints.
        - descriptors: Descriptors for the keypoints.
        """
        # Ensure the input is a 2D numpy array
        if not isinstance(image_array, np.ndarray) or image_array.ndim != 2:
            raise ValueError("Input must be a 2D numpy array")
        
        # Handle NaN and infinite values
        image_array = np.nan_to_num(image_array, nan=0.0, posinf=255.0, neginf=0.0)
        
        # Normalize and convert to uint8
        image_array = cv2.normalize(image_array, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        
        # Convert to a 3-channel BGR image
        image_bgr = cv2.merge([image_array, image_array, image_array])
        
        # Initialize ORB detector
        orb = cv2.ORB_create(nfeatures=nfeatures)
        
        # Detect keypoints and descriptors
        keypoints, descriptors = orb.detectAndCompute(image_bgr, None)
        
        return keypoints, descriptors# # #}

    def visualize_orb_features(self, image_array, keypoints):# # #{
        """
        Visualize ORB features on the image.
        
        Parameters:
        - image_array: 2D numpy array.
        - keypoints: Detected ORB keypoints.
        """
        # Ensure the input is a 2D numpy array
        if not isinstance(image_array, np.ndarray) or image_array.ndim != 2:
            raise ValueError("Input must be a 2D numpy array")
        
        # Handle NaN and infinite values
        image_array = np.nan_to_num(image_array, nan=0.0, posinf=255.0, neginf=0.0)
        
        # Normalize and convert to uint8
        image_array = cv2.normalize(image_array, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        
        # Convert to a 3-channel BGR image
        image_bgr = cv2.merge([image_array, image_array, image_array])
        
        # Draw keypoints on the image
        image_with_keypoints = cv2.drawKeypoints(image_bgr, keypoints, None, color=(0, 255, 0), flags=0)
        
        # Display the image with keypoints
        plt.figure(figsize=(10, 8))
        plt.imshow(cv2.cvtColor(image_with_keypoints, cv2.COLOR_BGR2RGB))
        plt.title('ORB Features')
        plt.axis('off')
        plt.show()# # #}

    def detect_sift_features(self, image_array, nfeatures=500):# # #{
        """
        Detect SIFT features in a 2D numpy array.
        
        Parameters:
        - image_array: 2D numpy array.
        - nfeatures: Number of keypoints to retain (default: 500).
        
        Returns:
        - keypoints: Detected SIFT keypoints.
        - descriptors: Descriptors for the keypoints.
        """
        # Ensure the input is a 2D numpy array
        if not isinstance(image_array, np.ndarray) or image_array.ndim != 2:
            raise ValueError("Input must be a 2D numpy array")
        
        # Handle NaN and infinite values
        image_array = np.nan_to_num(image_array, nan=0.0, posinf=255.0, neginf=0.0)
        
        # Normalize and convert to uint8
        image_array = cv2.normalize(image_array, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        
        # Convert to a 3-channel BGR image
        image_bgr = cv2.merge([image_array, image_array, image_array])
        
        # Initialize SIFT detector
        sift = cv2.SIFT_create(nfeatures=nfeatures)
        
        # Detect keypoints and descriptors
        keypoints, descriptors = sift.detectAndCompute(image_bgr, None)
        
        return keypoints, descriptors# # #}

    def visualize_sift_features(self, image_array, keypoints):# # #{
        """
        Visualize SIFT features on the image.
        
        Parameters:
        - image_array: 2D numpy array.
        - keypoints: Detected SIFT keypoints.
        """
        # Ensure the input is a 2D numpy array
        if not isinstance(image_array, np.ndarray) or image_array.ndim != 2:
            raise ValueError("Input must be a 2D numpy array")
        
        # Handle NaN and infinite values
        image_array = np.nan_to_num(image_array, nan=0.0, posinf=255.0, neginf=0.0)
        
        # Normalize and convert to uint8
        image_array = cv2.normalize(image_array, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        
        # Convert to a 3-channel BGR image
        image_bgr = cv2.merge([image_array, image_array, image_array])
        
        # Draw keypoints on the image
        image_with_keypoints = cv2.drawKeypoints(image_bgr, keypoints, None, color=(0, 255, 0), flags=0)
        
        # Display the image with keypoints
        plt.figure(figsize=(10, 8))
        plt.imshow(cv2.cvtColor(image_with_keypoints, cv2.COLOR_BGR2RGB))
        plt.title('SIFT Features')
        plt.axis('off')
        plt.show()# # #}

    def detect_harris_corners(self, image_array, block_size=2, ksize=3, k=0.04, threshold=0.01):# # #{
        """
        Detect Harris corners in a 2D numpy array.
        
        Parameters:
        - image_array: 2D numpy array.
        - block_size: Size of the neighborhood considered for corner detection (default: 2).
        - ksize: Aperture parameter of the Sobel derivative used (default: 3).
        - k: Harris detector free parameter (default: 0.04).
        - threshold: Threshold for detecting corners (default: 0.01).
        
        Returns:
        - keypoints: Detected corner points as a list of keypoints.
        """
        # Ensure the input is a 2D numpy array
        if not isinstance(image_array, np.ndarray) or image_array.ndim != 2:
            raise ValueError("Input must be a 2D numpy array")
        
        # Handle NaN and infinite values
        image_array = np.nan_to_num(image_array, nan=0.0, posinf=255.0, neginf=0.0)
        
        # Normalize and convert to uint8
        image_array = cv2.normalize(image_array, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        
        # Detect Harris corners
        dst = cv2.cornerHarris(image_array, block_size, ksize, k)
        
        # Normalize the result to range [0, 1]
        dst_norm = cv2.normalize(dst, None, 0, 1, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        
        # Thresholding to find corners
        corners = np.argwhere(dst_norm > threshold)
        
        # Convert corners to keypoints
        keypoints = [cv2.KeyPoint(float(pt[1]), float(pt[0]), 1) for pt in corners]
        
        return keypoints# # #}

    def visualize_harris_corners(self, image_array, keypoints):# # #{
        """
        Visualize Harris corners on the image.
        
        Parameters:
        - image_array: 2D numpy array.
        - keypoints: Detected corner points as a list of keypoints.
        """
        # Ensure the input is a 2D numpy array
        if not isinstance(image_array, np.ndarray) or image_array.ndim != 2:
            raise ValueError("Input must be a 2D numpy array")
        
        # Handle NaN and infinite values
        image_array = np.nan_to_num(image_array, nan=0.0, posinf=255.0, neginf=0.0)
        
        # Normalize and convert to uint8
        image_array = cv2.normalize(image_array, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        
        # Convert to a 3-channel BGR image
        image_bgr = cv2.merge([image_array, image_array, image_array])
        
        # Draw keypoints on the image
        image_with_keypoints = cv2.drawKeypoints(image_bgr, keypoints, None, color=(0, 255, 0))
        
        # Display the image with keypoints
        plt.figure(figsize=(10, 8))
        plt.imshow(cv2.cvtColor(image_with_keypoints, cv2.COLOR_BGR2RGB))
        plt.title('Harris Corners')
        plt.axis('off')
        plt.show()# # #}

if __name__ == "__main__":
    cp = CloudProcessing()


