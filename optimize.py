import yaml
import numpy as np
from PIL import Image

def load_occupancy_grid(yaml_path):
    # Step 1: Read the YAML metadata file
    with open(yaml_path, 'r') as file:
        map_metadata = yaml.safe_load(file)

    image_path = map_metadata['image']  # Map image file path
    resolution = map_metadata['resolution']  # Map resolution (meters/pixel)
    origin = map_metadata['origin']  # Map origin [x, y, theta]

    free_thresh = map_metadata.get('free_thresh', 0.25)  # Default free threshold
    occupied_thresh = map_metadata.get('occupied_thresh', 0.65)  # Default occupied threshold

    # Step 2: Load the map image (PGM format)
    image = Image.open(image_path)
    map_data = np.array(image)  # Convert image to NumPy array

    # Step 3: Normalize the map data to Occupancy Grid values
    occupancy_grid = np.zeros_like(map_data, dtype=np.int8)
    occupancy_grid[map_data >= occupied_thresh * 255] = 1  # Mark as obstacles
    occupancy_grid[map_data <= free_thresh * 255] = 0  # Mark as free space
    occupancy_grid[(map_data > free_thresh * 255) & (map_data < occupied_thresh * 255)] = -1  # Unknown space

    return occupancy_grid, resolution, origin

# Example usage
yaml_path = "my_map.yaml"  # Path to your map.yaml file
occupancy_grid, resolution, origin = load_occupancy_grid(yaml_path)

print("Occupancy Grid:")
print(occupancy_grid)
print("Grid Shape:", occupancy_grid.shape)
print("Resolution (m/pixel):", resolution)
print("Origin:", origin)
