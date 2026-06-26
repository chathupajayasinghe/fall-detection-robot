#!/usr/bin/env python3
import os
import sys
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid

def extract_largest_map(bag_path, output_prefix="home_map"):
    # Initialize reader
    reader = rosbag2_py.SequentialReader()
    
    # Configure storage options
    # ROS 2 Jazzy defaults to mcap, but can fallback to sqlite3 if needed
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Failed to open as MCAP: {e}. Trying sqlite3 storage format...")
        storage_options.storage_id = 'sqlite3'
        try:
            reader.open(storage_options, converter_options)
        except Exception as e_inner:
            print(f"Failed to open bag: {e_inner}")
            sys.exit(1)

    largest_msg = None
    largest_known_cells = -1

    print("Scanning bag for /map messages...")
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == '/map':
            msg = deserialize_message(data, OccupancyGrid)
            # Count the number of non-unknown cells (-1 represents unknown)
            known_cells = sum(1 for val in msg.data if val != -1)
            
            if known_cells > largest_known_cells:
                largest_known_cells = known_cells
                largest_msg = msg

    if largest_msg is None:
        print("No /map messages found in the bag file.")
        return

    print(f"\nSuccess! Found largest map with {largest_known_cells} mapped cells.")
    print(f"Dimensions: {largest_msg.info.width} x {largest_msg.info.height}")
    print(f"Resolution: {largest_msg.info.resolution} m/px")
    
    width = largest_msg.info.width
    height = largest_msg.info.height
    resolution = largest_msg.info.resolution
    origin = largest_msg.info.origin
    
    # Reshape and flip vertically (ROS bottom-left -> Image top-left)
    grid = [largest_msg.data[i * width : (i + 1) * width] for i in range(height)]
    grid.reverse()
    
    # Save as PGM P5 (binary)
    pgm_filename = f"{output_prefix}.pgm"
    with open(pgm_filename, 'wb') as f:
        f.write(b"P5\n")
        f.write(f"{width} {height}\n".encode())
        f.write(b"255\n")
        
        pgm_data = bytearray()
        for row in grid:
            for val in row:
                if val == -1:
                    pgm_data.append(205)  # Grey for unknown space
                elif val == 0:
                    pgm_data.append(254)  # White for free space
                elif val == 100:
                    pgm_data.append(0)    # Black for obstacles
                else:
                    # Scale intermediate occupancy values
                    pgm_data.append(int((100 - val) * 2.54))
        f.write(pgm_data)
        
    # Save as YAML (nav2 map saver standard)
    yaml_filename = f"{output_prefix}.yaml"
    yaml_data = {
        'image': pgm_filename,
        'resolution': resolution,
        'origin': [origin.position.x, origin.position.y, origin.position.z],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    with open(yaml_filename, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
        
    print(f"Saved: {pgm_filename} and {yaml_filename}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 extract_largest_map.py <path_to_bag_directory>")
        sys.exit(1)
    extract_largest_map(sys.argv[1])
