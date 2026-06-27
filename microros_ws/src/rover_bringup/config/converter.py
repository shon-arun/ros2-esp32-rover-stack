import os
import cv2
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader

def build_kalibr_dataset(bag_path, output_dir):
    # Ensure the extraction directory layout matches Kalibr's standard expectations
    os.makedirs(f'{output_dir}/cam0', exist_ok=True)
    
    bag_dir = Path(bag_path)
    
    # Open the output CSV and initialize the reader configuration
    with open(f'{output_dir}/imu0.csv', 'w') as imu_file, AnyReader([bag_dir]) as reader:
        # Write the strict column header format required by kalibr_bagcreater
        imu_file.write("timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z\n")
        
        image_count = 0
        imu_count = 0
        
        print(f"Opening bag directory: {bag_path}")
        print("Scanning database and extracting messages...")
        
        for connection, timestamp, rawdata in reader.messages():
            # Use forgiving substring matching to process any variations of the compressed topic
            if 'image' in connection.topic and 'compressed' in connection.topic:
                msg = reader.deserialize(rawdata, connection.msgtype)
                stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                
                # Decode the compressed image binary payload back to an uncompressed grayscale format
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
                
                # Save frame named exactly as its generation epoch nanosecond timestamp
                cv2.imwrite(f'{output_dir}/cam0/{stamp_ns}.png', cv_img)
                image_count += 1
                
            # Use forgiving substring matching to capture any variation of your IMU data topic
            elif 'imu' in connection.topic and 'data' in connection.topic:
                msg = reader.deserialize(rawdata, connection.msgtype)
                stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                
                # Extract the hardware observations and append to the table rows
                imu_file.write(f"{stamp_ns},{msg.angular_velocity.x},{msg.angular_velocity.y},{msg.angular_velocity.z},"
                               f"{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z}\n")
                imu_count += 1

        print("\n--- Extraction Summary ---")
        print(f"Successfully extracted: {image_count} images into '{output_dir}/cam0/'")
        print(f"Successfully extracted: {imu_count} IMU readings into '{output_dir}/imu0.csv'")
        
        if imu_count == 0:
            print("[WARNING] Zero IMU messages were parsed! Ensure your topic matches your criteria.")

if __name__ == '__main__':
    # Update these values to point to your input ROS2 folder and target output path
    INPUT_BAG_FOLDER = 'kalibr_data5'
    OUTPUT_DATASET_FOLDER = 'kalibr_data8'
    
    build_kalibr_dataset(INPUT_BAG_FOLDER, OUTPUT_DATASET_FOLDER)
