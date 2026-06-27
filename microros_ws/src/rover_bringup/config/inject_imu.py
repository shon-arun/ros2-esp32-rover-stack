import rosbag
import rospy
import csv
from sensor_msgs.msg import Imu

print("Opening kalibr_data7.bag to inject IMU data...")
# Open the bag in 'a' (append) mode so we don't overwrite the camera data
bag = rosbag.Bag('kalibr_data8.bag', 'a') 
count = 0

try:
    with open('kalibr_data8/imu0.csv', 'r') as f:
        reader = csv.reader(f)
        headers = next(reader) # Skip the header row
        
        for row in reader:
            # Safely skip blank lines that cause kalibr_bagcreater to silently crash
            if len(row) < 7: 
                continue
                
            imu = Imu()
            stamp_str = row[0].strip()
            
            # Split the 19-digit nanosecond timestamp into seconds and nanoseconds
            secs = int(stamp_str[:-9])
            nsecs = int(stamp_str[-9:])
            
            imu.header.stamp = rospy.Time(secs, nsecs)
            imu.header.frame_id = 'imu0'
            
            # Assign Gyroscope data
            imu.angular_velocity.x = float(row[1])
            imu.angular_velocity.y = float(row[2])
            imu.angular_velocity.z = float(row[3])
            
            # Assign Accelerometer data
            imu.linear_acceleration.x = float(row[4])
            imu.linear_acceleration.y = float(row[5])
            imu.linear_acceleration.z = float(row[6])
            
            # Write to the exact topic Kalibr is looking for
            bag.write('/imu0', imu, imu.header.stamp)
            count += 1
            
    print("SUCCESS: Injected " + str(count) + " IMU messages into /imu0 topic!")
except Exception as e:
    print("CRITICAL ERROR: " + str(e))
finally:
    bag.close()
