# Modifications made to pylon-ros-camera for AGNC applications

There are currently 3 main nodes in this repo that are used for AutoGNC applications. 
1. `/basler_cam/pylon_ros2_camera_node`
- This is the Basler camera wrapper and it publishes the raw camera stream on the `/basler_cam/pylon_ros2_camera_node/image_raw` topic.
    - The topic contains a `sensor_msgs/msg/Image` message, which will contain a `imgmsg` of the streamed image. 
- There is another topic called `/basler_cam/pylon_ros2_camera_node/image_compressed` that is configurable, which will instead of publishing the raw the camera data, will publish a compressed for of it. 
    - It is configured as follows in the the `pylon-ros-camera/pylon_ros2_camera_wrapper/config/cam_config_feb_11.yaml`
```yaml
publish_raw_image: false
publish_compressed_image: true
resize_compressed_image: true
```
- The config to set the image parameters under this compression are located here and include: `pylon-ros-camera/pylon_ros2_camera_wrapper/config/cam_config_feb_11.yaml`
```yaml
compressed_image_format: 'jpeg' # opitons: jpg, jpeg, png 
compressed_image_jpeg_quality: 80 # valid 1-100, higher means higher quality, less compression 
compressed_image_png_level: 3 # valid 0-9, higher means more compression
compressed_image_target_width: 1024
compressed_image_target_height: 768
```
- This process, by choosing, `publish_compressed_image: true` will load image as `OpenCV/Numpy` array, encode into the image format and apply compression, and then encode that back as a `sensor_msgs/msg/CompressedImage`. 

2. `image_saver_node`
- located here: `pylon-ros-camera/image_saver/image_saver/image_saver_node.py`
- This node turns live camera stream into image files on disk.
- To account for change to image topic 
```yaml
# image_topic: "/basler_cam/pylon_ros2_camera_node/image_raw"
image_topic: "/basler_cam/pylon_ros2_camera_node/image_compressed"
# is_compressed_image: false
is_compressed_image: true
```

3. `/stream_compressor_node`
- Subscribes to the raw camera stream, publishes data on `/basler_cam/pylon_ros2_camera_node/image_compressed_small`
- Downscales to a `512` width, 50 JPEG quality, and republish live stream on at 5 Hz. 

# Overal Process 
Driver built-in:
/basler_cam/pylon_ros2_camera_node/image_compressed
configured by pylon_ros2_camera_wrapper/config/cam_config_feb_11.yaml

Standalone compressor:
/basler_cam/pylon_ros2_camera_node/image_compressed_small
configured by image_saver/config/stream_compressor_config.yaml