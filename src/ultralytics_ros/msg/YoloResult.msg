std_msgs/Header header
vision_msgs/Detection2DArray detections
sensor_msgs/Image[] masks
# ByteTrack / BoT-SORT IDs (cùng thứ tự detections); -1 nếu không có track
int32[] track_ids
