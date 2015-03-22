Header header

# An image and a key to recall the image after aggregation (e.g. position when snapshot was taken)

float64 Key # Key used by the caller to recall the image 
sensor_msgs/Image Image # Acutal image data

# TODO: Add color filter data type to allow taking snapshots of multiple colors
#        by sending filtering parameters.
