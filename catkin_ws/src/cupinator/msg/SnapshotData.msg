Header header

# An image and a key to recall the image (i.e. position when snapshot was taken)

float64 Key # Key used by the caller to recall the image from queries made to the 
            #    node who recieves this message
sensor_msgs/Image Image # Acutal image data

# TODO: Add color filter data type to allow taking snapshots of multiple colors
#        by sending filtering parameters.
