# ROS2 package for movement data publisher node


This publisher node will send a message of type <CircleMovement> to the topic, and will eventually be received and processed by a subscriber node.

The message consists of two float values, indicating the linear-x-direction movement and angular-z-direction movement respectively.