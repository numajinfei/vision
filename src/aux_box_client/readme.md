## 1. description
A tcp client node to communication with AuxBox by RJ45 cable.

Prepare:
- IP: set or get tcp server's ip address
- param: config/params.yaml set ip and port(default 6001)

## 2. message data structure:
- pub:
  
  node: /aux_box_client/pub 
  
  data: shared_interface/msg/AuxBox
  
    - type: 'LaserRange' or 'Inclinometer'
    - v1： LaserRange's distance(Positive is normal,Negative is error code) or Inclinomer's angle x
    - v2:  Inclinomer's angle y

    - terminal cmd:
      ros2 topic pub --once /aux_box_client_node/sub std_msgs/String "data: LaserRange"

      ros2 topic pub --once /aux_box_client_node/sub std_msgs/String "data: Inclinometer"
  
- sub:
  
  node: /aux_box_client/sub

  data: std_msgs/msg/String
  
    - "LaserRange": sub LaserRange's distance data.
    - "Inclinomter": sub sensor's angle x and y data.

    - terminal cmd:
      ros2 topic echo /aux_box_client_node/pub
  
