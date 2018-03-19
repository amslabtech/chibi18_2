chibi18_2
=========
構成
---
### localizer ###
パーティクルフィルタによる自己位置推定を行う.  
尤度は実スキャンと参照スキャンの残差平方和から求める.  
* Published topics
    * /tf (tf/tfMessage)  
      from "odom" to "map"
    * /chibi18/estimated_pose (geometry_msgs/PoseStamped)  
      the particle that has largest importance weight
    * /chibi18/poses (geometry_msgs/PoseArray)  
      for debug
* Subscribed topics
    * /scan (sensor_msgs/Laserscan)
    * /tf (tf/tfMessage)  
      "laser" <- "base_link" <- "odom"
    * /map (nav_msgs/OccupancyGrid)

### local_path_planner ###
DWAにより,とるべき速度と角速度を決定する.
* Published topics
    * /chibi18/velocity (geometry_msgs/Twist)
* Subscribed topics
    * /scan (sensor_msgs/Laserscan)
    * /tf (tf/tfMessage)  
      "laser" <- "base_link" <- "odom"
    * /chibi18/target (geometry_msgs/PoseStamped)
    * /roomba/odometry (nav_msgs/Odometry)
    * /chibi18/stop (std_msgs/Bool)  
      if *false* subscribed, roomba does not move. 

### global_path_planner ###
A*により,経路を作成する.
waypointはrvizの2dnavgoalから与える.
* Published topics
    * /chibi18/global_path (nav_msgs/Path)
    * /chibi18/cost_map (nav_msgs/OccupancyGrid)
    * /chibi18/target (geometry_msgs/PoseStamped)
* Subscribed topics
    * /map (nav_msgs/OccupancyGrid)
    * /tf (tf/tfMessage)  
      "laser" <- "base_link" <- "odom"
    * /move_base_simple/goal (geometry_msgs/PoseStamped)
    * /chibi18/estimated_pose
