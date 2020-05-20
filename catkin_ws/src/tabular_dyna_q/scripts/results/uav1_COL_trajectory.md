

<!--
 * @Author       : LI Jinjie
 * @Date         : 2020-05-20 20:02:14
 * @LastEditors  : LI Jinjie
 * @LastEditTime : 2020-05-20 20:02:34
 * @Units        : None
 * @Description  : file content
 * @Dependencies : None
 * @NOTICE       : None
--> 

rostopic pub -r 10 /uav1/pose_cmd geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 1.5}, angular: {x: 0.0, y: 0.0, z: 0.10}}'