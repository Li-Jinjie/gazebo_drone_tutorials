

<!--
 * @Author       : LI Jinjie
 * @Date         : 2020-05-16 20:58:48
 * @LastEditors  : LI Jinjie
 * @LastEditTime : 2020-05-16 20:58:50
 * @Units        : None
 * @Description  : file content
 * @Dependencies : None
 * @NOTICE       : None
--> 

rostopic pub -r 10 /uav1/pose_cmd geometry_msgs/Twist '{linear: {x: 0.1, y: -0.1, z: 1.5}, angular: {x: 0.0, y: 0.0, z: -0.2}}'