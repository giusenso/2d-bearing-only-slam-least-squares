Project 02 - Bearing only SLAM

The dataset is composed by a g2o file which contain poses and bearing observations. The file contain also odometry edges that are use to construct the initial guess
for the problem. The use of those edges is your own choice if you think it's necessary.

Hint :
    
    - Parse the whole dataset and initialize the landmarks (Linear Triangulation) by using at least two bearing observations with the proper parallax
    - Setup a LS optimization that involves all the poses and landmarks that you have initialized
    - In the bearing edges the ID of the landmarkd is reported, use it to identify them for both the triangulation and the global optimization
         
        -EDGE_BEARING_SE2_XY id_pose id_landmark bearing fake_value_from_g2o(don't use it)

Expected output :
  - Robot trajectory and Map


For any question concerning the initialization part feel free to contact me
