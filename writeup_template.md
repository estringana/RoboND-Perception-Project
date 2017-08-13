## Project: Perception Pick & Place
### Alejandro Estringana Ruiz
#### Project: [https://github.com/estringana/RoboND-Perception-Project](https://github.com/estringana/RoboND-Perception-Project)
---

[//]: # (Image References)

[Clusters]: ./ToDeliver/images/Clusters.png
[Table]: ./ToDeliver/images/Table.png
[Objects]: ./ToDeliver/images/Objects.png
[BeforeRemoveZ]: ./ToDeliver/images/BeforeRemoveZ.png
[RemovedZ]: ./ToDeliver/images/RemovedZ.png
[RemovedY]: ./ToDeliver/images/RemovedY.png
[RemovedX]: ./ToDeliver/images/RemovedX.png
[DownSample]: ./ToDeliver/images/DownSample.png
[NoNoisy]: ./ToDeliver/images/NoNoisy.png
[initial]: ./ToDeliver/images/initial.png
[TrainedModel1]: ./ToDeliver/images/TrainedModel1.png
[CameraWithLabel1]: ./ToDeliver/images/CameraWithLabel1.png
[TrainedModel2]: ./ToDeliver/images/TrainedModel2.png
[CameraWithLabel2]: ./ToDeliver/images/CameraWithLabel2.png
[TrainedModel3]: ./ToDeliver/images/TrainedModel3.png
[CameraWithLabel3]: ./ToDeliver/images/CameraWithLabel3.png


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  See the example `output.yaml` for details on what the output should look like.  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

# Code explanation
First thing you need to do is to convert the given cloud points given by Ros Point Cloud to PCL PointXYZRGB. This is required in order to use the python functions I will be explaining on the coming sections.

```
cloud = ros_to_pcl(pcl_msg)
```

We started with the following input PCL.
![][Initial]


### Statistical Outlier Filtering
This project also simulates the camera have lot of noisy. This happen on conventional cameras as well. However on the previous exercises we didn't have to do this. The way you get rid of all these points is by applying an Statistical Outlier Filtering. How it works is by checking all the neighbours of every point cloud and it discards all those ones which didn't have an average of neigbours over the defined K. On my case, after checking several combinations I fond that k=10 and x=0.2 removes all the noisy dots.

```
 outlier_filter = cloud.make_statistical_outlier_filter()
outlier_filter.set_mean_k(10)
x = 0.2
outlier_filter.set_std_dev_mul_thresh(x)
cloud = outlier_filter.filter()
```

You can see this image is much more clear
![][NoNoisy]


 
### Voxel Grid Downsampling
Next step on the point cloud manipulation is to apply a Voxel Grid Downsampling filter. This filter remove cloup point to help the image manipulation. For the purpose of the project I used the same leaf size I used on the exercises `0.01`

```
# Voxel Grid filter
# Create a VoxelGrid filter object for our input point cloud
vox = cloud.make_voxel_grid_filter()
# Choose a voxel (also known as leaf) size
# Note: this (1) is a poor choice of leaf size
# Experiment and find the appropriate size!
LEAF_SIZE = 0.01
# Set the voxel (or leaf) size
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
# Call the filter function to obtain the resultant downsampled point cloud
cloud_filtered = vox.filter()
```

On this image we can see less points clouds than in the previous
![][DownSample]



### PassThrough filter
Here things become interesting. As we are going to be presented the object on the same place, we can discard all those areas of the image which we know, there won't be any object. The whole purpose of all these filter is to get to a point where we only have the object we are insterested on.

I took a slightly different approach here to the one on the previous exercises. On the previous exercises we were ok with only removing PCL on the Z axis. However, here, in order to optimize I removed PCL using the three axis.

#### Axis X
Here I will remove all the background PCL which we are not interested on. I found the min and max after playing with the numbers. This was easy.

```
# Create a PassThrough filter object.
passthrough = cloud_filtered.make_passthrough_filter()
# Assign axis and range to the passthrough filter object.
filter_axis = 'x'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.35
axis_max = 1.0
passthrough.set_filter_limits(axis_min, axis_max)
# Finally use the filter function to obtain the resultant point cloud.
cloud_filtered = passthrough.filter()
```

This filter removes all PCL behind the table 
![][RemovedX]

#### Axis Y
After removing from X, it leaves a big table with two wings on the sides. We know those two wings won't help in any way, so I removed them by applying the PassThrough filter. Here, it took me a while to realised that I had to used a negative min. Once I found it, it made sense.

```
passthrough = cloud_filtered.make_passthrough_filter()
# Assign axis and range to the passthrough filter object.
filter_axis = 'y'
passthrough.set_filter_field_name(filter_axis)
axis_min = -0.47
axis_max = 0.47
passthrough.set_filter_limits(axis_min, axis_max)
# Finally use the filter function to obtain the resultant point cloud.
cloud_filtered = passthrough.filter()
```

This one removes the wings of the table
![][RemovedY]

#### Axis Z
Lastly, I realised that the table had a small leg. As the leg is on the Z axis, I had to used also the filter here. I played with the min and max until I got rid of it.

```
passthrough = cloud_filtered.make_passthrough_filter()
# Assign axis and range to the passthrough filter object.
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.5
axis_max = 1.5
passthrough.set_filter_limits(axis_min, axis_max)
# Finally use the filter function to obtain the resultant point cloud.
cloud_filtered = passthrough.filter()
```

Lastly we can see the leg of the table on the first image and after the filter it's not there(second image)
![][BeforeRemoveZ]
![][RemovedZ]


### RANSAC Plane Segmentation
Last filter I had to apply was the Ransac filter. What it does, it helps to identify figures(inliers) like planes, esferics, tubes, etc. On this case, we needed the plane one to to identify the table and then, get the objectsd from what wasn't detected as table(outliers).

```
# Create the segmentation object
seg = cloud_filtered.make_segmenter()
# Set the model you wish to fit
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
# Max distance for a point to be considered fitting the model
# Experiment with different values for max_distance
# for segmenting the table
max_distance = 0.01
seg.set_distance_threshold(max_distance)
# Call the segment function to obtain set of inlier indices and model coefficients
inliers, coefficients = seg.segment()

# TODO: Extract inliers and outliers
cloud_table = cloud_filtered.extract(inliers, negative=False)
cloud_objects = cloud_filtered.extract(inliers, negative=True)
```

Table
![][Table]

Object
![][Objects]



### Euclidean Clustering
At this point we have a bunch of points belonging to something. However, we don't know yet how many objects we have. By applying Euclidean Clustering, it detects which points belongs to one object and which ones to a different one. On this, you need to specify the minimum amout of points needed to generate an object. This specific one gets harder on the world 3 as there is a glue behind another object and it's too small. You also need to specify the max. I didn't worry much about the max as it didn't seem to be a problem. Last thing, you need to set a tolerance. Here, I played with it until I found the disered result.

```
white_cloud = XYZRGB_to_XYZ(cloud_objects)  # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()

# Create a cluster extraction object
ec = white_cloud.make_EuclideanClusterExtraction()
# Set tolerances for distance threshold
# as well as minimum and maximum cluster size (in points)
# NOTE: These are poor choices of clustering parameters
# Your task is to experiment and find values that work for segmenting objects.
ec.set_ClusterTolerance(0.015)
ec.set_MinClusterSize(50)
ec.set_MaxClusterSize(1500)
# Search the k-d tree for clusters
ec.set_SearchMethod(tree)
# Extract indices for each of the discovered clusters
cluster_indices = ec.Extract()

# TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
# Assign a color corresponding to each segmented object in scene
cluster_color = get_color_list(len(cluster_indices))

color_cluster_point_list = []

for j, indices in enumerate(cluster_indices):
    for i, indice in enumerate(indices):
        color_cluster_point_list.append([white_cloud[indice][0],
                                         white_cloud[indice][1],
                                         white_cloud[indice][2],
                                         rgb_to_float(cluster_color[j])])

# Create new cloud containing all clusters, each with unique color
cluster_cloud = pcl.PointCloud_PointXYZRGB()
cluster_cloud.from_list(color_cluster_point_list)
```

Here we can see each cluster on a different color
![][Clusters]


### Getting features and make a prediction
Now we have all the clusters, we need to iterate over them and extract the features of each object. These features will be compared to the ones we extracted from training our model. The feautres we are extracting are based on colors and normals. We could have more features extracted here, however, for the purpose of the project, these two seem ok.

```
# Grab the points for the cluster
pcl_cluster = cloud_objects.extract(pts_list)
ros_cloud_objects = pcl_to_ros(pcl_cluster)
# Compute the associated feature vector
chists = compute_color_histograms(ros_cloud_objects, using_hsv=True)
normals = get_normals(ros_cloud_objects)
nhists = compute_normal_histograms(normals)
feature = np.concatenate((chists, nhists))
```
Now we have the features, we need generate a predicion about which object those features belongs to:

```
# Make the prediction
prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
label = encoder.inverse_transform(prediction)[0]
detected_objects_labels.append(label)
```


### Actioning the robot to pick the items
I will go trhough this very quick the steps required for this are very detailed on the documentation.

The way I approached this was by concatenating two loops. The first one iterates over the list of objects to pick. The second loop iterates until it finds such object on the list of detected objects.

Once it is found, I extracted the required parameters, and send them to the robot as well as saved them in YAML format on a dictionary.

Eventually I output this dictionary to the output_X.yml file where X is the world I am running on.

```
for item in object_list_param:
object_group = item['group']

# TODO: Get the PointCloud for a given object and obtain it's centroid
for object in object_list:
    if object.label == item['name']:
        # Name
        object_name = String()
        object_name.data = item['name']

        # TODO: Assign the arm to be used for pick_place
        arm_name = String()
        if object_group == 'green':
            arm_name.data = 'right'
        else:
            arm_name.data = 'left'

        # Pick pose
        points_arr = ros_to_pcl(object.cloud).to_array()
        mean = np.mean(points_arr, axis=0)[:3]
        pick_pose = Pose()
        pick_pose.position.x = np.asscalar(mean[0])
        pick_pose.position.y = np.asscalar(mean[1])
        pick_pose.position.z = np.asscalar(mean[2])

        # TODO: Create 'place_pose' for the object
        place_pose = Pose()
        place_pose.position.x = poses_parsed[arm_name.data][0]
        place_pose.position.y = poses_parsed[arm_name.data][1]
        place_pose.position.z = poses_parsed[arm_name.data][2]

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ", resp.success)
```


```
filename = 'output_' + str(test_scene_num.data) + '.yml'
send_to_yaml(filename, dict_list)
```

Before all this I map the poses into a dictionary. This helps later on to find the pose I am looking for

```
poses_parsed = {}
for pose in poses:
    poses_parsed[pose['name']] = pose['position']
```

# Output

## World 1
Matrixes after training model
![][TrainedModel1]

Visual of the robot detecting objects
![][CameraWithLabel1]

[Output generated in YML file](./ToDeliver/output_1.yml)

## World 2
Matrixes after training model
![][TrainedModel2]

Visual of the robot detecting objects
![][CameraWithLabel2]

[Output generated in YML file](./ToDeliver/output_2.yml)

## World 3
Matrixes after training model
![][TrainedModel3]

Visual of the robot detecting objects
![][CameraWithLabel3]

[Output generated in YML file](./ToDeliver/output_3.yml)