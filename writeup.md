# Project: Perception Pick & Place

## Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

## Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

[//]: # (Image References)

[image1]: ./misc_images/simulat.png
[image2]: ./misc_images/noise.png
[image3]: ./misc_images/voxel.png
[image4]: ./misc_images/pass_y.png
[image5]: ./misc_images/RANSAC.png
[image6]: ./misc_images/table.png
[image7]: ./misc_images/objects.png
[image8]: ./misc_images/cluster_v.png
[image9]: ./misc_images/SVM_result.png
[image10]: ./misc_images/perception_world1.png
[image11]: ./misc_images/perception_world2.png
[image12]: ./misc_images/perception_world3.png
[image13]: ./misc_images/pickplace_srv.png
[image14]: ./misc_images/stuck_table.png

---
### Writeup / README

#### 1. This writeup will include below section:
##### Perception
* How did I implement the pipeline for filtering.
* How did I implement the pipeline for cluster and object segmentation.
* How did I implement the pipeline for object recognition with SVM.
* How about the result of my pipeline for perception.
* What to improve.
##### Action
* Get message type from the .srv file.
* Get variables from perception section.
* Get centroids from the posion of object.
* Get yaml files to save posion.
* What to improve.

### Perception
#### 1. Pipeline for filtering. 
The PR2 has a RGBD camera to capture the data of view. The simulat I use is Gazebo and Rviz, as below:  
![alt text][image1] 
All of the perception section would use the pcl point cloud to be the data. So I use the ros_to_pcl function in pcl_helper.py to convert ROS message.  
<pre><code>cloud = ros_to_pcl(pcl_msg)</code></pre>     
##### 1.1  Statistical Outlier Filtering
The first thing is to reduce the noise of point cloud, I use the Statistical Outlier Filtering. I use parameters the class suggested. 
![alt text][image2]
<pre><code>
outlier_filter = cloud.make_statistical_outlier_filter()
outlier_filter.set_mean_k(50)
x = 1.0
outlier_filter.set_std_dev_mul_thresh(x)
cloud_filtered = outlier_filter.filter()
</code></pre>  
##### 1.2  Voxel Grid Downsampling
Downsampling is useful to reduce points of cloud but remain the feature which belong to shape of the cloud. The voxel grid downsampling will make a grid for cloud data as below, and the point of lines-cross will be saved:
![alt text][image3]
The result of Voxel Grid Downsampling is like this:
<pre><code>
[pic downsampling]
vox = cloud.make_voxel_grid_filter()
LEAF_SIZE = 0.003
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
cloud_filtered = vox.filter()
</code></pre>
##### 1.3  PassThrough Filter 
Sometimes few objects can be filtered directly. In this project, I just need to keep the table and objects to recognize. If I just make passthrough with 'z' plane as like the process in class. The two boxes will in the view of camera.
![alt text][image4]
So I do the same thing to 'y' plane.  
<pre><code>
passThrough_z = cloud_filtered.make_passthrough_filter()
filter_axis = 'z'
passThrough_z.set_filter_field_name(filter_axis)
axis_min = 0.6
axis_max = 1.1
passThrough_z.set_filter_limits(axis_min, axis_max)
cloud_filtered = passThrough_z.filter()

passThrough_y = cloud_filtered.make_passthrough_filter()
filter_axis = 'y'
passThrough_y.set_filter_field_name(filter_axis)
axis_min = -0.5
axis_max = 0.5
passThrough_y.set_filter_limits(axis_min, axis_max)
cloud_filtered = passThrough_y.filter()
</code></pre>
##### 1.4  RANSAC Plane Segmentation
The process above hasn't seperated the table and objects. So I use RANSAC Plane Segmentation, which will create a model to keep the thing near to lines or curve. Color of objects will be used to caculate the distance between feature and model-line.
![alt text][image5]
The effect of RANSAC as below:
![alt text][image6]![alt text][image7]
Table is a good choice to do RANSAC Plane Segmentation. So make the parameters "negative=True" to get objects.
<pre><code>
seg = cloud_filtered.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
max_distance = 0.01
seg.set_distance_threshold(max_distance)
inliers, coefficients = seg.segment()
extracted_inliers = cloud_filtered.extract(inliers, negative=False)
extracted_outliers = cloud_filtered.extract(inliers, negative=True)
</code></pre>
#### 2. Pipeline for cluster
In this section, I make the cluster of pixel of point cloud. It'll make PR2 know which is a can, or which is a book. Take mixture to be individual one. I use Euclidean Clustering, 
##### 2.3  Euclidean Clustering
The Euclidean Clustering use a [k-d tree](http://pointclouds.org/documentation/tutorials/kdtree_search.php)
<pre><code>
white_cloud = XYZRGB_to_XYZ(extracted_outliers)
tree = white_cloud.make_kdtree()
ec = white_cloud.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance(0.01)
ec.set_MinClusterSize(100)
ec.set_MaxClusterSize(50000)
ec.set_SearchMethod(tree)
cluster_indices = ec.Extract()
</code></pre>
##### 2.4  Create Cluster-Mask Point Cloud to visualize each cluster separately 
Use next code you can get the visualization of cluster. The effect is like this:  
![alt text][image8]
<pre><code>
cluster_color = get_color_list(len(cluster_indices))
color_cluster_point_list = []
for j, indices in enumerate(cluster_indices):
    for i, indice in enumerate(indices):
        color_cluster_point_list.append([white_cloud[indice][0],
                                        white_cloud[indice][1],
                                        white_cloud[indice][2],
                                         rgb_to_float(cluster_color[j])])
cluster_cloud = pcl.PointCloud_PointXYZRGB()
cluster_cloud.from_list(color_cluster_point_list)
</code></pre>
#### 3. pipeline for object recognition.
In this section, SVM is used to recognition the object. With the process of Machine Learning, first get some datas for training. Then use these datas to train a model. Finally do recognition by the model and pick the object in next section.
##### 3.1 Get datas for training
I'll use the sensor_stice exercise to get all kinds of objects will be used in next section. The "pick_list_*.yaml" find record the objects will be used. In this project, these objects is in yaml files:
* 'sticky_notes'
* 'book'
* 'snacks'
* 'biscuits'
* 'eraser'
* 'soap2'
* 'soap'
* 'glue'  
Use "capture_features.py" I can get data file.  
##### 3.2 Train and get model file
The accuracy of SVM depend on the number of datas and the parameter choice of SVM. I use 100 for data range and sigmoid to be the kernel of SVM. The key code as below:
<pre><code>
clf = svm.SVC(C=1, kernel='sigmoid')
kf = cross_validation.KFold(len(X_train),
                    n_folds=5,
                    shuffle=True,
                    random_state=1)
scores = cross_validation.cross_val_score(cv=kf,
                     estimator=clf,
                     X=X_train, 
                     y=y_train,
                     scoring='accuracy'
                                    )
print('Scores: ' + str(scores))
print('Accuracy: %0.2f (+/- %0.2f)' % (scores.mean(), 2*scores.std()))
</code></pre>
The accuracy as below:
![alt text][image9]
##### 3.3 Classify
First load the model:
<pre><code>
model = pickle.load(open('model.sav', 'rb'))
clf = model['classifier']
encoder = LabelEncoder()
encoder.classes_ = model['classes']
scaler = model['scaler']
</code></pre>
Then return pcl datas to ros msg, and caculate feature. Last do the prediction. The "detected_objects" is my ROS datas to publish.
<pre><code>
detected_objects_labels = []
detected_objects = []
for index, pts_list in enumerate(cluster_indices):
    pcl_cluster = extracted_outliers.extract(pts_list)
    cluster_ros = pcl_to_ros(pcl_cluster)
    chists = compute_color_histograms(cluster_ros, using_hsv=True)
    normals = get_normals(cluster_ros)
    nhists = compute_normal_histograms(normals)
    feature = np.concatenate((chists, nhists))
    prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
    label = encoder.inverse_transform(prediction)[0]
    detected_objects_labels.append(label)
    label_pos = list(white_cloud[pts_list[0]])
    label_pos[2] += .4
    object_markers_pub.publish(make_label(label,label_pos, index))
    do = DetectedObject()
    do.label = label
    do.cloud = cluster_ros
    detected_objects.append(do)
rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

detected_objects_pub.publish(detected_objects)
</code></pre>
##### 3.4  Make a logic to determine whether or not the object detections are robust.
I use set() for labels detected and received because there may be the same object with the wrong prediction.
<pre><code>
object_list_param = rospy.get_param('/object_list')
objects_list = []
for i in range(len(object_list_param)):
    objects_list.append(object_list_param[i]['name'])
if set(detected_objects_labels) == set(objects_list):
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass
</code></pre>
##### 3.5  Result
This project supply three group of objects, calls three world. So my result for three world as below.
* World 1:
![alt text][image10]
* World 2:
![alt text][image11]
* World 3:
![alt text][image12]

By five times tests with all three worlds, my code can recognize 3/3 objects in the first world, 4/5 objects in the second world, 7/8 objects in the third world on average.
#### 4 What to improve in Perception Section
The accuracy of perception is the main one to improve. The higher accuracy can be get when using DeepLearning in this project.

###  Action
The action part is mostly depend on the result of perception. Get the objects name and the position of centroid from perception. Then publish all of msg which we get msg_type from "PickPlace.srv". The "pick_place_routine" will make PR2 do the grap. The function "pr2_mover" will cover these.
#### 1. Get message type.
The messages in "PickPlace.srv" are described as below:
![alt text][image13]
So I need do this job at the end of function "pr2_mover":
<pre><code>
pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)
</code></pre>
#### 2. Get variables
These variables should be gotten "TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE".
TEST_SCENE_NUM is a int num to be set.
OBJECT_NAME can be gotten from rospy.get_param('/object_list')
WHICH_ARM can be gotten from rospy.get_param('/dropbox'), the param will tell PR2 the object should drop to which dropbox, that's mean which arm should be used.
PICK_POSE can be gotten with dropbox's position
<pre><code>
labelPosition = labels.index(OBJECT_NAME.data)
PICK_POSE.position.x = np.asscalar(centroids[labelPosition][0])
PICK_POSE.position.y = np.asscalar(centroids[labelPosition][1])
PICK_POSE.position.z = np.asscalar(centroids[labelPosition][2])
</code></pre>
PLACE_POSE can be gotten with centroid's position
<pre><code>
if object_group == dropbox_group[0]:
    WHICH_ARM.data = dropbox_name[0]
    PLACE_POSE.position.x = dropbox_position[0][0]
    PLACE_POSE.position.y = dropbox_position[0][1]
    PLACE_POSE.position.z = dropbox_position[0][2]
elif object_group == dropbox_group[1]:
    WHICH_ARM.data = dropbox_name[1]
    PLACE_POSE.position.x = dropbox_position[1][0]
    PLACE_POSE.position.y = dropbox_position[1][1]
    PLACE_POSE.position.z = dropbox_position[1][2]
</code></pre>
#### 3. Get centroids
To get centroid of object. I have to transform ROS msg into PCL file. And caculate the mean to be the centroid.
<pre><code>
centroids = []

for one_object in object_list:
    labels.append(one_object.label)
    points_arr = ros_to_pcl(one_object.cloud).to_array()
    centroids.append(np.mean(points_arr, axis=0)[:3])
</code></pre>
#### 4. Get yaml files
Thanks for help functions with this project, I just need to make a list and append msg in it.
<pre><code>
outputfile = "output_*.yaml"

dict_list = []
yaml_dict = make_yaml_dict(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)
dict_list.append(yaml_dict)

send_to_yaml(outputfile, dict_list)
</code></pre>
#### 5. Collision Avoidance
This is what I'm working on and also this is what to improve. The way to arrive it is make a 3D collision map. Object the PR2 is picking runtime will not in this collision map. But the table and remaining objects should be in collision map, this is useful to avoid PR2 stuck in the table as below pic. And the PLACE_POSE should be little different to avoid objects like a tower.
![alt text][image14]
