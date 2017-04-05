# Registration server

This package provides services for the registration of RGBD views of an object (with texture). The services are defined [here](../registration_services). 


## RegistrationService

```
sensor_msgs/PointCloud2[] rgbd_views
geometry_msgs/Transform[] rgbd_views_odometry_transforms # can be blank
---
geometry_msgs/Transform[] rgbd_view_transforms
int32[] rgbd_view_correspondences
```

The result is stored in `rgbd_view_transforms` - corresponding to the transforms which align the views with each other, along with `rgbd_view_correspondences` denoting the number of [SIFT](../siftgpu) correspondences used to compute the transforms. 


## Start the system 

To start the registration server run:

```
rosrun registration_server rgbd_view_registration_server
```

For debugging, the result of the registration (i.e. registered point cloud) is published on the topic `/rgbd_view_registration/rgbd_view_cloud` (downsampled).

## Testing

A sample executable is provided to test the registration [here](test/test_rgbd_view_registration.cpp) (it also shows how to convert data in the right format, etc.):

```
rosrun registration_server test_rgbd_view_registration /path_to_folder_with_pcds
```

After finishing the registration, a PCL visualizer window will be opened showing the result. 

**Note** this has been tested with Asus Xtion point clouds, ~ 10-20 files. 
