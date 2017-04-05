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

