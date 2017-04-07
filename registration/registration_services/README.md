Registration services
=================================


This package defines a service message, which should be included by the packages using RGBD view registration (thus avoiding a dependency on the registration package, which in turn depends on CUDA). 


####  	RegistrationService

```

sensor_msgs/PointCloud2[] additional_views
geometry_msgs/Transform[] additional_views_odometry_transforms
---
geometry_msgs/Transform[] additional_view_transforms
int32[] additional_view_correspondences
```

This service registers the RGBD views acquired for a specific object a) with respect to each other. The underlying registration is done using [siftgpu](../siftgpu) and the CERES optimization engine. The registered poses of the object RGBD views are recorded in the vector `rgbd_view_transforms`, while the number of correspondences used per view are stored in `rgbd_view_correspondences`. 
