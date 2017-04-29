# rgbd_grabber node

This node listens for point clouds on a topic (default `/camera/depth_registered/points`) and saves them to disk as pcds when receiving a control message (default topic `/teleop/keyinput`) or when pressing `s`. 

## Usage

```
rosrun rgbd_grabber rgbd_grabber [folder_to_save_data] [pcd_topic] [control_topic]
```
