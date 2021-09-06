`quickplot` is able to receive any serialized ROS2 message, but the library to deserialize it may not be installed.
In this case, the program should show a warning.
A simple way to test this is installing a message type, which is not installed on the host, to a Docker container, and publishing it.
`quickplot` must gracefully handle the discovered topic when run on the host.

```bash
# usage example
# vision_msgs was not required on the host system for me, but was released to galactic
# you can test with any message type that contains a numeric field
docker build --network host . -t quickplot_unknown_type
docker run --network host quickplot_unknown_type ros2 topic pub /object vision_msgs/msg/ObjectHypothesis
```