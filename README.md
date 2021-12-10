A realtime plotting tool for ROS2.

# usage

```bash
ros2 run quickplot quickplot [config_file.yaml] [--ros-args -p use_sim_time:=true]
```

Plot config files are designed to be concise, with ROS topics as first-class citizens, so they can potentially be hand-written.

```
# example to plot speed and angular velocity of a Twist message on two axes
plots:
  - axes:
      - y_min: -2
        y_max: 2
      - y_min: -0.5
        y_max: 0.5
    sources:
      - topic_name: /cmd_vel
        member_path:
          - twist
          - linear
          - x
      - topic_name: /cmd_vel
        member_path:
          - twist
          - angular
          - z
        axis: 2
```

# roadmap

Bugs and bug sources

* [ ] click on member entry immediately adds to plot, but should allow dragdrop

features

* [ ] diagnostics per topic, with warning symbol in the list
* [ ] pausing
* [ ] plot length of geometry_msgs/msg/Vector3
* [ ] unit suggestions based on convention
* [ ] range suggestions based on unit (angles -pi to pi)
* [ ] suggest auto-fit if all y values are off-plot
* [ ] pass flag to not start with default config
* [ ] show error states on topic communication
* [ ] warning about NaN values
* [ ] if the config path in the argument does not exist, create new one
* [ ] show array elements, default to first item, aggregate array elements
* [ ] mouse wheel scroll time

* [ ] covariance viz
* [ ] array viz

## tests

To test GUI features, use the provided scripts in the `test` directory

* `test/publish_sim_twist.py` publishes velocity in sim time, and a sim time clock; the application should display a warning if launched with `use_sim_time:=false`
