Realtime plotting tool for ROS2.

Bugs and bug sources

* [ ] initial window dockspaces visible
* [ ] closable plot windows (but not topic view)
* [ ] click on member entry immediately adds to plot, but should allow dragdrop
* [ ] handle errors in configuration file and display on GUI
* [ ] show error states on topic communication
* [ ] warning about NaN values
* [ ] avoid plot colors of insufficient contrast
* [ ] don't crash when member is missing
* [ ] don't display sim time warnings if there is no data
* [ ] don't reassign line colors when switching from unavailable to available

features

* [ ] diagnostics per topic, with warning symbol
* [ ] sorting topics
* [ ] filter topics/types/members in list
* [ ] pausing function
* [ ] plot length of geometry_msgs/msg/Vector3
* [ ] unit suggestions based on convention
* [ ] range suggestions based on unit (angles -pi to pi)
* [ ] suggest auto-fit if all y values are off-plot
* [ ] pass flag to not start with default config
* [ ] if the config path in the argument does not exist, create new one
* [ ] show array elements, default to first item, aggregate array elements

## tests

To test GUI features, use the provided scripts in the `test` directory

* `test/publish_sim_twist.py` publishes velocity in sim time, and a sim time clock; the application should display a warning if launched with `use_sim_time:=false`
