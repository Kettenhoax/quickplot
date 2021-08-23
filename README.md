Realtime plotting tool for ROS2.

TODOs

Bugs and bug sources

* [ ] initial window dockspaces visible
* [ ] avoid field duplicates in config files
* [ ] closable plot windows, but not topic view
* [ ] click on topic triggers activate, but should allow dragdrop
* [ ] handle errors in configuration file and display on GUI
* [ ] show error states on topic communication
* [ ] warning about NaN values
* [ ] warning if timestamps are out of frame (forgot use_sim_time if required)
* [ ] store program config in ubuntu-recommended user directory rather than $HOME
* [ ] avoid plot colors
* [ ] /clock resets should trigger data clear
* [ ] add sim time note at time axis
* [ ] multiple remove buttons?

features

* [ ] sorting topics
* [ ] filter topics/types/members in list
* [ ] pausing function
* [ ] plot length of vector
* [ ] unit suggestions based on convention
* [ ] tooltip: click to add to plot i
* [ ] suggest auto-fit if all y values are off-plot

## Tests

To test GUI features, use the provided scripts in the `test` directory

* `test/publish_sim_twist.py` publishes velocity in sim time, and a sim time clock; the application should display a warning if launched with `use_sim_time:=false`
