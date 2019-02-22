## Questions for first meeting:
- Design : How and where do we save the trajectory data? Is there a predefined standard for AIDO? 
- HW : Stock of parts for new towers, should we rebuild the old ones (with new design)
- Should we stick with how the network is set up now, or should we make a dedicated network?
- Better way to increase the camera resolution?
- [for Aleks] HW exercises platform now urgent!!!

## Topics to handle
### Accuracy criterions
**Goals**:
- 1 cm diff
- 5° diff
- 1 or 2 Hz update (seems to be working, need more online tests)
- good MPE (maximum probable explanation) of path --> get rid of outliers --> robust kernels ? (see below)


### Usability
- [ ] Find a better way to map duckiebot-names to their apriltags numbers
- [x] Put all parameters in launch (or yaml files)
  - [ ] auto-importing of parameters? Param class?
- [ ] Better access to launch variables (configuration server)
- [ ] Exporting experiment data --> all robot paths
- [ ] Separate the different part of our work into different git repo! (acquisition/graph)


### Optimization time and accuracy
- [ ] Robust kernels in g2o : reduce outlier issues:
  - [x] Add the possibility to set them
  - [ ] testing:
    - [x] small values (b = 0.01 (1cm)) --> give bad results but faster convergence
    - [x] medium value (b = 0.2 or 0.1) --> give rather good results --> keeping this for real time test
    - [x] big values   (b = 1 or 2) --> No noticable difference with when using no kernel
    - [ ] real time test with medium values (all above is on same bag file for now)
  - [ ] find out what parameters should be applied
- [ ] Better retro-interpolation policy ? 
- [ ] Removing old vertices and edges: see "long term experiment" section
  - [x] Removing old vertices
  - [x] Stocking time defined in params
  - [x] Removing old vertices by chunck of half the stocking size
  - [x] "Free" indices to be used again --> make a cyclic counter
  - [x] Add emergency trigger to remove vertices if cyclic counter is overflown
  - [ ] Actually save trajectories to files (function is there, just need to code it)--> design TBD
- [ ] Edge measurement info : 6*6 covariance matrix
  - [x] Add the possibility to set them
  - [ ] Devise nice heuristics and implement them
- [ ] Test AprilTag 3 library
- [ ] Increase the camera resolution *(Aleks working on it)*
- [x] Make sense of the underused locks in duckietown_graph_builder
  - [x] reuse same lock as custom lock in g2ographbuilder
  - [x] protect graph modifications
  - [x] test on rosbag to lookout for deadlocks
  - [ ] real time test to spot out deadlocks 
- [ ] Find heuristic to setting watchtowers (and eventually roadsigns) fixed
- [x] Factorizing duckietown graph builder in mutliple classes (node class for instance!!):
  --> Made the code much more readable and easy to change and maintain!!

### Odometry
- Using visual odometry :
  - [x] Reading their code --> some unclear things:
    - [ ] asking Guillem to read us through it
  - [ ] Making sure it works
  - [x] Being careful that the time stamps are the ones we want --> they are not:
    - [ ] fixing this
  - [ ] Testing it!



### Long term experiments
**Reminder**: In the long term we want it to run 24/7, so we should anticipate this.

Ideas:  
- Setting watchtower to fixed state after a certain time. Maybe create a parallel g2o graph with only watchtowers and ground april tag, optimize it, and update the watchtower's positions in the complete one from time to time (to take into account that somebody or some bot moved a watchtower). Time interval to be defined.
- Removing old edges and point as time passes. Maybe just storing all robot paths as we suppress them from the graph. This way complexity stays limited even with long or "infinite" experiments. Need to find a way to cycle (or re-use) the integer mapping from timestamps to g2o integer identifiers. Amaury is starting to code this part (see his branch).


### Questions for g2o team

- Do we need to call "compute_active_errors" and "compute_initial_guess" at each optimization iteration?
- What exact purpose does the "online" argument of "optimize" serve? It seems to be used when optimization is done recursivly, which is our case. We might want to try it. We tried it and the g2o c++ lib crashes!
- What really are robust Kernel. The value we put in it corresponds to the "b" value in chapter 5 on robust kernels?
- In remove_vertex, what does the "detach" bool do?

### TROUBLES

- [ ] Graph builder sometimes crashes and sometimes not on same rosbags!
  extract of one of the logs from transform_listener_ros.py: I don't know though if that is why it crashed or because the c++ part crashes.
``` 
rospy.internal][WARNING] 2019-02-14 14:31:41,485: Unknown error initiating TCP/IP socket to amaury-FX503VM:36401 (http://amaury-FX503VM:44989/): Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/impl/tcpros_base.py", line 557, in connect
    self.read_header()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/impl/tcpros_base.py", line 650, in read_header
    self._validate_header(read_ros_handshake_header(sock, self.read_buff, self.protocol.buff_size))
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosgraph/network.py", line 362, in read_ros_handshake_header
    d = sock.recv(buff_size)
error: [Errno 104] Connection reset by peer

[rospy.internal][INFO] 2019-02-14 14:31:41,485: topic[/poses_acquisition/odometry] removing connection to http://amaury-FX503VM:44989/
[rospy.internal][WARNING] 2019-02-14 14:31:41,485: Unknown error initiating TCP/IP socket to amaury-FX503VM:36401 (http://amaury-FX503VM:44989/): Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/impl/tcpros_base.py", line 557, in connect
    self.read_header()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/impl/tcpros_base.py", line 650, in read_header
    self._validate_header(read_ros_handshake_header(sock, self.read_buff, self.protocol.buff_size))
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosgraph/network.py", line 362, in read_ros_handshake_header
    d = sock.recv(buff_size)
error: [Errno 104] Connection reset by peer
``` 
