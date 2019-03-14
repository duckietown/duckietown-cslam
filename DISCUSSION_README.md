## Questions for first meeting:
- NETWORK RELIABILITY!??
    - Sascha, Mac, Selcuk, Andrea, Jacopo
    - Systematic debugging (i.e. remove internet)
    - Check the router specs and statics
    - avahi 
    - check if we changed something in the configuration of the duckiebots (net related)
- Should we stick with how the network is set up now, or should we make a dedicated network?
- HW : Stock of parts for new towers, should we rebuild the old ones (with new design) check RPis of old towers
- Design : How and where do we save the trajectory data? Is there a predefined standard for AIDO? 
- Better way to increase the camera resolution?
- Apriltags3 debugging
- We should start with performance testing ASAP
- [for Aleks] HW exercises platform now urgent!!!
- We need CAD models of MegaBots, old and new watchtowers to get transforms to their baselink from cameras/mounted tags
- It would also be good to have CAD models for watchtowes for better visualization (low priority)
- Existing framework/example of web interface for controlling watchtowers
- Distance measurement laser to validate measurements of apriltags (they should be probably approximately correct but its good to verify) -> Marcus
- CHECK the camera distortion models
- Andrea said something about having local bag files (which are split every 5 mins) in the meeting on Wed. Is there any existing code to manage that as well as to auto retreive those from the duckiebots/watchtowers. Although I don't trust bag files after the sys-id assignment, maybe compressed image bags are better? If we can have proper frequency on bag files locally recorded, we can merge them and that will definitely give better results than running the graph optimizer online. Possible issue: Time on all watchtowers/duckiebots should be in perfect sync. This sync can cause problems while doing graph optimization online as well. 
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
- [x] Better retro-interpolation policy
  - [x] Re-read by Amaury, no idea to make it better
  - [ ] Re read by somebody else?
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
  - [x] Make a python binding
  - [ ] Test it online
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



### Debugging
 - Web interface for watchtowers
   - [ ] Support to launch acquisition
   - [ ] Support to view images
   - [ ] Add functions of existing diagnostics UI to this

### Long term experiments
**Reminder**: In the long term we want it to run 24/7, so we should anticipate this.

Ideas:  
- Setting watchtower to fixed state after a certain time. Maybe create a parallel g2o graph with only watchtowers and ground april tag, optimize it, and update the watchtower's positions in the complete one from time to time (to take into account that somebody or some bot moved a watchtower). Time interval to be defined.
- Removing old edges and point as time passes. Maybe just storing all robot paths as we suppress them from the graph. This way complexity stays limited even with long or "infinite" experiments. Need to find a way to cycle (or re-use) the integer mapping from timestamps to g2o integer identifiers.

### Questions for g2o team

- Do we need to call "compute_active_errors" and "compute_initial_guess" at each optimization iteration?
- What exact purpose does the "online" argument of "optimize" serve? It seems to be used when optimization is done recursivly, which is our case. We might want to try it. We tried it and the g2o c++ lib crashes!
- What really are robust Kernel. The value we put in it corresponds to the "b" (or b squared) value in chapter 5 on robust kernels?
- In remove_vertex, what does the "detach" bool do?

### TROUBLES
- The sky is clear


