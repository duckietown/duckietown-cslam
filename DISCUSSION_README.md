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
    - [x] medium value (b = 0.2 or 0.1) --> give rather good results
    - [ ] big values   (b = 1 or 2)
    - [ ] real time test (all above is on same bag file for now)
  - [ ] find out what parameters should be applied
- [ ] Better retro-interpolation policy ? 
- [ ] Removing old vertices and edges and fixing watchtowers : see "long term experiment" section
- [ ] Edge measurement info : 6*6 covariance matrix
  - [x] Add the possibility to set them
  - [ ] Devise nice heuristics and implement them
- [ ] Test AprilTag 3 library
- [ ] Increase the camera resolution *(Aleks working on it)*

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
