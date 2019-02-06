### Topics to handle:

- Robust kernels in g2o : reduce outlier issues


### Questions for g2o team:

- Do we need to call "compute_active_errors" and "compute_initial_guess" at each optimization iteration?
- What exact purpose does the "online" argument of "optimize" serve? It seems to be used when optimization is done recursivly, which is our case. We might want to try it.
