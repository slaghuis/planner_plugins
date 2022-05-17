# Planner Plugins
Plugins to the [Navigation Lite](https://github.com/slaghuis/navigation_lite/tree/octomap) Planner server.  These plugins, or similar onces are required by the Navigation Lite stack to run.

## Theta Star 
A Theta Star algoritm implementation.  Successfully tested in the simulator.  This plugin does path planning through a popoltaed Octomap

## No Plan
Suitible for testing of other logic.  No planning takes place.  The "planner" simply returns the provided start and end goal as the path.  CAUTION - NO PATH PLANNING IS DONE.
