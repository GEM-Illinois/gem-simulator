# Lane detection module • David Young • 30 May 2019

# Synopsis

The lane detection module is responsible for finding the boundaries and
center line of a paved road or walkway using video from the GEM
vehicle's front-facing camera.

Lane detection will support *centering* the car on campus roads and
sidewalks so that the vehicle does not leave the pavement or collide
with a curb.

Lane-detection parameters include the height of the camera above the
ground, the camera's position with respect to the wheels, the
horizontal/vertical viewing angle of the camera, and whatever else it
takes to locate and orient the camera frustrum with respect to the
vehicle and road.

The module assumes that the road is flat and the vehicle is level.

The module subscribes to the video stream over the ROS software bus.
It detects edges in the video and fits curves to the edges that appear
to comprise the left and right boundaries of the lane.  The module
publishes polynomial coefficients or a piecewise linear approximation
of the lane boundaries and/or center line.  The ??? module subscribes
to lane detection.

# Personnel

Tianqi and David will start the development under guidance from Katie
and Sayan.  We may enlist Hussein, Peter, and Richard to help.

# Acceptance criteria

* Simulation: the simulated vehicle (in Gazebo?) detects the boundaries
  and center line of a simulated path.  Detected boundaries & center line are
  within tolerances TBD.  Impairments (gaps, smudges, branches?) are tolerated.

* Lab: in the IRL corral, the vehicle detects lanes marked with tape
  on the floor.  Impairments tolerated.

* Campus: on CSL Studio drive, the vehicle detects lanes acceptably during
  manual driving.

# Solution architecture

TBD TBD TBD
If any diagrams are appropriate, use them.

# Interfaces consumed & produced

E.g., sensors, input & output messages, user interface. 

Please link to the input specifications.

To begin with, mention what agents will consume the output
and the abstract content of the output.

Output specs will evolve.  Specify your outputs in as much
detail as possible.

# Documentation plan

What documentation do we need for the finished module/activity?
A user's manual?  Design & implementation overview?

Consider installation, configuration, user controls,
diagnostics.

# Software test plan

How will you verify the quality of your solution and guard
against regressions during development?

E.g., unit & integration tests in the simulator or other
test fixtures.

# Field experiment plan

How will we evaluate this module using the vehicle in
real-world conditions?  Does a test course need to be
constructed?  How many people need to take part in the
experiment?

# Effort estimate

Let the architecture, documentation & software test plan
guide your effort estimate.

Identify high-level tasks, then subdivide.  Try to subdivide
until you can picture yourself performing the subtask in
one day or less.

Take this opportunity to identify subtasks that you had
not thought of before.

# Further reading

If books, journal articles, or web pages will help other
researchers understand your goal and solution, you may
mention them here.

