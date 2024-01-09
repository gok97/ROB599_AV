# ROB599_AV

### Project Description
The project focuses on the development of a path-
planning and motion-controlling algorithm to enable autonomous
navigation of a static world by a delivery quadcopter subject to
wind disturbances. The developed path-planning algorithm relies
on RRT* to generate a viable path through obstacles, which is
then refined to guarantee time optimality and minimum snap
throughout navigation. The motion controller is a Non-Linear
Time-Varying MPC that fully captures the severe non-linearities
and couplings present in a quadrotor’s dynamics and successfully
rejects disturbances to its motion. The overall system is shown
to be highly effective at tracking complex trajectories in city-like
and forest-like environments with high obstacle density.

#### Motion-Planning
A two-stage planner is implemented: a high-level planner
first defines the general path the vehicle should take between obstacles; a low-level planner subsequently defines and smooth and efficient trajectory following said path.

RRT* is chosen as the high-level planner.

The following are the low-level planners experimented with:
1. Triangular V-Profile
2. Time Optimal
3. Time Optimal + Minimum Snap

<img src="Submissions\SubmissionFinal\Traj_Example.png" width="300"/>

#### MPC Controller
Non-Linear MPC is used as the choice of controller to perform trajectory-tracking. Linear as well as Non-Linear MPC were implemented and their performance were analysed.

The following graphics shows the trajectory tracking and disturbance rejection (wind, discrete mass drop) capabilities of both the Linear and Non-Linear MPC controllers.

![](Submissions\Submission2\Checkpoint2\DeliveryMission.gif)

