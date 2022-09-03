# f110-mpc
model predictive control in f1tenth simulator
![ezgif com-gif-maker](https://user-images.githubusercontent.com/75038294/188287172-a3a5bced-eb3e-4f0f-b888-33585b3a36c6.gif)

This project is a racing strategy for autonomous racing for the f1tenth platform. The goal of this project is to compete head to head with opponents on the race track at high speeds without crashing into the opponent or any other obstacles.

This approach uses a offline pre-computed optimised trajectory around the racetrack for global planning. The challenge was to obtain a optimised race line around the racetrack which the car would follow to get the quickest laps. This race line optimisation is out of scope for this project.

The next part of this challenge was to track this race line accurately at high speed. A basic pure pursuit algorithm was used for path tracking, this is a great approach because of its simplicity ,but this doesn't take care of obstacle avoidance and its performance depends a lot on the look ahead distance tuning and could leading to the car drifting at high speeds.

So a Model Predictive Controller was used for local planning as well as tracking:
1. In this approach , we use 20 offline calculated trajectories for local planning (mini paths). These trajectories are generated using a simple mathematical model of the car (bicycle model) and forward simulating it for a certain time dt with constant speed and curvature.
These 20 trajectories are checked for collisions. The ones which collide with a obstacle are discarded .Out of the remaining trajectories, the one with its end point closest to the race line is selected and fed into the MPC for tracking.This process is repeated once the car get some threshold distance closer the end of the mini path.

2. In the second MPC approach we can combine MPC and obstacle avoidance by enforcing additional constraints.  For this we find the largest gap in the laser scan points and then obtain the line equations of the start and end points of the gap from the laser frame origin. These constraints ensure the resultant MPC trajectory to stay between the lines hence avoiding any obstacles.
(mention drift control)

3.The Third approach(ongoing) is useful for overtaking maneuvers. In this approach the optimised race line is used to create 6 more race lines(lanes) that are concentric but spread outwards on the racetrack. This enables the racing agent to have multiple look ahead points on every lane. So if the opponent agent is roughly occupying a certain lane , then the ego agent can switch lanes and overtake the opponent agent. Also after the overtake the ego agent can keep checking the inner lanes for availability and switch to inner most lanes for optimality.  
  
