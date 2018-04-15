# Cheezoid
Cheese smelling robot, also with petrol detection.

Bugs: 
- Reverse scan throws up 0 scans.

Thoughts:

- Do forwardScans and a velocity calculation while moving.
  This will allow the robot to determine if it has got stuck, e.g. wheels go stuck.


Findings:

- The nxt CAN NOT detect sensor angle in the region where it moves loosely.
Therefore we have to make an adjustment for the scans depending on the direction of movement.
By about 15 degrees
- 360/0 degree infront
- 270 is to the left
- 90 is to the right
- 180 is behind


Plan:

- Make a scan interpretter from the raw data


Maybe:

Rather than just doing plain angles.
Identify where features are for sure.
Then work on botSim with those.


ToDo:

NXTlocalise with new functions.
Finish Scan clear and make that align with the botscans
Make robot.m inherit from BotSim()
Update pathplanning functions to work with nxt.
Make move function detect actual velocity with forwardScan()

