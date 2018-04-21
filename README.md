# Cheezoid
Cheese smelling robot, also with petrol detection.

Bugs: 
- Reverse scan throws up 0 scans.

Thoughts:

- Do forwardScans and a velocity calculation while moving.
  This will allow the robot to determine if it has got stuck, e.g. wheels go stuck.
- Make sure initial via point is quite close, this will allow to validate the movement.
- Add in checks to the move to ensure it doesn't crash.
    -these checks need to be robust to noise / failed scans etc. ie a reading <5 doesn't make sense


Findings:

- The nxt CAN NOT detect sensor angle in the region where it moves loosely.
Therefore we have to make an adjustment for the scans depending on the direction of movement.
By about 15 degrees
- 360/0 degree infront
- 270 is to the left
- 90 is to the right
- 180 is behind

-IN BOT SIM: positive angle rotates anti-clockwise
-This is also true with our robot. 
-In robot mA is on left, mB is on right.

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
Have some live plotting
Modify nxtcallibration to work with left right too.
    - For motor A and Motor B
    - Add new callibration variables to properties, to load callibration 
      and save to callibration.mat.
Fix my bodge of double scan, then delete scan cleaner
Make forward scan function

Structure:
- Run localise to find position. (does this run into a wall if you start it facing a wall?)
- Update bot.pos
- Run path planning
- Path follow (maybe every ~30cm, do a quick scan to update position estimate?)
- maybe while moving do forward scans, if this very different to what is expected then stop and rescan.
        This would allow us to only stop and scan when we need to.
- Reach destination. On stage of path one before the end. Do a scan, then make the final movement.
   I think this would be a combination of speed and accuracy.
- Play mario theme.



Apparently:

Modelling the sensor noise is very important.
So figure out when the angles are acute in the botSim
Then model them as unreliable for the actual robot.
Then 

