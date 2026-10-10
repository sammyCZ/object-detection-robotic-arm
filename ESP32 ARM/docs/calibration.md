How to calibrate each joint
Leave the pulse range at 500/2500 µs, the standard mapping for 180° servos, so the angles you type roughly match the servo's real angle. What you're finding is each joint's usable range: where it stops before hitting an end stop, itself, the table, or straining a cable.

Open the monitor and type calib on. Then, for each joint (gripper first, shoulder last):

Find the minimum. From 90°, step down 5° at a time, slowly:

joint 0 85 1000
joint 0 80 1000
...
Stop at the first sign of trouble: it hits something, it buzzes when stopped, it stalls, or a cable pulls tight. Go back up 5–10°. That angle is minDeg.
Find the maximum the same way, stepping up from 90°. That's maxDeg.
Return to 90° before moving on to the next joint (joint 0 90 1500).
Joint-specific tips:

Gripper: "open" is fully open without strain. "Closed" is where the fingers just touch. Don't let it squeeze until it stalls; a stalled servo draws its maximum current and overheats. Those two angles are its min and max; which is which depends on the servo's direction.
Shoulder and elbow: test these with the arm stretched out, where gravity loads them most, and watch for sagging or buzzing. Also try them in combination, because the elbow may hit the base only when the shoulder is low. Pick limits that are safe whatever the other joint is doing.
Base: watch the cables as it turns; they're usually what limits it.
Home pose: choose a pose that's stable and folded in, where the snap at power-up won't hit anything. Find it with pose ..., then read the numbers from status.
When you're done, type calib off; any joint outside the old limits eases back in.

Recording the results
Fill these in and send them to me (or edit config.h yourself):


joint      min   max   home   direction of +angle (e.g. "closes", "turns left", "lifts")
base
shoulder
elbow
wrist
gripper
Once I have them, I'll update config.h and widen the limits. If a joint's direction feels backwards, I'll flip it by swapping its two pulse values. Then I'll flash and we'll check the whole range with calibration mode off.

This is the quick manual version. Stage 5 adds a proper tool that measures the exact pulse ranges and records taught positions for the camera mapping. Accurate angles only matter if we add inverse kinematics later; the taught-grid approach doesn't need them.

I can save this procedure to docs/calibration.md next to the wiring doc if you want it for reference.