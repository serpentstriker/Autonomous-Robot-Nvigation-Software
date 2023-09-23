# Autonomous-Robot-Navigation-Software

My Team and I developed an autonomous robot for Dalhousie University Gregson Design Challenge 2022. The rules and scoring are listed below. We placed first in the competition by disarming 6 mines and removing 3.

![image](https://user-images.githubusercontent.com/61470710/210140986-bf64b6a3-d9f3-4bea-a9c2-a78d8064e948.png)


1. Robots must operate safely without endangering the users or the public. A pre-inspection of your
robot will be performed, and only approved robots are permitted in the building/on the course.

2. No modifications to the course structure, wildlife, or the deployed mines are permitted.

3. No pneumatic or hydraulic systems of any kind are permitted.

4. No voltages over 24V are permitted.

5. Only one physical robot can be deployed; it may then separate on the course if designed to do so.

6. Your team of 3 students is one group. One group is permitted 3 attempts on the course or a total of
15-minutes, whichever condition is fulfilled first

7. Only parts provided in the initial kit and those paid for from the flexible budget can be used.

8. The diameter of the robot must be less than 1.5’ or 46cm.


Scoring
9. [20 pts] Awarded if the robot leaves the 2’ box at coordinates marked “Deployed.” The robot must
move away from the deployment zone to commence mine sweeping operations.

10. [15 pts] Max. awarded if the robot reaches 3 known mine locations, and confirms presence by flashing
a light or other suitable indicator. The locations are provided in a pre-specified waypoint list and are
shown in the boxes above. Each mine is located at a unique position [X, Y, Theta], and there are 5
points awarded per mine reached.

11. [15 pts] Max. awarded if the robot can find 3 unknown mine locations and relay the pose [X, Y, Theta]
to the user via Terminal. Detection range tolerance must be ±1’ or better, and 5 points will be
awarded for each correct mine detected and pose relayed.

12. [18 pts] Max. awarded if the robot deactivates all mines. This will be judged by observing if your robot
communicates with the mine effectively and results in a deactivated mine at location. Each mine
deactivated will award the team 3 points.

13. [18 pts] Max. awarded if the robot disposes all mines after deactivation. This will be judged by
observing if your robot moves the deactivated mine effectively to the disposal location ±1’. Each mine
disposed will award the team 3 points.

14. [14 pts] Max. awarded for swift operation. Time is of the essence and completely removing all 6
mines within 10 minutes will award 14 points. Beyond 10 minutes, 3 points are deducted for every
minute into overage; for example, at 12 minutes 30 seconds, 5 points would be awarded for
timeliness. If mines remain on the field at the conclusion of the operation, no points will be award
for timeliness.

15. [-10 pts per] Any physical contact with the fence or wildlife results in forfeiting 10 points as the robot
will be deemed unsafe. Accordingly, the vehicle must have an obstacle detection and path planning
routine active. Every contact is a 10 point deduction.

16. [-20 pts per] Any physical contact with an active mine results in forfeiting 20 points as the robot will
sustain heavy damage. Every contact is a 20 point deduction.
Mine characteristics (may be subject to minor changes)
- 3D printed cylinder; diameter to be 4-5”, base 1” tall, overhang handles around perimeter that
are 0.5-1” tall, total height 2-3” tall
- Mine will have metal tape.
- LEDs are red when active; LEDs are green when deactivate.
- Activated on power cycle.
- Deactivated when
a. a photodiode receives a frequency of 4 kHz;
b. signal is received within ~360 degree light acceptance;
c. signal is received within 2’ diameter range.
d. there may be gaps in disarming signal range due to environmental decay

- If contacted or bumped (accelerometer) when activated, the mine lights will flash.






