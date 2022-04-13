# Simulation

_We are using the map worlds/iris\_arducopter\_runway.world_

In all the experiments we took off with the drone to a certain height, set an alternative home-point far away from it and then changed the yaw using RC untill the heading was good.
When the yaw was correct, we changed the pitch using RC commands to make the drone fly straight.

* **All of the problems written here, does not take affect with the ardupilot SITL simulator.**
* It is recomended to use the SITL simulator, and for now, abandon gazebo.

## Mode Change From AltHold To Guided/Auto At High Speed

In the simulation, if the drone is to fast (above 20 m/s) then when it tries to stop (In guided or auto mode) it handles it very poorly while yawing aggressively and may even crash.

**This is probably a simulation bug**

## Drone Achieving Very High Speeds

The simulation does not simulate drag force, so the drone is accelerating to very high speeds - up to 50 m/s.

I looks like when the drone in the simulation passes the 10\~20 m/s barrier its behavior is partly undefined.

## Random Crashes (At High Speeds)

It looks like the drone crashes into something after a long period of time.

* When going west where there are hills, the height starts to decrement until it becomes 0 and then the drone start to turn around very fast (looks like a crash).
  * It seems that it crashes into the hills.
* When going north to where it looks like a city in the map, it just crashes in mid-air (It happend when the drone height was 300 meters). Maybe the simulator simulates high buildings?

The height in the simulator shows the height relatively to the takeoff point and should not simulate topography, so the written above is not the case.

**This is probably a simulation bug.**

### Experiment #5 - Crash

![](experiments/expr5-300mHeight_unexpectedCrash_directionFixInTheMiddle/Screenshot%20from%202022-04-03%2017-08-26.png)
![](experiments/expr5-300mHeight_unexpectedCrash_directionFixInTheMiddle/Screenshot%20from%202022-04-03%2017-08-35.png)
![](experiments/expr5-300mHeight_unexpectedCrash_directionFixInTheMiddle/Screenshot%20from%202022-04-03%2017-08-44.png)
![](experiments/expr5-300mHeight_unexpectedCrash_directionFixInTheMiddle/Screenshot%20from%202022-04-03%2017-08-45.png)
![](experiments/expr5-300mHeight_unexpectedCrash_directionFixInTheMiddle/Screenshot%20from%202022-04-03%2017-08-49.png)
![](experiments/expr5-300mHeight_unexpectedCrash_directionFixInTheMiddle/Screenshot%20from%202022-04-03%2017-08-51.png)
![](experiments/expr5-300mHeight_unexpectedCrash_directionFixInTheMiddle/Screenshot%20from%202022-04-03%2017-08-55.png)
![](experiments/expr5-300mHeight_unexpectedCrash_directionFixInTheMiddle/Screenshot%20from%202022-04-03%2017-09-00.png)

## Flying Direction

As described above, the only RC command the drone receives is pitch forward but the movement direction heading drifts along the drone flight. Its movement looks like an curve that drifts to a random direction (most of the times I think that the arc direction was counterclockwise).

* The heading of the drone (given by the compass) is correct along all the flight.
* There should not be wind in the simulation.
* The heading direction gets to be sometimes up to 90 degrees from the starting point.

### Solution: Pitch angle

We gave the drone 0.1 RC pitch forward command that apperantly converted to be less then 2 degrees which is a low value for pitching.

#### Why Does It Matter?

* It seems that the simulation random errors are to significant for being able to pitch 2 degrees stabely and that creates random curves as seen untill now.

#### Fixing This Problem

* Giving the drone bigger pitch angle can help. After giving it 0.5 RC pitch forward command, the pitch angle was close to 15 degrees. The drone successfully flew straight.

#### Problems

* It seems that when the drone angle is 15 degrees it really struggles to climb up.
* In the simulation the drone gets to rediculus speeds (up to 50 m/s), see (#Air Speed)

**As said above, this is a simulation error and does not happen with the ardupilot SITL.**

### Experiment #1 - Curved flight

![](experiments/expr1/Screenshot%20from%202022-04-03%2011-29-32.png)
![](experiments/expr1/Screenshot%20from%202022-04-03%2011-29-55.png)

### Experiment #2 - Curved flight

![](experiments/expr2-2_sessions/Screenshot%20from%202022-04-03%2011-44-37.png)
![](experiments/expr2-2_sessions/Screenshot%20from%202022-04-03%2011-44-56.png)
![](experiments/expr2-2_sessions/Screenshot%20from%202022-04-03%2011-45-11.png)
![](experiments/expr2-2_sessions/Screenshot%20from%202022-04-03%2011-45-26.png)
![](experiments/expr2-2_sessions/Screenshot%20from%202022-04-03%2011-45-42.png)

## Thoughts

When giving the drone fixed pitch in the RC controls, its speed gets **very** high (up to 30~40 m/s). This may cause problems.

* When setting the drone on auto mode and giving it a mission to loop on (In the north where there is a kind of city), it does it great by default, but when done after setting the flight speed to 30 the same "crashing" simptomes apear near corners of the mission.
  * Sharp height loss.
  * Sharp yawing that seems uncontrolled.
  * High and sharply changed roll and pitch angles.

These things are probably happening because of the drone trying to stop itself very fast.

There is a weird phenomenon - When the drone is on alt_hold mode **with valid gps** it goes in a straigt line, but when it is on alt_hold mode **without a valid gps** it starts to do the weird curves.

* Something to pay attantion is that when we disable the gps by setting the SIM_GPS_DISABLE paramter to 1, a message in the console shows: **AP: EKF3 IMU0 stopped aiding** and **AP: EKF3 IMU1 stopped aiding**. Maybe this command disables the imu too?
