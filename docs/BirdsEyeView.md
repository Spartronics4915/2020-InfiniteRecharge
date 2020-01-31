# Bird's Eye View for Programming 2020
- [Bird's Eye View for Programming 2020](#birds-eye-view-for-programming-2020)
  - [Operator Interface (Justice? ...)](#operator-interface-justice)
  - [Drivetrain / vslam (Henry, Declan, Jeffrey, ..)](#drivetrain--vslam-henry-declan-jeffrey)
  - [Control-panel (Noah)](#control-panel-noah)
  - [Intake (Freeman)](#intake-freeman)
  - [Indexer (Camden)](#indexer-camden)
  - [Turret Control (Ethan)](#turret-control-ethan)
  - [Climber (Austin)](#climber-austin)
  - [Vision (Darwin, Jeffrey)](#vision-darwin-jeffrey)
  - [Driver Cameras (Martin)](#driver-cameras-martin)
  - [Dashboard (Martin)](#dashboard-martin)
  - [Bling (Maya?)](#bling-maya)
  - [Scouting (Henry)](#scouting-henry)

## Operator Interface (Justice? ...)

* buttonboard negotiations
* coordinate with drivercams to cause correct camera feed to show on 
  driver station window(s)

## Drivetrain / vslam (Henry, Declan, Jeffrey, ..)

* excellent manual control over manoevering robot
* odometry / tracking via intel T265 / vslam
    * validation
        * drive characterization (for both main and test robots)
        * accuracy testing (ongoing)
* autonomous modes
    * any preplanned paths? 
        * paths repository class
        * resolve units disparity between field and dashboard/network tables
    * use of robot pose to control turret
        * only on the shooting portion of the field?
    * coordinate with vision to update/reset robot pose (correct for vslam drift)
    * control panel
        * careful approach (via sonar or odometry)
        * tweak-mode drive to reduce change of breakage?

## Control-panel (Noah)

* raise/lower spinner arm
    * manage fragility of the arm
        * sonar-approach (with drivetrain team)
        * distance-approach (with drivetrain team)
        * only raise when we're close and safe and slow
* control spin amount
    * color sensor mounting and validation
    * understand requirements of spinning motor / drivetrain coupling
        * allow driver position/pressure tweaks?
        * tradeoff between motor rotation speed and power and slippage
    * consider whether driver control is valid option

## Intake (Freeman)

* collaborate with Indexer to ensure intake is active when we're in
  collection mode.

## Indexer (Camden)

* collaborate with Intake to store balls
    * monitor sensors to collect balls. Signal full to disable intake
        and trigger last-ball-storage in intake.
* collaborate with Turret to shoot balls
    * kick ball(s) sequentially into Turret/shooter, shooter must be
        ready to receive balls (ie: flywheel up-to-speed & turret aimed)

## Turret Control (Ethan)

* given a robot pose, determine correct turret pose and flywheel speed to score.
    * characterize flywheel, spinup time
    * pid tuning
* turret aiming (horizontal & vertical)
    * is this a position control?
    * when is this active?
        * collaborate with Vision/Drivetrain to cause turret to 
        track/approximate target

## Climber (Austin)

* extend light-saber to attach to bar
  * what if we miss?  redo?
* after attach, trigger winch, stop when "appropriate"
  * do we need to monitor or limit current here?
* work with drivetrain team to determine if climber state should depend on
  robot position.

## Vision (Darwin, Jeffrey)

* strategy 1 (fire-and-forget PNP)
    * states:
        * quiet
        * searching (low res, center-bbox) -> xerror, yerror 
            * inputs: how do we enter this mode?
            * output:  
                /Vision/bboxErrorX, /Vision/bboxErrorY, /Vision/framerate
        * acquiring (high res, pnp) -> camera-local P of center (turret coords)
            * inputs: how do we enter this mode? (button push)
            * output:  /Vision/PnP/targetCenterX,Y,Z (Z is distance)
            * a one-off state, we proceed to quiet or searching (ignored)
    * unknowns/risks
        * is there a cost to resolution/change during pipeline (?)
        * what are the accuracy requirements of acquire moder?
            * eg: 2 inches at 10 feet?
            * reliability of shooter itself
        * trig for inner-vs-outer error metric
* strategy 2 (run-n-gun)
    * consider whether it's possible to shoot multiple balls while moving
* strategy 3 (just inner pid)
    * no PnP, just use bbox and known geo associated with target to
      produce distance and angle estimates.

## Driver Cameras (Martin)

* camera placement and count
  * front, back, up (?)
* determine raspi count
  * validate camera switcher - if it works, then 1 raspi may suffice
* update to WPI 2020 img

## Dashboard (Martin)

* final drive-team layout of cameras and match controls
    * heads-up display
        * reticles?
        * climber current?
        * battery? 
        * gamestate? (match-time, target color, ...)
    * camera feed switching
    * visualization of robot and turret poses on the field
        * abstract robot display so we don't get a cone on last years robot.
        * two cones might be needed (turret FOV vs turret angle)
* diagnostics layout
    * subsystem status and current commands
    * graphs
        * climber current ? - work with climber team
        * drivetrain motor PIDs (?) - work with drivetrain team
        * sonar
        * color wheel sensor
        * color wheel distance sensor (?)
    * path planning
        * update robot specs for 2020
        * work with drivetrain team to determine if useful

## Bling (Maya?)

* work with OI team to map buttons and robot state to different Bling patterns
* make sure we have a backup solution for LED strip and arduino?

## Scouting (Henry)

* work with strategy team to determine what data we want to scout
* develop a variety of views of the data
* develop / improve data gathering tools ?  AppSheet app.

