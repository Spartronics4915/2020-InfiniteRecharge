package com.spartronics4915.frc2020;

import com.spartronics4915.lib.math.threedim.*;

/**
 * CoordSysMgr2020 extends CoordSysMgr. See comments therein. We extend 
 * CoordSysMgr with the season-specific mounting of the camera onto the robot.
 */

 /* 
 * Atm: we envision two scenarios:
 * 
 * 1. camera mounted on moving launcher turret
 * 2. camera mounted rigidly onto robot
 * 
 * Robot - turret is mounted relative to robot origin by some amount in
 * x, y and z (z for robot origin is assumed 0 the floor).  Turret may 
 * also be rotated to point opposite to the robot front.
 * 
 *                       y
 *                 ._____|____. 
 *                 |     |    |             
 *                 |     o-----x
 *              x--- T        |
 *                 |_|________| 
 *                   y
 * 
 * Turret - x coord points to center of shooting.  Turret angle of 0
 * means shoot straight. We expect a range of turret angles from -45 to 45 deg.
 * Camera - mounted on turret offset by some amount in x, y (and possibly z).
 * Camera may also be tilted upward to minimize the pixels displaying the
 * ground.  This tilt is currently assumed to be around the camera's x axis.
 * 
 *                   , - ~ ~ 
 *            , '               ' ,
 *          ,           x           ,
 *         ,            |      -z    ,
 *        ,             |       |     ,
 *        ,     y-------o       C--x  ,   # at C, y is towards viewer
 *        ,                           ,
 *         ,                         ,
 *          ,                       ,
 *            ,                  , '
 *              ' - , _ _ _ ,  '
 * 
 * Camera - xy plane is camera plane.  This makes -z the direction that
 * the camera is pointing toward.
 * 
 *           y 
 *           |
 *           |
 *           C ------ -z
 *          /
 *         x
 *       
 */

public class CoordSysMgr2020 extends CoordSysMgr
{
    // camera to mount -----------------------------------------------------
    // camTilt represents the amount the camera is pointed up, measured in
    // degrees around camera's x axis. To measure it, point the camera at
    // a wall from a perpendicular orientation.  Mark the center of the 
    // camera with a mark.  Measure the distance from height of the mark
    // to the camera origin.  Measure the distance from the wall.  Use
    // degrees(atan2(mark, dist)) to compute the tilt angle.
    public static final double kCamTilt = 15; 

    // camPos represents offset of camera's origin relative to turret origin.
    // (measured in inches).  Camera origin is the center of the focal point.
    // So we do expect some z offset as well as x.
    public static final Vec3 kCamPos = new Vec3(0, -12, 0); 

    // camFlips maps camera axes to mount axes. eg: cam x is turret -y (above)
    public static final Vec3 kCamFlipX = new Vec3(0, -1, 0);  // cam +x is turret's -y
    public static final Vec3 kCamFlipY = new Vec3(0, 0, 1);   // cam +y is turret's +z
    public static final Vec3 kCamFlipZ = new Vec3(-1, 0, 0);  // cam +z is turret's -x

    // mount to robot ------------------------------------------------------
    // - Turret is mounted at robot back+right, up from ground
    // - Turret's x axis is opposite robot's 
    // NB: this mMntPos is also in Constants.Launcher.kRobotToTurret.
    // We interpret this as:  3.72 inches behind and 5.264 inches
    // to the right of the robot's origin. Current twodim code doesn't
    // consider the z origin (offset from floor). This becomes important
    // as we consider the camera "pitch"/tilt. Currently the height of
    // the turret origin relative to the robot origin (on the carpet), is
    // only guessed to be 8 inches.
    public static final Vec3 kMntPos = new Vec3(-3.72, 5.264, 8);
    public static final Vec3 kMntAxis = Vec3.ZAxis;
    public static final Affine3 kMntFlip = Affine3.fromRotation(180, kMntAxis);

    /**
     * Constructor for the CamToField pipeline for 2020 robot.  Presumably
     * we need only a single instance.
     */
    public CoordSysMgr2020()
    {
        /* NB: see CoordSysMgrTest for validation. The file is located in
        * the tests subtree (rather than here) to prevent pollution of
        * production jar by tests.
        */
        super();
        Affine3 camFlips = Affine3.fromAxes(kCamFlipX, kCamFlipY, kCamFlipZ);
        Affine3 camTilt = Affine3.fromRotation(kCamTilt, Vec3.XAxis);
        Affine3 camRot = Affine3.concatenate(camFlips, camTilt);
        Affine3 camOffset = Affine3.fromTranslation(kCamPos);
        Affine3 camToMount = Affine3.concatenate(camOffset, camRot);
        this.setCamToMount(camToMount);
        this.updateTurretAngle(0);
    }

    /**
     * Updates the mountToRobot coordinate system conversion. Should
     * reflect the current state of the turret angle.  This plus the
     * Robot's field pose is sufficient to calculate the conversion
     * of Vision targets into field targets. 
     * @param angle - turret angle; 0 is "straight" (degrees)
     */
    public void updateTurretAngle(double angle)
    {
        Affine3 m2rAim = Affine3.fromRotation(angle, Vec3.ZAxis);
        Affine3 m2rRot = Affine3.concatenate(kMntFlip, m2rAim);
        Affine3 m2rOffset = Affine3.fromTranslation(kMntPos);
        Affine3 mntToRobot = Affine3.concatenate(m2rOffset, m2rRot);
        this.setMountToRobot(mntToRobot);
    }

};