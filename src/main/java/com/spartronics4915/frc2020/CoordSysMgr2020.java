package com.spartronics4915.frc2020;

import com.spartronics4915.lib.math.threedim.*;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;

/**
 * CamToField2020 extends CameraToField. See coordinate system comments
 * therein. We extend CameraToField with the season-specific mounting of
 * the camera onto the robot.
 * 
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
    private final double mCamTilt = 15; // if you change this also change

    // camPos represents offset of camera's origin relative to turret origin.
    // (measured in inches).  Camera origin is the center of the focal point.
    // So we do expect some z offset as well as x.
    private final Vec3 mCamPos = new Vec3(0, -12, 0); 

    // camFlips maps camera axes to mount axes. eg: cam x is turret -y (above)
    private final Vec3 mCamFlipX = new Vec3(0, -1, 0);  // cam +x is turret's -y
    private final Vec3 mCamFlipY = new Vec3(0, 0, 1);   // cam +y is turret's +z
    private final Vec3 mCamFlipZ = new Vec3(-1, 0, 0);  // cam +z is turret's -x

    // mount to robot ------------------------------------------------------
    // turret is mounted at robot back (3.72 inches behind robot center)
    // turret's x axis is opposite robot's 
    private final Vec3 mMntPos = new Vec3(-3.72, 5.264, 8); 
    private final Vec3 mMntAxis = Vec3.ZAxis;
    private final Affine3 mMntFlip = Affine3.fromRotation(180, this.mMntAxis);

    /**
     * Constructor for the CamToField pipeline for 2020 robot.  Presumably
     * we need only a single instance.
     */
    public CoordSysMgr2020()
    {
        /* NB: see CamToFieldTests for validation. The file is located in
        * the tests subtree (rather than here) to prevent pollution of
        * production jar by tests.
        */
        super();
        Affine3 camFlips = Affine3.fromAxes(mCamFlipX, mCamFlipY, mCamFlipZ);
        Affine3 camTilt = Affine3.fromRotation(mCamTilt, Vec3.XAxis);
        Affine3 camRot = Affine3.concatenate(camFlips, camTilt);
        Affine3 camOffset = Affine3.fromTranslation(mCamPos);
        Affine3 camToMount = Affine3.concatenate(camOffset, camRot);
        this.setCamToMount(camToMount);
        this.updateTurretAngle(0);
    }

    /**
     * Called periodically to update the camToField conversion.
     * @param angle - turret angle; 0 is "straight" (degrees)
     */
    public void updateTurretAngle(double angle)
    {
        Affine3 m2rAim = Affine3.fromRotation(angle, Vec3.ZAxis);
        Affine3 m2rRot = Affine3.concatenate(mMntFlip, m2rAim);
        Affine3 m2rOffset = Affine3.fromTranslation(mMntPos);
        Affine3 mntToRobot = Affine3.concatenate(m2rOffset, m2rRot);
        this.setMountToRobot(mntToRobot);
    }

};