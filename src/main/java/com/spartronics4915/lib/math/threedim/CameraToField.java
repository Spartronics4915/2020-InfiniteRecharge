package com.spartronics4915.lib.math.threedim;

/**
 * CameraToField captures the coordinate-system chain (kinematics) associated
 * with a camera mounted on a potentially-moving mount attached to a robot.
 * To produce season-specific behavior, please subclass this class.
 * 
 * Robot field pose (ie: robot position/heading in field coordinates)
 * aka the robotToField 
 *
 * field coords (z is up)
 * field is approx x: [0, 52*12], y: [-26*12, 26*12] (z is up), in theory we 
 * don't need to know its exact coords.
 * 
 * Field
 *                    y (+) (-312,312)
 *                    |_____ ______. 
 *     Blue Alliance  |            |  Red Alliance
 *                    o-----x      | (0,640)
 *                    |         R  |
 *                    |____________| 
 *
 * Robot
 *                       y
 *                 ._____|____. 
 *                 |     |    |             
 *                 |     o-----x
 *              x--- M        |
 *                 |_|________| 
 *                   y
 * 
 * Mount                    |     Camera (axometric view) - camera plane is xy
 *                .___.     |
 *                | C |     |          y 
 *                |   |     |          |
 *                | z |     |          |
 *                | | |     |          C ------ -z
 *         x -----|-M |     |         /
 *                |___|     |        x
 */

public class CameraToField
{
    private Affine3 mCamToMount;
    private Affine3 mMountToRobot;
    private Affine3 mCamToRobot;
    private Affine3 mRobotToField;
    private Affine3 mCamToField;
    private String mRobotPose;
    private double mTimestamp;
    private boolean mDirty;

    public CameraToField()
    {
        mCamToMount = new Affine3();
        mMountToRobot = new Affine3();
        mCamToRobot = new Affine3();
        mRobotToField = new Affine3();
        mCamToField = new Affine3();
        mRobotPose = null;
        mTimestamp = 0;
        mDirty = true;
    }

    /**
     * Sets the *fixed* mounting of the camera on the mount or robot.
     * We expect this method to be called upon robot's subsystem
     * initialization (and stored in a Constants file).
     * @param camPose- a string representing the pose obtained by
     *   manually constructing an Affine3, then invoking its asString() method.
     *   The string representation is compact and can be atomically transmitted
     *   via network tables.
     */
    public void setCamToMount(String camPose)
    {
        this.mCamToMount = new Affine3(camPose);
    }

    public void setCamToMount(final Affine3 cToM)
    {
        this.mCamToMount = cToM;
    }

    /**
     * Sets the mounting of the mount onto the robot.  If
     * the camera is mounted directly to the robot this is unneeded.
     * Otherwise we expect thie method to be called upon robot's subsystem
     * intialization (and stored in a Constants file) and potentially 
     * periodically during the match.
     * @param mountPose - a string representing the pose obtained by
     *   manually constructing an Affine3, then invoking its asString() method.
     *   The string representation is compact and can be atomically transmitted
     *   via network tables.
     */
    public void setMountToRobot(String mountPose)
    {
        this.setMountToRobot(new Affine3(mountPose));
    }

    /**
     * If the vision camera is mounted on an articulated mechanism (eg turret)
     * we expect to receive an update to the mount description.  The format
     * of mountPose may differ from season-to-season.  In the case of a turret
     * we expect that the mountPose is merely an angle to capture the current
     * turret angle.  Note that this must be composed with other mount 
     * characteristics (ie: its offset and orientation of the zero angle)
     */
    public void setMountToRobot(Affine3 mToR)
    {
        this.mMountToRobot = mToR;
        this.mDirty = true;
    }

    /**
     * Periodically update the robot pose via a string
     * @param rposeString "x y angle" (angle in degrees)
     * @param timestamp time associated with pose
     */
    public void updateRobotPose(String rposeString, double timestamp)
    {
        this.mRobotPose = rposeString;
        String[] vals = rposeString.split(" ");
        assert vals.length == 3;
        double x = Double.parseDouble(vals[0]);
        double y = Double.parseDouble(vals[1]);
        double angle = Double.parseDouble(vals[2]);
        this.updateRobotPose(x, y, angle, timestamp);
    }

    /**
     * Periodically update internal notion of robot pose via numbers.
     * @param x - x coordinate of robot origin on field
     * @param y - y coordinate of robot origin on field
     * @param angle - heading of robot in degrees.  If x of robot points to 
     * x of field, the angle is zero.
     * @param timestamp
     */
    public void updateRobotPose(double x, double y, double angle, 
                               double timestamp)
    {
        assert(timestamp > this.mTimestamp);
        this.mTimestamp = timestamp;
        Affine3 xlate =  Affine3.fromTranslation(x, y, 0);
        Affine3 rot = Affine3.fromRotation(angle, new Vec3(0,0,1));
        this.mRobotToField = Affine3.concatenate(xlate, rot);
        this.mDirty = true;
    }

    /**
     * This is the primary method to convert a camera-space point (a vision target)
     * into field coordinates suitable for consumption by the RobotStateEsimator.
     * @param pt - coordinates of target in camera space.  A point at the center
     * of the camera frame 10 feet away would be (0, 0, -120).
     * @return the target point converted to field coordinates.
     */
    public Vec3 transformPoint(Vec3 pt)
    {
        this._rebuildTransforms();
        return this.mCamToField.transformPoint(pt);

    }

    /**
     * This is the primary method to convert a camera-space direction into
     * into field coordinates. May be useful to ensure that the transform
     * chain has the expected result.
     * @param dir - unit-length direction vector in camera space.
     * @return the camera direction represented in field coordinates.
     */
    public Vec3 transformVector(Vec3 dir)
    {
        this._rebuildTransforms();
        return this.mCamToField.transformVector(dir);
    }

    /**
     * updates the transform chain whenever any inputs have changed.
     */
    private void _rebuildTransforms()
    {
        if(this.mDirty)
        {
            this.mCamToRobot = Affine3.concatenate(this.mMountToRobot,
                                                this.mCamToMount);
            this.mCamToField = Affine3.concatenate(this.mRobotToField, 
                                                this.mCamToRobot);
            this.mDirty = false;
        }
    }

}