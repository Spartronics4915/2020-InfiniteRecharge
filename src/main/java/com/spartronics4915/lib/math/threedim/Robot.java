package com.spartronics4915.lib.math.threedim;

class Robot
{
    private Affine3 mCamToMount;
    private Affine3 mMountToRobot;
    private Affine3 mCamToRobot;
    private Affine3 mRobotToField;
    private Affine3 mCamToField;
    private double mTimestamp;
    private boolean mDirty;
    private String mRobotPose = null;

    public Robot()
    {
        mCamToMount = new Affine3();
        mMountToRobot = new Affine3();
        mCamToRobot = new Affine3();
        mRobotToField = new Affine3();
        mCamToField = new Affine3();
        mTimestamp = 0;
        mDirty = true;
    }

    /**
     * Sets the *fixed* mounting of the camera on the robot or turret.
     * We expect this method to be called upon robot's subsystem
     * initialization (and stored in a Constants file).
     * @param camPose
     */
    public void setCameraPose(String camPose)
    {
        this.mCamToMount = new Affine3(camPose);
    }

    public void setMountPose(String mountPose)
    {
        this.mMountToRobot = new Affine3(mountPose);
        this.mDirty = true;
    }

    /*
    # Robot field pose (ie: robot position/heading in field coordinates)
    # aka the robotToField 
    #
    # field coords (z is up)
    # field is approx x: [0, 52*12], y: [-26*12, 26*12] (z is up)
    #  in theory we don't need to know the exactl coords of the field
    #
    #                       y
    #                 ._____|____. 
    #  Blue Alliance  |     |    |  Red Alliance
    #                 |     o-----x
    #                 |__________| 
    #
    */

    /**
     * Periodically update the robot pose via a string
     * @param rposeString "x y angle" (angle in degrees)
     * @param timestamp time associated with pose
     */
    public void updateRobotPose(String rposeString, double timestamp)
    {
        String[] vals = rposeString.split(" ");
        assert vals.length == 3;
        double x = Double.parseDouble(vals[0]);
        double y = Double.parseDouble(vals[1]);
        double angle = Double.parseDouble(vals[2]);
        this.updateRobotPose(x,y,angle, timestamp);
    }

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
     * If the vision camera is mounted on an articulated mechanism (eg turret)
     * we expect to receive an update to the mount description.  The format
     * of mountPose may differ from season-to-season.  In the case of a turret
     * we expect that the mountPose is merely an angle to capture the current
     * turret angle.  Note that this must be composed with other mount 
     * characteristics (ie: its offset and orientation of the zero angle)
     */
    public void updateMountPose(String mposeStr, double timestamp)
    {
    }

    public Vec3 transformPoint(Vec3 pt)
    {
        if(this.mDirty)
        {
            this._rebuildTransforms();
            this.mDirty = false;
        }
        return this.mCamToField.transformPoint(pt);

    }

    public Vec3 transformVector(Vec3 dir)
    {
        if(this.mDirty)
        {
            this._rebuildTransforms();
            this.mDirty = false;
        }
        return this.mCamToField.transformVector(dir);
    }

    private void _rebuildTransforms()
    {
        this.mCamToRobot = Affine3.concatenate(this.mCamToMount, 
                                               this.mMountToRobot);
        this.mCamToField = Affine3.concatenate(this.mCamToRobot,
                                               this.mRobotToField);
    }

}