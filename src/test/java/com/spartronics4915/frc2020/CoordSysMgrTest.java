package com.spartronics4915.frc2020;

import com.spartronics4915.lib.math.threedim.math3.*;
import com.spartronics4915.lib.math.twodim.geometry.*;
import com.spartronics4915.lib.util.Units;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

class CoordSysMgrTest
{
    double kEpsilon = 1e-9;

    @Test
    void usageExample()
    {
        CoordSysMgr2020 ctof = new CoordSysMgr2020();
        ctof.updateTurretAngle(0);

        // robot at center of field, pointing right
        Vec3 robotPt = new Vec3(320, 0, 0);
        double robotHeading = 0;
        ctof.updateRobotPose(robotPt.getX(), robotPt.getY(), robotHeading);

        // This should give us the location of the camera relative to the robot
        // when turret angle is 0.
        Vec3 camOnRobot = ctof.camPointToRobot(Vec3.ZeroPt);
        double dist = camOnRobot.length(), distXY;
        assert (dist > 20 && dist < 25);
        // for a robot and turret angle of 0, predict camera position on field.
        Vec3 camPtOnField = ctof.camPointToField(new Vec3(0, 0, 0));
        Vec3 camDirOnField = ctof.camDirToField(new Vec3(0, 0, -1));
        assertEquals(camDirOnField.getX(), -1, .2); // camera tilts into z-up
        assertEquals(camDirOnField.getY(), 0, kEpsilon); // camera tilts into z-up
        distXY = camPtOnField.subtract(robotPt).lengthXY();
        assert (distXY < 18);

        // Let's see where a point 100 inches away from camera is
        // should be "behind the robot" offset by combination of
        // turret and camera offsets.
        Vec3 tgt = new Vec3(0, 0, -100);
        Vec3 p0 = ctof.camPointToRobot(tgt);
        assert (p0.getX() < -100); // mostly behind the robot (x is front)
        distXY = p0.lengthXY();
        assert (distXY > 100); // farther from robot than camera (even xy only)

        // Now point turret directly toward turret right (robot left)
        ctof.updateTurretAngle(-90); // turret x is fwd, +y is left
        p0 = ctof.camPointToRobot(tgt);
        assert (p0.getY() > 100); // mostly to the left of the robot (+y is left)
        distXY = p0.lengthXY();
        assert (distXY > 100); // since robot origin is farther from target

        // point turret directly to its left (robot right)
        ctof.updateTurretAngle(90);
        p0 = ctof.camPointToRobot(tgt);
        assert (p0.getY() < -90); // mostly to the left of the robot (+y is left)
        double distXY2 = p0.length();
        // robot origin farther from points turret sees to its right.
        assert (distXY2 > distXY);

        // robot heading north, target below/right, 45 degrees,
        // variant of updateRobotPose that estimates the robot transform
        // given a measurement and an expected location.
        ctof.updateTurretAngle(0);
        robotHeading = 90;
        Vec3 robotRelativeTarget = new Vec3(-100, -100, 96);
        Vec3 knownFieldLocationTarget = new Vec3(628, -67.5, 96);
        ctof.updateRobotPose(robotHeading, robotRelativeTarget, knownFieldLocationTarget);
        Vec3 robotLocation = ctof.robotPointToField(Vec3.ZeroPt);
        Vec3 tMinusR = knownFieldLocationTarget.subtract(robotLocation);
        assertEquals(robotRelativeTarget.length(), tMinusR.length(), kEpsilon);
        assertEquals(tMinusR.getX(), 100, kEpsilon);
        assertEquals(tMinusR.getY(), -100, kEpsilon);
        assertEquals(tMinusR.getZ(), 96, kEpsilon);

        // same as above, but now the target is exactly behind the robot so
        // the y coord of tMinusR is 0
        robotHeading = 135;
        ctof.updateRobotPose(robotHeading, robotRelativeTarget, knownFieldLocationTarget);
        robotLocation = ctof.robotPointToField(Vec3.ZeroPt);
        tMinusR = knownFieldLocationTarget.subtract(robotLocation);
        assertEquals(robotRelativeTarget.length(), tMinusR.length(), kEpsilon);
        assertEquals(tMinusR.getY(), 0, kEpsilon);

        // robot heading west (field x+) at midfield
        robotHeading = 0;
        Vec3 robotPoint = new Vec3(320, 0, 0);
        // target point is blue alliance target in lower right
        Vec3 targetPoint = new Vec3(628, -67.5, 0);
        Vec3 targetDir = targetPoint.subtract(robotPoint).asUnit();
        ctof.updateRobotPose(320, 0, robotHeading);
        // mount direction should be negative (ie > 90 && < 270)
        // note that this angle will always be positive and doesn't
        // imply the sign of the rotation angle for the turret
        Vec3 mountDir = ctof.fieldDirToMount(targetDir);
        double mountAngle = mountDir.angleOnXYPlane();
        // for turret pointing x forward, angle near 160 is
        // behind and left
        // x
        // |
        // y---o
        assertEquals(mountAngle, 167.6, .05);

        // now let's reverse our robot
        robotHeading = 180;
        ctof.updateRobotPose(320, 0, robotHeading);
        mountDir = ctof.fieldDirToMount(targetDir);
        mountAngle = mountDir.angleOnXYPlane();
        assertEquals(mountAngle, -12.4, .05);
    }

    private Affine3 getRToField(Vec3 pos, double heading)
    {
        return Affine3.fromTranslation(pos).rotate(heading, Vec3.ZAxis);
    }

    @Test
    void robotToField()
    {
        // Simple test
        Vec3 pos = new Vec3(320, 0, 0);
        double heading = 0;
        Affine3 rtof = this.getRToField(pos, heading);
        Vec3 fieldLoc = rtof.transformPoint(Vec3.ZeroPt); // robot origin
        assert (fieldLoc.equals(pos, kEpsilon));

        Vec3 fieldDir = rtof.transformVector(Vec3.XAxis);
        assert (fieldDir.equals(Vec3.XAxis));

        // More interesting test
        pos = new Vec3(320, -26, 0);
        heading = 20;
        rtof = this.getRToField(pos, heading);
        fieldLoc = rtof.transformPoint(Vec3.ZeroPt); // robot origin
        assert (fieldLoc.equals(pos, kEpsilon));
        fieldDir = rtof.transformVector(Vec3.XAxis);
        double rads = Math.acos(Vec3.XAxis.dot(fieldDir.asUnit()));
        assertEquals(rads, Math.PI * heading / 180., kEpsilon);
    }

    @Test
    void pipelineValidate()
    {
        /* validate camera flip */
        Vec3 tgX = new Vec3(0, -1, 0);
        Vec3 tgY = new Vec3(0, 0, 1);
        Vec3 tgZ = new Vec3(-1, 0, 0);
        Affine3 camFlips = Affine3.fromAxes(tgX, tgY, tgZ);
        Vec3 x = camFlips.transformVector(Vec3.XAxis);
        assert (x.equals(tgX));
        Vec3 y = camFlips.transformVector(Vec3.YAxis);
        assert (y.equals(tgY));
        Vec3 z = camFlips.transformVector(Vec3.ZAxis);
        assert (z.equals(tgZ));

        /* validate camera tilt plus tilt */
        double tiltAngle = 45; /* extreme, praps */
        Affine3 camTilt = Affine3.fromRotation(tiltAngle, Vec3.XAxis);
        Affine3 camRot = Affine3.concatenate(camFlips, camTilt);
        Vec3 camOff = new Vec3(0, -12, 0); // right of turret origin
        Affine3 camTrans = Affine3.fromTranslation(camOff);
        Affine3 camToMount = Affine3.concatenate(camTrans, camRot);
        Affine3 mountToCam = camToMount.asInverse();

        // verify that origin in camera space maps to mount position
        Vec3 o = camToMount.transformPoint(Vec3.ZeroPt);
        assert (o.equals(camOff, kEpsilon));

        // verify that mount point maps to camera origin
        o = mountToCam.transformPoint(camOff);
        assert (o.equals(Vec3.ZeroPt, kEpsilon));

        // verify that the camera tilt works as expected
        // a vector to a centered target, results in a vector
        // relative to the turret that is upward in z and with
        // 0 y component (ie: centered but up).
        o = camToMount.transformVector(new Vec3(0, 0, -10));
        assert (o.equals(new Vec3(7.071068, 0, 7.071068), 1e-6));

        // Here we test point transformations. Since the camera is offset
        // from turret origin, we expect points to be as well.
        Vec3 target = new Vec3(0, 0, -120); // center of camera 12 ft away
        Vec3 tgtPtMount = camToMount.transformPoint(target);
        assertEquals(camOff.getY(), tgtPtMount.getY(), kEpsilon); // y coords match

        // Check for non-zero angle between tgtPtMount and x axis
        double cosAngle = tgtPtMount.asUnit().dot(Vec3.XAxis);
        assert (cosAngle > .001);

        // Mount to robot
        // origin is back and up from robot's
        Vec3 mountPos = new Vec3(-15, 0, 8);
        double mountAngle = 0;
        Affine3 mountRot = Affine3.fromRotation(180 - mountAngle, Vec3.ZAxis);
        Affine3 mountOffset = Affine3.fromTranslation(mountPos);
        Affine3 mountToRobot = Affine3.concatenate(mountOffset, mountRot);

        // verify that turret's negative x is robot's x
        Vec3 mntNegX = mountToRobot.transformVector(new Vec3(-1, 0, 0));
        assert (mntNegX.equals(Vec3.XAxis, kEpsilon));
        // verify turret's mount offset
        assert (mountToRobot.transformPoint(Vec3.ZeroPt).equals(mountPos, kEpsilon));

        Affine3 camToRobot = Affine3.concatenate(mountToRobot, camToMount);
        Vec3 camOnRobot = camToRobot.transformPoint(Vec3.ZeroPt);
        assert (camOnRobot.equals(new Vec3(-15, 12, 8)));

        Affine3 robotToField = this.getRToField(new Vec3(320, 0, 0), 0);
        Affine3 camToField = Affine3.concatenate(robotToField, camToRobot);
        Vec3 camOnField = camToField.transformPoint(Vec3.ZeroPt);
        assert (camOnField.equals(new Vec3(305, 12, 8), kEpsilon));
    }

    private static class TargetInfo
    {
        public Pose2d fieldToTurret;
        public Rotation2d turretAngle;
        public double distance;
        public double degrees;
    }

    // mostly the same as LauncherCommands.trackTarget()
    // issue there is distance calculation
    private TargetInfo trackTarget(final Pose2d robot, final Pose2d target)
    {
        TargetInfo result = new TargetInfo();
        result.fieldToTurret = robot.transformBy(Constants.Launcher.kRobotToTurret);
        Pose2d turretToTarget = result.fieldToTurret.inFrameReferenceOf(target);
        Rotation2d fieldAnglePointingToTarget = new Rotation2d(
            turretToTarget.getTranslation().getX(),
            turretToTarget.getTranslation().getY(),
            true /*normalize*/);
        result.turretAngle = fieldAnglePointingToTarget.rotateBy(
            result.fieldToTurret.getRotation().inverse());
        double wrongDistance = target.distance(result.fieldToTurret);
        result.distance = Math.hypot(turretToTarget.getTranslation().getX(),
                                turretToTarget.getTranslation().getY());
        result.distance = Units.metersToInches(result.distance);
        result.degrees = result.turretAngle.getDegrees();
        return result;
    }

    @Test
    void turretTrackTest()
    {
        // cf: LauncherCommands.trackTarget()
        // Pose2d is always in meters (by convention)
        Pose2d matchTargetMeters = new Pose2d(
            Units.inchesToMeters(Constants.Vision.kAllianceGoalCoords[0]),
            Units.inchesToMeters(Constants.Vision.kAllianceGoalCoords[1]),
            Rotation2d.fromDegrees(180));

        // all our validations are in inches
        double turretDist = Math.hypot(
            Constants.Launcher.kRobotToTurret.getTranslation().getX(),
            Constants.Launcher.kRobotToTurret.getTranslation().getY());
        turretDist = Units.metersToInches(turretDist);

        /*
        // start with a trivial/verifiable case:
        // robot 5m distant, points away from target, in perfect horizontal 
        // alignment (in Red-Alliance coords).
        //                       y
        //                 ._____|____. 
        //              x--|-T   |    |             
        //  Goal           | |   o-----x
        //                 | y        |
        //                 |__________| 
        */
        double expectedDist = Units.metersToInches(5);
        double robotX = matchTargetMeters.getTranslation().getX() - 5;
        double robotY = matchTargetMeters.getTranslation().getY();
        Pose2d r0 = new Pose2d(robotX, robotY, 180);
        TargetInfo i0 = this.trackTarget(r0, matchTargetMeters);
        assert(Math.abs(i0.distance - expectedDist) < turretDist);
        assert(i0.degrees > 0 && i0.degrees < 2); // ie: a small positive turn

        /*
         *  Now rotate our robot 35, turret needs to turn by negative equiv
         */
        Pose2d r1 = new Pose2d(robotX, robotY, 180+35);
        TargetInfo i1 = this.trackTarget(r1, matchTargetMeters);
        assert(Math.abs(i1.distance - expectedDist) < turretDist);
        assert(i1.degrees < -33 && i1.degrees > -35); // ie: a large negative turn

        /*
         *  Now rotate our robot 35, turret needs to turn by negative equiv
         */
        Pose2d r2 = new Pose2d(robotX, robotY, 180-35);
        TargetInfo i2 = this.trackTarget(r2, matchTargetMeters);
        assert(Math.abs(i2.distance - expectedDist) < turretDist);
        assert(i2.degrees > 35 && i1.degrees < 37); // ie: a large negative turn
    }

    private static class AltTargetInfo
    {
        public double distance;
        public double yawAngle;
        public double pitchAngle;
        public double innerDistance;
        public double innerYawAngle;
        public double innerPitchAngle;
        public double fixYaw(double a)
        {
            if(a > 180)
                a = -(360 - yawAngle);
            return a;
        }
        public double fixDist(double d)
        {
            return d; // no fixup, since we're operating in inches
        }
    }

    private AltTargetInfo trackTargetAlt(CoordSysMgr2020 ctof, 
                                        final Pose2d robot, 
                                        final Vec3 targetInches,
                                        final Vec3 innerTargetInches)
    {
        AltTargetInfo result = new AltTargetInfo();
        // Pose2d is presumed always in meters (by convention)
        ctof.updateRobotPose(robot); 

        // but CoordSysMgr currently works in inches
        Vec3 targetInMnt = ctof.fieldPointToMount(targetInches);
        Vec3 innerTargetInMnt = ctof.fieldPointToMount(innerTargetInches);

        // return our results in inches and degrees
        result.distance = result.fixDist(targetInMnt.lengthXY());
        result.yawAngle = result.fixYaw(targetInMnt.angleOnXYPlane());
        result.pitchAngle = targetInMnt.angleOnXZPlane();
        result.innerDistance = result.fixDist(innerTargetInMnt.lengthXY());
        result.innerYawAngle = result.fixYaw(innerTargetInMnt.angleOnXYPlane());
        result.innerPitchAngle = innerTargetInMnt.angleOnXZPlane();
        return result;
    }
    @Test
    void turretTrackAltTest()
    {
        CoordSysMgr2020 ctof = new CoordSysMgr2020();
        Vec3 matchTargetInches = new Vec3(Constants.Vision.kAllianceGoalCoords);
        Vec3 innerTargetInches = new Vec3(Constants.Vision.kAllianceInnerGoalCoords);
        Pose2d matchTargetMeters = new Pose2d(
            Units.inchesToMeters(Constants.Vision.kAllianceGoalCoords[0]),
            Units.inchesToMeters(Constants.Vision.kAllianceGoalCoords[1]),
            Rotation2d.fromDegrees(180));
        double robotX = matchTargetMeters.getTranslation().getX() - 5;
        double robotY = matchTargetMeters.getTranslation().getY();
        double turretDist = Math.hypot(
            Constants.Launcher.kRobotToTurret.getTranslation().getX(),
            Constants.Launcher.kRobotToTurret.getTranslation().getY());
        turretDist = Units.metersToInches(turretDist);

        // robot 5m distant, points away from target, in horizontal alignment
        double expectedDist = Units.metersToInches(5);
        Pose2d r0 = new Pose2d(robotX, robotY, 180);
        AltTargetInfo i0 = this.trackTargetAlt(ctof, r0, matchTargetInches, 
                                            innerTargetInches);
        assert(Math.abs(i0.distance - expectedDist) < turretDist);
        assert(i0.yawAngle > 0 && i0.yawAngle < 2); // ie: a small positive turn
        // at 5m, the difference in pitch angle between inner and outer is 
        // significant around 2.5 degrees, but the yaw angle isn't
        assert(Math.abs(i0.innerYawAngle - i0.yawAngle) < .25); // 1/4 degree
        assertEquals((i0.pitchAngle - i0.innerPitchAngle), 2.5, .3); 

        /*
         *  Now rotate our robot 35, turret needs to turn by negative equiv
         */
        Pose2d r1 = new Pose2d(robotX, robotY, 180+35);
        AltTargetInfo i1 = this.trackTargetAlt(ctof, r1, matchTargetInches,
                                                innerTargetInches);
        assert(Math.abs(i1.distance - expectedDist) < turretDist);
        assert(i1.yawAngle < -33 && i1.yawAngle > -35); // ie: a large negative turn
        assert(Math.abs(i1.innerYawAngle - i1.yawAngle) < .25); // 1/4 degree
        assertEquals((i0.pitchAngle - i0.innerPitchAngle), 2.5, .3); 

        /*
         *  Now rotate our robot -35, turret needs to turn by positive equiv
         */
        Pose2d r2 = new Pose2d(robotX, robotY, 180-35);
        AltTargetInfo i2 = this.trackTargetAlt(ctof, r2, matchTargetInches,
                                                innerTargetInches);
        assert(Math.abs(i2.distance - expectedDist) < turretDist);
        assert(i2.yawAngle > 35 && i2.yawAngle < 37); // ie: a large negative turn
        assert(Math.abs(i2.innerYawAngle - i2.yawAngle) < .25); // 1/4 degree
        assertEquals((i0.pitchAngle - i0.innerPitchAngle), 2.5, .3); 

        /*
         * Explore targeting from up-close. The apex of the "target zone"
         * triangle is 26in away.
         */
        // robot-center 26in distant, points away from target, in horizontal alignment
        // - the pitch is 74 degreees, 58 degrees to inner
        // - yaw angles are substantially different (13 and 6 degrees)
        expectedDist = 26; 
        robotX = matchTargetMeters.getTranslation().getX() - Units.inchesToMeters(26);
        robotY = matchTargetMeters.getTranslation().getY();
        Pose2d r3 = new Pose2d(robotX, robotY, 180);
        AltTargetInfo i3 = this.trackTargetAlt(ctof, r3, matchTargetInches,
                                                innerTargetInches);
        assert(Math.abs(i3.distance - expectedDist) < turretDist);
        assertEquals(i3.innerDistance, 52, .5);
        assertEquals(i3.yawAngle, 13, .5);
        assertEquals(i3.innerYawAngle, 6, .5);
        assertEquals(i3.pitchAngle, 75, .5);
        assertEquals(i3.innerPitchAngle, 58, .5);
    }
}
