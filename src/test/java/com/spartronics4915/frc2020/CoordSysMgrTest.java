package com.spartronics4915.frc2020;

import com.spartronics4915.lib.math.threedim.*;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;


class CoordSysMgrTest
{
    double kEpsilon = 1e-9;

    @Test
    public void usageExample()
    {
        CoordSysMgr2020 ctof = new CoordSysMgr2020();
        ctof.updateTurretAngle(0);

        // robot at center of field, pointing right
        Vec3 robotPt = new Vec3(320, 0, 0);
        double robotHeading = 0;
        ctof.updateRobotPose(robotPt.a1, robotPt.a2, robotHeading);

        // This should give us the location of the camera relative to the robot
        // when turret angle is 0.
        Vec3 camOnRobot = ctof.camPointToRobot(Vec3.ZeroPt);
        double dist = camOnRobot.length(), distXY;
        assert(dist > 5 && dist < 20);
        // for a robot and turret angle of 0, predict camera position on field.
        Vec3 camPtOnField = ctof.camPointToField(new Vec3(0, 0, 0));
        Vec3 camDirOnField = ctof.camDirToField(new Vec3(0, 0, -1));
        assertEquals(camDirOnField.a1, -1, .2); // camera tilts into z-up
        assertEquals(camDirOnField.a2, 0, kEpsilon); // camera tilts into z-up
        distXY = camPtOnField.subtract(robotPt).lengthXY();
        assert(distXY < 18);

        // Let's see where a point 100 inches away from camera is
        // should be "behind the robot" offset by combination of
        // turret and camera offsets.
        Vec3 p0 = ctof.camPointToRobot(new Vec3(0, 0, -100));
        assert(p0.a1 < -100); // mostly behind the robot (x is front)
        distXY = p0.lengthXY();
        assert(distXY > 100); // farther from robot than camera (even xy only)

        // Now point turret directly toward turret right (robot left)
        ctof.updateTurretAngle(-90); // turret x is fwd, +y is left
        p0 = ctof.camPointToRobot(new Vec3(0, 0, -100));
        assert(p0.a2 > 100); // mostly to the left of the robot (+y is left)
        distXY = p0.lengthXY();
        assert(distXY > 100); // since robot origin is farther from target

        // point turret directly left
        ctof.updateTurretAngle(90);
        p0 = ctof.camPointToRobot(new Vec3(0, 0, -100));
        assert(p0.a2 < -90); // mostly to the left of the robot (+y is left)
        double distXY2 = p0.length();
        // robot origin closer to points turret sees to its right.
        assert(distXY2 < distXY); 

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
        assertEquals(tMinusR.a1, 100, kEpsilon);
        assertEquals(tMinusR.a2, -100, kEpsilon);
        assertEquals(tMinusR.a3, 96, kEpsilon);

        // same as above, but now the target is exactly behind the robot so
        // the y coord of tMinusR is 0
        robotHeading = 135;
        ctof.updateRobotPose(robotHeading, robotRelativeTarget, knownFieldLocationTarget);
        robotLocation = ctof.robotPointToField(Vec3.ZeroPt);
        tMinusR = knownFieldLocationTarget.subtract(robotLocation);
        assertEquals(robotRelativeTarget.length(), tMinusR.length(), kEpsilon);
        assertEquals(tMinusR.a2, 0, kEpsilon);


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
        //         x
        //         |
        //     y---o
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
    public void robotToField()
    {
        // Simple test
        Vec3 pos = new Vec3(320, 0, 0);
        double heading = 0;
        Affine3 rtof = this.getRToField(pos, heading);
        Vec3 fieldLoc = rtof.transformPoint(Vec3.ZeroPt); // robot origin
        assert(fieldLoc.equals(pos, kEpsilon));

        Vec3 fieldDir = rtof.transformVector(Vec3.XAxis);
        assert(fieldDir.equals(Vec3.XAxis));

        // More interesting test
        pos = new Vec3(320, -26, 0);
        heading = 20;
        rtof = this.getRToField(pos, heading);
        fieldLoc = rtof.transformPoint(Vec3.ZeroPt); // robot origin
        assert(fieldLoc.equals(pos, kEpsilon));
        fieldDir = rtof.transformVector(Vec3.XAxis);
        double rads = Math.acos(Vec3.XAxis.dot(fieldDir.asUnit()));
        assertEquals(rads, Math.PI * heading/180.,  kEpsilon);
    }

    @Test
    public void pipelineValidate()
    {
        /* validate camera flip */
        Vec3 tgX = new Vec3(0, -1, 0);
        Vec3 tgY = new Vec3(0, 0, 1);
        Vec3 tgZ = new Vec3(-1, 0, 0);
        Affine3 camFlips = Affine3.fromAxes(tgX, tgY, tgZ);
        Vec3 x = camFlips.transformVector(Vec3.XAxis);
        assert(x.equals(tgX));
        Vec3 y = camFlips.transformVector(Vec3.YAxis);
        assert(y.equals(tgY));
        Vec3 z = camFlips.transformVector(Vec3.ZAxis);
        assert(z.equals(tgZ));

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
        assert(o.equals(camOff, kEpsilon));

        // verify that mount point maps to camera origin
        o = mountToCam.transformPoint(camOff);
        assert(o.equals(Vec3.ZeroPt, kEpsilon));

        // verify that the camera tilt works as expected
        // a vector to a centered target, results in a vector
        // relative to the turret that is upward in z and with 
        // 0 y component (ie: centered but up).
        o = camToMount.transformVector(new Vec3(0,0,-10));
        assert(o.equals(new Vec3(7.071068, 0, 7.071068), 1e-6));

        // Here we test point transformations.  Since the camera is offset
        // from turret origin, we expect points to be as well.
        Vec3 target = new Vec3(0, 0, -120); // center of camera 12 ft away
        Vec3 tgtPtMount = camToMount.transformPoint(target);
        assertEquals(camOff.a2, tgtPtMount.a2, kEpsilon); // y coords match

        // Check for non-zero angle between tgtPtMount and x axis
        double cosAngle = tgtPtMount.asUnit().dot(Vec3.XAxis);
        assert(cosAngle > .001);

        // Mount to robot
        //   origin is back and up from robot's
        Vec3 mountPos = new Vec3(-15, 0, 8); 
        double mountAngle = 0;
        Affine3 mountRot = Affine3.fromRotation(180-mountAngle, Vec3.ZAxis);
        Affine3 mountOffset = Affine3.fromTranslation(mountPos);
        Affine3 mountToRobot = Affine3.concatenate(mountOffset, mountRot);

        // verify that turret's negative x is robot's x
        Vec3 mntNegX = mountToRobot.transformVector(new Vec3(-1, 0, 0));
        assert(mntNegX.equals(Vec3.XAxis, kEpsilon));
        // verify turret's mount offset
        assert(mountToRobot.transformPoint(Vec3.ZeroPt).equals(mountPos, kEpsilon));

        Affine3 camToRobot = Affine3.concatenate(mountToRobot, camToMount);
        Vec3 camOnRobot = camToRobot.transformPoint(Vec3.ZeroPt);
        assert(camOnRobot.equals(new Vec3(-15, 12, 8)));

        Affine3 robotToField = this.getRToField(new Vec3(320, 0, 0), 0);
        Affine3 camToField = Affine3.concatenate(robotToField, camToRobot);
        Vec3 camOnField = camToField.transformPoint(Vec3.ZeroPt);
        assert(camOnField.equals(new Vec3(305, 12, 8), kEpsilon));

    }

}