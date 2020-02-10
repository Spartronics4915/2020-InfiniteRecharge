package com.spartronics4915.frc2020;

import com.spartronics4915.lib.math.threedim.*;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;


class CamToFieldTests
{
    double kEpsilon = 1e-9;

    @Test
    public void usageExample()
    {
        double timestamp = 0.01;
        CamToField2020 ctof = new CamToField2020();
        ctof.updateTurretAngle(0);

        // robot at center of field, pointing right
        ctof.updateRobotPose(320, 0, 0, timestamp); 
        timestamp += .01;

        Vec3 camOnField = ctof.getPointOnField(new Vec3(0, 0, 0));
        Vec3 camDirOnField = ctof.getDirOnField(new Vec3(0, 0, -1));

        // let's see where a point 100 inches away from camera is
        // should be "behind the robot" on at y == 0
        Vec3 p1 = ctof.getPointOnField(new Vec3(0, 0, -100));
        p1.print();
        assertEquals(p1.a2, 0, kEpsilon);
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

        Affine3 camToRobot = Affine3.concatenate(camToMount, mountToRobot);
        Vec3 camOnRobot = camToRobot.transformPoint(Vec3.ZeroPt);
        System.out.println("cam relative to robot  ");
        camOnRobot.print();

        Affine3 robotToField = this.getRToField(new Vec3(320, 0, 0), 0);
        Affine3 camToField = Affine3.concatenate(camToRobot, robotToField);
        Vec3 camOnField = camToField.transformPoint(Vec3.ZeroPt);
        System.out.println("cam relative to field  ");
        camOnField.print();

    }

}