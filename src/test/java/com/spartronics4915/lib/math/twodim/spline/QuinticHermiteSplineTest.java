/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.spartronics4915.lib.math.twodim.spline;

import java.util.List;

import org.junit.jupiter.api.Test;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.spline.SplineParameterizer.MalformedSplineException;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class QuinticHermiteSplineTest
{
    private static final double kMaxDx = 0.127;
    private static final double kMaxDy = 0.00127;
    private static final double kMaxDtheta = 0.0872;

    @SuppressWarnings({"ParameterName", "PMD.UnusedLocalVariable"})
    private void run(Pose2d a, Pose2d b)
    {
        // Start the timer.
        var start = System.nanoTime();

        // Generate and parameterize the spline.
        var controlVectors = SplineHelper.getQuinticControlVectorsFromWaypoints(List.of(a, b));
        var spline = SplineHelper
            .getQuinticSplinesFromControlVectors(controlVectors.toArray(new Spline.ControlVector[0]))[0];
        var poses = SplineParameterizer.parameterize(spline);

        // End the timer.
        var end = System.nanoTime();

        // Calculate the duration (used when benchmarking)
        var durationMicroseconds = (end - start) / 1000.0;

        for (int i = 0; i < poses.size() - 1; i++)
        {
            var p0 = poses.get(i);
            var p1 = poses.get(i + 1);

            // Make sure the twist is under the tolerance defined by the Spline class.
            var twist = p1.getPose().inFrameReferenceOf(p0.getPose()).log();
            assertAll(() -> assertTrue(Math.abs(twist.dx) < kMaxDx),
                () -> assertTrue(Math.abs(twist.dy) < kMaxDy),
                () -> assertTrue(Math.abs(twist.dtheta.getRadians()) < kMaxDtheta));
        }

        // Check first point
        assertAll(
            () -> assertEquals(a.getTranslation().getX(), poses.get(0).getPose().getTranslation().getX(),
                1E-9),
            () -> assertEquals(a.getTranslation().getY(), poses.get(0).getPose().getTranslation().getY(),
                1E-9),
            () -> assertEquals(a.getRotation().getRadians(),
                poses.get(0).getPose().getRotation().getRadians(), 1E-9));

        // Check last point
        assertAll(
            () -> assertEquals(b.getTranslation().getX(),
                poses.get(poses.size() - 1).getPose().getTranslation().getX(), 1E-9),
            () -> assertEquals(b.getTranslation().getY(),
                poses.get(poses.size() - 1).getPose().getTranslation().getY(), 1E-9),
            () -> assertEquals(b.getRotation().getRadians(),
                poses.get(poses.size() - 1).getPose().getRotation().getRadians(), 1E-9));
    }

    @SuppressWarnings("PMD.JUnitTestsShouldIncludeAssert")
    @Test
    void testStraightLine()
    {
        run(new Pose2d(), new Pose2d(3, 0, new Rotation2d()));
    }

    @SuppressWarnings("PMD.JUnitTestsShouldIncludeAssert")
    @Test
    void testSimpleSCurve()
    {
        run(new Pose2d(), new Pose2d(1, 1, new Rotation2d()));
    }

    @SuppressWarnings("PMD.JUnitTestsShouldIncludeAssert")
    @Test
    void testSquiggly()
    {
        run(new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
            new Pose2d(-1, 0, Rotation2d.fromDegrees(90)));
    }

    @Test
    void testMalformed()
    {
        assertThrows(MalformedSplineException.class,
            () -> run(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(1, 0, Rotation2d.fromDegrees(180))));
        assertThrows(MalformedSplineException.class,
            () -> run(new Pose2d(10, 10, Rotation2d.fromDegrees(90)),
                new Pose2d(10, 11, Rotation2d.fromDegrees(-90))));
    }
}
