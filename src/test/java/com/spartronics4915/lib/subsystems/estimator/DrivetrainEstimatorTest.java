package com.spartronics4915.lib.subsystems.estimator;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.math.twodim.trajectory.TrajectoryGenerator;
import com.spartronics4915.lib.util.Kinematics;

import org.junit.jupiter.api.Test;
import org.knowm.xchart.SwingWrapper;
import org.knowm.xchart.XYChartBuilder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.math.StateSpaceUtils;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;

public class DrivetrainEstimatorTest
{
    @Test
    public void testEstimator()
    {
        var stateStdDevs = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01);
        var measurementStdDevs = new MatBuilder<>(Nat.N6(), Nat.N1()).fill(0.1, 0.1, 0.1, 0.005,
            0.005, 0.002);
        var est = new DrivetrainEstimator(stateStdDevs, measurementStdDevs, 3, new Pose2d());

        final double dt = 0.01;
        final double visionUpdateRate = 0.2;

        var traj = TrajectoryGenerator.defaultTrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(),
                new Pose2d(3, 3, new Rotation2d())
                // new Pose2d(), new Pose2d(20, 20, Rotation2d.fromDegrees(0)),
                // new Pose2d(23, 23, Rotation2d.fromDegrees(173)),
                // new Pose2d(54, 54, new Rotation2d())
            ),
            List.of(),
            0, 0,
            1, 1,
            false);

        var kinematics = new DifferentialDriveKinematics(1);
        Pose2d lastPose = null;

        List<Double> trajXs = new ArrayList<>();
        List<Double> trajYs = new ArrayList<>();
        List<Double> observerXs = new ArrayList<>();
        List<Double> observerYs = new ArrayList<>();
        List<Double> slamXs = new ArrayList<>();
        List<Double> slamYs = new ArrayList<>();
        List<Double> visionXs = new ArrayList<>();
        List<Double> visionYs = new ArrayList<>();

        var rand = new Random();
        final double steadyStateErrorX = 1.0;
        final double steadyStateErrorY = 1.0;

        double t = 0.0;
        Pose2d lastVisionUpdate = null;
        double lastVisionUpdateT = Double.NEGATIVE_INFINITY;
        double maxError = Double.NEGATIVE_INFINITY;
        double errorSum = 0;
        while (t <= traj.getTotalTime())
        {
            t += dt;

            var groundtruthState = traj.sample(t);
            var input = kinematics
                .toWheelSpeeds(new ChassisSpeeds(groundtruthState.state.velocity, 0.0,
                    // ds/dt * dtheta/ds = dtheta/dt
                    groundtruthState.state.velocity * groundtruthState.state.state.getCurvature()));

            Matrix<N3, N1> u = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
                input.leftMetersPerSecond * dt,
                input.rightMetersPerSecond * dt, 0.0);
            if (lastPose != null)
            {
                u.set(2, 0,
                    groundtruthState.state.state.getRotation().getRadians()
                        - lastPose.getRotation().getRadians());
            }
            u = u.plus(StateSpaceUtils.makeWhiteNoiseVector(Nat.N3(),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.002, 0.002, 0.001)));
            lastPose = groundtruthState.state.state.getPose();

            Pose2d realPose = groundtruthState.state.state.getPose();

            if (lastVisionUpdateT + visionUpdateRate < t)
            {
                if (lastVisionUpdate != null)
                {
                    est.addVisionMeasurement(lastVisionUpdate, lastVisionUpdateT);
                }

                lastVisionUpdateT = t;
                lastVisionUpdate = realPose.getPose()
                    .transformBy(new Pose2d(rand.nextGaussian() * 0.05, rand.nextGaussian() * 0.05,
                        Rotation2d.fromRadians(rand.nextGaussian() * 0.002)));

                visionXs.add(lastVisionUpdate.getTranslation().getX());
                visionYs.add(lastVisionUpdate.getTranslation().getY());
            }

            double dist = realPose.getTranslation().getDistance(new Translation2d());
            Pose2d measurementVSlam = realPose.getPose()
                .transformBy(new Pose2d(new Translation2d(steadyStateErrorX * (dist / 76.0),
                    steadyStateErrorY * (dist / 76.0)), new Rotation2d()))
                .transformBy(new Pose2d(rand.nextGaussian() * 0.05, rand.nextGaussian() * 0.05,
                    Rotation2d.fromRadians(rand.nextGaussian() * 0.001)));
            var xHat = est.update(measurementVSlam, u.get(0, 0), u.get(1, 0), u.get(2, 0), t);

            double error = groundtruthState.state.state.getTranslation()
                .getDistance(xHat.getTranslation());
            if (error > maxError)
            {
                maxError = error;
            }
            errorSum += error;

            trajXs.add(groundtruthState.state.state.getTranslation().getX());
            trajYs.add(groundtruthState.state.state.getTranslation().getY());
            observerXs.add(xHat.getTranslation().getX());
            observerYs.add(xHat.getTranslation().getY());
            slamXs.add(measurementVSlam.getTranslation().getX());
            slamYs.add(measurementVSlam.getTranslation().getY());
        }

        var chartBuilder = new XYChartBuilder();
        chartBuilder.title = "The Magic of Sensor Fusion";
        var chart = chartBuilder.build();

        chart.addSeries("vSLAM", slamXs, slamYs);
        chart.addSeries("Vision", visionXs, visionYs);
        chart.addSeries("Trajectory", trajXs, trajYs);
        chart.addSeries("xHat", observerXs, observerYs);

        System.out.println("Mean error (meters): " + errorSum / (traj.getTotalTime() / dt));
        System.out.println("Max error (meters):  " + maxError);

        // Uncomment for fun graphs
        // new SwingWrapper<>(chart).displayChart();
        // try
        // {
        //     Thread.sleep(1000000000);
        // }
        // catch (InterruptedException e)
        // {
        // }
    }
}
