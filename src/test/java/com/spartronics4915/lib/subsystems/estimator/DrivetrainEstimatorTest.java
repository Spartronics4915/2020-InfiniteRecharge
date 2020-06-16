package com.spartronics4915.lib.subsystems.estimator;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.trajectory.TrajectoryGenerator;
import com.spartronics4915.lib.util.VecBuilder;

import org.junit.jupiter.api.Test;
import org.knowm.xchart.SwingWrapper;
import org.knowm.xchart.XYChartBuilder;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class DrivetrainEstimatorTest
{
    @Test
    public void testEstimator()
    {
        final double dt = 0.02;
        final double visionUpdateRate = 0.2;

        var stateStdDevs = VecBuilder.fill(0.02, 0.02, 0.01, 0.1, 0.1);
        var localMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 0.5, 0.5, 0.5, 0.05);
        var globalMeasurementStdDevs = VecBuilder.fill(5, 5, 1);
        var est = new DrivetrainEstimator(
                new Rotation2d(), new Pose2d(),
                stateStdDevs, localMeasurementStdDevs, globalMeasurementStdDevs,
                dt
        );

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

        List<Double> trajXs = new ArrayList<>();
        List<Double> trajYs = new ArrayList<>();
        List<Double> observerXs = new ArrayList<>();
        List<Double> observerYs = new ArrayList<>();
        List<Double> slamXs = new ArrayList<>();
        List<Double> slamYs = new ArrayList<>();
        List<Double> visionXs = new ArrayList<>();
        List<Double> visionYs = new ArrayList<>();

        var rand = new Random(4915);

        double distanceLeft = 0.0;
        double distanceRight = 0.0;

        double t = 0.0;
        Pose2d lastVisionUpdate = null;
        double lastVisionUpdateTime = Double.NEGATIVE_INFINITY;

        double maxError = Double.NEGATIVE_INFINITY;
        double errorSum = 0;
        while (t <= traj.getTotalTime())
        {
            var groundtruthState = traj.sample(t);
            Pose2d realPose = groundtruthState.state.state.getPose();

            if (lastVisionUpdateTime + visionUpdateRate < t)
            {
                if (lastVisionUpdate != null)
                {
                    est.addVisionMeasurement(lastVisionUpdate, lastVisionUpdateTime);
                }

                lastVisionUpdateTime = t;
                lastVisionUpdate = new Pose2d(
                    realPose.getTranslation().getX() + rand.nextGaussian() * 0.5,
                    realPose.getTranslation().getY() + rand.nextGaussian() * 0.5,
                    realPose.getRotation().getRadians() + rand.nextGaussian() * 0.01
                );

                visionXs.add(lastVisionUpdate.getTranslation().getX());
                visionYs.add(lastVisionUpdate.getTranslation().getY());
            }

            Pose2d measurementVSlam = new Pose2d(
                realPose.getTranslation().getX() + rand.nextGaussian() * 0.05,
                realPose.getTranslation().getY() + rand.nextGaussian() * 0.05,
                realPose.getRotation().getRadians() + rand.nextGaussian() * 0.5
            );

            var input = kinematics
                    .toWheelSpeeds(new ChassisSpeeds(groundtruthState.state.velocity, 0.0,
                            // ds/dt * dtheta/ds = dtheta/dt
                            groundtruthState.state.velocity * groundtruthState.state.state.getCurvature()));

            input.leftMetersPerSecond += rand.nextGaussian() * 0.02;
            input.rightMetersPerSecond += rand.nextGaussian() * 0.02;

            distanceLeft += input.leftMetersPerSecond * dt;
            distanceRight += input.rightMetersPerSecond * dt;

            var xHat = est.updateWithTime(
                    t,
                    realPose.getRotation(),//.rotateBy(Rotation2d.fromRadians(rand.nextGaussian() * 0.05)),
                    input,
                    measurementVSlam,
                    distanceLeft,
                    distanceRight
            );

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

            t += dt;
        }

        System.out.println("Mean error (meters): " + errorSum / (traj.getTotalTime() / dt));
        System.out.println("Max error (meters):  " + maxError);

        try
        {
            var chartBuilder = new XYChartBuilder();
            chartBuilder.title = "The Magic of Sensor Fusion";
            var chart = chartBuilder.build();

            chart.addSeries("vSLAM", slamXs, slamYs);
            chart.addSeries("Vision", visionXs, visionYs);
            chart.addSeries("Trajectory", trajXs, trajYs);
            chart.addSeries("xHat", observerXs, observerYs);
            // Uncomment for fun graphs
            new SwingWrapper<>(chart).displayChart();
            try
            {
                Thread.sleep(1000000000);
            }
            catch (InterruptedException e)
            {
            }
        }
        catch(java.awt.HeadlessException ex)
        {
            System.out.println("skipping charts in headless mode");
        }

    }
}
