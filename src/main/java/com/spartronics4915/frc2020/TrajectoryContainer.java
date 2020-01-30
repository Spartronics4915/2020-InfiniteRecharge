package com.spartronics4915.frc2020;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.trajectory.TrajectoryGenerator;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.TimingConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;

public class TrajectoryContainer {
    public static enum Destination {
        LeftTrenchBack(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        leftTrenchFront(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        rightTrenchBack(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        rightTrenchFront(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        inFrontOfLeftPowerPort(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        inFrontOfRightPowerPort(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        frontOfShieldGenerator(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        backOfShieldGenerator(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        leftOfShieldGenerator(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        rightOfShieldGenerator(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        public final Pose2d pose;

        private Destination(Pose2d pose) {
            this.pose = pose;
        }
    }
    public static final class TrajectoryCollection {
        private Pose2d mStartPoint;
        private Map<Destination, TimedTrajectory<Pose2dWithCurvature>> mTrajectories;
        public TrajectoryCollection(Pose2d startPoint) {
            mStartPoint = startPoint;
        }
        public void generateTrajectories(Map<Destination, List<Pose2d>> destinations) {

            for (var entry : destinations.entrySet()) {
                List<Pose2d> waypoints = new ArrayList<Pose2d>();
                for (Pose2d pose : entry.getValue()) {
                    waypoints.add(pose);
                }
                waypoints.add(entry.getKey().pose);
                waypoints.add(mStartPoint);
                List<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<TimingConstraint<Pose2dWithCurvature>>();
                var trajectory = TrajectoryGenerator.defaultTrajectoryGenerator.generateTrajectory(waypoints, constraints, Constants.Trajectory.kStartVelocityMetersPerSec, Constants.Trajectory.kEndVelocityMetersPerSec, Constants.Trajectory.kMaxVelocityMetersPerSec, Constants.Trajectory.kMaxAccelerationMeterPerSecSq, false);
                mTrajectories.put(entry.getKey(), trajectory);
            }
        }
        public TimedTrajectory<Pose2dWithCurvature> getTrajectory(Destination name) {
            return mTrajectories.get(name);
        } 
    }
    public static final TrajectoryCollection left = new TrajectoryCollection(Constants.Trajectory.kStartPointLeft);
    public static final TrajectoryCollection middle = new TrajectoryCollection(Constants.Trajectory.kStartPointMiddle);
    public static final TrajectoryCollection right = new TrajectoryCollection(Constants.Trajectory.kStartPointRight);
    static {
        Map<Destination, List<Pose2d>> destinations = new HashMap<Destination, List<Pose2d>>();
        destinations.put(Destination.backOfShieldGenerator, Arrays.asList(new Pose2d(50, 50, Rotation2d.fromDegrees(0))));
        left.generateTrajectories(destinations);
        middle.generateTrajectories(destinations);
        right.generateTrajectories(destinations);
    }
}