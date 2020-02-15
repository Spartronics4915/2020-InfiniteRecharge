package com.spartronics4915.frc2020;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rectangle2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.math.twodim.trajectory.TrajectoryGenerator;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.TimingConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;

import edu.wpi.first.wpilibj.util.Units;

public class TrajectoryContainer
{
    public TimedTrajectory<Pose2dWithCurvature> throughTrench(RobotStateEstimator stateEstimator)
    {
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        Pose2d[] intermediate = new Pose2d[]
        {
            new Pose2d(Units.inchesToMeters(424), Units.inchesToMeters(135),
                Rotation2d.fromDegrees(180)),
            new Pose2d(Units.inchesToMeters(207), Units.inchesToMeters(135),
                Rotation2d.fromDegrees(180))
        };
        for (int i = 0; i < intermediate.length; i++)
        {
            Pose2d pose = intermediate[i];
            intermediate[i] = new Pose2d(pose.getTranslation().getX() - Units.inchesToMeters(312.5),
                pose.getTranslation().getY(), pose.getRotation());
        }
        RobotStateMap stateMap = stateEstimator.getEncoderRobotStateMap();
        Pose2d robotPose = stateMap.getLatestState().pose;
        double robotX = robotPose.getTranslation().getX();
        if (robotX < Units.inchesToMeters(312.5))
        {
            for (int i = 0; i < intermediate.length; i++)
            {
                Pose2d pose = intermediate[i];
                intermediate[i] = new Pose2d(-pose.getTranslation().getX(),
                    pose.getTranslation().getY(), Rotation2d.fromDegrees(0));
            }
        }
        for (int i = 0; i < intermediate.length; i++)
        {
            Pose2d pose = intermediate[i];
            intermediate[i] = new Pose2d(pose.getTranslation().getX() + Units.inchesToMeters(312.5),
                pose.getTranslation().getY(), pose.getRotation());
        }
        waypoints.add(robotPose);
        for (Pose2d pose : intermediate)
        {
            waypoints.add(pose);
        }
        ArrayList<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<TimingConstraint<Pose2dWithCurvature>>();
        return TrajectoryContainer.generateTrajectory(waypoints, constraints);
    }

    public TimedTrajectory<Pose2dWithCurvature> toControlPanel(RobotStateEstimator stateEstimator)
    {
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        Pose2d pose = stateEstimator.getEncoderRobotStateMap().getLatestState().pose;
        waypoints.add(pose);
        Translation2d p = pose.getTranslation();
        Pose2d nextToControlPanel;
        if (p.getX() < Units.inchesToMeters(359))
        {
            nextToControlPanel = new Pose2d(Units.inchesToMeters(328), Units.inchesToMeters(135),
                Rotation2d.fromDegrees(0));
        }
        else
        {
            nextToControlPanel = new Pose2d(Units.inchesToMeters(390), Units.inchesToMeters(135),
                Rotation2d.fromDegrees(180));
        }
        waypoints.add(nextToControlPanel);
        ArrayList<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<TimingConstraint<Pose2dWithCurvature>>();
        constraints.add(new VelocityLimitRegionConstraint(new Rectangle2d(
            new Translation2d(Units.inchesToMeters(290), Units.inchesToMeters(161.6)),
            new Translation2d(Units.inchesToMeters(428), Units.inchesToMeters(90))), .5));
        return TrajectoryContainer.generateTrajectory(waypoints, constraints);
    }

    public static enum Destination
    {
        LeftTrenchFar(385, 134, 0), LeftShootingPosition(508, 5, 328.69), RightTrenchFar(394, -134, 180),
        RightTrenchNear(242, -134, 180), RightShootingPosition(421, -121, 14.36),
        ShieldGeneratorFarRight(400, -40, 135), MiddleShootingPosition(456, -67, 0);

        public final Pose2d pose;

        private Destination(double x, double y, double heading)
        {
            this.pose = new Pose2d(Units.inchesToMeters(x), Units.inchesToMeters(y),
                    Rotation2d.fromDegrees(heading));
        }

        private Destination(Pose2d pose)
        {
            Translation2d trans = pose.getTranslation();
            this.pose = new Pose2d(Units.inchesToMeters(trans.getX()),
                    Units.inchesToMeters(trans.getY()), pose.getRotation());
        }
    }

    public static final class DestinationCouple
    {
        private Destination mStart;
        private Destination mEnd;

        public DestinationCouple(Destination a, Destination b)
        {
            mStart = a;
            mEnd = b;
        }

        public TimedTrajectory<Pose2dWithCurvature> createTrajectory(Pose2d startPoint,
                List<Pose2d> midpoints)
        {
            Pose2d start;
            Pose2d end = mEnd.pose;
            if (mStart == null)
                start = startPoint;
            else
                start = mStart.pose;
            ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
            waypoints.add(start);
            for (Pose2d pose : midpoints)
            {
                waypoints.add(pose);
            }
            waypoints.add(end);
            List<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<TimingConstraint<Pose2dWithCurvature>>();
            return TrajectoryContainer.generateTrajectory(waypoints, constraints);
        }

        @Override
        public boolean equals(Object obj)
        {
            if (obj == this)
                return true;
            else if (obj == null)
                return false;
            else if (!(obj instanceof DestinationCouple))
                return false;

            var dest = ((DestinationCouple) obj);
            return dest.mEnd == this.mEnd && dest.mStart == this.mStart;
        }

        @Override
        public int hashCode()
        {
            return Objects.hash(mStart, mEnd);
        }
    }

    public static final class TrajectoryCollection
    {
        public final Pose2d mStartPoint;
        private Map<DestinationCouple, TimedTrajectory<Pose2dWithCurvature>> mTrajectories;

        public TrajectoryCollection(Pose2d startPoint)
        {
            mStartPoint = startPoint;
            mTrajectories = new HashMap<DestinationCouple, TimedTrajectory<Pose2dWithCurvature>>();
        }

        public void generateTrajectories(Map<DestinationCouple, List<Pose2d>> trajectories)
        {
            for (var entry : trajectories.entrySet())
            {
                var trajectory = entry.getKey().createTrajectory(mStartPoint, entry.getValue());

                mTrajectories.put(entry.getKey(), trajectory);
            }
        }

        public TimedTrajectory<Pose2dWithCurvature> getTrajectory(Destination start,
                Destination end)
        {
            return mTrajectories.get(new DestinationCouple(start, end));
        }
    }

    public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints,
            List<TimingConstraint<Pose2dWithCurvature>> constraints)
    {
        var trajectory = TrajectoryGenerator.defaultTrajectoryGenerator.generateTrajectory(
                waypoints, constraints, Constants.Trajectory.kStartVelocityMetersPerSec,
                Constants.Trajectory.kEndVelocityMetersPerSec,
                Constants.Trajectory.kMaxVelocityMetersPerSec,
                Constants.Trajectory.kMaxAccelerationMeterPerSecSq, false);
        return trajectory;
    }

    public static final TrajectoryCollection left = new TrajectoryCollection(
            Constants.Trajectory.kStartPointLeft);
    public static final TrajectoryCollection middle = new TrajectoryCollection(
            Constants.Trajectory.kStartPointMiddle);
    public static final TrajectoryCollection right = new TrajectoryCollection(
            Constants.Trajectory.kStartPointRight);
    static
    {
        // left
        var leftTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        leftTrajectories.put(new DestinationCouple(null, Destination.LeftTrenchFar),
                Arrays.asList());
        leftTrajectories.put(
                new DestinationCouple(Destination.LeftTrenchFar, Destination.LeftShootingPosition),
                Arrays.asList());

        left.generateTrajectories(leftTrajectories);

        // middle
        var middleTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        middleTrajectories.put(new DestinationCouple(null, Destination.ShieldGeneratorFarRight),
                Arrays.asList());
        middleTrajectories.put(new DestinationCouple(Destination.ShieldGeneratorFarRight,
                Destination.MiddleShootingPosition), Arrays.asList());

        middle.generateTrajectories(middleTrajectories);
        // right
        var rightTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        rightTrajectories.put(new DestinationCouple(null, Destination.RightTrenchFar),
                Arrays.asList());
        rightTrajectories.put(
                new DestinationCouple(Destination.RightTrenchFar, Destination.RightTrenchNear),
                Arrays.asList());
        rightTrajectories.put(new DestinationCouple(Destination.RightTrenchNear,
                Destination.RightShootingPosition), Arrays.asList());

        right.generateTrajectories(rightTrajectories);
    }
}
