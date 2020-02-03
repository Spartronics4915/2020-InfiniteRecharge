package com.spartronics4915.frc2020;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.SortedMap;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.math.twodim.trajectory.TrajectoryGenerator;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.TimingConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;

import edu.wpi.first.wpilibj.util.Units;

public class TrajectoryContainer
{
    public static enum Destination
    {
        LeftTrenchFar(385, 134, 0), LeftShootingPosition(508, 5, 0), RightTrenchFar(394, -134, 0),
        RightTrenchNear(242, -134, 0), RightShootingPosition(421, -121, 0),
        ShieldGeneratorFarRight(400, -40, 0), MiddleShootingPosition(456, -67, 0);

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
        public int hashCode()
        {
            return Objects.hash(mStart, mEnd);
        }
    }

    public static final class TrajectoryCollection
    {
        private Pose2d mStartPoint;
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
                System.out.println(entry.getKey().hashCode());
                var trajectory = entry.getKey().createTrajectory(mStartPoint, entry.getValue());
                mTrajectories.put(entry.getKey(), trajectory);
            }
        }

        public TimedTrajectory<Pose2dWithCurvature> getTrajectory(Destination start,
                Destination end)
        {
            return mTrajectories.get(new DestinationCouple(start, end));
        }

        public TimedTrajectory<Pose2dWithCurvature> getTrajectory(Destination end)
        {
            return mTrajectories.get(new DestinationCouple(null, end));
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
