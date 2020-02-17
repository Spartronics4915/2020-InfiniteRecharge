package com.spartronics4915.frc2020;

import java.security.KeyStore.Entry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.SortedMap;

import com.spartronics4915.frc2020.Constants.Trajectory;
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
        kLeftTrenchFar(394, 134, 120), 
        kLeftShootingPosition(508, 5, 148.69),
        kRightTrenchFar(304, -134, 180), 
        kRightTrenchVeryFar(404, -134, 180),
        kRightTrenchNear(242, -134, 180), 
        kRightShootingPosition(421, -121, 194.36),
        kShieldGeneratorFarRight(390, -38, 115),
        kShieldGeneratorNearRight(340, -60, 60),
        kMiddleShootingPosition(456, -65, 180),
        kJustAhead(120, 0, 0),
        kJustBehind(-120, 0, 0);

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
        private boolean mReversed;

        public DestinationCouple(Destination a, Destination b, boolean reversed)
        {
            mStart = a;
            mEnd = b;
            mReversed = reversed;
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
            return TrajectoryContainer.generateTrajectory(waypoints, constraints, mReversed);
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
            return mTrajectories.get(new DestinationCouple(start, end, false));
        }
    }

    public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints,
            List<TimingConstraint<Pose2dWithCurvature>> constraints, boolean reversed)
    {
        var trajectory = TrajectoryGenerator.defaultTrajectoryGenerator.generateTrajectory(
                waypoints, constraints, Constants.Trajectory.kStartVelocityMetersPerSec,
                Constants.Trajectory.kEndVelocityMetersPerSec,
                Constants.Trajectory.kMaxVelocityMetersPerSec,
                Constants.Trajectory.kMaxAccelerationMeterPerSecSq, reversed);
        return trajectory;
    }

    public static final TrajectoryCollection left = new TrajectoryCollection(
            Constants.Trajectory.kStartPointLeft);
    public static final TrajectoryCollection middle = new TrajectoryCollection(
            Constants.Trajectory.kStartPointMiddle);
    public static final TrajectoryCollection right = new TrajectoryCollection(
            Constants.Trajectory.kStartPointRight);
        public static final TrajectoryCollection eightBall = new TrajectoryCollection(
                Constants.Trajectory.kStartPointMiddle);
    public static final TrajectoryCollection driveStraight = new TrajectoryCollection(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    public static final TrajectoryCollection driveStraightReversed = new TrajectoryCollection(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    static
    {
        // left
        var leftTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        leftTrajectories.put(new DestinationCouple(null, Destination.kLeftTrenchFar, false),
                Arrays.asList());
        leftTrajectories.put(
                new DestinationCouple(Destination.kLeftTrenchFar, Destination.kLeftShootingPosition, true),
                Arrays.asList());

        left.generateTrajectories(leftTrajectories);

        // middle
        var middleTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        middleTrajectories.put(new DestinationCouple(null, Destination.kShieldGeneratorNearRight, false),
                Arrays.asList());
        middleTrajectories.put(new DestinationCouple(Destination.kShieldGeneratorNearRight, Destination.kShieldGeneratorFarRight, true), Arrays.asList());        
        middleTrajectories.put(new DestinationCouple(Destination.kShieldGeneratorFarRight, Destination.kMiddleShootingPosition, true), Arrays.asList());

        middle.generateTrajectories(middleTrajectories);
        // right
        var rightTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        rightTrajectories.put(new DestinationCouple(null, Destination.kRightTrenchFar, false),
                Arrays.asList());
        rightTrajectories.put(new DestinationCouple(Destination.kRightTrenchFar,
                Destination.kRightShootingPosition, true), Arrays.asList());

        right.generateTrajectories(rightTrajectories);

        //eight ball
        var eightBallTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        eightBallTrajectories.put(new DestinationCouple(null, Destination.kShieldGeneratorNearRight, false), Arrays.asList());
        eightBallTrajectories.put(new DestinationCouple(Destination.kShieldGeneratorNearRight, Destination.kShieldGeneratorFarRight, false), Arrays.asList());
        eightBallTrajectories.put(new DestinationCouple(Destination.kShieldGeneratorFarRight, Destination.kMiddleShootingPosition, true), Arrays.asList());
        eightBallTrajectories.put(new DestinationCouple(Destination.kMiddleShootingPosition, Destination.kRightTrenchVeryFar, false), Arrays.asList());
        eightBallTrajectories.put(new DestinationCouple(Destination.kRightTrenchVeryFar, Destination.kRightTrenchFar, false), Arrays.asList());
        eightBallTrajectories.put(new DestinationCouple(Destination.kRightTrenchFar, Destination.kRightShootingPosition, true), Arrays.asList());
        eightBall.generateTrajectories(eightBallTrajectories);

        var driveStraightTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        driveStraightTrajectories.put(new DestinationCouple(null, Destination.kJustAhead, false), Arrays.asList());
        driveStraight.generateTrajectories(driveStraightTrajectories);

        var driveStraightReversedTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        driveStraightReversedTrajectories.put(new DestinationCouple(null, Destination.kJustBehind, true), Arrays.asList());
        driveStraightReversed.generateTrajectories(driveStraightReversedTrajectories);
    }
}
