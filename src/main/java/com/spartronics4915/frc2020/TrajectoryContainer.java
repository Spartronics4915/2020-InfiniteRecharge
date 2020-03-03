package com.spartronics4915.frc2020;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import com.spartronics4915.frc2020.commands.StateMapResetCommand;
import com.spartronics4915.frc2020.commands.SuperstructureCommands;
import com.spartronics4915.frc2020.subsystems.Drive;
import com.spartronics4915.lib.math.twodim.control.TrajectoryTracker;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.geometry.Rectangle2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.geometry.Translation2d;
import com.spartronics4915.lib.math.twodim.trajectory.TrajectoryGenerator;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.TimingConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import com.spartronics4915.lib.math.twodim.trajectory.types.TimedTrajectory;
import com.spartronics4915.lib.subsystems.drive.TrajectoryTrackerCommand;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TrajectoryContainer
{
    public static class AutoMode
    {
        public final String name;
        public final Command command;

        public AutoMode(String name, Command command)
        {
            this.name = name;
            this.command = command;
        }
    }

    public static enum Destination
    {
        kLeftTrenchFar(380, 134, 120),
        kLeftShootingPosition(508, 5, 148.69),
        kLeftSittingPosition(420, 8, 180),
        kRightTrenchFar(364, -140, 180),
        kMiddleBack(490, -65, 180),
        kRightTrenchVeryFar(404, -145, 180),
        kRightTrenchIntermediate(443, -88, 240),
        kRightTrenchNear(242, -134, 180),
        kEightBallIntermediate(456, -134, 180),
        kRightShootingPosition(400, -140, 180),
        kShieldGeneratorFarRight(386, -46, 140),
        kHalfWayToShielGenerator(450, -80, 170),
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
        private final Destination mStart;
        private final Destination mEnd;
        private final boolean mReversed;
        private final List<TimingConstraint<Pose2dWithCurvature>> mExtraConstraints;

        public DestinationCouple(Destination a, Destination b, boolean reversed)
        {
            mStart = a;
            mEnd = b;
            mReversed = reversed;
            mExtraConstraints = new ArrayList<>();
        }

        public DestinationCouple(Destination a, Destination b, boolean reversed,
            TimingConstraint<Pose2dWithCurvature>... constraints)
        {
            mStart = a;
            mEnd = b;
            mReversed = reversed;
            mExtraConstraints = Arrays.asList(constraints);
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
            constraints.add(new CentripetalAccelerationConstraint(0.762));
            constraints.addAll(mExtraConstraints);
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
            var startPose = start == null ? null : start.pose;
            var endPose = end == null ? null : end.pose;

            //  System.out.println("getTrajectory: " + mStartPoint + ", " + startPose + ", " + endPose);

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
    public static final TrajectoryCollection eightBall = new TrajectoryCollection(
        Constants.Trajectory.kStartPointMiddle);
    public static final TrajectoryCollection driveStraight = new TrajectoryCollection(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    public static final TrajectoryCollection driveStraightReversed = new TrajectoryCollection(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    public static final AutoMode kDefaultAutoMode = new AutoMode("All: Do Nothing", new Command()
    {
        @Override
        public Set<Subsystem> getRequirements()
        {
            return Set.of();
        }
    });

    public static AutoMode[] getAutoModes(RobotStateEstimator stateEstimator, Drive drive,
        TrajectoryTracker ramseteController, SuperstructureCommands superstructureCommands)
    {
        AutoMode[] autoModes = new AutoMode[] {kDefaultAutoMode,
            // new AutoMode("Drive Straight", new SequentialCommandGroup(
            //     new StateMapResetCommand(stateEstimator,
            //         TrajectoryContainer.driveStraight.mStartPoint),
            //     new TrajectoryTrackerCommand(drive,
            //         TrajectoryContainer.driveStraight.getTrajectory(null, Destination.kJustAhead),
            //         ramseteController, stateEstimator.getBestRobotStateMap()))),
            // new AutoMode("Drive Straight Reversed",
            //     new SequentialCommandGroup(
            //         new StateMapResetCommand(stateEstimator,
            //             TrajectoryContainer.driveStraightReversed.mStartPoint),
            //         new TrajectoryTrackerCommand(drive,
            //             TrajectoryContainer.driveStraightReversed.getTrajectory(null,
            //                 Destination.kJustBehind),
            //             ramseteController, stateEstimator.getBestRobotStateMap()))),
            new AutoMode("Left",
                new SequentialCommandGroup(
                    new WaitCommand(5),
                    superstructureCommands.new LaunchSequence(4),
                    new StateMapResetCommand(stateEstimator, TrajectoryContainer.left.mStartPoint),
                    new TrajectoryTrackerCommand(drive,
                        TrajectoryContainer.left.getTrajectory(null,
                            Destination.kLeftSittingPosition),
                        ramseteController, stateEstimator.getBestRobotStateMap()))),
            new AutoMode("Middle 3 Ball",
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new StateMapResetCommand(stateEstimator,
                        TrajectoryContainer.middle.mStartPoint),
                    superstructureCommands.new LaunchSequence(4),
                    new TrajectoryTrackerCommand(drive,
                            TrajectoryContainer.middle.getTrajectory(
                                null, Destination.kMiddleBack),
                            ramseteController, stateEstimator.getBestRobotStateMap())
            )),
            new AutoMode("Middle 5 Ball",
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new StateMapResetCommand(stateEstimator,
                        TrajectoryContainer.middle.mStartPoint),
                    superstructureCommands.new LaunchSequence(4),
                    new ParallelRaceGroup(
                        new TrajectoryTrackerCommand(drive,
                            TrajectoryContainer.middle.getTrajectory(
                                Destination.kRightTrenchIntermediate,
                                Destination.kRightTrenchFar),
                            ramseteController, stateEstimator.getBestRobotStateMap()),
                        new SequentialCommandGroup(
                            superstructureCommands.new Intake(),
                            superstructureCommands.new Intake())),
                    new TrajectoryTrackerCommand(drive,
                        TrajectoryContainer.middle.getTrajectory(Destination.kRightTrenchFar,
                            Destination.kRightShootingPosition),
                        ramseteController, stateEstimator.getBestRobotStateMap()),
                    superstructureCommands.new LaunchSequence(5)
            )),
            // new AutoMode("Eight Ball", new SequentialCommandGroup(
            // new StateMapResetCommand(stateEstimator,
            // TrajectoryContainer.eightBall.mStartPoint),
            // superstructureCommands.new LaunchSequence(3),
            // new ParallelCommandGroup(
            // new TrajectoryTrackerCommand(
            // drive,
            // TrajectoryContainer.eightBall.getTrajectory(null,
            // Destination.kShieldGeneratorFarRight),
            // ramseteController, stateEstimator.getBestRobotStateMap()),
            // new SequentialCommandGroup(
            // superstructureCommands.new Intake(),
            // superstructureCommands.new Intake())),
            // new TrajectoryTrackerCommand(drive,
            // TrajectoryContainer.eightBall.getTrajectory(
            // Destination.kShieldGeneratorFarRight, Destination.kEightBallIntermediate),
            // ramseteController, stateEstimator.getBestRobotStateMap()),
            // new ParallelCommandGroup(
            // new TrajectoryTrackerCommand(
            // drive,
            // TrajectoryContainer.eightBall.getTrajectory(
            // Destination.kEightBallIntermediate, Destination.kRightTrenchFar),
            // ramseteController, stateEstimator.getBestRobotStateMap()),
            // new SequentialCommandGroup(
            // superstructureCommands.new Intake(),
            // superstructureCommands.new Intake(),
            // superstructureCommands.new Intake())),
            // new TrajectoryTrackerCommand(drive,
            // TrajectoryContainer.eightBall.getTrajectory(Destination.kRightTrenchFar,
            // Destination.kRightShootingPosition),
            // ramseteController, stateEstimator.getBestRobotStateMap()),
            // superstructureCommands.new LaunchSequence(5))),
            // new AutoMode("Characterize Drive",
            // new CharacterizeDriveBaseCommand(drive, Constants.Drive.kWheelDiameter)),
            // new AutoMode("Right: Through Trench",
            // new TrajectoryTrackerCommand(drive,
            // TrajectoryContainer.left.getTrajectory(null, Destination.kLeftTrenchFar),
            // ramseteController, stateEstimator.getBestRobotStateMap()))

        };
        return autoModes;
    }

    static
    {
        // left
        var leftTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        leftTrajectories.put(new DestinationCouple(null, Destination.kLeftSittingPosition, false),
            Arrays.asList());
        // leftTrajectories.put(new DestinationCouple(Destination.kLeftTrenchFar,
        // Destination.kLeftShootingPosition, true), Arrays.asList());

        left.generateTrajectories(leftTrajectories);

        // middle
        var rightTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        rightTrajectories.put(new DestinationCouple(Destination.kRightTrenchIntermediate,
            Destination.kRightTrenchFar, false,
            new VelocityLimitRegionConstraint(
                new Rectangle2d(
                    new Translation2d(Units.inchesToMeters(410), Units.inchesToMeters(-105)),
                    new Translation2d(Units.inchesToMeters(205), Units.inchesToMeters(-160))),
                0.3)),
            Arrays.asList());
        rightTrajectories.put(new DestinationCouple(Destination.kRightTrenchFar,
            Destination.kRightShootingPosition, true), Arrays.asList());
        rightTrajectories.put(new DestinationCouple(null, Destination.kMiddleBack, false), Arrays.asList());

        middle.generateTrajectories(rightTrajectories);

        // eight ball
        var eightBallTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        eightBallTrajectories.put(
            new DestinationCouple(null, Destination.kShieldGeneratorFarRight, false),
            Arrays.asList());
        eightBallTrajectories.put(new DestinationCouple(Destination.kShieldGeneratorFarRight,
            Destination.kEightBallIntermediate, true), Arrays.asList());
        eightBallTrajectories.put(new DestinationCouple(Destination.kEightBallIntermediate,
            Destination.kRightTrenchFar, false), Arrays.asList());
        eightBallTrajectories.put(new DestinationCouple(Destination.kRightTrenchFar,
            Destination.kRightShootingPosition, true), Arrays.asList());

        eightBall.generateTrajectories(eightBallTrajectories);

        var driveStraightTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        driveStraightTrajectories.put(new DestinationCouple(null, Destination.kJustAhead, false),
            Arrays.asList());

        driveStraight.generateTrajectories(driveStraightTrajectories);

        var driveStraightReversedTrajectories = new HashMap<DestinationCouple, List<Pose2d>>();
        driveStraightReversedTrajectories
            .put(new DestinationCouple(null, Destination.kJustBehind, true), Arrays.asList());

        driveStraightReversed.generateTrajectories(driveStraightReversedTrajectories);
    }

    public static TimedTrajectory<Pose2dWithCurvature> throughTrench(
        RobotStateEstimator stateEstimator)
    {
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        Pose2d[] intermediate = new Pose2d[] {
            new Pose2d(Units.inchesToMeters(424), Units.inchesToMeters(135),
                Rotation2d.fromDegrees(180)),
            new Pose2d(Units.inchesToMeters(207), Units.inchesToMeters(135),
                Rotation2d.fromDegrees(180))};
        for (int i = 0; i < intermediate.length; i++)
        {
            Pose2d pose = intermediate[i];
            intermediate[i] = new Pose2d(pose.getTranslation().getX() - Units.inchesToMeters(312.5),
                pose.getTranslation().getY(), pose.getRotation());
        }
        RobotStateMap stateMap = stateEstimator.getBestRobotStateMap();
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
        return TrajectoryContainer.generateTrajectory(waypoints, constraints, false);
    }

    public static TimedTrajectory<Pose2dWithCurvature> toControlPanel(
        RobotStateEstimator stateEstimator)
    {
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        Pose2d pose = stateEstimator.getBestRobotStateMap().getLatestState().pose;
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
        return TrajectoryContainer.generateTrajectory(waypoints, constraints, false);
    }
}
