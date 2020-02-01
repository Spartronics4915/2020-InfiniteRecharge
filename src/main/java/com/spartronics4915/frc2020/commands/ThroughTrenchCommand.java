package com.spartronics4915.frc2020.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.spartronics4915.frc2020.TrajectoryContainer;
import com.spartronics4915.lib.math.twodim.control.TrajectoryTracker;
import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.math.twodim.trajectory.constraints.TimingConstraint;
import com.spartronics4915.lib.subsystems.drive.AbstractDrive;
import com.spartronics4915.lib.subsystems.drive.TrajectoryTrackerCommand;
import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ThroughTrenchCommand extends TrajectoryTrackerCommand
{
    public ThroughTrenchCommand(AbstractDrive drive, TrajectoryTracker tracker,
            RobotStateMap stateMap)
    {
        super(drive, TrajectoryContainer.generateTrajectory(Arrays.asList(), Arrays.asList()),
                tracker, stateMap);
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        ArrayList<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<TimingConstraint<Pose2dWithCurvature>>();
        var trajectory = TrajectoryContainer.generateTrajectory(waypoints, constraints);
        mTrajectory = () -> trajectory;
    }
}
