package com.spartronics4915.frc2020.commands;

import java.util.Set;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class StateMapResetCommand extends InstantCommand {
    public StateMapResetCommand(RobotStateEstimator stateEstimator, Pose2d start) {
        super(() -> {
            stateEstimator.resetRobotStateMaps(start);
        });
    }

    @Override
    public Set<Subsystem> getRequirements()
    {
        return Set.of();
    }

    @Override
    public void execute() {

    }
}