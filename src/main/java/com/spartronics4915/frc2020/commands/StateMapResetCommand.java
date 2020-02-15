package com.spartronics4915.frc2020.commands;

import java.util.Set;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.subsystems.estimator.RobotStateEstimator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

class StateMapResetCommand extends CommandBase {
    public StateMapResetCommand(RobotStateEstimator stateEstimator, Pose2d start) {
        stateEstimator.resetRobotStateMaps(start);
    }
    @Override
    public Set<Subsystem> getRequirements()
    {
        // TODO Auto-generated method stub
        return Set.of();
    }
    @Override
    public void execute() {

    }
}