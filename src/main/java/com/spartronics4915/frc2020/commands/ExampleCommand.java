package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExampleCommand extends CommandBase {

    private Climber mClimber;
    // You should only use one subsystem per command. If multiple are needed, use a CommandGroup.
    public ExampleCommand(Climber climber) {
        mClimber = climber;
        addRequirements(mClimber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // ex. mClimber.raise();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end (boolean interrupted) {
        // ex. mClimber.stop();
    }
}
