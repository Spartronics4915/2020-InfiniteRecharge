package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ClimberCommands
{
    private final Climber mClimber;

    public ClimberCommands(Climber Climber)
    {
        mClimber = Climber;
    }

    public class Extend extends StartEndCommand
    {
        public Extend()
        {
            super(mClimber::extend, mClimber::stop, mClimber);
        }
    }

    public class Retract extends StartEndCommand
    {
        public Retract()
        {
            super(mClimber::retract, mClimber::stop, mClimber);
        }
    }

    /**
     * Commands with simple logic statements should be implemented as a
     * FunctionalCommand. This saves the overhead of a full CommandBase, but still
     * allows us to deal with isFinished.
     */
    public class WinchPrimary extends FunctionalCommand
    {
        public WinchPrimary()
        {
            super(() -> {}, () -> mClimber.winch(!Constants.Climber.kStalled),
                (Boolean b) -> mClimber.stop(), mClimber::isStalled, mClimber);
        }
    }

    /**
     * A StartEndCommand allows us to specify an execute() and end() condition,
     * and runs until interrupted.
     */
    public class WinchSecondary extends StartEndCommand
    {
        public WinchSecondary()
        {
            super(() -> mClimber.winch(Constants.Climber.kStalled), mClimber::stop, mClimber);
        }
    }
}
