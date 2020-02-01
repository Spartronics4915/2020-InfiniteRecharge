package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ClimberCommands
{
    public class Extend extends StartEndCommand
    {
        public Extend(Climber Climber)
        {
            super(Climber::extend, Climber::stop, Climber);
        }
    }

    public class Retract extends StartEndCommand
    {
        public Retract(Climber Climber)
        {
            super(Climber::retract, Climber::stop, Climber);
        }
    }

    /**
     * Commands with simple logic statements should be implemented as a
     * FunctionalCommand. This saves the overhead of a full CommandBase, but still
     * allows us to deal with isFinished.
     */
    public class WinchPrimary extends FunctionalCommand
    {
        public WinchPrimary(Climber Climber)
        {
            super(() -> {}, () -> Climber.winch(!Constants.Climber.kStalled),
                (Boolean b) -> Climber.stop(), Climber::isStalled, Climber);
        }
    }

    /**
     * A StartEndCommand allows us to specify an execute() and end() condition,
     * and runs until interrupted.
     */
    public class WinchSecondary extends StartEndCommand
    {
        public WinchSecondary(Climber Climber)
        {
            super(() -> Climber.winch(Constants.Climber.kStalled), Climber::stop, Climber);
        }
    }
}
