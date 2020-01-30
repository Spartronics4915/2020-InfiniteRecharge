package com.spartronics4915.frc2020.commands;

import javax.xml.namespace.QName;

import com.spartronics4915.frc2020.subsystems.Climber;
import com.spartronics4915.frc2020.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ClimberCommands
{
    private Climber mClimber;

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

    public class WinchPrimary extends FunctionalCommand
    {
        public WinchPrimary(Climber mClimber)
        {
            super(() -> {}, () -> mClimber.winch(!Constants.Climber.kStalled), 
                (Boolean b) -> mClimber.stop(), mClimber::isStalled, mClimber);
        }
    }

    public class WinchSecondary extends StartEndCommand
    {
        public WinchSecondary(Climber mClimber)
        {
            super(() -> mClimber.winch(Constants.Climber.kStalled), mClimber::stop, mClimber);
        }
    }
}
