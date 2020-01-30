package com.spartronics4915.frc2020.commands;

import javax.xml.namespace.QName;

import com.spartronics4915.frc2020.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ClimberCommands
{
    private Climber mClimber;

    ClimberCommands(Climber Climber)
    {
        mClimber = Climber;
    }

    public class Raise extends StartEndCommand
    {
        public Raise()
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

    public class Winch extends CommandBase
    {
public Winch()
    }
}
