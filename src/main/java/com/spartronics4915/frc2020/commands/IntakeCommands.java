package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class IntakeCommands
{
    public class Intake extends StartEndCommand
    {
        public Intake(Intake Intake)
        {
            super(Intake::intake, Intake::stop, Intake); // FIXME
        }
    }

    // An InstantCommand runs an action and immediately exits.
    public class Stop extends InstantCommand
    {
        public Stop(Intake Intake)
        {
            super(Intake::stop, Intake); // FIXME
        }
    }

    public class Eject extends StartEndCommand
    {
        public Eject(Intake Intake)
        {
            super(Intake:reverse, Intake::stop, Intake); // FIXME
        }
    }
}
