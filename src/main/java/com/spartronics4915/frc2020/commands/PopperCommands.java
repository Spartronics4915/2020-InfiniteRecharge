package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.Popper;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class PopperCommands
{
    public class Pop extends StartEndCommand
    {
        public Pop(Popper popper)
        {
            super(popper::pop, popper::stop, popper);
        }
    }

    public class Stop extends RunCommand
    {
        public Stop(Popper popper)
        {
            super(popper::stop);
        }
    }
}
