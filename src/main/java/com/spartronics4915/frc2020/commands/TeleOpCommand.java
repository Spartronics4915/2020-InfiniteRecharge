package com.spartronics4915.frc2020.commands;

import java.util.Set;

import com.spartronics4915.frc2020.subsystems.Drive;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TeleOpCommand extends CommandBase
{
    private final Drive mDrive;
    private final Joystick mJoystick;

    public TeleOpCommand(Drive drive, Joystick joy)
    {
        mDrive = drive;
        mJoystick = joy;
    }
    @Override
    public Set<Subsystem> getRequirements()
    {
        return Set.of(mDrive);
    }

    @Override
    public void execute()
    {
        mDrive.arcadeDrive(mJoystick.getY(), mJoystick.getX());
    }
}
