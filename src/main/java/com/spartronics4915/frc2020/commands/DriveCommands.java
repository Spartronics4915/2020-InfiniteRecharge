package com.spartronics4915.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

import com.spartronics4915.frc2020.subsystems.Drive;


public class DriveCommands
{
    private final Drive mDrive;
    private final Joystick mJoystick;

    public DriveCommands(Drive drive, Joystick joystick)
    {
        mDrive = drive;
        mJoystick = joystick;
        mDrive.setDefaultCommand(new TeleOpCommand());
    }

    public class TeleOpCommand extends CommandBase
    {
        public TeleOpCommand()
        {
            addRequirements(mDrive);
        }

        @Override
        public void execute()
        {
            // To invert joystick controller, so forward... is forward
            mDrive.arcadeDrive(mJoystick.getY(), mJoystick.getX());
        }
    }
}
