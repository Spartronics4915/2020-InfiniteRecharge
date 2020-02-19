package com.spartronics4915.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

import com.spartronics4915.frc2020.subsystems.Drive;


public class DriveCommands
{
    private Drive mDrive;

    public DriveCommands(Drive drive)
    {
        mDrive = drive;
    }

    public class TeleOpCommand extends CommandBase
    {
        private final Joystick mJoystick;
        public TeleOpCommand(Joystick joy)
        {
            mJoystick = joy;
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
