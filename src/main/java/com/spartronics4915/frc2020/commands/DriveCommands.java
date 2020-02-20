package com.spartronics4915.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.subsystems.Drive;

public class DriveCommands
{
    private final Drive mDrive;
    private final Joystick mJoystick;
    private boolean mInverted;
    private boolean mSlow;

    public DriveCommands(Drive drive, Joystick joystick)
    {
        mDrive = drive;
        mJoystick = joystick;
        // universal joystick convention is that forward and left are negative
        // as such, mInverted starts as true
        mInverted = true;
        mSlow = false;
        mDrive.setDefaultCommand(new TeleOpCommand());
    }

    /**
     * Runs the standard arcadeDrive, but with conditions (inverted / in slow mode)
     * given by a set of toggleable booleans.
     * <p>
     * Uses the ternary operator to avoid readable code.
     */
    public class TeleOpCommand extends CommandBase
    {
        public TeleOpCommand()
        {
            addRequirements(mDrive);
        }

        @Override
        public void execute()
        {
            // before:
            // (mInverted ? -1 : 1) * (mSlow ? Constants.Drive.kSlowModeMultiplier : 1) * mJoystick.getY(),
            // (mInverted ? -1 : 1) * (mSlow ? Constants.Drive.kSlowModeMultiplier : 1) * mJoystick.getX());

            // after (with helpful comments)
            double x = mJoystick.getX();
            double y = mJoystick.getY(); 
            y = -1;  // reverse the sense of joystick y, fwd should be positive
            if(mSlow)
            {
                x *= Constants.Drive.kSlowModeMultiplier;
                y *= Constants.Drive.kSlowModeMultiplier;
            }
            if(mInverted)
            {
                x *= -1;
                y *= -1;
            }
            mDrive.arcadeDrive(y, x);
        }
    }

    /**
     * Written exclusively for the trigger, this command merely sets
     * the drivetrain to slow mode. A corresponding UnsetSlow does not
     * exist, as ToggleSlow would function fine.
     */
    public class SetSlow extends CommandBase
    {
        public SetSlow()
        {
            addRequirements(mDrive);
        }

        @Override
        public void initialize()
        {
            mSlow = true;
        }

        @Override
        public boolean isFinished()
        {
           return true;
        }
    }

    /**
     * Instantly ends, and goes back to the default TeleOpCommand.
     */
    public class ToggleSlow extends CommandBase
    {
        public ToggleSlow()
        {
            addRequirements(mDrive);
        }

        @Override
        public void initialize()
        {
            mSlow = !mSlow;
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }

    /**
     * Instantly ends, and goes back to the default TeleOpCommand.
     */
    public class ToggleInverted extends CommandBase
    {
        public ToggleInverted()
        {
            addRequirements(mDrive);
        }

        @Override
        public void initialize()
        {
            mInverted = !mInverted;
        }

        @Override
        public boolean isFinished()
        {
            return true;
        }
    }
}
