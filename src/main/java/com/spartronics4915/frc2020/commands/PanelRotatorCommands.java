package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.subsystems.PanelRotator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PanelRotatorCommands
{
    private final PanelRotator mPanelRotator;

    public PanelRotatorCommands(PanelRotator panelRotator)
    {
        mPanelRotator = panelRotator;
        mPanelRotator.setDefaultCommand(new Stop());
    }

    /**
     * This Raise {@link CommandBase} calls {@link PanelRotator}.raise
     * repeatedly, until the upper optical flag is broken, at which point the
     * motor will stop.
     * <p>
     * The motor will also stop raising if interrupted by another Command.
     */
    public class Raise extends CommandBase
    {
        public Raise()
        {
            addRequirements(mPanelRotator);
        }

        @Override
        public void execute()
        {
            mPanelRotator.raise();
        }

        @Override
        public boolean isFinished()
        {
            return mPanelRotator.getOpticalFlagUp();
        }

        @Override
        public void end(boolean interrupted)
        {
            mPanelRotator.stop();
        }
    }

    /**
     * This Lower {@link CommandBase} will call {@link PanelRotator}.lower
     * repeatedly, until the down limit switch is pressed, at which point the
     * motor will stop.
     * <p>
     * The motor will also stop lowering if interrupted by another Command.
     */
    public class Lower extends CommandBase
    {
        public Lower()
        {
            addRequirements(mPanelRotator);
        }

        @Override
        public void execute()
        {
            mPanelRotator.lower();
        }

        @Override
        public boolean isFinished()
        {
            return mPanelRotator.getLimitSwitchDown();
        }

        @Override
        public void end(boolean interrupted)
        {
            mPanelRotator.stop();
        }
    }

    /**
     * This {@link CommandBase} will spin the color wheel to the correct color as broadcast
     * by the FMS.
     * No "optimization" is implemented - the spinner only moves clockwise/counterclockwise.
     * <p>
     * It requires the {@link PanelRotator} to be raised first to work properly.
     */
    public class SpinToColor extends CommandBase
    {
        private String mTargetColor;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public SpinToColor()
        {
            addRequirements(mPanelRotator);
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize()
        {
            // We set mTargetColor only once, after this Command is first called.
            mTargetColor = mPanelRotator.getTargetColor();
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mPanelRotator.spin();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished()
        {
            // Note that this is a comparison of Strings.
            // Conversions from native ColorSensorV3 values to one of four values is
            // done in PanelRotator.
            if (mPanelRotator.getRotatedColor().equals(mTargetColor))
                return true;
            else if (mPanelRotator.getRotatedColor().equals("Error"))
            {
                mPanelRotator.logError("Color Sensor: No data provided");
                return true;
            }
            else if (mPanelRotator.getLimitSwitchDown())
            {
                mPanelRotator.logError("Arm is down, so the wheel is stopped");
                return true;
            }
            else
                return false;

        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted)
        {
            mPanelRotator.stop();
        }
    }

    /**
     * The {@link CommandBase} SpinRotation calls {@link PanelRotator}.spin until
     * it detects (through use of the color sensor) that the wheel has been spun
     * one full rotation.
     * <p>
     * Do note that it only spins the Color Wheel once. The operator will have to
     * push the corresponding button at least three times to complete Stage Two.
     */
    public class SpinRotation extends CommandBase
    {
        private int eighths;
        private String currentColor;
        private String lastColor;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public SpinRotation()
        {
            addRequirements(mPanelRotator);
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize()
        {
            eighths = 0;
            currentColor = mPanelRotator.getRotatedColor();
            lastColor = currentColor;
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
            mPanelRotator.spin();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished()
        {
            // If the confidence in Color is too low, we're likely looking up at the ceiling and
            // not aligned with the Control Panel.
            if (mPanelRotator.getColorConfidence() < Constants.PanelRotator.kConfidenceMinimum)
            {
                mPanelRotator.logError("Confidence too low!");
                return true;
            }
            else if (mPanelRotator.getRotatedColor().equals("Error"))
            {
                mPanelRotator.logError("Color Sensor: No data provided");
                return true;
            }
            else if (mPanelRotator.getLimitSwitchDown())
            {
                mPanelRotator.logError("Arm is down, so the wheel is stopped");
                return true;
            }

            // If the detected color has changed, iterate the eighths counter.
            currentColor = mPanelRotator.getRotatedColor();
            if (currentColor != lastColor)
                eighths++;
            lastColor = currentColor;

            // The color wheel is made up of two each of four total colors,
            // for a total of eight.
            if (eighths == 8)
                return true;
            else
                return false;
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted)
        {
            mPanelRotator.stop();
        }
    }

    /**
     * This {@link RunCommand} stops all PanelRotator motors by calling
     * PanelRotator.stop() repeatedly.
     * <p>
     * It also happens to be our default command.
     */
    public class Stop extends RunCommand
    {
        public Stop()
        {
            super(mPanelRotator::stop, mPanelRotator);
        }
    }

    /* These won't work without backing up the robot.
    public class AutoSpinRotation extends SequentialCommandGroup
    {
        public AutoSpinRotation()
        {
            super(new Raise(), new SpinRotation(), new SpinRotation(), new SpinRotation(),
                new SpinRotation(), new Lower());
        }
    }

    public class AutoSpinToColor extends SequentialCommandGroup
    {
        public AutoSpinToColor()
        {
            super(new Raise(), new SpinToColor(), new Lower());
        }
    }
    */
}
