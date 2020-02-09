package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.subsystems.PanelRotator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class PanelRotatorCommands
{
    /**
     * Commands with simple logic statements should be implemented as a
     * {@link FunctionalCommand}. This saves the overhead of a full
     * {@link CommandBase}, but still allows us to deal with isFinished.
     * <p>
     * A FunctionalCommand takes five inputs:
     * @param Runnable onInit
     * @param Runnable onExecute
     * @param Consumer<Boolean> onEnd (boolean interrupted)
     * @param BooleanSupplier isFinished
     * @param Subsystem requirement For both the CommandScheduler and the above method references.
     * <p>
     * Each of these parameters corresponds with a method in the CommandBase class.
     */

    /**
     * This Raise {@link FunctionalCommand} calls {@link PanelRotator}.raise
     * repeatedly, until the upper optical flag is broken, at which point the
     * motor will stop.
     * <p>
     * The motor will also stop raising if interrupted by another Command.
     */
    public class Raise extends FunctionalCommand
    {
        public Raise(PanelRotator panelRotator)
        {
            super(() -> {}, panelRotator::raise, (Boolean b) -> panelRotator.stop(),
                panelRotator::getOpticalFlagUp, panelRotator);
        }
    }

    /**
     * This Lower {@link FunctionalCommand} will call {@link PanelRotator}.lower
     * repeatedly, until the down limit switch is pressed, at which point the
     * motor will stop.
     * <p>
     * The motor will also stop lowering if interrupted by another Command.
     */
    public class Lower extends FunctionalCommand
    {
        public Lower(PanelRotator panelRotator)
        {
            super(() -> {}, panelRotator::lower, (Boolean b) -> panelRotator.stop(),
                panelRotator::getLimitSwitchDown, panelRotator);
        }
    }

    /**
     * Commands that are "complex", or have > simple logic within them,
     * should be put here. They simply extend {@link CommandBase} and
     * are written as such.
     */

    /**
     * This {@link CommandBase} will spin the color wheel to the correct color as broadcast
     * by the FMS.
     * No "optimization" is implemented - the spinner only moves clockwise/counterclockwise.
     * <p>
     * It requires the {@link PanelRotator} to be raised first to work properly.
     */
    public class SpinToColor extends CommandBase
    {
        private final PanelRotator mPanelRotator;
        private String mTargetColor;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public SpinToColor(PanelRotator panelRotator)
        {
            mPanelRotator = panelRotator;
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
    public class SpinOnce extends CommandBase
    {
        private final PanelRotator mPanelRotator;

        private int eighths;
        private String currentColor;
        private String lastColor;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public SpinOnce(PanelRotator panelRotator)
        {
            mPanelRotator = panelRotator;
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
            // If the detected color has changed, iterate the eighths counter.
            currentColor = mPanelRotator.getRotatedColor(); // TODO: does this have issues with the white stripes?
            if (currentColor != lastColor)
                eighths++;
            lastColor = currentColor;

            // The color wheel is made up of two each of four total colors,
            // for a total of eight.
            if (eighths == 8) // TODO: double check for off-by-one errors
                return true;
            else
                return false;

            // TODO: In the event we drive away from the control panel while spinning it,
            // this subsystem should be able to tell and end appropriately.
            // Look into exposing and using the confidence value of the ColorMatch.
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted)
        {
            mPanelRotator.stop();
        }
    }

    public class ColorSensorTesting extends CommandBase
    {
        private final PanelRotator mPanelRotator;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public ColorSensorTesting(PanelRotator panelRotator)
        {
            mPanelRotator = panelRotator;
            addRequirements(mPanelRotator);
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize()
        {
            mPanelRotator.logDebug("Predicted color: " + mPanelRotator.getActualColor());
            mPanelRotator.logDebug("18-bit: " + mPanelRotator.get18BitRGB());
            mPanelRotator.logDebug("Float: " + mPanelRotator.getFloatRGB());
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute()
        {
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished()
        {
            return true;
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted)
        {
        }
    }
}
