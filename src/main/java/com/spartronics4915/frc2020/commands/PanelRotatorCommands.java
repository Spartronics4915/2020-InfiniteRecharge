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
     * repeatedly, until the upper beam sensor is broken, at which point the
     * motor will stop.
     * <p>
     * The motor will also stop raising if interrupted by another Command.
     */
    public class Raise extends FunctionalCommand
    {
        public Raise(PanelRotator PanelRotator)
        {
            super(() -> {}, PanelRotator::raise, (Boolean b) -> PanelRotator.stop(),
                PanelRotator::getBeamSensorUp, PanelRotator);
        }
    }

    /**
     * This Lower {@link FunctionalCommand} will call {@link PanelRotator}.lower
     * repeatedly, until the down beam sensor is broken, at which point the
     * motor will stop.
     * <p>
     * The motor will also stop lowering if interrupted by another Command.
     */
    public class Lower extends FunctionalCommand
    {
        public Lower(PanelRotator PanelRotator)
        {
            super(() -> {}, PanelRotator::lower, (Boolean b) -> PanelRotator.stop(),
                PanelRotator::getBeamSensorDown, PanelRotator);
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
        // TODO: This might be better as a FunctionalCommand.
        private final PanelRotator mPanelRotator;
        private String mTargetColor;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public SpinToColor(PanelRotator PanelRotator)
        {
            mPanelRotator = PanelRotator;
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
            if (mPanelRotator.getActualColor() == mTargetColor)
                return true;
            else
                return false;

            if (mPanelRotator.getActualColor() == "Error")
            {
                System.out.println("error—no data provided");
                return true;
            }

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

        public int eighths;
        public String currentColor;
        public String lastColor;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public SpinOnce(PanelRotator PanelRotator)
        {
            mPanelRotator = PanelRotator;
            addRequirements(mPanelRotator);
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize()
        {
            eighths = 0;
            currentColor = mPanelRotator.getActualColor();
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
            currentColor = mPanelRotator.getActualColor();
            if (currentColor != lastColor)
                eighths++;
            lastColor = currentColor;

            // The color wheel is made up of two each of four total colors,
            // for a total of eight.
            if (eighths == 8) // TODO: double check for off-by-one errors
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

    public class ColorSensorTesting extends CommandBase
    {
        private final PanelRotator mPanelRotator;

        // You should only use one subsystem per command. If multiple are needed, use a
        // CommandGroup.
        public ColorSensorTesting(PanelRotator PanelRotator)
        {
            mPanelRotator = PanelRotator;
            addRequirements(mPanelRotator);
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize()
        {
            System.out.println("\nPredicted color: " + mPanelRotator.getActualColor());
            System.out.println("18-bit: " + mPanelRotator.get18BitRGB());
            System.out.println("Float: " + mPanelRotator.getFloatRGB());
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
