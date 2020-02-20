package com.spartronics4915.frc2020.commands;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.RobotContainer;
import com.spartronics4915.frc2020.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ClimberCommands
{
    private final Climber mClimber;

    public ClimberCommands(Climber climber)
    {
        mClimber = climber;
        mClimber.setDefaultCommand(new Stop());
    }

    /**
     * A {@link StartEndCommand} allows us to specify an execute() and end()
     * condition, and runs until interrupted.
     *
     * @param Runnable onInit Runs over and over until interrupted
     * @param Runnable onEnd (boolean interrupted) Method to call once when ended
     * @param Subsystem requirement For both the CommandScheduler and the above method references.
     */

    /**
     * This {@link StartEndCommand} extends the "lightsaber" while held down.
     * <p>
     * It does so by calling {@link Climber}.extend until stopped.
     * There is no end condition aside from direct termination.
     * <p>
     * Note that the "while held down" functionality is defined in {@link RobotContainer}.
     */
    public class Extend extends StartEndCommand
    {
        public Extend()
        {
            super(mClimber::extend, mClimber::stop, mClimber);
        }
    }

    /**
     * This {@link StartEndCommand} extends the "lightsaber" while held down.
     * <p>
     * It does so by calling {@link Climber}.retract until stopped.
     * There is no end condition aside from direct termination.
     * <p>
     * Note that the "while held down" functionality is defined in {@link RobotContainer}.
     */
    public class Retract extends StartEndCommand
    {
        public Retract()
        {
            super(mClimber::retract, mClimber::stop, mClimber);
        }
    }

    public class ExtendMin extends ParallelRaceGroup
    {
        public ExtendMin()
        {
            super(new Extend(), new WaitCommand(Constants.Climber.kTimerExtenderMin));
        }
    }

    public class ExtendMax extends ParallelRaceGroup
    {
        public ExtendMax()
        {
            super(new Extend(), new WaitCommand(Constants.Climber.kTimerExtenderMax));
        }
    }

    /**
     * The {@link CommandBase} WinchPrimary is part of a two-step winching Command chain.
     * <p>
     * This primary functionality activates first, winching the rope by turning the motor a direction
     * (! {@link Constants}.Climber.kStalled) until it detects a stall ({@link Climber.isStalled}),
     * in which case {@link WinchSecondary} follows.
     * <p>
     * A switch controls this Command, and will always run it first. In the event the Climber is
     * already winched in this motor direction, it will quickly detect the stall and go to
     * {@link WinchSecondary}.
     */
    public class WinchPrimary extends CommandBase
    {
        public WinchPrimary()
        {
            addRequirements(mClimber);
        }

        @Override
        public void execute()
        {
            mClimber.winch(!Constants.Climber.kStalled);
        }

        @Override
        public boolean isFinished()
        {
            return mClimber.isStalled();
        }

        @Override
        public void end(boolean interrupted)
        {
            mClimber.stop();
        }
    }

    /**
     * The {@link CommandBase} WinchSecondary is part of a two-step winching Command chain.
     * <p>
     * This secondary functionality activates after {@link WinchPrimary}, winching the rope by turning
     * the motor the opposite direction as {@link WinchPrimary} ({@link Constants}.Climber.kStalled).
     * <p>
     * A switch controls this Command, and will always run it after {@link WinchPrimary}, even if
     * {@link WinchPrimary} is already winched enough in that direction.
     */
    public class WinchSecondary extends CommandBase
    {
        public WinchSecondary()
        {
            addRequirements(mClimber);
        }

        @Override
        public void execute()
        {
            mClimber.winch(Constants.Climber.kStalled);
        }

        @Override
        public void end(boolean interrupted)
        {
            mClimber.stop();
        }
    }

    /**
     * This {@link SequentialCommandGroup} merely calls {@link WinchPrimary}
     * and {@link WinchSecondary}.
     * <p>
     * WinchPrimary runs the winch in high-speed mode and ends when it detects a stall.
     * WinchSecondary runs the winch in high-torque mode and ends on the user's input.
     */
    public class Winch extends SequentialCommandGroup
    {
        public Winch()
        {
            super(new WinchPrimary(), new WinchSecondary());
        }
    }

    /**
     * This {@link RunCommand} stops all Climber motors by calling
     * Climber.stop() over and over.
     * <p>
     * It also happens to be our default command.
     */
    public class Stop extends RunCommand
    {
        public Stop()
        {
            super(mClimber::stop, mClimber);
        }
    }
}
