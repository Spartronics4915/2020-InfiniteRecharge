package com.spartronics4915.frc2020.commands;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Here's an example Command "Factory".
 * This example represents a way to have a per-subsystem factory
 * that will allow us to factor/separate/encapsulate commands for
 * a subsystem into a single file, rather than mooshed into robotContainer
 * or spread across multiple files.
 *
 * This examples operates on a single subsystem.  Note that some commands may
 * require or operate on more than one subsystem.  This example could be extended
 * to accept an ArrayList<SpartronicsSubsystem> or Set<SpartronicsSubsystem>.
 * Or explicitly require, say, the DriveTrain subsystem.
 *
 * usage:
 *  in RobotContainer():
 *     this.mExampleCmdFactory = new ExampleCommandFactory(mCamera);
 *
 *  in RobotContainer::configureJoystickButtons()
 *     new JoystickButton(...).whenPressed(this.mExampleCmdFactory.GetCommand("test1"));
 *     new JoystickButton(...).whenPressed(new ExampleCommandFactory.Test1());
 *
 */
public class ExampleCommandFactory
{
    private final SpartronicsSubsystem mSubsys;

    ExampleCommandFactory(SpartronicsSubsystem subsys)
    {
        this.mSubsys = subsys;
    }

    public CommandBase GetCommand(String nm)
    {
        CommandBase result = null;
        switch (nm)
        {
            case "test1":
                result = new InstantCommand(() -> mSubsys.logInfo("running instant command test1"));
                break;
            case "test2":
                result = new InstantCommand(() -> mSubsys.logInfo("running instant command test2"));
                break;
            default:
        }
        return result;
    }

    public class Test3 extends CommandBase
    {

        private int mCount = 0;

        public Test3()
        {
            addRequirements(mSubsys);
        }

        public void execute()
        {
            mSubsys.dashboardPutNumber("executionCount", this.mCount++);
        }

        public boolean isFinished()
        {
            return this.mCount > 1000;
        }

        public void done()
        {
            mSubsys.logInfo("Test1 command is done");
        }
    }

    public class Test4 extends SequentialCommandGroup
    {

        Test4()
        {
            addRequirements(mSubsys);

            // add subcommands here
        }

    }
}
