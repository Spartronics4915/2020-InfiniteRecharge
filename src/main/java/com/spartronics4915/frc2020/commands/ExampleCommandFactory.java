package com.spartronics4915.frc2020.commands;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * Here's an example Command "Factory".
 * This example represents a way to have a per-subsystem "command factory"
 * that will allow us to factor/separate/encapsulate commands for
 * a subsystem into a single file, rather than mooshed into robotContainer
 * or spread across multiple files.
 *
 * This examples operates on a single subsystem.  Note that some commands may
 * require or operate on more than one subsystem.  This example could be 
 * extended to accept an ArrayList<SpartronicsSubsystem> or 
 * Set<SpartronicsSubsystem>. Or explicitly require, say, the DriveTrain 
 * subsystem.
 *
 * We present at least 3 different styles/use-cases.  Choose the one that's
 * best for your application, though option #1 is likely preferred.
 *
 * 1. Contextualized Innerclass 
 *  - access to outerclass member variables
 *  - requires unusual, but legal, java syntax for construction
 * 2. Uncontextualized Innerclass
 *  - no access to outerclass instance (ie: more of a namespace implemenation)
 *  - easier construction, but no shared state across innerclass instances.
 * 3. Enumerated GetCommand method
 *  - easy to read and use, but the dispatch is switch-based
 *
 * usage:
 *  in RobotContainer():
 *     this.mCamera = new CameraSubsystem();
 *     this.mCamCmds = new ExampleCommandFactory(mCamera);
 *
 *  in RobotContainer::configureJoystickButtons(), for example:
 *
 *  // style #1
 *  new JoystickButton(...).whenPressed(this.mCamCmds.new Test3());
 *
 *  // style #2
 *  new JoystickButton(...).whenPressed(new ExampleCommandFactory.Test5(mCamera));
 * 
 *  // style #3
 *  new JoystickButton(...).whenPressed(mCamCmds.GetCommand("test1"));
 *
 */
public class ExampleCommandFactory
{
    private final SpartronicsSubsystem mSubsys;

    public ExampleCommandFactory(SpartronicsSubsystem subsys)
    {
        this.mSubsys = subsys;
    }

    public CommandBase GetCommand(String nm)
    {
        CommandBase result = null;
        switch (nm)
        {
            case "test1":
                result = new InstantCommand(() ->
                {
                    mSubsys.logInfo("running instant command test1");
                });
                break;
            case "test2":
                result = new InstantCommand(() ->
                {
                    mSubsys.logInfo("running instant command test2");
                });
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
        public Test4()
        {
            addRequirements(mSubsys);

            // add subcommands here
        }
    }

    // Example InstantCommand. Why not just place the labmda expression in the
    // caller? - to encapsulate the details of the expression in this file
    // and not the caller (ownership of files is clearer)
    public class Test5 extends InstantCommand
    {
        public Test5(SpartronicsSubsystem subsys)
        {
            // Can't access mSubsys here, so we require that it be
            // passed in...
            super(() -> subsys.logInfo("InstantCommand Test5"));
        }
    }

    // Another subclassing example. Here we subclass StartEndCommand
    // and take care to pass our start and end Runnables as parameters
    // to the superclass constructor.
    public class Test6 extends StartEndCommand
    {
        public Test6(SpartronicsSubsystem subsys)
        {
            super(() ->
            {
                subsys.logInfo("Start of StartEndCommand");
            }, () ->
            {
                subsys.logInfo("End of StartEndCommand");
            }, subsys);
        }
    }
}
