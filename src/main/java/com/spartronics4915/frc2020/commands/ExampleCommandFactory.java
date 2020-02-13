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
 * or spread across multiple files. We rely on Java "nested classes" to
 * define multiple classes in a single file. More detail can be found
 * here: https://docs.oracle.com/javase/tutorial/java/javaOO/nested.html.
 * An inner class is associated with an instance of its enclosing class and 
 * has direct access to that object's methods and fields.  This is generally
 * true, only after it's fully been constructed. Sadly, due to the 
 * representation of various wpilib2 commands, this means that we must
 * pass in parameters that might more naturally be represented as outer
 * class member variables.
 *
 * This examples operates on a single subsystem.  Note that some commands may
 * require or operate on more than one subsystem.  This example could be 
 * extended to accept an ArrayList<SpartronicsSubsystem> or 
 * Set<SpartronicsSubsystem>. Or explicitly require, say, the DriveTrain 
 * subsystem.
 *
 * We present at 3 different styles/use-cases.  Choose the one that's
 * best for your application, though option #1 is likely preferred.
 *
 * 1. Contextualized inner class 
 *  - access to outerclass member variables, only not in constructor.
 *  - requires unusual, but legal, java syntax for construction
 * 2. Independant inner class
 *  - no access to outerclass member variables
 *  - yes access to outerclass static variables
 *  - standard constructor syntax:  new ExampleCommandFactory.Test7(mCameraSys)
 * 3. Enumerated MakeCmd method
 *  - easy to read and use, but the dispatch is switch-based
 *
 * usage:
 *  in RobotContainer():
 *     this.mCamera = new CameraSubsystem();
 *      
 *     // ExampleCommandFactory constructor accepts subsystem and
 *     // misc parameters to store in the instance.  These will be
 *     // be available to methods of inner classes unless the methods
 *     // are defined in the constructor or passed to the super during
 *     // construction. 
 *     this.mCamCmds = new ExampleCommandFactory(mCamera, ...parameters...); 
 * 
 *  See RobotContainer::configureTestCommands(), for instantiatioon examples.
 *
 */
public class ExampleCommandFactory
{
    public enum CmdEnum
    {
        kTest1,
        kTest2
    };

    private final SpartronicsSubsystem mSubsys;

    public ExampleCommandFactory(SpartronicsSubsystem subsys)
    {
        this.mSubsys = subsys;
    }

    public CommandBase MakeCmd(CmdEnum c)
    {
        CommandBase result = null;
        switch (c)
        {
            case kTest1:
                result = new InstantCommand(() ->
                {
                    mSubsys.logInfo("running instant command test1");
                });
                break;
            case kTest2:
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

    // Example InstantCommand. Question: why not just place the lambda 
    // expression in the caller? Answer: to encapsulate the details of the 
    // expression in this file and not the caller (ownership of files is clearer)
    public class Test5 extends InstantCommand
    {
        public Test5(SpartronicsSubsystem subsys)
        {
            // Can't access outer class member variables in a Runnable passed
            // to super since it may run it before we're fully constructed.
            // We require that the subsystem be passed in if this InstantCommand
            // 'requires' it. Error message:
            //   cannot refer to 'this' nor 'super' while explicitly invoking a 
            //   constructorJava(134217866)
            super(() -> { subsys.logInfo("Run Test5"); }, subsys);
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
                // look we can put comments in our code!
                // this is the Start command
                subsys.logInfo("Start of StartEndCommand");
            }, () ->
            {
                // look we can put comments in our code!
                // this is the end command.
                subsys.logInfo("End of StartEndCommand");
            }, subsys);
        }
    }

    // A public static (inner) class allows us to reference
    // this class without contructing an instance of our outer
    // class.
    public static class Test7 extends StartEndCommand
    {
        public Test7(SpartronicsSubsystem subsys)
        {
            super(() ->
            {
                // look we can put comments in our code!
                // this is the Start command
                subsys.logInfo("Start of Test7/StartEndCommand");
            }, () ->
            {
                // look we can put comments in our code!
                // this is the end command.
                subsys.logInfo("End of Test7/StartEndCommand");
            }, subsys);
        }
    }
}
