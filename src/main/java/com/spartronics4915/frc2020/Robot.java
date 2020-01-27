package com.spartronics4915.frc2020;

import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.frc2020.subsystems.LED.BlingState;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot
{
    private Command mAutonomousCommand;
    private RobotContainer mRobotContainer;

    @Override
    public void robotInit()
    {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        mRobotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit()
    {
        // TODO: verify call to DISABLED bling state
        Logger.notice("@disabledInit: Requested BlingState.DISABLED");
        mRobotContainer.setBlingState(BlingState.DISABLED);
    }

    @Override
    public void disabledPeriodic()
    {
    }

    /**
     * This autonomous runs the autonomous command selected by our {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit()
    {
        mAutonomousCommand = mRobotContainer.getAutonomousCommand();

        if (mAutonomousCommand != null)
        {
            mAutonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic()
    {
    }

    @Override
    public void teleopInit()
    {
        if (mAutonomousCommand != null)
        {
            mAutonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic()
    {
    }

    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic()
    {
    }
}
