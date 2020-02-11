package com.spartronics4915.frc2020.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.spartronics4915.frc2020.commands.IndexerCommands;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;

public class TestIndexer
{
    @Test
    public void testLaunch() 
    {
        Indexer mIndexer = new Indexer();
        IndexerCommands mIndexerCommands = new IndexerCommands();

        // Defining Commands
        Command startLaunch = mIndexerCommands.new StartLaunch(mIndexer);
        Command endLaunch = mIndexerCommands.new EndLaunch(mIndexer);
        Command loadBallToSlot = mIndexerCommands.new LoadBallToSlot(mIndexer, 0);
        Command loadToLauncher = mIndexerCommands.new LoadToLauncher(mIndexer);

        // Defining Motors
        SpartronicsMotor simmedLoaderMotor = SpartronicsSimulatedMotor.getFromId(Constants.Indexer.Loader.kMotorId);
        SpartronicsMotor simmedIndexerMotor = SpartronicsSimulatedMotor.getFromId(Constants.Indexer.Spinner.kMotorId);

        DriverStationSim mSim = new DriverStationSim();
        mSim.setAutonomous(false);
        mSim.setEnabled(true);

        // Testing startlaunch
        mIndexer.logInfo("Testing StartLaunch...");
        startLaunch.schedule();
        assertTrue(startLaunch.isScheduled()); // make sure it doesn't just crash
        assertEquals(simmedLoaderMotor, 1.0);
        mIndexer.logInfo("Success!");

        // Testing endlaunch
        mIndexer.logInfo("Testing EndLaunch...");
        endLaunch.schedule();
        assertTrue(endLaunch.isScheduled());
        mIndexer.logInfo("Success!");

        // Testing loadBallToSlot
        mIndexer.logInfo("Testing LoadBallToSlot...");
        loadBallToSlot.schedule();
        assertTrue(loadBallToSlot.isScheduled());
        mIndexer.logInfo("Success!");

        // Testing loadToLauncher
        mIndexer.logInfo("Testing LoadToLauncher...");
        loadToLauncher.schedule();
        assertTrue(loadToLauncher.isScheduled());
        mIndexer.logInfo("Success!");

        mIndexer.logInfo("Loading Test successful!");
    }

    @Test
    public void testIndexerIntake()
    {
        Indexer mIndexer = new Indexer();
        IndexerCommands mIndexerCommands = new IndexerCommands();

        // Defining Command
        Command loadFromIntake = mIndexerCommands.new LoadFromIntake(mIndexer);

        DriverStationSim mSim = new DriverStationSim();
        mSim.setAutonomous(false);
        mSim.setEnabled(true);

        // Testing LoadFromIntake
        mIndexer.logInfo("Testing LoadFromIntake...");
        loadFromIntake.schedule();
        assertTrue(loadFromIntake.isScheduled());
        mIndexer.logInfo("Success!!");
    }
}
