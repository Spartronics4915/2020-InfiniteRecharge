package com.spartronics4915.frc2020.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.spartronics4915.frc2020.commands.IndexerCommands;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.sim.DriverStationSim;

public class TestIndexer
{
    @Test
    public void testLaunch() {
        var indexer = new Indexer();
        var commands = new IndexerCommands();

        /*****Defining Commands*****/
        var startLaunch = commands.new StartLaunch(indexer);

        var endLaunch = commands.new EndLaunch(indexer);

        var loadBallToSlot = commands.new LoadBallToSlot(indexer, 0);

        var loadToLauncher = commands.new LoadToLauncher(indexer);
        /******Defining Motors******/
        var simmedLoaderMotor = SpartronicsSimulatedMotor.getFromId(Indexer.Motors.LOADER.valueOf());
        var simmedIndexerMotor = SpartronicsSimulatedMotor.getFromId(Indexer.Motors.INDEXER.valueOf());

        var sim = new DriverStationSim();
        sim.setAutonomous(false);
        sim.setEnabled(true);

        
        // testing startlaunch
        indexer.logInfo("Testing StartLaunch...");
        startLaunch.schedule();
        assertTrue(startLaunch.isScheduled()); // make sure it doesn't just crash
        assertEquals(simmedLoaderMotor, 1.0);
        indexer.logInfo("Success!");
        
        // testing endlaunch
        indexer.logInfo("Testing EndLaunch...");
        endLaunch.schedule();
        assertTrue(endLaunch.isScheduled());
        indexer.logInfo("Success!");

        // testing loadBallToSlot
        indexer.logInfo("Testing LoadBallToSlot...");
        loadBallToSlot.schedule();
        assertTrue(loadBallToSlot.isScheduled());
        indexer.logInfo("Success!");

        // testing loadToLauncher
        indexer.logInfo("Testing LoadToLauncher...");
        loadToLauncher.schedule();
        assertTrue(loadToLauncher.isScheduled());
        indexer.logInfo("Success!");

        indexer.logInfo("Loading Test successful!");
    }

    @Test
    public void testIndexerIntake()
    {
        var indexer = new Indexer();
        var commands = new IndexerCommands();

        /****Defing Command****/
        var loadFromIntake = commands.new LoadFromIntake(indexer);

        var sim = new DriverStationSim();
        sim.setAutonomous(false);
        sim.setEnabled(true);

        // testing LoadFromIntake
        indexer.logInfo("Testing LoadFromIntake...");
        loadFromIntake.schedule();
        assertTrue(loadFromIntake.isScheduled());
        indexer.logInfo("Success!!");
    }
}