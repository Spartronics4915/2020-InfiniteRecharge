package com.spartronics4915.frc2020.subsystems;

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
        System.out.print("Testing StartLaunch...");
        startLaunch.schedule();
        assertTrue(startLaunch.isScheduled()); // make sure it doesn't just crash
        System.out.println("Success!");
        
        // testing endlaunch
        System.out.print("Testing EndLaunch...");
        endLaunch.schedule();
        assertTrue(endLaunch.isScheduled());
        System.out.println("Success!");

        // testing loadBallToSlot
        System.out.print("Testing LoadBallToSlot...");
        loadBallToSlot.schedule();
        assertTrue(loadBallToSlot.isScheduled());
        System.out.println("Success!");

        // testing loadToLauncher
        System.out.print("Testing LoadToLauncher...");
        loadToLauncher.schedule();
        assertTrue(loadToLauncher.isScheduled());
        System.out.println("Success!");

        System.out.println("Loading Test successful!");
    }
}