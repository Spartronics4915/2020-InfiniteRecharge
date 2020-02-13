package com.spartronics4915.frc2020.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.commands.IndexerCommands;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;

public class TestIndexer
{
    private static Indexer sIndexer = new Indexer();
    private IndexerCommands mCommands;
    private static DriverStationSim sSim;

    public TestIndexer() {
        mCommands = new IndexerCommands();
        sSim = new DriverStationSim();
        sSim.setAutonomous(false);
        sSim.setEnabled(true);
    }

    @Test
    public void testLaunch() {
        /*****Defining Commands*****/
        var startLaunch = mCommands.new StartLaunch(sIndexer);

        var endLaunch = mCommands.new EndLaunch(sIndexer);

        var loadBallToSlot = mCommands.new LoadBallToSlot(sIndexer, 0);

        var loadToLauncher = mCommands.new LoadToLauncher(sIndexer);
        /******Defining Motors******/
        var simmedLoaderMotor = SpartronicsSimulatedMotor.getFromId(Indexer.Motors.LOADER.valueOf());
        var simmedIndexerMotor = SpartronicsSimulatedMotor.getFromId(Indexer.Motors.INDEXER.valueOf());

        // var sim = new DriverStationSim();
        // sim.setAutonomous(false);
        // sim.setEnabled(true);

        
        // testing startlaunch
        sIndexer.logInfo("Testing StartLaunch...");
        startLaunch.schedule();
        assertTrue(startLaunch.isScheduled()); // make sure it doesn't just crash
        // assertEquals(simmedLoaderMotor, 1.0);
        // CommandScheduler.getInstance().cancel(startLaunch);
        sIndexer.logInfo("Success!");
        
        // testing endlaunch
        sIndexer.logInfo("Testing EndLaunch...");
        endLaunch.schedule();
        assertTrue(endLaunch.isScheduled());
        sIndexer.logInfo("Success!");

        // testing loadBallToSlot
        sIndexer.logInfo("Testing LoadBallToSlot...");
        loadBallToSlot.schedule();
        assertTrue(loadBallToSlot.isScheduled());
        sIndexer.logInfo("Success!");

        // testing loadToLauncher
        sIndexer.logInfo("Testing LoadToLauncher...");
        loadToLauncher.schedule();
        assertTrue(loadToLauncher.isScheduled());
        sIndexer.logInfo("Success!");

        sIndexer.logInfo("Loading Test successful!");
    }

    @Test
    public void testIndexerIntake()
    {
<<<<<<< HEAD
        /****Defing Command****/
        var loadFromIntake = mCommands.new LoadFromIntake(sIndexer);

        // testing LoadFromIntake
        sIndexer.logInfo("Testing LoadFromIntake...");
        loadFromIntake.schedule();
        assertTrue(loadFromIntake.isScheduled());
        sIndexer.logInfo("Success!!");
        // CommandScheduler.getInstance().cancel(loadFromIntake);

        // sim.setEnabled(false);
=======
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
>>>>>>> 24ebc2140eacc1cdadb4f0f8fa247496194ec984
    }
}
