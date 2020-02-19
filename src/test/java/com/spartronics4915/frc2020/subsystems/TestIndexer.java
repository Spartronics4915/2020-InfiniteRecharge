package com.spartronics4915.frc2020.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;
import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.commands.IndexerCommands;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.sim.DriverStationSim;

public class TestIndexer
{
    private static Indexer sIndexer = new Indexer();
    private IndexerCommands mCommands;
    private static DriverStationSim sSim;

    public TestIndexer() {
        mCommands = new IndexerCommands(sIndexer);
        sSim = new DriverStationSim();
        sSim.setAutonomous(false);
        sSim.setEnabled(true);
    }

    @Test
    public void testLaunch() {
        /*****Defining Commands*****/
        var startLaunch = mCommands.new StartKicker(sIndexer);

        var endLaunch = mCommands.new EndKicker(sIndexer);

        var loadBallToSlot = mCommands.new LoadBallToSlotGroup(sIndexer, 0);

        var loadToLauncher = mCommands.new LoadToLauncher(sIndexer);
        /******Defining Motors******/
        var simmedLoaderMotor = SpartronicsSimulatedMotor.getFromId(Constants.Indexer.Loader.kMotorId);
        var simmedIndexerMotor = SpartronicsSimulatedMotor.getFromId(Constants.Indexer.Spinner.kMotorId);

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
        /****Defing Command****/
        var loadFromIntake = mCommands.new LoadFromIntake(sIndexer);

        var bulkHarvest = mCommands.new BulkHarvest(sIndexer);

        // testing LoadFromIntake
        sIndexer.logInfo("Testing LoadFromIntake...");
        loadFromIntake.schedule();
        assertTrue(loadFromIntake.isScheduled());
        sIndexer.logInfo("Success!!");


        sIndexer.logInfo("Testing BulkHarvest...");
        bulkHarvest.schedule();
        assertTrue(bulkHarvest.isScheduled());
        sIndexer.logInfo("Success!!");
        // sim.setEnabled(false);
    }
}
