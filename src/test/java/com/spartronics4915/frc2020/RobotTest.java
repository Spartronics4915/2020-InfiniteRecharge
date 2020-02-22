package com.spartronics4915.frc2020;

import com.spartronics4915.frc2020.commands.*;
import com.spartronics4915.frc2020.subsystems.*;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.hal.HAL;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * a simple test of basic robot init.  This turns out to be quite
 * slippery in the context of the HALJni configuration.  We apparently
 * can't expect to run this interactively but the thread below provides some
 * vscode settings that I've verified as functiona.  It appears to be very 
 * sensitive to static construction sequence, etc.  I landed at this point
 * by search-and-destroy methods.   Hopefully we can learn more
 * about the inherent subtlties.
 * 
 * https://www.chiefdelphi.com/t/unit-testing-java-io-ioexception-wpihaljni/372288/8
 */
class RobotTest
{
    static final Robot sRobot;
    static
    {
        if (!HAL.initialize(500, 0)) 
            Logger.warning("HAL already initialized");

        sRobot = new Robot();
        sRobot.robotInit();
    }

    @Test
    public void initTest()
    {        
        Logger.notice("initTest found this config: " + Constants.sConfig);
    }

    /* this test, when run interactively is sorta interesting, we
     * disable it until such time as we can determine how to request exit.
     * Issues that a manual run points out:
     *      - Built by: null  on: null  (null) (even though Manifest looks fine)
     *      - missing native libspartronicsnative.so
     *      - missing USB/serial throws an error
     *      - CTR: CAN frame not received/too-stale.
     *      - Joystick Button 1 on port 0 not available, check if controller is plugged in
    */
    @Test
    public void robotInitTest()
    {
        assert(sRobot.mInitialized);
    }

    @Test
    public void indexerTest()
    {
        assert(sRobot.mInitialized);
        var sim = new DriverStationSim();
        sim.setAutonomous(false);
        sim.setEnabled(true);
        var cmds = sRobot.mRobotContainer.mIndexerCommands;
        var indexer = sRobot.mRobotContainer.mIndexer;
        var startLaunch = cmds.new StartKicker();
        var endLaunch = cmds.new EndKicker();
        var loadBallToSlot = cmds.new LoadBallToSlotGroup(0);
        var loadToLauncher = cmds.new LoadToLauncher();

        // testing startlaunch
        indexer.logInfo("Testing StartLaunch...");
        startLaunch.schedule();

        assertTrue(startLaunch.isScheduled()); // make sure it doesn't just crash
        // assertEquals(simmedLoaderMotor, 1.0);
        // CommandScheduler.getInstance().cancel(startLaunch);
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

        sim.setEnabled(false);
    }
}