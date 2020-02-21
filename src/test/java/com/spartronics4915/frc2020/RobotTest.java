package com.spartronics4915.frc2020;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;
import com.spartronics4915.lib.util.Logger;
import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.wpilibj.RobotBase;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

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
    static class emptySubsystem extends SpartronicsSubsystem
    {
        public emptySubsystem() {}
    }

    private static emptySubsystem sSubsys = new emptySubsystem();
    private static DriverStationSim sSim;

    RobotTest()
    {
        sSim = new DriverStationSim();
        //sSim.setAutonomous(false); // these cause a crash
        //sSim.setEnabled(true);
    }

    @Test
    public void initTest()
    {
        sSubsys.logNotice("initTest found this config: " + Constants.sConfig);        
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
    // @Test
    public void initRobot()
    {
        RobotBase.startRobot(Robot::new);
        sSim.setEnabled(false);
    }
}