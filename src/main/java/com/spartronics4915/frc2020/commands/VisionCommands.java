package com.spartronics4915.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.frc2020.subsystems.Vision;

/**
 * Very little going on here currently:
 *  1. (driver or autonomous) control over VisionLED releay
 *  2. default behavior listening for dashboard request to change LED state.
 * 
 * NB: turret angle updates are under the control of LauncherCommands.
 *   we assume that the updated turret angle is correctly delivered
 *   to the CoordSysMgr.
 */
public class VisionCommands
{
    private Vision mVision;

    public VisionCommands(Vision subsys)
    {
        mVision = subsys;
        mVision.setDefaultCommand(new DefaultCommand());
    }

    public static enum LEDStateChange
    {
        kOff,
        kOn,
        kToggle
    };

    /**
     * An instant command that can turn on, off or toggle the VisionLED relay.
     * This probably should be invoked automatically when odometry determines
     * that we're in a place where vision targeting is enabled.
     * 
     * XXX: perhaps this should be the central control for acquisition:
     *   ie: light is off, we could notify raspi to rest 
     */
    public class SetLEDRelay extends InstantCommand
    {
        LEDStateChange mStateChange;
        public SetLEDRelay(LEDStateChange c)
        {
            super(); 
            // NB: I think it's legit to say 'no subsystem requirements'.
            // since we're the only one who cares about this relay.
            // Now we won't unschedule our default command on each toggle.
            mStateChange = c;
        }

        @Override
        public void initialize()
        {
            // this is where InstantCommand does its thing
            boolean oldstate = mVision.isLEDOn(), newstate;
            switch(mStateChange)
            {
            case kOff:
                newstate = false;
                break;
            case kOn:
                newstate = true;
                break;
            case kToggle:
            default:
                newstate = !oldstate;
                break;
            }
            mVision.setLED(newstate);
            mVision.dashboardPutBoolean(Constants.Vision.kLEDRelayKey, newstate);
        }
    }

    /**
     * We listen for changes to the relay state nettab value - possibly
     * changed by dashboard or even raspi vision.
     */
    private class DefaultCommand extends CommandBase
    {
        public DefaultCommand()
        {
            this.addRequirements(mVision);
        }

        @Override
        public void execute()
        {
            // synchronize mLEDRelay with LEDRelay network table value.
            boolean newstate = mVision.dashboardGetBoolean(Constants.Vision.kLEDRelayKey, true);
            boolean oldstate = mVision.isLEDOn();
            if(newstate != oldstate)
                mVision.setLED(newstate);
        }

        @Override
        public boolean isFinished()
        {
            return false; // for clarity, we're always in this mode
        }
    }

}