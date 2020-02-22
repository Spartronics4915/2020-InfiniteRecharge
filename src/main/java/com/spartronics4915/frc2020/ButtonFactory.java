package com.spartronics4915.frc2020;

import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;

public class ButtonFactory
{
    Constants.OI.DeviceSpec[] mDeviceList;
    ButtonFactory()
    {
        /* our job is to configure Constants.OI.deviceList according
         * to runtime configuration inputs
         */
        String config; 
        mDeviceList = Constants.OI.deviceList;
        if(!RobotBase.isReal())
        {
            config = "noOI";
            Logger.notice("ButtonFactory: configuring buttons for no OI");
        }
        else
            config = Constants.sConfig; // come from runtime bootstrapping

        switch(config)
        {
        case "noOI":
        case "testbed": // testbed might have its own config, which we embody here.
            break;
        case "default":
            for(int i=0;i<mDeviceList.length;i++)
                mDeviceList[i].joystick = new Joystick(mDeviceList[i].portId);
            break;
        }
    }

    Joystick getJoystick(int devid)
    {
        return this.mDeviceList[devid].joystick;
    }

    /**
     * create a joystick button if the current machine configuration
     * supports it. We dance this little jig to reduce console spewage.
     * @param deviceid
     * @param buttonid
     * @return either a Joystick button or an InvalidButton
     */
    Button create(int deviceid, int buttonid)
    {
        if(deviceid < this.mDeviceList.length)
        {
            var dev = this.mDeviceList[deviceid];
            // buttonIds indexOrigin is 1
            if(dev.joystick != null && buttonid <= dev.numButtons)
                return new JoystickButton(dev.joystick, buttonid);
        }
        return new InvalidButton(0, buttonid);
    }

    /* we don't extend JoystickButton because the super is always
     * called and this is the source of some spewage when missing.
     */
    private static class InvalidButton extends Button
    {
        public InvalidButton(int port, int button) 
        {
        }

        /* overriding core Trigger methods gets us most of the way since Button
         * is a thin vaneer atop Trigger
         */
        @Override
        public Trigger whenActive(final Command toRun, boolean inter)
        {
            // no-op
            return this;
        }

        @Override
        public Trigger whileActiveContinuous(final Command toRun, boolean inter)
        {
            // no-op
            return this;
        }

        @Override
        public Trigger whileActiveOnce(final Command toRun, boolean inter)
        {
            // no-op
            return this;
        }

        @Override
        public Button whenInactive(final Command toRun, boolean intr)
        {
            // no-op
            return this;
        }

        @Override
        public Button toggleWhenActive(final Command toRun, boolean intr)
        {
            // no-op
            return this;
        }

        @Override
        public Button cancelWhenActive(final Command toRun)
        {
            // no-op
            return this;
        }

    }
}