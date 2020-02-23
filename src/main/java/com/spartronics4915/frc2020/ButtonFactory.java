package com.spartronics4915.frc2020;

import java.util.HashSet;
import java.util.Objects;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;

import com.spartronics4915.lib.util.Logger;

public class ButtonFactory
{

    private Constants.OI.DeviceSpec[] mDeviceList;
    private HashSet<ButtonSpec> mInUse;

    ButtonFactory()
    {
        /* our job is to configure Constants.OI.deviceList according
         * to runtime configuration inputs
         */
        String config; 
        mDeviceList = Constants.OI.deviceList;
        mInUse = new HashSet<>();
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
        case "test chassis": // main drive tester
            {
                int id = Constants.OI.kJoystickId;
                mDeviceList[id].joystick = new Joystick(id);
                mDeviceList[id].numButtons = 4; // maybe 0 is better?
            }
            break;
        case "Athena":
        case "robot2020":
        case "default":
            // here we trust in Constants.OI values
            for(int i=0;i<mDeviceList.length;i++)
                mDeviceList[i].joystick = new Joystick(mDeviceList[i].portId);
            break;
        }
    }

    /**
     * return the joystick object for the given port.
     * NB: the result may be null!
     */
    Joystick getJoystick(int port)
    {
        return this.mDeviceList[port].joystick;
    }

    /**
     * create a joystick button if the current machine configuration
     * supports it. We dance this little jig to reduce console spewage.
     * @param deviceid
     * @param buttonid
     * @return either a Joystick button or an InvalidButton
     */
    Button create(int deviceId, int buttonId)
    {
        var spec = new ButtonSpec(deviceId, buttonId);
        if(mInUse.contains(spec))
        {
            Logger.warning("ButtonFactory button collision " + 
                            deviceId + " " + buttonId);
        }
        mInUse.add(spec);
        if(deviceId < this.mDeviceList.length)
        {
            var dev = this.mDeviceList[deviceId];
            // buttonIds indexOrigin is 1
            if(dev.joystick != null && buttonId <= dev.numButtons)
                return new JoystickButton(dev.joystick, buttonId);
        }
        return new InvalidButton(0, buttonId);
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

    private static class ButtonSpec
    {
        private final int port;
        private final int bId;

        public ButtonSpec(int usbport, int buttonId)
        {
            port = usbport;
            bId = buttonId;
        }

        // https://stackoverflow.com/questions/262367/type-safety-unchecked-cast
        @SuppressWarnings("unchecked")
        @Override
        public boolean equals(Object o)
        {
            if (this == o)
                return true;
            if (o == null)
                return false;
            if (!(o instanceof ButtonSpec))
                return false;
            return port == ((ButtonSpec) o).port && 
                   bId == ((ButtonSpec) o).bId;
        }

        @Override
        public int hashCode()
        {
            return (port << 8)  + bId;
        }

    }
}