package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import com.spartronics4915.lib.util.Logger;
import java.util.Arrays;
import com.fazecast.jSerialComm.SerialPort;

/**
 * The subsystem that controls the LED.
 * Please see the README.md in the Arduino_Bling directory for details.
 */
public class LED extends SpartronicsSubsystem
{
    /**
     * jSerialComm is used for SerialPort communication to identify Arduino device.
     * while on linux systems we can use `lsusb -v` to get USB info, this does not
     * work on roboRIO. On Linux: "Arduino SA Uno R3 (CDC ACM)"
     *
     * See enumerateAvailablePorts() for more info. Note, @robotInit(),
     * prints out a list of available serial ports on the system.
     *
     * LED subsystem only 'write' to the port, writeBytes() method is used.
     */

    // output from kPortDescription is the output from the .getPortDescription()
    private static final String kPortDescription = "USB-Based Serial Port";
    private SerialPort mBlingPort = null;

    private static LED sInstance = null;
    private static BlingState mBlingState;

    public static LED getInstance()
    {
        if (sInstance == null)
        {
            sInstance = new LED();
            mBlingState = BlingState.BLING_COMMAND_TELEOP;
        }
        return sInstance;
    }

    private LED()
    {
        try
        {
            mBlingPort = Arrays.stream(SerialPort.getCommPorts())
                .filter((SerialPort p) -> p.getPortDescription().equals(kPortDescription)
                    && !p.isOpen())
                .findFirst()
                .orElseThrow(() -> new RuntimeException("Device not found: " + kPortDescription));

            mBlingPort.setComPortParameters(9600, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
            mBlingPort.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
            mBlingPort.openPort();

            logInitialized(true);
            Logger.notice("LED: Initialized!");
        }
        catch (Exception e)
        {
            logException("LED: Couldn't initialize SerialPort", e);
            logInitialized(false);
        }
    }

    // LED has no need for periodic updates. State change info is updated in the method
    // setBlingState().
    @Override
    public void periodic()
    {
        super.periodic();
    }

    /**
     * This enum is giving the possible styles we can have the Arduino express.
     */
    public enum BlingState
    {
        // blingStates MUST match Arduino sketch code
        // bling code is passed in for use in the BlingState methods
        BLING_COMMAND_OFF("0"), // turn off bling
        BLING_COMMAND_DISABLED("1"), // robot powered on but disabled (in disabledInit())
        BLING_COMMAND_AUTOMODE("2"), // autonomous init
        BLING_COMMAND_TELEOP("3"), // teleop init and general driving
        BLING_COMMAND_LAUNCH("4"), // ...
        BLING_COMMAND_INTAKE("5"), // ...
        BLING_COMMAND_DRIVE_SLOW("6"), // ...
        BLING_COMMAND_CLIMBING("7"), // ...
        BLING_COMMAND_VISION("8"), // ...
        BLING_COMMAND_EJECT("9"), // ...
        ; // semicolon to state more to follow

        private final String blingCode;

        BlingState(String code)
        {
            this.blingCode = code;
        }

        public byte[] getBlingMessage()
        {
            return this.blingCode.getBytes();
        }
    }

    /**
     * Method to send a request to the Arduino to play the animation associated with the desired bling state.
     */
    public void setBlingState(BlingState blingState)
    {
        // If NOT initialized we should not be calling any LED methods
        if (!isInitialized())
        {
            logError("LED: setBlingState called but LED subsystem is NOT initialized!");
            dashboardPutString("LED state:", "NOT initialized");
            return;
        }

        // Save current blingState
        mBlingState = blingState;

        // Convert state to byte message and sent to serial port
        byte[] message = mBlingState.getBlingMessage();
        if (mBlingPort.writeBytes(message, message.length) == -1)
        {
            logError("LED: Error writing to SerialPort - uninitializing LED subsystem");
            dashboardPutString("LED state:", "Write Error!");
            logInitialized(false);
        }
        else
        {
            dashboardPutString("LED state:", mBlingState.toString());
            logDebug("LED state: " + mBlingState.toString());
        }
    }

    /**
     * Generate a list of available serial ports on the system; example output:
     * DEBUG Port Listing Start: -----------
     * DEBUG ttyUSB1|USB-to-Serial Port (pl2303)|USB-to-Serial Port (pl2303)
     * DEBUG ttyUSB2|USB-to-Serial Port (ftdi_sio)|TTL232R-3V3
     * DEBUG ttyUSB0|USB-to-Serial Port (cp210x)|CP2104 USB to UART Bridge Controller
     * DEBUG ttyS1|Physical Port S1|Physical Port S1
     * DEBUG ttyACM0|USB-Based Serial Port|USB-Based Serial Port
     * DEBUG Port Listing End: -----------
     */
    public void enumerateAvailablePorts()
    {
        logDebug("Port Listing Start: ============================================");
        SerialPort ports[] = SerialPort.getCommPorts();
        if (ports.length == 0)
        {
            logDebug("ERROR: No available serial ports found!");
            return;
        }
        for (SerialPort port : ports)
        {
            String info = port.getSystemPortName() + "|" + port.getDescriptivePortName() + "|"
                + port.getPortDescription() + "\n";
            logDebug(info);
        }
        logDebug("Port Listing End: ==============================================");
    }
}
