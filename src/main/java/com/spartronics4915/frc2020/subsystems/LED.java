package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import java.util.Arrays;
import com.fazecast.jSerialComm.SerialPort;


/**
 * The subsystem that controls the LED.
 * Please see the README.md in the Arduino_Bling directory for details.
 */
public class LED extends SpartronicsSubsystem
{
    /**
     * jSerialComm is used for SerialPort communication to identify
     * Arduino device.
     *  - Use `$ lsusb -v` to get info on USB device
     *  - kPortDescription is used for port enumeration
     *  - Since LED subsystem only 'write' to the port, writeBytes()
     *    method is used
     * TODO -- verify Arduino returns port description and matches kPortDescription
     */
    private static final String kPortDescription = "Arduino SA Uno R3 (CDC ACM)";
    private SerialPort mBlingPort = null;

    private static LED sInstance = null;
    private static BlingState mBlingState;

    public static LED getInstance()
    {
        if (sInstance == null)
        {
            sInstance = new LED();
            mBlingState = BlingState.BLING_COMMAND_STARTUP;
        }
        return sInstance;
    }

    private LED()
    {
        try
        {
            mBlingPort = Arrays.stream(SerialPort.getCommPorts())
                .filter(
                    // TODO -- verify Arduino returns port description
                    (SerialPort p) -> p.getPortDescription().equals(kPortDescription) && !p.isOpen())
                .findFirst().orElseThrow(() -> new RuntimeException("Device not found: " + kPortDescription));

            mBlingPort.setComPortParameters(9600, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
            mBlingPort.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
            mBlingPort.openPort();

            logInitialized(true);
        }
        catch (Exception e)
        {
            logException("LED: Couldn't initialize SerialPort", e);
            logInitialized(false);
        }
    }

    // Update LED blingState to dashboard
    @Override
    public void periodic()
    {
        super.periodic();
        this.dashboardPutString("LED state:", mBlingState.toString());
    }

    /**
     * This enum is giving the possible styles we can have the Arduino express.
     */
    public enum BlingState
    {
        // TODO: Add blingStates & ensure it matches Arduino sketch code
        // bling code is passed in for use in the BlingState methods
        BLING_COMMAND_OFF("0"), // turn off bling
        BLING_COMMAND_STARTUP("1"), // Startup phase
        BLING_COMMAND_DISABLED("2"), // robot powered on but disabled (in disabledInit())
        BLING_COMMAND_AUTOMODE("3"), // ...
        BLING_COMMAND_SHOOTING("4"), // ...
        BLING_COMMAND_PICKUP("5"), // ...
        BLING_COMMAND_LOADING("6"), // ...
        BLING_COMMAND_CLIMBING("7"), // ...
        BLING_COMMAND_VISION("8"), // ...
        BLING_COMMAND_DEFAULT("9"), // ...
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
        // TODO: review the process of setting initialization -- if not
        // initialized we should not be calling any LED methods
        if (!isInitialized())
        {
            logError("LED: setBlingState called but LED subsystem is NOT initialized!");
            return;
        }

        // Save current blingState for smartdashboard display
        mBlingState = blingState;

        // Convert state to byte message and sent to serial port
        byte[] message = blingState.getBlingMessage();
        if (mBlingPort.writeBytes(message, message.length) == -1)
        {
            logError("LED: Error writing to SerialPort - uninitializing LED subsystem");
            logInitialized(false);
        }
    }
}
