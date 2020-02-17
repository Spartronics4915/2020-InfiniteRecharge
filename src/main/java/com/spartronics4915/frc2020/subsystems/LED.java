package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The subsystem that controls the LED.
 * Please see the README.md in the Arduino_Bling directory for details.
 */
public class LED extends SpartronicsSubsystem
{
    private SerialPort mBling;

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
            mBling = new SerialPort(9600, SerialPort.Port.kUSB);
            logInitialized(true);
			Logger.notice("LED: Initialized!");
        }
        catch (Exception e)
        {
            logException("LED: Couldn't initialize SerialPort", e);
            logInitialized(false);
        }
    }

    // TODO: validate desired periodic() function for LED subsystem
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
            return;
        }

        // Save current blingState for smartdashboard display
        mBlingState = blingState;

        // Convert state to byte message and sent to serial port
        byte[] message = blingState.getBlingMessage();

        // TODO: check return value of write and log
        // Logger.notice("LED: (UNINITIALIZED) setBlingState is set to: " + blingState.name());
        mBling.write(message, message.length);
        Logger.notice("LED: setBlingState is set to: " + blingState.name());
    }
}
