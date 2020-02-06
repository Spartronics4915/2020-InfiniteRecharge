package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SensorModel;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DigitalInput;

public class PanelRotator extends SpartronicsSubsystem
{
    private SpartronicsMotor mSpinMotor;
    private SpartronicsMotor mRaiseMotor;
    private final DigitalInput mOpticalFlagUp;
    private final DigitalInput mLimitSwitchDown;
    private final ColorSensorV3 mColorSensor;

    private String sensedColor;
    private int red;
    private int green;
    private int blue;

    private final ColorMatch mColorMatcher = new ColorMatch();

    public PanelRotator()
    {
        mOpticalFlagUp = new DigitalInput(Constants.PanelRotator.kOpticalFlagUpId);
        mLimitSwitchDown = new DigitalInput(Constants.PanelRotator.kLimitSwitchDownId);
        mSpinMotor = SpartronicsMax.makeMotor(Constants.PanelRotator.kSpinMotorId,
            SensorModel.fromMultiplier(1));
        mRaiseMotor = SpartronicsSRX.makeMotor(Constants.PanelRotator.kRaiseMotorId,
            SensorModel.fromMultiplier(1));

        mColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        mColorMatcher.addColorMatch(Constants.PanelRotator.kRedTarget);
        mColorMatcher.addColorMatch(Constants.PanelRotator.kGreenTarget);
        mColorMatcher.addColorMatch(Constants.PanelRotator.kBlueTarget);
        mColorMatcher.addColorMatch(Constants.PanelRotator.kYellowTarget);

        if (mSpinMotor.hadStartupError() || mRaiseMotor.hadStartupError())
        {
            mSpinMotor = new SpartronicsSimulatedMotor(Constants.PanelRotator.kSpinMotorId);
            mRaiseMotor = new SpartronicsSimulatedMotor(Constants.PanelRotator.kRaiseMotorId);
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }
    }

    /**
     * Raises the arm holding the spinner at a set speed
     */
    public void raise()
    {
        mRaiseMotor.setDutyCycle(Constants.PanelRotator.kRaiseSpeed);
    }

    /**
     * Lowers the arm holding the spinner at a set speed
     */
    public void lower()
    {
        mRaiseMotor.setDutyCycle(Constants.PanelRotator.kLowerSpeed);
    }

    /**
     * Spins the wheel to move the control panel
     */
    public void spin()
    {
        mSpinMotor.setDutyCycle(Constants.PanelRotator.kSpinSpeed);
    }

    // TODO: What will this return before Stage Two?
    /**
     * Gets the color the robot needs to spin to through game specific messages
     * @return A String color - either Red, Blue, Yellow, or Green
     */
    public String getTargetColor()
    {
        return DriverStation.getInstance().getGameSpecificMessage();
    }

    /** 
     * this gets the 18-bit output (max is 2^18 - 1, I think)
     */
    public String get18BitRGB()
    {
        int red = mColorSensor.getRed();
        int green = mColorSensor.getGreen();
        int blue = mColorSensor.getBlue();

        String RGB = red + ", " + green + ", " + blue;

        return RGB;
    }

    /** 
     * this gets the 18-bit output but divided by 262143 to make a fraction between 0 & 1
     */
    public String getFloatRGB()
    {
        int redFloat = mColorSensor.getRed() / 262143;
        int greenFloat = mColorSensor.getGreen() / 262143;
        int blueFloat = mColorSensor.getBlue() / 262143;

        String RGB = redFloat + ", " + greenFloat + ", " + blueFloat;

        return RGB;
    }

    /**
     * Finds what color the color sensor is seeing.
     * @return A String color - either Red, Blue, Yellow, or Green
     */
    public String getActualColor()
    {
        // TODO: You will need to verify that this builtin functionality works.
        Color detectedColor = mColorSensor.getColor();
        ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);

        if (match.color.equals(Constants.PanelRotator.kRedTarget))
            sensedColor = "Red";
        else if (match.color.equals(Constants.PanelRotator.kGreenTarget))
            sensedColor = "Green";
        else if (match.color.equals(Constants.PanelRotator.kBlueTarget))
            sensedColor = "Blue";
        else if (match.color.equals(Constants.PanelRotator.kYellowTarget))
            sensedColor = "Yellow";
        else
            sensedColor = "Error";

        System.out.println(sensedColor);
        return sensedColor;
    }

    /** 
     * Gets the color that the field's sensors are seeing
     */
    public String getColorForDashboard()
    {
        Color detectedColor = mColorSensor.getColor();
        ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);

        if (match.color.equals(Constants.PanelRotator.kRedTarget))
            sensedColor = "Blue";
        else if (match.color.equals(Constants.PanelRotator.kGreenTarget))
            sensedColor = "Yellow";
        else if (match.color.equals(Constants.PanelRotator.kBlueTarget))
            sensedColor = "Red";
        else if (match.color.equals(Constants.PanelRotator.kYellowTarget))
            sensedColor = "Green";
        else
            sensedColor = "Error";

        return sensedColor;
    }

    /**
     * Sees if the beam sensor on the top is triggered
     * @return whether the optical flag is broken
     */
    public boolean getOpticalFlagUp()
    {
        return mOpticalFlagUp.get() == Constants.PanelRotator.kOpticalFlagBroken; // TODO: adjust the constant if backwards
    }

    // TODO: Double-check this
    /**
     * @return if the bottom limit switch is triggered
     */
    public boolean getLimitSwitchDown()
    {
        return mLimitSwitchDown.get();
    }

    // TODO: Multiple stop() methods are redundant unless we use motor safety

    /**
     * Stops the extension motor
     */
    public void stopRaiseMotor()
    {
        mRaiseMotor.setDutyCycle(0);
    }

    /**
     * Stops the wheel motor
     */
    public void stopSpin()
    {
        mSpinMotor.setDutyCycle(0);
    }

    /** 
     * Universal stop method
     */
    public void stop()
    {
        mSpinMotor.setDutyCycle(0);
        mRaiseMotor.setDutyCycle(0);
    }
}
