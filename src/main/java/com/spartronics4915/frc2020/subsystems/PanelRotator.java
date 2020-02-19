package com.spartronics4915.frc2020.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.spartronics4915.frc2020.Constants;
import com.spartronics4915.lib.hardware.motors.SpartronicsMax;
import com.spartronics4915.lib.hardware.motors.SpartronicsMotor;
import com.spartronics4915.lib.hardware.motors.SpartronicsSRX;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;
import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class PanelRotator extends SpartronicsSubsystem
{
    private final DigitalInput mOpticalFlagUp;
    private final DigitalInput mLimitSwitchDown;
    private SpartronicsMotor mSpinMotor;
    private SpartronicsMotor mRaiseMotor;
    private final ColorSensorV3 mColorSensor;

    private String sensedColor;
    private String rotatedColor;

    private final ColorMatch mColorMatcher = new ColorMatch();

    public PanelRotator()
    {
        mOpticalFlagUp = new DigitalInput(Constants.PanelRotator.kOpticalFlagUpId);
        mLimitSwitchDown = new DigitalInput(Constants.PanelRotator.kLimitSwitchDownId);

        mSpinMotor = SpartronicsMax.makeMotor(Constants.PanelRotator.kSpinMotorId);
        mSpinMotor.setBrakeMode(true);

        mRaiseMotor = SpartronicsSRX.makeMotor(Constants.PanelRotator.kRaiseMotorId);
        mRaiseMotor.setBrakeMode(true);

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
     * Gets the color the field needs to see to through game-specific messages
     *
     * @return A String color - either Red, Blue, Yellow, or Green
     */
    public String getTargetColor()
    {
        // return DriverStation.getInstance().getGameSpecificMessage();
        //TODO: this is a placeholder because we don't have the field output in tests; change it before we are actually in the game
        return "Red";
    }

    // TODO: What will this return before Stage Two?
    /**
     * Gets the color the robot needs to spin to through game specific messages
     *
     * @return A String color - either Red, Blue, Yellow, or Green
     */
    public String getRobotTargetColor()
    {
        String robotTargetColor;

        if (getTargetColor() == "Red")
            robotTargetColor = "Blue";
        else if (getTargetColor() == "Green")
            robotTargetColor = "Yellow";
        else if (getTargetColor() == "Blue")
            robotTargetColor = "Red";
        else if (getTargetColor() == "Yellow")
            robotTargetColor = "Green";
        else
            robotTargetColor = "Error";

        return robotTargetColor;
    }

    /**
     * This gets the 18-bit output (max is 2^18 - 1, I think)
     *
     * @return a comma-separated String of raw RGB values
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
     * This gets the 18-bit output but divided by 262143 to make a fraction between 0 & 1
     *
     * @return a comma-separated String of RGB values, as a percentage
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
     * Finds what actual color the color sensor is seeing.
     *
     * @return A String color - either Red, Blue, Yellow, or Green
     */
    public String getActualColor()
    {
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

        return sensedColor;
    }

    /**
     * Finds the distance between the sensor and what it is looking at
     * @return 11-bit (0-2047) value
     */
    public int getDistance()
    {
        int distance = mColorSensor.getProximity();
        
        return distance;
    }

    /**
     * This code could be less redundant by taking a String parameter and converting it,
     * but it'll work out to be the same amount of code anyways, and this is clearer.
     *
     * @return The current Color of the wheel as detected by the FMS.
     */
    public String getRotatedColor()
    {
        Color detectedColor = mColorSensor.getColor();
        ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);

        if (match.color.equals(Constants.PanelRotator.kRedTarget))
            rotatedColor = "Blue";
        else if (match.color.equals(Constants.PanelRotator.kGreenTarget))
            rotatedColor = "Yellow";
        else if (match.color.equals(Constants.PanelRotator.kBlueTarget))
            rotatedColor = "Red";
        else if (match.color.equals(Constants.PanelRotator.kYellowTarget))
            rotatedColor = "Green";
        else
            rotatedColor = "Error";

        return rotatedColor;
    }

    /**
     * {@link ColorMatchResult} includes a confidence value.
     *
     * @return a percentage value from 0 to 1 with the
     */
    public double getColorConfidence()
    {
        Color detectedColor = mColorSensor.getColor();
        ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);

        return match.confidence;
    }

    /**
     * Checks if the top optical flag is broken
     *
     * @return whether the PanelManipulator is raised
     */
    public boolean getOpticalFlagUp()
    {
        // TODO: Double-check this
        return mOpticalFlagUp.get();
    }

    /**
     * Checks if the bottom limit switch is triggered
     *
     * @return whether the PanelManipulator is lowered
     */
    public boolean getLimitSwitchDown()
    {
        // TODO: Double-check this
        return mLimitSwitchDown.get();
    }

    /**
     * Universal stop method
     */
    public void stop()
    {
        mSpinMotor.setDutyCycle(0);
        mRaiseMotor.setDutyCycle(0);
    }

    /**
     * According to Dana this runs all the time.
     */
    public void periodic()
    {
        // these methods are put here to output stuff to the driver station
        dashboardPutNumber("Color sensor IR distance", getDistance());
        dashboardPutString("Color seen by robot", getActualColor());
        dashboardPutString("Color seen by FMS", getRotatedColor());
        dashboardPutNumber("Color match confidence", getColorConfidence());
        dashboardPutString("Color sensor target (what the FIELD wants to see)", getTargetColor());
        dashboardPutString("Color sensor target (what the ROBOT wants to see)", getRobotTargetColor());
    }
}
