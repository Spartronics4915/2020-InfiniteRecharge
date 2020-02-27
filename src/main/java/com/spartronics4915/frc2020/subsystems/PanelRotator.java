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
    private String rotatedColor;

    private final ColorMatch mColorMatcher = new ColorMatch();

    public PanelRotator()
    {
        mOpticalFlagUp = new DigitalInput(Constants.PanelRotator.kOpticalFlagUpId);
        mLimitSwitchDown = new DigitalInput(Constants.PanelRotator.kLimitSwitchDownId);
        mSpinMotor = SpartronicsMax.makeMotor(Constants.PanelRotator.kSpinMotorId);
        mRaiseMotor = new SpartronicsSimulatedMotor(42);//SpartronicsSRX.makeMotor(Constants.PanelRotator.kRaiseMotorId);

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
        mRaiseMotor.setPercentOutput(Constants.PanelRotator.kRaiseSpeed);
    }

    /**
     * Lowers the arm holding the spinner at a set speed
     */
    public void lower()
    {
        mRaiseMotor.setPercentOutput(Constants.PanelRotator.kLowerSpeed);
    }

    /**
     * Spins the wheel to move the control panel
     */
    public void spin()
    {
        mSpinMotor.setPercentOutput(Constants.PanelRotator.kSpinSpeed);
    }

    // TODO: What will this return before Stage Two?
    /**
     * Gets the color the robot needs to spin to through game specific messages
     *
     * @return A String color - either Red, Blue, Yellow, or Green
     */
    public String getTargetColor()
    {
        return DriverStation.getInstance().getGameSpecificMessage();
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

        dashboardPutString("Current Color (robot)", sensedColor);
        dashboardPutNumber("ColorMatch Confidence", match.confidence);
        return sensedColor;
    }

    /**
     * The position of our color sensor and the field's has a difference of π/2, so
     * we need to adjust targets accordingly.
     * <p>
     * See https://drive.google.com/uc?id=1BfoFJmpJg31txUqTG-OrJjeWgQdQsCNC
     * for a diagram of how these line up.
     * <p>
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

        dashboardPutString("currentColor", rotatedColor);
        dashboardPutNumber("currentColorConfidence", match.confidence);
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
        mSpinMotor.setPercentOutput(0);
        mRaiseMotor.setPercentOutput(0);
    }
}
