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
    private SpartronicsMotor mExtendMotor;

    private final DigitalInput mBeamSensorUp;
    private final DigitalInput mBeamSensorDown;

    private final ColorSensorV3 mColorSensor;

    public String sensedColor;

    private int red;
    private int green;
    private int blue;

    private final ColorMatch mColorMatcher = new ColorMatch();

    //see https://drive.google.com/file/d/1BfoFJmpJg31txUqTG-OrJjeWgQdQsCNC/view for a diagram of how these line up
    //TODO: test all these

    //Blue
    private final Color kRedTarget = ColorMatch.makeColor(0, 1, 0);
    //Yellow
    private final Color kGreenTarget = ColorMatch.makeColor(1, 1, 0);
    //Red
    private final Color kBlueTarget = ColorMatch.makeColor(1, 0, 0);
    //Green
    private final Color kYellowTarget = ColorMatch.makeColor(0, 1, 0);

    public PanelRotator()
    {
        mBeamSensorUp = new DigitalInput(Constants.PanelRotator.kBeamSensorUpId);
        mBeamSensorDown = new DigitalInput(Constants.PanelRotator.kBeamSensorDownId);
        mSpinMotor = SpartronicsMax.makeMotor(Constants.PanelRotator.kSpinMotorId,
            SensorModel.fromMultiplier(1));
        mExtendMotor = SpartronicsSRX.makeMotor(Constants.PanelRotator.kExtendMotorId,
            SensorModel.fromMultiplier(1));
        if (mSpinMotor.hadStartupError() || mExtendMotor.hadStartupError())
        {
            mSpinMotor = new SpartronicsSimulatedMotor();
            mExtendMotor = new SpartronicsSimulatedMotor();
            logInitialized(false);
        }
        else
        {
            logInitialized(true);
        }
        // mSpinMotor = new CANSparkMax(Constants.PanelRotator.kSpinMotorID,
        // MotorType.kBrushless);
        // mExtendMotor = new TalonSRX(Constants.PanelRotator.kExtendMotorID);
        mColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        mColorMatcher.addColorMatch(kRedTarget);
        mColorMatcher.addColorMatch(kGreenTarget);
        mColorMatcher.addColorMatch(kBlueTarget);
        mColorMatcher.addColorMatch(kYellowTarget);
    }

    /**
     * Raises the arm holding the spinner at a set speed
     */
    public void raise()
    {
        mExtendMotor.setDutyCycle(Constants.PanelRotator.kRaiseSpeed);
    }

    /**
     * Lowers the arm holding the spinner at a set speed
     */
    public void lower()
    {
        mExtendMotor.setDutyCycle(Constants.PanelRotator.kLowerSpeed);
    }

    /**
     * Spins the wheel to move the control panel
     */
    public void spin()
    {
        mSpinMotor.setDutyCycle(Constants.PanelRotator.kSpinMotorSpeed);
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

    /** this gets the 18-bit output (max is 2^18 - 1, I think) -*/
    public String get18BitRGB()
    {
        int red = mColorSensor.getRed();
        int green = mColorSensor.getGreen();
        int blue = mColorSensor.getBlue();

        String RGB = red + ", " + green + ", " + blue;

        return RGB;
    }

    /** this gets the 18-bit output but divided by 262143 to make a fraction between 0 & 1 -*/
    public String getFloatRGB()
    {
        int redFloat = mColorSensor.getRed() / 262143;
        int greenFloat = mColorSensor.getGreen() / 262143;
        int blueFloat = mColorSensor.getBlue() / 262143;

        String RGB = redFloat + ", " + greenFloat + ", " + blueFloat;

        return RGB;
    }

    /** finds what color the color sensor is seeing  (Red, Blue, Yellow, or Green); currently just a placeholder for output */

    // TODO: Implement this method!! !
    // https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/Java/Color%20Match/src/main/java/frc/robot/Robot.java
    /**
     * Finds what color the color sensor is seeing - currently just a placeholder for output
     *
     * @return A String color - either Red, Blue, Yellow, or Green
     */
    public String getActualColor()
    {
        Color detectedColor = mColorSensor.getColor();

        ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);

        if (match.color == kRedTarget)
        {
            sensedColor = "Red";
        }
        else if (match.color == kGreenTarget)
        {
            sensedColor = "Green";
        }
        else if (match.color == kBlueTarget)
        {
            sensedColor = "Blue";
        }
        else if (match.color == kYellowTarget)
        {
            sensedColor = "Yellow";
        }
        else
        {
            sensedColor = "Error";
        }

        System.out.println(sensedColor);
        return sensedColor;

        // return "method not complete";
    }

    /**
     * Sees if the bottom beam sensor is triggered
     */
    public boolean getBeamSensorDown()
    {
        // TODO: maybe backwards
        return mBeamSensorDown.get();
    }

    /**
     * Sees if the top beam sensor is triggered
     */
    public boolean getBeamSensorUp()
    {
        return mBeamSensorUp.get(); // TODO: maybe backwards
    }

    // TODO: A discussion needs to be had on the relevance and implementation of getRotations...

    // TODO: Multiple stop() methods are redundant unless we use motor safety

    /**
     * Stops the extension motor
     */
    public void stopExtendMotor()
    {
        mExtendMotor.setDutyCycle(0);
    }

    /**
     * Stops the wheel motor
     */
    public void stopSpin()
    {
        mSpinMotor.setDutyCycle(0);
    }

    /** stops the two motors */
    public void stop()
    {
        mSpinMotor.setDutyCycle(0);
        mExtendMotor.setDutyCycle(0);
    }
}
