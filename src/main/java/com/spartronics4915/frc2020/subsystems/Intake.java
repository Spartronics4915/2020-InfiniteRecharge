package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

/** The Intake subsystem takes balls from 
 * the playing field and outputs them to storage
 * 
 * harvestintake: activates the mechanum 
 * "vector" wheels in partial intake, hopefully
 * in tandem with the prism roller
 * 
 * ingestintake: activates the prism roller
 * in partial intake, hopefully in tandem with
 * the mechanum "vector" wheels
 * 
 * ingestreverse: reverses the prism roller
 * 
 * harvestreverse: reverses the mechanum
 * wheels
 * 
 * ingeststop: stops the prism roller
 * 
 * harveststop: stops the mechanum wheels
 * 
 * isballheld: checks to see if there's a ball in the intake chamber
 * 
 * stop: a universal stop method
 * 
 */
public class Intake extends SpartronicsSubsystem 
{
    private CANSparkMax mIntakeMotor1;
    private CANSparkMax mIntakeMotor2;

    /** constructor **/
    public Intake()
    {
        mIntakeMotor1 = new CANSparkMax(12, MotorType.kBrushless);
        mIntakeMotor2 = new CANSparkMax(13, MotorType.kBrushless);
    }

    /** starts vector wheels **/
    public void harvestIntake() 
    {
        mIntakeMotor1.set(0.5);
    }

    /** starts prism roller **/
    public void ingestIntake() 
    {
        mIntakeMotor2.set(0.5);
    }

    /** reverses vector wheels **/
    public void harvestReverse() 
    {
        mIntakeMotor1.set(-0.5);
    }

    /** reverses prism roller **/
    public void ingestReverse() 
    {
        mIntakeMotor2.set(-0.5);
    }

    /** stops vector wheels **/
    public void harvestStop() 
    {
        mIntakeMotor1.set(0.0);
    }

    /** stops prism roller **/
    public void ingestStop() 
    {
        mIntakeMotor2.set(0.0);
    }

    /** checks to see if ball is held in intake chamber **/
    public void isBallHeld() 
    {

    }

    /** universal stop method **/
    public void stop()
    {
        mIntakeMotor1.set(0.0);
        mIntakeMotor2.set(0.0);
    }
}
