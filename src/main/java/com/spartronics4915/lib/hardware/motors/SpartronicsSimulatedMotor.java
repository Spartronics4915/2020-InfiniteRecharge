package com.spartronics4915.lib.hardware.motors;

/**
 * This class provides a simulated, easy-to-inspect, implementor of SpartronicsMotor.
 */
// TODO: Keep track of motor state
public class SpartronicsSimulatedMotor implements SpartronicsMotor {

    @Override
    public SpartronicsEncoder getEncoder() {
        return new SpartronicsEncoder() {
        
            @Override
            public void setPhase(boolean isReversed) {
            }
        
            @Override
            public double getVelocity() {
                return 0;
            }
        
            @Override
            public double getPosition() {
                return 0;
            }
        };
    }

    @Override
    public SensorModel getSensorModel() {
        return SensorModel.fromMultiplier(1);
    }

    @Override
    public boolean hadStartupError() {
        return false;
    }

    @Override
    public double getVoltageOutput() {
        return 0;
    }

    @Override
    public boolean getOutputInverted() {
        return false;
    }

    @Override
    public void setOutputInverted(boolean inverted) {

    }

    @Override
    public boolean getBrakeMode() {
        return false;
    }

    @Override
    public void setBrakeMode(boolean mode) {

    }

    @Override
    public double getVoltageCompSaturation() {
        return 0;
    }

    @Override
    public void setVoltageCompSaturation(double voltage) {

    }

    @Override
    public double getMotionProfileCruiseVelocity() {
        return 0;
    }

    @Override
    public void setMotionProfileCruiseVelocity(double velocityMetersPerSecond) {

    }

    @Override
    public double getMotionProfileMaxAcceleration() {
        return 0;
    }

    @Override
    public void setMotionProfileMaxAcceleration(double accelerationMetersSecSq) {

    }

    @Override
    public void setUseMotionProfileForPosition(boolean useMotionProfile) {

    }

    @Override
    public void setDutyCycle(double dutyCycle, double arbitraryFeedForwardVolts) {

    }

    @Override
    public void setDutyCycle(double dutyCycle) {

    }

    @Override
    public void setVelocity(double velocityMetersPerSecond) {

    }

    @Override
    public void setVelocity(double velocityMetersPerSecond, double arbitraryFeedForwardVolts) {

    }

    @Override
    public void setVelocityGains(double kP, double kD) {

    }

    @Override
    public void setVelocityGains(double kP, double kI, double kD, double kF) {

    }

    @Override
    public void setPosition(double positionMeters) {

    }

    @Override
    public void setPositionGains(double kP, double kD) {

    }

    @Override
    public void setPositionGains(double kP, double kI, double kD, double kF) {

    }

    @Override
    public void setNeutral() {

    }

}