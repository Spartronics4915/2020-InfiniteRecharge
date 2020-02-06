package com.spartronics4915.frc2020.subsystems;

import com.spartronics4915.frc2020.commands.ClimberCommands;
import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.sim.DriverStationSim;

public class TestClimber
{
    @Test
    public static void testExtend()
    {
        (new ClimberCommands()).new Extend(new Climber());

        var sim = new DriverStationSim();
        sim.setAutonomous(false);
        sim.setEnabled(true);
    }
}