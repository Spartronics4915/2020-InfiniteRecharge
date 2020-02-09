
// package com.spartronics4915.frc2020.subsystems;

// import static org.junit.jupiter.api.Assertions.assertFalse;
// import static org.junit.jupiter.api.Assertions.assertTrue;

// import com.spartronics4915.frc2020.commands.ClimberCommands;
// import com.spartronics4915.lib.hardware.motors.SpartronicsSimulatedMotor;

// import org.junit.jupiter.api.Test;

// import edu.wpi.first.hal.sim.DriverStationSim;

// public class TestClimber
// {
//     @Test
//     public void testExtend()
//     {
//         var commandToRun = new ClimberCommands().new WinchPrimary(new Climber());

//         var sim = new DriverStationSim();
//         // sim.setAutonomous(false);
//         // sim.setEnabled(true);

//         commandToRun.schedule();

//         var simmedMotor = SpartronicsSimulatedMotor.getFromId(6);
//         assertTrue(commandToRun.isScheduled());
//         simmedMotor.setOutputCurrent(11);
//         assertFalse(commandToRun.isScheduled());
//     }
// }