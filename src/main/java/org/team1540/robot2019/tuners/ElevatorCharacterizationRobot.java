package org.team1540.robot2019.tuners;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.rooster.preferencemanager.PreferenceManager;

public class ElevatorCharacterizationRobot extends TimedRobot {

  private static Elevator elevator;

  private Joystick joystick;

  private SimpleRegression regression = new SimpleRegression();

  private double setpoint;
  private boolean running;
  private static final double RAMP_RATE = 0.25; // volts per second

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("elevatorChar");
  private NetworkTableEntry slopeEntry = table.getEntry("slope");
  private NetworkTableEntry interceptEntry = table.getEntry("intercept");
  private NetworkTableEntry rSquaredEntry = table.getEntry("rSquared");


  @Override
  public void robotInit() {
    PreferenceManager.getInstance().add(new Tuning());

    Scheduler.getInstance().run();
    Hardware.initElevator();

    joystick = new Joystick(0);

    elevator = new Elevator();

    elevator.setEnableController(false);
  }

  @Override
  public void teleopInit() {
    if (joystick.getRawButton(1)) {
      if (!running) {
        running = true;
        regression.clear();
        setpoint = 0;
        elevator.setRaw(0);
      } else {
        setpoint +=
            (RAMP_RATE / 12) * 0.02; // 20ms loop time and the elevator takes throttle not volts
        elevator.setRaw(setpoint);

        if (elevator.getVelocity() != 0) {
          regression.addData(elevator.getVelocity(), elevator.getThrottle());
        }
      }
    } else {
      running = false;
      setpoint = 0;
      elevator.setRaw(0);
    }
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();

    if (!Double.isNaN(regression.getSlope())) {
      slopeEntry.forceSetNumber(regression.getSlope());
      interceptEntry.forceSetNumber(regression.getIntercept());
      rSquaredEntry.forceSetNumber(regression.getRSquare());
    } else {
      slopeEntry.forceSetNumber(0);
      interceptEntry.forceSetNumber(0);
      rSquaredEntry.forceSetNumber(0);
    }
  }
}
