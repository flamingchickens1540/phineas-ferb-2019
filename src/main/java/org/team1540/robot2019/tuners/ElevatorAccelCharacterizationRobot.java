package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.LinkedList;
import java.util.List;
import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.rooster.preferencemanager.PreferenceManager;

public class ElevatorAccelCharacterizationRobot extends TimedRobot {

  private static final int ACCEL_MEASUREMENT_WINDOW = 6;
  private static final int SETPOINT = 6;
  private Notifier notifier;

  @Override
  public void robotInit() {
    PreferenceManager.getInstance().add(new Tuning());
    Scheduler.getInstance().run(); // process preferences
    Hardware.initElevator();
    elevator = new Elevator();

    notifier = new Notifier(this::run);

    notifier.startPeriodic(0.01);
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run(); // process preferences
  }

  private static Elevator elevator;

  private Joystick joystick = new Joystick(0);

  private List<Double> velocities = new LinkedList<>();
  private List<Double> voltages = new LinkedList<>();
  private List<Double> times = new LinkedList<>();

  private PrintWriter csvWriter = null;

  private void run() {
    if (!isOperatorControl() && csvWriter != null) {
      csvWriter.close();
      csvWriter = null;
    }
    if (velocities.size() == ACCEL_MEASUREMENT_WINDOW) {
      velocities.remove(0);
      voltages.remove(0);
      times.remove(0);
    }
    double velocity = elevator.getVelocity();

    velocities.add(velocity);

    double accelCausingVoltage = elevator.getVoltage() - (Tuning.elevatorKV * velocity
        + Tuning.elevatorVIntercept);
    voltages.add(accelCausingVoltage);
    times.add((double) System.currentTimeMillis() / 1000.0);

    if (joystick.getRawButton(1)) { // if button A is pressed
      if (csvWriter == null) {
        // create a new CSV writer, reset everything
        try {
          File dir = new File("/home/lvuser/elevator/");
          if (!dir.exists()) {
            dir.mkdirs();
          }
          csvWriter = new PrintWriter(
              dir.toPath().resolve("measureaccel-" + System.currentTimeMillis() + ".csv").toFile());
          csvWriter.println("voltage,accel");
        } catch (FileNotFoundException e) {
          throw new RuntimeException(e);
        }
        elevator.setRaw(0);
      } else {
        if (velocities.size() == ACCEL_MEASUREMENT_WINDOW) {
          double accel = bestFitSlope(times, velocities);
          csvWriter.println(voltages.get(1) + "," + accel);
          System.out.println(velocities.toString());
          System.out.println(times.toString());
          System.out.println(accel);
        }

        elevator.setRaw(SETPOINT);
      }
    } else {
      if (csvWriter != null) {
        csvWriter.close();
        csvWriter = null;
      }
      elevator.setRaw(0);
    }
  }

  private static double bestFitSlope(List<Double> xVals, List<Double> yVals) {
    SimpleRegression reg = new SimpleRegression();
    for (int i = 0; i < xVals.size(); i++) {
      reg.addData(xVals.get(i), yVals.get(i));
    }
    return reg.getSlope();
  }
}
