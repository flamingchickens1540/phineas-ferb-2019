package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.subsystems.*;
import org.team1540.rooster.Utilities;

public class ElevatorTuningRobot extends TimedRobot {

  private static Elevator elevator;
  private Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {
    Hardware.initElevator();

    elevator = new Elevator();

    elevator.setEnableController(false);
  }

  @Override
  public void teleopPeriodic() {
    elevator.setRaw(Utilities.processDeadzone(joystick.getRawAxis(1), 0.1));
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }
}
