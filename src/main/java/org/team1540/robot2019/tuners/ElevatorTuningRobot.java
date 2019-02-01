package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.rooster.preferencemanager.PreferenceManager;

public class ElevatorTuningRobot extends TimedRobot {

  private Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {
    Logger.getRootLogger().setLevel(Level.DEBUG);
    PreferenceManager.getInstance().add(new Tuning());
    Scheduler.getInstance().run();

    Hardware.initElevator();

    Robot.elevator = new Elevator();

    new JoystickButton(joystick, 1).whenPressed(new MoveElevatorToPosition(0));
    new JoystickButton(joystick, 2).whenPressed(new MoveElevatorToPosition(20));
    new JoystickButton(joystick, 3).whenPressed(new MoveElevatorToPosition(28));
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }
}
