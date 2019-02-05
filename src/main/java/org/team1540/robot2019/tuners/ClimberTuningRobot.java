package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.subsystems.Climber;
import org.team1540.robot2019.subsystems.Drivetrain;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.rooster.Utilities;
import org.team1540.rooster.preferencemanager.PreferenceManager;
import org.team1540.rooster.util.SimpleCommand;

public class ClimberTuningRobot extends TimedRobot {

  private static Climber climber;
  private Joystick joystick = new Joystick(0);
  private JoystickButton buttonA = new JoystickButton(joystick, 1);
  private JoystickButton buttonB = new JoystickButton(joystick, 2);
  private JoystickButton buttonX = new JoystickButton(joystick, 3);
  private JoystickButton buttonY = new JoystickButton(joystick, 4);
  private JoystickButton buttonSt = new JoystickButton(joystick, 8);


  @Override
  public void robotInit() {
    Hardware.initClimber();
    Hardware.initElevator();
    Hardware.initDrive();

    climber = new Climber();
    Robot.elevator = new Elevator();
    Robot.drivetrain = new Drivetrain();

    PreferenceManager.getInstance().add(new Tuning());
    Scheduler.getInstance().run();

    new MoveElevatorToPosition(Tuning.elevatorDownPosition).start();
  }

  @Override
  public void teleopInit() {
//    new MoveElevatorToPosition(Tuning.elevatorDownPosition).start();
  }

  @Override
  public void teleopPeriodic() {
    climber.setArms(-Utilities.processDeadzone(joystick.getRawAxis(5), 0.1));

    double drive = -Utilities.processDeadzone(joystick.getRawAxis(2), 0.1);
    Robot.drivetrain.setThrottle(drive, drive);

//    elevator.setRaw(Utilities.processDeadzone(joystick2.getRawAxis(1), 0.1) / 5);
//    elevator.setRaw(0.04);

    SmartDashboard.putNumber("climber current left", climber.getCurrentLeft());
    SmartDashboard.putNumber("climber current right", climber.getCurrentRight());

//    buttonA.whenPressed(new MoveElevatorToPosition(Tuning.elevatorUpPosition));
    buttonA.whenPressed(new SimpleCommand("start climbing", climber::startClimbing));
    buttonB.whenPressed(new SimpleCommand("retract cylinder", climber::onPlatform));

    buttonX.whenPressed(new MoveElevatorToPosition(Tuning.elevatorUpPosition));
    buttonY.whenPressed(new MoveElevatorToPosition(Tuning.elevatorDownPosition));
//    buttonSt.whenPressed(new SimpleCommand("arms back", climber::backward));
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }
}
