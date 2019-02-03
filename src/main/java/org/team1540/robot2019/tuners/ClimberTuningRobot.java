package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.subsystems.Climber;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.rooster.Utilities;
import org.team1540.rooster.util.SimpleCommand;

public class ClimberTuningRobot extends TimedRobot {

  private static Climber climber;
  private Joystick joystick = new Joystick(0);
  private JoystickButton button1 = new JoystickButton(joystick, 1);
  private JoystickButton button2 = new JoystickButton(joystick, 2);

  @Override
  public void robotInit() {
    Hardware.initClimber();

    climber = new Climber();
  }

  @Override
  public void teleopPeriodic() {
//    climber.setArms(-Utilities.processDeadzone(joystick.getRawAxis(5), 0.1));

//    elevator.setRaw(Utilities.processDeadzone(joystick2.getRawAxis(1), 0.1) / 5);
//    elevator.setRaw(0.04);

    SmartDashboard.putNumber("climber current left", climber.getCurrentLeft());
    SmartDashboard.putNumber("climber current right", climber.getCurrentRight());

//    button1.whenPressed(new MoveElevatorToPosition(Tuning.elevatorCargoShipPosition));
    button1.whenPressed(new SimpleCommand("start climbing", climber::startClimbing));
    button2.whenPressed(new SimpleCommand("retract cylinder", climber::onPlatform));
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }
}
