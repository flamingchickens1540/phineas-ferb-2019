package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.subsystems.Climber;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.rooster.Utilities;

public class ClimberTuningRobot extends TimedRobot {

  private static Climber climber;
  private static Elevator elevator;
  private Joystick joystick = new Joystick(0);
  private Joystick joystick2 = new Joystick(1);
//  private JoystickButton button1 = new JoystickButton(joystick, 1);
//  private JoystickButton button2 = new JoystickButton(joystick, 2);

  @Override
  public void robotInit() {
    Hardware.initClimber();
    Hardware.initElevator();

    climber = new Climber();
    elevator = new Elevator();
  }

  @Override
  public void teleopPeriodic() {
    climber.setArms(Utilities.processDeadzone(joystick.getRawAxis(5), 0.1));

    elevator.setRaw(Utilities.processDeadzone(joystick2.getRawAxis(1), 0.1) / 5);
//    elevator.setRaw(0.04);

    SmartDashboard.putNumber("climber current left", climber.getCurrentLeft());
    SmartDashboard.putNumber("climber current right", climber.getCurrentRight());

//    button1.whenPressed(new SimpleCommand("cylinder down", climber::cylinderDown));
//    button2.whenPressed(new SimpleCommand("cylinder up", climber::cylinderUp));
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }
}
