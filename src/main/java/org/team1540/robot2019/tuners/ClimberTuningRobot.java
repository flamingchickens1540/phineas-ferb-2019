package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.subsystems.Climber;
import org.team1540.robot2019.subsystems.Wrist;
import org.team1540.rooster.Utilities;
import org.team1540.rooster.util.SimpleCommand;

public class ClimberTuningRobot extends TimedRobot {

  private static Climber climber;
  private Joystick joystick = new Joystick(0);
  private JoystickButton button1 = new JoystickButton(joystick, 1);
  private JoystickButton button2 = new JoystickButton(joystick, 2);

  @Override
  public void robotInit() {
    Hardware.initWrist();

    climber = new Climber();
  }

  @Override
  public void teleopPeriodic() {
    climber.setRawArms(Utilities.processDeadzone(joystick.getRawAxis(1), 0.1));

    button1.whenPressed(new SimpleCommand("cylinder down", climber::legsDown));
    button2.whenPressed(new SimpleCommand("cylinder up", climber::legsUp));
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }
}
