package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.wrist.LowerWrist;
import org.team1540.robot2019.commands.wrist.RaiseWrist;
import org.team1540.robot2019.subsystems.Wrist;

public class WristLogicTuningRobot extends TimedRobot {

  private Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {
    Hardware.initWrist();

    Robot.wrist = new Wrist();

    new JoystickButton(joystick, 1).whenPressed(new LowerWrist());
    new JoystickButton(joystick, 2).whenPressed(new RaiseWrist());
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

}
