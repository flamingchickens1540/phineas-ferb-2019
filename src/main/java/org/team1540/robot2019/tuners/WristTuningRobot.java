package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.subsystems.Wrist;
import org.team1540.rooster.Utilities;

public class WristTuningRobot extends TimedRobot {

  private static Wrist wrist;
  private Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {
    Hardware.initWrist();

    wrist = new Wrist();
  }

  @Override
  public void teleopPeriodic() {
    wrist.set(Utilities.processDeadzone(joystick.getRawAxis(1), 0.1));

    wrist.setCylinder(joystick.getRawButton(1));
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }
}
