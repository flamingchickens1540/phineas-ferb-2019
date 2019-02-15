package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.wrist.LowerWrist;
import org.team1540.robot2019.subsystems.Wrist;
import org.team1540.rooster.preferencemanager.PreferenceManager;

public class WristLogicTuningRobot extends TimedRobot {

  private Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {
    Logger.getRootLogger().setLevel(Level.DEBUG);
    PreferenceManager.getInstance().add(new Tuning());
    Scheduler.getInstance().run();

    Hardware.initWrist();
    Hardware.initCompressor();

    Robot.wrist = new Wrist();

    new JoystickButton(joystick, 1).whenPressed(new CommandGroup() {
      {
        addSequential(new LowerWrist());
        addSequential(new TimedCommand(1));
      }
    });
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
  }

}
