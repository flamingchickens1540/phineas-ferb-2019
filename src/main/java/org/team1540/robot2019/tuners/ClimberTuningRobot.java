package org.team1540.robot2019.tuners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.ShuffleboardDisplay;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.climber.ClimbLevelThreeGyro;
import org.team1540.robot2019.commands.climber.PrepareForClimb;
import org.team1540.robot2019.subsystems.Climber;
import org.team1540.robot2019.subsystems.Drivetrain;
import org.team1540.robot2019.subsystems.Elevator;
import org.team1540.rooster.preferencemanager.PreferenceManager;
import org.team1540.rooster.util.SimpleCommand;

public class ClimberTuningRobot extends TimedRobot {

    private Joystick joystick = new Joystick(1);
    private JoystickButton button1 = new JoystickButton(joystick, 1);
    private JoystickButton button2 = new JoystickButton(joystick, 2);
    private JoystickButton button3 = new JoystickButton(joystick, 3);
    private JoystickButton button4 = new JoystickButton(joystick, 4);

    @Override
    public void robotInit() {
        PreferenceManager.getInstance().add(new Tuning());

        Scheduler.getInstance().run();

        Hardware.initClimber();
        Hardware.initNavX();
        Hardware.initElevator();
        Hardware.initDrive();
        Hardware.initPressureSensor();

        ShuffleboardDisplay.init();

        Robot.climber = new Climber();
        Robot.elevator = new Elevator();
        Robot.drivetrain = new Drivetrain();

        button1.whenPressed(new ClimbLevelThreeGyro());
        button2.whenPressed(new PrepareForClimb());
        button3
            .whenPressed(new SimpleCommand("Raise Cylinder", Robot.climber::cylinderUp, Robot.climber));
        button3.whenPressed(new SimpleCommand("stahp", () -> Robot.climber.setArms(0), Robot.climber));
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();
    }
}
