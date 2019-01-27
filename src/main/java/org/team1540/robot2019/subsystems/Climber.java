package org.team1540.robot2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.commands.climber.JoystickClimberArms;

import static org.team1540.robot2019.Hardware.climberArmLeft;
import static org.team1540.robot2019.Hardware.climberArmRight;

public class Climber extends Subsystem {

    public void joystickArms(double value) {
        climberArmLeft.set(ControlMode.PercentOutput, value);
        climberArmRight.set(ControlMode.PercentOutput, value);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new JoystickClimberArms());
    }

}
