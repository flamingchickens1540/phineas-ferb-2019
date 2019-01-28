package org.team1540.robot2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.commands.climber.JoystickClimberArms;

import static org.team1540.robot2019.Hardware.*;

public class Climber extends Subsystem {

    public void legsDown() {
        climberCylinder1.set(true);
        climberCylinder2.set(false);
    }

    public void legsUp() {
        climberCylinder1.set(false);
        climberCylinder2.set(true);
    }

    public void setRawArms(double value) {
        climberArmLeft.set(ControlMode.PercentOutput, value);
        climberArmRight.set(ControlMode.PercentOutput, value);
    }

    @Override
    protected void initDefaultCommand() {
    }

}
