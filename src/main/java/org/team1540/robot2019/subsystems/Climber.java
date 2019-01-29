package org.team1540.robot2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.commands.climber.JoystickClimberArms;

import static org.team1540.robot2019.Hardware.*;

public class Climber extends Subsystem {

    public void cylinderDown() {
        climberCylinder1.set(true);
        climberCylinder2.set(false);
    }

    public void cylinderUp() {
        climberCylinder1.set(false);
        climberCylinder2.set(true);
    }

    public void setArms(double value) {
        climberArmLeft.set(ControlMode.PercentOutput, -value);
//        climberArmRight.set(ControlMode.PercentOutput, value);
    }

    public double getCurrentLeft() {
        return climberArmLeft.getOutputCurrent();
    }

    public double getCurrentRight() {
        return climberArmRight.getOutputCurrent();
    }

    @Override
    protected void initDefaultCommand() {
    }

}
