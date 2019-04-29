package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class VelocityTankDrive extends Command {

    public VelocityTankDrive() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void execute() {
        double triggerValue = OI.getTankdriveForwardsAxis() - OI.getTankdriveBackwardsAxis();
        double leftSetpoint = (triggerValue - OI.getTankdriveLeftAxis()) * Tuning.drivetrainMaxVelocity;
        double rightSetpoint = (triggerValue - OI.getTankdriveRightAxis()) * Tuning.drivetrainMaxVelocity;
        Robot.drivetrain.setLeftVelocityTicks(leftSetpoint);
        Robot.drivetrain.setRightVelocityTicks(rightSetpoint);
        if (Robot.debugMode) {
            SmartDashboard.putNumber("debug-setpoint-left", leftSetpoint * 10 / Tuning.drivetrainTicksPerMeter);
            SmartDashboard.putNumber("debug-setpoint-right", rightSetpoint * 10 / Tuning.drivetrainTicksPerMeter);
            SmartDashboard.putNumber("debug-setpoint-left-raw", leftSetpoint);
            SmartDashboard.putNumber("debug-setpoint-right-raw", rightSetpoint);
            SmartDashboard.putNumber("debug-velocity-left", Robot.drivetrain.getLeftVelocityMetersPerSecond());
            SmartDashboard.putNumber("debug-velocity-right", Robot.drivetrain.getRightVelocityMetersPerSecond());
            SmartDashboard.putNumber("debug-velocity-left-raw", Robot.drivetrain.getLeftVelocityTicks());
            SmartDashboard.putNumber("debug-velocity-right-raw", Robot.drivetrain.getRightVelocityTicks());
            SmartDashboard.putNumber("debug-velocity-right-error", Hardware.driveRightMotorA.getClosedLoopError());
            SmartDashboard.putNumber("debug-velocity-left-error", Hardware.driveLeftMotorA.getClosedLoopError());
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
