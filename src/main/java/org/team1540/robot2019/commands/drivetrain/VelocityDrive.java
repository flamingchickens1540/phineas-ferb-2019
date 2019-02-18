package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class VelocityDrive extends Command {

    public VelocityDrive() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        Robot.drivetrain.configTalonsForVelocity();
  }

  @Override
  protected void execute() {
    double triggerValue = OI.getTankdriveForwardsAxis() - OI.getTankdriveBackwardsAxis();
    double leftSetpoint = (triggerValue - OI.getTankdriveLeftAxis()) * Tuning.drivetrainMaxVelocity;
    double rightSetpoint = (triggerValue - OI.getTankdriveRightAxis()) * Tuning.drivetrainMaxVelocity;
      Robot.drivetrain.setLeftVelocityTPU(leftSetpoint);
      Robot.drivetrain.setRightVelocityTPU(rightSetpoint);
      SmartDashboard.putNumber("debug-setpoint-left", leftSetpoint * 10 / Tuning.drivetrainTicksPerMeter);
      SmartDashboard.putNumber("debug-setpoint-right", rightSetpoint * 10 / Tuning.drivetrainTicksPerMeter);
    SmartDashboard.putNumber("debug-setpoint-left-raw", leftSetpoint);
    SmartDashboard.putNumber("debug-setpoint-right-raw", rightSetpoint);
      SmartDashboard.putNumber("debug-velocity-left", Robot.drivetrain.getLeftVelocityMetersPerSecond());
      SmartDashboard.putNumber("debug-velocity-right", Robot.drivetrain.getRightVelocityMetersPerSecond());
      SmartDashboard.putNumber("debug-velocity-left-raw", Robot.drivetrain.getLeftVelocityTPU());
      SmartDashboard.putNumber("debug-velocity-right-raw", Robot.drivetrain.getRightVelocityTPU());
    SmartDashboard.putNumber("debug-velocity-right-error", Hardware.driveRightMotorA.getClosedLoopError());
    SmartDashboard.putNumber("debug-velocity-left-error", Hardware.driveLeftMotorA.getClosedLoopError());
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
