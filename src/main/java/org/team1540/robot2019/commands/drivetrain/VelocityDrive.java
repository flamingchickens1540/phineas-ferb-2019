package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.subsystems.Drivetrain;

public class VelocityDrive extends Command {

  private final Drivetrain dt;

  public VelocityDrive(Drivetrain dt) {
    this.dt = dt;
    requires(dt);
//    dt.reset();
    dt.configTalonsForVelocity();
  }

  @Override
  protected void execute() {
//    double triggerValue = OI.getTankdriveForwardsAxis() - OI.getTankdriveBackwardsAxis();
//    double leftSetpoint = (triggerValue - OI.getTankdriveLeftAxis()) * Tuning.drivetrainMaxVelocity;
//    double rightSetpoint = (triggerValue - OI.getTankdriveRightAxis()) * Tuning.drivetrainMaxVelocity;
//    dt.setLeftVelocity(leftSetpoint);
//    dt.setRightVelocity(rightSetpoint);
    double triggerValue = OI.getTankdriveForwardsAxis() - OI.getTankdriveBackwardsAxis();
    double leftSetpoint = (triggerValue - OI.getTankdriveLeftAxis()) * Tuning.drivetrainMaxVelocity;
    double rightSetpoint = (triggerValue - OI.getTankdriveRightAxis()) * Tuning.drivetrainMaxVelocity;
    dt.setLeftVelocity(leftSetpoint);
    dt.setRightVelocity(rightSetpoint);
    SmartDashboard.putNumber("debug-setpoint-left", leftSetpoint*10/Tuning.driveTicksPerMeter);
    SmartDashboard.putNumber("debug-setpoint-right", rightSetpoint*10/Tuning.driveTicksPerMeter);
    SmartDashboard.putNumber("debug-setpoint-left-raw", leftSetpoint);
    SmartDashboard.putNumber("debug-setpoint-right-raw", rightSetpoint);
    SmartDashboard.putNumber("debug-velocity-left", dt.getLeftVelocityMetersPerSecond());
    SmartDashboard.putNumber("debug-velocity-right", dt.getRightVelocityMetersPerSecond());
    SmartDashboard.putNumber("debug-velocity-left-raw", dt.getLeftVelocity());
    SmartDashboard.putNumber("debug-velocity-right-raw", dt.getRightVelocity());
    SmartDashboard.putNumber("debug-velocity-right-error", Hardware.driveRightMotorA.getClosedLoopError());
    SmartDashboard.putNumber("debug-velocity-left-error", Hardware.driveLeftMotorA.getClosedLoopError());
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
