package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class LiftGyroStabilizeLevel3 extends PIDCommand {

    public static double CLIMBER_GYRO_P = Tuning.climberGyroP;
    public static double CLIMBER_GYRO_I = Tuning.climberGyroI;
    public static double CLIMBER_GYRO_D = Tuning.climberGyroD;
    public static double CLIMBER_GYRO_GOAL = 0;

    public LiftGyroStabilizeLevel3() {
        super(0, 0, 0);
        SmartDashboard.putNumber("LiftGyroStabilizeLevel3/CLIMBER_GYRO_P", CLIMBER_GYRO_P);
        SmartDashboard.putNumber("LiftGyroStabilizeLevel3/CLIMBER_GYRO_I", CLIMBER_GYRO_I);
        SmartDashboard.putNumber("LiftGyroStabilizeLevel3/CLIMBER_GYRO_D", CLIMBER_GYRO_D);
        SmartDashboard.putNumber("LiftGyroStabilizeLevel3/CLIMBER_GYRO_GOAL", CLIMBER_GYRO_GOAL);

        requires(Robot.climber);
    }

    @Override
    protected void initialize() {
        CLIMBER_GYRO_P = SmartDashboard.getNumber("LiftGyroStabilizeLevel3/CLIMBER_GYRO_P", CLIMBER_GYRO_P);
        CLIMBER_GYRO_I = SmartDashboard.getNumber("LiftGyroStabilizeLevel3/CLIMBER_GYRO_I", CLIMBER_GYRO_I);
        CLIMBER_GYRO_D = SmartDashboard.getNumber("LiftGyroStabilizeLevel3/CLIMBER_GYRO_D", CLIMBER_GYRO_D);
        CLIMBER_GYRO_GOAL = SmartDashboard.getNumber("LiftGyroStabilizeLevel3/CLIMBER_GYRO_GOAL", CLIMBER_GYRO_GOAL);

        this.getPIDController().setPID(CLIMBER_GYRO_P, CLIMBER_GYRO_I, CLIMBER_GYRO_D);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected double returnPIDInput() {
        double rawPitchDegrees = Hardware.navx.getRawPitchDegrees() + CLIMBER_GYRO_GOAL;
        SmartDashboard.putNumber("LiftGyroStabilizeLevel3/rawPitchDegrees", rawPitchDegrees);
        return rawPitchDegrees;
    }

    @Override
    protected void usePIDOutput(double v) {
        if (Robot.climber.getArmsPosition() > Tuning.climberStartPosLevel3 || v > 0) {
            Robot.climber.setArms(v + Tuning.climberGyroFF);
        } else {
            Robot.climber.setArms(0);
        }
    }
}
