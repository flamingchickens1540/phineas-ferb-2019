package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;

public class LiftGyroStabilizeLevel2 extends Command {

    public static double ANGLE = 0;

    public LiftGyroStabilizeLevel2() {
        SmartDashboard.putNumber("Level2Angle", ANGLE);
        requires(Robot.climber);
    }

    @Override
    protected void initialize() {
        ANGLE = SmartDashboard.getNumber("Level2Angle", ANGLE);
    }

    @Override
    protected void execute() {
        if (Hardware.navx.getRawPitchDegrees() > ANGLE) {
            Robot.climber.lowerCylinder();
        } else {
            Robot.climber.raiseCylinder();
        }

//        if (Robot.climber.getArmsPosition() < Tuning.climberArmsFwdLimit) {
//        Robot.climber.setArms(OI.getManualClimberArmsAxis());
//        } else {
//            Robot.climber.setArms(0);
//        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}
