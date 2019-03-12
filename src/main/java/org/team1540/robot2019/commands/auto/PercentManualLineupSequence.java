package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.drivetrain.PointDrive;

public class PercentManualLineupSequence extends CommandGroup {

    private PercentManualLineup command;

    public PercentManualLineupSequence() {
//        addSequential(new WaitCommand(0.05)); // Wait for leds to turn on

        command = new PercentManualLineup();
        addSequential(command);

        SmartDashboard.setDefaultNumber("PercentLineup/ANGULAR_KP", PercentManualLineup.ANGULAR_KP); // TODO: Remove temporary tuning (yaml ftw)
        SmartDashboard.setDefaultNumber("PercentLineup/ANGULAR_KI", PercentManualLineup.ANGULAR_KI);
        SmartDashboard.setDefaultNumber("PercentLineup/ANGULAR_KD", PercentManualLineup.ANGULAR_KD);
        SmartDashboard.setDefaultNumber("PercentLineup/MIN_VEL_THETA", PercentManualLineup.MIN_VEL_THETA);
        SmartDashboard.setDefaultNumber("PercentLineup/DEADZONE_VEL_THETA", PercentManualLineup.DEADZONE_VEL_THETA);
        SmartDashboard.setDefaultNumber("PercentLineup/MAX_VEL_THETA", PercentManualLineup.MAX_VEL_THETA);
        SmartDashboard.setDefaultNumber("PercentLineup/ANGLE_OFFSET", PercentManualLineup.ANGLE_OFFSET);
    }

    @Override
    protected void initialize() {
        PercentManualLineup.ANGULAR_KP = SmartDashboard.getNumber("PercentLineup/ANGULAR_KP", PercentManualLineup.ANGULAR_KP); // TODO: Remove temporary tuning (yaml ftw)
        PercentManualLineup.ANGULAR_KI = SmartDashboard.getNumber("PercentLineup/ANGULAR_KI", PercentManualLineup.ANGULAR_KI);
        PercentManualLineup.ANGULAR_KD = SmartDashboard.getNumber("PercentLineup/ANGULAR_KD", PercentManualLineup.ANGULAR_KD);
        PercentManualLineup.MIN_VEL_THETA = SmartDashboard.getNumber("PercentLineup/MIN_VEL_THETA", PercentManualLineup.MIN_VEL_THETA);
        PercentManualLineup.DEADZONE_VEL_THETA = SmartDashboard.getNumber("PercentLineup/DEADZONE_VEL_THETA", PercentManualLineup.DEADZONE_VEL_THETA);
        PercentManualLineup.MAX_VEL_THETA = SmartDashboard.getNumber("PercentLineup/MAX_VEL_THETA", PercentManualLineup.MAX_VEL_THETA);
        PercentManualLineup.ANGLE_OFFSET = SmartDashboard.getNumber("PercentLineup/ANGLE_OFFSET", PercentManualLineup.ANGLE_OFFSET);

        command = new PercentManualLineup();

        if (SmartDashboard.getBoolean("TurnOffLimelightWhenNotInUse", true)) {
            Robot.limelight.prepForVision();
        }
    }

    @Override
    protected void interrupted() {
        end(); // CommandGroups don't call end by default on interrupted
    }

    @Override
    protected void end() {
        PointDrive.manualResetGoal();
        if (SmartDashboard.getBoolean("TurnOffLimelightWhenNotInUse", true)) {
            Robot.limelight.prepForDriverCam();
        }
    }
}
