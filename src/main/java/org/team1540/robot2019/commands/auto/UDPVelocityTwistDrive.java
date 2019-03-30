package org.team1540.robot2019.commands.auto;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.twod.Transform2D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.networking.TEBConfig;

// TODO: Use proper logging class
public class UDPVelocityTwistDrive extends Command {

    private static final Logger logger = Logger.getLogger(UDPVelocityTwistDrive.class);

    public UDPVelocityTwistDrive() {
        SmartDashboard.setDefaultNumber("test-goal/position/x", 2);
        SmartDashboard.setDefaultNumber("test-goal/position/y", 0);
        SmartDashboard.setDefaultNumber("test-goal/orientation/z", 0);

        requires(Robot.drivetrain);
        requires(Robot.tebPlanner);

//        TankDriveTwist2DInput twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters); // TODO: Test closed loop
//        Executable pipeline = twist2DInput
//            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
//            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
//            .then(new CTREOutput(Hardware.driveLeftMotorA, Hardware.driveRightMotorA, true));
    }

    @Override
    protected void initialize() {
        double xGoal = SmartDashboard.getNumber("test-goal/position/x", 1);
        double yGoal = SmartDashboard.getNumber("test-goal/position/y", 0);
        double angleGoal = SmartDashboard.getNumber("test-goal/orientation/z", 0);
        System.out.println("Updated goal!");

        Transform2D goal = Robot.odometry.getOdomToBaseLink().toTransform2D().add(new Transform2D(xGoal, yGoal, angleGoal));
        Robot.tebPlanner.setGoal(goal);

        TEBConfig cfg = new TEBConfig(); // placeholder
        Robot.tebPlanner.setCfg(cfg);

        try {
            Robot.tebPlanner.sendGoalAndConfig();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void execute() {
        Twist2D cmdVel = Robot.tebPlanner.getCmdVel();
        Robot.drivetrain.setTwist(cmdVel);

//    twist2DInput.setTwist(cmdVel); // TODO: Test closed loop
//    pipeline.execute();

        if (Robot.debugMode) {
            double leftSetpoint = (cmdVel.getX() - cmdVel.getOmega() * Tuning.drivetrainRadiusMeters);
            double rightSetpoint = (cmdVel.getX() + cmdVel.getOmega() * Tuning.drivetrainRadiusMeters);
            SmartDashboard.putNumber("debug-setpoint-left", leftSetpoint * 10 / Tuning.drivetrainTicksPerMeter);
            SmartDashboard.putNumber("debug-setpoint-right", rightSetpoint * 10 / Tuning.drivetrainTicksPerMeter);
            SmartDashboard.putNumber("debug-velocity-left", Robot.drivetrain.getLeftVelocityMetersPerSecond());
            SmartDashboard.putNumber("debug-velocity-right", Robot.drivetrain.getRightVelocityMetersPerSecond());
            NetworkTableInstance.getDefault().flush();
        }
    }

    @Override
    protected boolean isFinished() {
        double distanceError = Robot.odometry.getOdomToBaseLink().toTransform2D().getPositionVector().distance(Robot.tebPlanner.getGoal().getPositionVector()); // TODO: This should use javaTF
        double angleError = TrigUtils.signedAngleError(Hardware.navx.getYawRadians(), Robot.tebPlanner.getGoal().getTheta());

        return distanceError < 0.1 && Math.abs(angleError) < Math.toRadians(10);
    }

    @Override
    protected void end() {
        logger.debug("Goal reached!");
        Robot.drivetrain.stop();
    }
}
