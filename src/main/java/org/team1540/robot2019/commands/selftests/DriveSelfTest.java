package org.team1540.robot2019.commands.selftests;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.stat.descriptive.SummaryStatistics;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.rooster.wrappers.ChickenController;

public class DriveSelfTest extends Command {
    private static final Logger logger = Logger.getLogger(DriveSelfTest.class);

    private boolean finished = false;
    private int currentMotor;
    private List<Double> motorAvgCurrents = new ArrayList<>(6);
    private List<Double> motorMaxCurrents = new ArrayList<>(6);
    private List<Double> motorAvgVels = new ArrayList<>(6);
    private SummaryStatistics currentMotorStatsCurrent = new SummaryStatistics();
    private SummaryStatistics currentMotorStatsVelocity = new SummaryStatistics();
    private Timer runTimer = new Timer();
    private Timer waitTimer = new Timer();

    private boolean motorRunning;

    public DriveSelfTest() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        logger.info("Starting drivetrain self test");
        motorAvgCurrents.clear();
        motorMaxCurrents.clear();
        motorAvgVels.clear();

        resetPerMotorStats();

        currentMotor = 0;
        finished = false;
        motorRunning = false;
        runTimer.start();
        Robot.drivetrain.splitFollowers();
    }

    @Override
    protected void execute() {
        if (runTimer.get() > (Tuning.driveTestTime)) {
            Hardware.driveMotorAll[currentMotor].neutralOutput();

            motorMaxCurrents.add(currentMotorStatsCurrent.getMax());
            motorAvgCurrents.add(currentMotorStatsCurrent.getMean());
            motorAvgVels.add(currentMotorStatsVelocity.getMean());

            resetPerMotorStats();

            if (currentMotor < 5) {
                currentMotor++;
                logger.info("Testing motor " + getName(currentMotor));
            } else {
                finished = true;
            }
            runTimer.stop();
            runTimer.reset();
            motorRunning = false;
            Robot.drivetrain.setBrake(true);
        } else {
            if (Math.abs(getVel(currentMotor)) < Tuning.driveTestStopTolerance && !motorRunning) {
                runTimer.start();
                ChickenController motor = Hardware.driveMotorAll[currentMotor];

                motor.set(ControlMode.PercentOutput, Tuning.driveTestMotorThrot);
                motorRunning = true;
                Robot.drivetrain.setBrake(false);
            }

            if (motorRunning) {
                currentMotorStatsCurrent.addValue(getCurrent(currentMotor));
                currentMotorStatsVelocity.addValue(getVel(currentMotor));
            }
        }
    }

    @Override
    protected void end() {
        logger.info("Drive self-test complete");

        String avgCurrentInfo = constructMotorStatString(motorAvgCurrents, "A");
        logger.info("Avg currents were " + avgCurrentInfo);

        String maxCurrentInfo = constructMotorStatString(motorMaxCurrents, "A");
        logger.info("Max currents were " + maxCurrentInfo);

        String avgVelInfo = constructMotorStatString(motorAvgVels, " m/s");
        logger.info("Avg velocities were " + avgVelInfo);

        Robot.drivetrain.joinFollowers();
        Robot.drivetrain.setBrake(true);
    }

    @Override
    protected void interrupted() {
        Robot.drivetrain.joinFollowers();
        Robot.drivetrain.setBrake(true);
    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

    private void resetPerMotorStats() {
        currentMotorStatsCurrent.clear();
        currentMotorStatsVelocity.clear();
    }

    private static double getVel(int motorIndex) {
        return motorIndex < 3 ? Robot.drivetrain.getLeftVelocityMetersPerSecond()
            : Robot.drivetrain.getRightVelocityMetersPerSecond();
    }

    private static double getCurrent(int motorIndex) {
        switch (motorIndex) {
            case 0:
                return Hardware.getDriveLeftACurrent();
            case 1:
                return Hardware.getDriveLeftBCurrent();
            case 2:
                return Hardware.getDriveLeftCCurrent();
            case 3:
                return Hardware.getDriveRightACurrent();
            case 4:
                return Hardware.getDriveRightBCurrent();
            case 5:
                return Hardware.getDriveRightCCurrent();
        }
        logger.error("Current was requested for non-existent drive motor index " + motorIndex);
        return 0;
    }

    private static String getName(int motorIndex) {
        switch (motorIndex) {
            case 0:
                return "LA";
            case 1:
                return "LB";
            case 2:
                return "LC";
            case 3:
                return "RA";
            case 4:
                return "RB";
            case 5:
                return "RC";
        }
        logger.error("Name was requested for non-existent drive motor index " + motorIndex);
        return "??";
    }

    private String constructMotorStatString(List<Double> stats, String unit) {
        StringBuilder builder = new StringBuilder();

        for (int i = 0; i < 6; i++) {
            builder.append(getName(i)).append(" ").append(stats.get(i)).append(unit);

            if (i < 5) {
                builder.append("; ");
            }
        }

        return builder.toString();
    }
}
