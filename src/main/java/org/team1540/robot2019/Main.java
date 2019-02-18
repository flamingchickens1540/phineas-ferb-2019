package org.team1540.robot2019;

import edu.wpi.first.wpilibj.RobotBase;

public class Main {

    public static void main(String[] args) {
        if (RobotBase.isSimulation()) {
            // workaround for java simulator not catching standard error
            System.setErr(System.out);
        }

        RobotBase.startRobot(Robot::new);
    }
}
