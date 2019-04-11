package org.team1540.robot2019.commands.drivetrain.pointdrive;

public interface PointAngleProvider {

    void initialize();

    PointControlConfig getPointControlConfig();

    double returnAngleError(double defaultError);
}
