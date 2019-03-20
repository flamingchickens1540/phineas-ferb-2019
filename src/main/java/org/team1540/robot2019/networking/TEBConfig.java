package org.team1540.robot2019.networking;

public class TEBConfig {

    private double maxVelX = 1.5;
    private double maxVelXBackwards = 1.5;
    private double accLimX = 1.5;
    private double maxVelTheta = 5.0;
    private double accLimTheta = 15.0;
    private double minTurningRadius = 0;
    private double weightKinematicsForwardDrive = 0.7;

    public TEBConfig() {

    }

    public TEBConfig(double maxVelX, double maxVelXBackwards, double accLimX, double maxVelTheta, double accLimTheta, double minTurningRadius, double weightKinematicsForwardDrive) {
        this.maxVelX = maxVelX;
        this.maxVelXBackwards = maxVelXBackwards;
        this.accLimX = accLimX;
        this.maxVelTheta = maxVelTheta;
        this.accLimTheta = accLimTheta;
        this.minTurningRadius = minTurningRadius;
        this.weightKinematicsForwardDrive = weightKinematicsForwardDrive;
    }

    public double getMaxVelX() {
        return maxVelX;
    }

    public void setMaxVelX(double maxVelX) {
        this.maxVelX = maxVelX;
    }

    public double getMaxVelXBackwards() {
        return maxVelXBackwards;
    }

    public void setMaxVelXBackwards(double maxVelXBackwards) {
        this.maxVelXBackwards = maxVelXBackwards;
    }

    public double getAccLimX() {
        return accLimX;
    }

    public void setAccLimX(double accLimX) {
        this.accLimX = accLimX;
    }

    public double getMaxVelTheta() {
        return maxVelTheta;
    }

    public void setMaxVelTheta(double maxVelTheta) {
        this.maxVelTheta = maxVelTheta;
    }

    public double getAccLimTheta() {
        return accLimTheta;
    }

    public void setAccLimTheta(double accLimTheta) {
        this.accLimTheta = accLimTheta;
    }

    public double getMinTurningRadius() {
        return minTurningRadius;
    }

    public void setMinTurningRadius(double minTurningRadius) {
        this.minTurningRadius = minTurningRadius;
    }

    public double getWeightKinematicsForwardDrive() {
        return weightKinematicsForwardDrive;
    }

    public void setWeightKinematicsForwardDrive(double weightKinematicsForwardDrive) {
        this.weightKinematicsForwardDrive = weightKinematicsForwardDrive;
    }
}
