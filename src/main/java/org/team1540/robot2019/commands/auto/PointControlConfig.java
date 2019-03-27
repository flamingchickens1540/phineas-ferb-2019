package org.team1540.robot2019.commands.auto;

public class PointControlConfig {

    public double OUTPUT_SCALAR = 20;

    // Max/Min angular velocity
    private double MIN = 0;
    private double MAX = 10;
    private double DEADZONE = 0.05;

    // Constants for angular VPID controller
    private double P = 0.39;
    private double I = 0;
    private double D = 0.6;

    private double THROTTLE_CONSTANT = 3;

    public PointControlConfig(double OUTPUT_SCALAR, double MIN, double MAX, double DEADZONE, double p, double i, double d, double THROTTLE_CONSTANT) {
        this.OUTPUT_SCALAR = OUTPUT_SCALAR;
        this.MIN = MIN;
        this.MAX = MAX;
        this.DEADZONE = DEADZONE;
        P = p;
        I = i;
        D = d;
        this.THROTTLE_CONSTANT = THROTTLE_CONSTANT;
    }

    public double getOUTPUT_SCALAR() {
        return OUTPUT_SCALAR;
    }

    public double getMIN() {
        return MIN;
    }

    public double getMAX() {
        return MAX;
    }

    public double getDEADZONE() {
        return DEADZONE;
    }

    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }

    public double getTHROTTLE_CONSTANT() {
        return THROTTLE_CONSTANT;
    }
}
