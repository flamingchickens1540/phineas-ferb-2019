package org.team1540.robot2019.datastructures.utils;

public class UnitsUtils {

    private static final double INCHES_PER_METER = 39.3701;
    private static final double INCHES_PER_FOOT = 12;
    private static final double DECISECONDS_PER_SECOND = 10;

    public static double inchesToMeters(double inches) {
        return inches / INCHES_PER_METER;
    }

    public static double metersToInches(double meters) {
        return meters * INCHES_PER_METER;
    }
}
