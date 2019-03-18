package org.team1540.robot2019.utils;

import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import java.util.HashMap;
import java.util.Map;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2019.Tuning;
import org.team1540.rooster.Utilities;
import org.team1540.rooster.triggers.AxisButton;
import org.team1540.rooster.triggers.DPadAxis;
import org.team1540.rooster.triggers.MultiAxisButton;
import org.team1540.rooster.triggers.StrictDPadButton;

public class ChickenXboxController {

    public enum XboxAxis {
        LEFT_X(0),
        LEFT_Y(1),
        LEFT_TRIG(2),
        RIGHT_TRIG(3),
        RIGHT_X(4),
        RIGHT_Y(5);

        @SuppressWarnings("MemberName")
        public final int value;
        @SuppressWarnings("PMD.UseConcurrentHashMap")
        private static final Map<Integer, XboxAxis> map = new HashMap<>();

        XboxAxis(int value) {
            this.value = value;
        }

        static {
            for (XboxAxis axisId : XboxAxis.values()) {
                map.put(axisId.value, axisId);
            }
        }

        public static XboxAxis of(int value) {
            return map.get(value);
        }
    }

    public enum XboxButton { // Hmm, I know about XboxController.Button but like...
        A(1),
        B(2),
        X(3),
        Y(4),
        LB(5),
        RB(6),
        BACK(7),
        START(8),
        LEFT_PRESS(9),
        RIGHT_PRESS(10);

        @SuppressWarnings("MemberName")
        public final int value;
        @SuppressWarnings("PMD.UseConcurrentHashMap")
        private static final Map<Integer, XboxButton> map = new HashMap<>();

        XboxButton(int value) {
            this.value = value;
        }

        static {
            for (XboxButton axisId : XboxButton.values()) {
                map.put(axisId.value, axisId);
            }
        }

        public static XboxButton of(int value) {
            return map.get(value);
        }
    }

    private XboxController controller;

    public ChickenXboxController(int port) {
        this.controller = new XboxController(port);
    }

    public double getX(Hand hand) {
        return controller.getX(hand);
    }

    public double getY(Hand hand) {
        return controller.getY(hand);
    }

    /**
     * Gets angle from a 2D joystick
     * @param hand Left vs right joystick of the xbox controller
     * @return Angle in radians counter-clockwise from 12 o'clock
     */
    public double get2DJoystickAngle(Hand hand) { // TODO: Migrate to ROOSTER
        double x = -controller.getY(hand);
        double y = -controller.getX(hand);
        return Math.atan2(y, x);
    }

    public double get2DJoystickMagnitude(Hand hand) { // TODO: Migrate to ROOSTER
        double x = controller.getX(hand);
        double y = controller.getY(hand);
        return Utilities.processDeadzone(new Vector2D(x, y).distance(Vector2D.ZERO), Tuning.driveDeadzone);
    }

    public StrictDPadButton getDPadButton(DPadAxis button) {
        return new StrictDPadButton(controller, 0, button);
    }

    public JoystickButton getButton(XboxButton button) {
        return new JoystickButton(controller, button.value);
    }

    public AxisButton getAxisButton(double threshold, XboxAxis axis) {
        return new AxisButton(controller, threshold, axis.value);
    }

    public XboxController getController() {
        return controller;
    }

    public MultiAxisButton getMultiAxisButton(double threshold, XboxAxis[] axes) {
        int[] axesIds = new int[axes.length];
        for (int i = 0; i < axes.length; i++)
            axesIds[i] = axes[i].value;
        return new MultiAxisButton(controller, threshold, axesIds);
    }

    // Delegated methods

    public double getTriggerAxis(Hand hand) {
        return controller.getTriggerAxis(hand);
    }

    public boolean getBumper(Hand hand) {
        return controller.getBumper(hand);
    }

    public boolean getBumperPressed(Hand hand) {
        return controller.getBumperPressed(hand);
    }

    public boolean getBumperReleased(Hand hand) {
        return controller.getBumperReleased(hand);
    }

    public boolean getStickButton(Hand hand) {
        return controller.getStickButton(hand);
    }

    public boolean getStickButtonPressed(Hand hand) {
        return controller.getStickButtonPressed(hand);
    }

    public boolean getStickButtonReleased(Hand hand) {
        return controller.getStickButtonReleased(hand);
    }

    public boolean getAButton() {
        return controller.getAButton();
    }

    public boolean getAButtonPressed() {
        return controller.getAButtonPressed();
    }

    public boolean getAButtonReleased() {
        return controller.getAButtonReleased();
    }

    public boolean getBButton() {
        return controller.getBButton();
    }

    public boolean getBButtonPressed() {
        return controller.getBButtonPressed();
    }

    public boolean getBButtonReleased() {
        return controller.getBButtonReleased();
    }

    public boolean getXButton() {
        return controller.getXButton();
    }

    public boolean getXButtonPressed() {
        return controller.getXButtonPressed();
    }

    public boolean getXButtonReleased() {
        return controller.getXButtonReleased();
    }

    public boolean getYButton() {
        return controller.getYButton();
    }

    public boolean getYButtonPressed() {
        return controller.getYButtonPressed();
    }

    public boolean getYButtonReleased() {
        return controller.getYButtonReleased();
    }

    public boolean getBackButton() {
        return controller.getBackButton();
    }

    public boolean getBackButtonPressed() {
        return controller.getBackButtonPressed();
    }

    public boolean getBackButtonReleased() {
        return controller.getBackButtonReleased();
    }

    public boolean getStartButton() {
        return controller.getStartButton();
    }

    public boolean getStartButtonPressed() {
        return controller.getStartButtonPressed();
    }

    public boolean getStartButtonReleased() {
        return controller.getStartButtonReleased();
    }

    public double getX() { // Don't use this
        return controller.getX();
    }

    public double getY() { // Don't use this
        return controller.getY();
    }

    public boolean getRawButton(int button) {
        return controller.getRawButton(button);
    }

    public boolean getRawButtonPressed(int button) {
        return controller.getRawButtonPressed(button);
    }

    public boolean getRawButtonReleased(int button) {
        return controller.getRawButtonReleased(button);
    }

    public double getRawAxis(int axis) {
        return controller.getRawAxis(axis);
    }

    public int getPOV(int pov) {
        return controller.getPOV(pov);
    }

    public int getPOV() {
        return controller.getPOV();
    }

    public int getAxisCount() {
        return controller.getAxisCount();
    }

    public int getPOVCount() {
        return controller.getPOVCount();
    }

    public int getButtonCount() {
        return controller.getButtonCount();
    }

    public HIDType getType() {
        return controller.getType();
    }

    public String getName() {
        return controller.getName();
    }

    public int getAxisType(int axis) {
        return controller.getAxisType(axis);
    }

    public int getPort() {
        return controller.getPort();
    }

    public void setOutput(int outputNumber, boolean value) {
        controller.setOutput(outputNumber, value);
    }

    public void setOutputs(int value) {
        controller.setOutputs(value);
    }

    public void setRumble(RumbleType type, double value) {
        controller.setRumble(type, value);
    }
}
