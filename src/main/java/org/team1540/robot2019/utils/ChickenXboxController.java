package org.team1540.robot2019.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import java.util.HashMap;
import java.util.Map;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.rooster.triggers.AxisButton;
import org.team1540.rooster.triggers.DPadAxis;
import org.team1540.rooster.triggers.MultiAxisButton;
import org.team1540.rooster.triggers.StrictDPadButton;

public class ChickenXboxController extends XboxController {

    /**
     * Construct an instance of a joystick. The joystick index is the USB port on the drivers station.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public ChickenXboxController(int port) {
        super(port);
    }

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

    public double getRawAxis(XboxAxis axis) {
        return super.getRawAxis(axis.value);
    }

    /**
     * Get the X axis value of the controller in the official 1540 coordinate system
     *
     * @param hand Side of controller whose value should be returned.
     * @return The X axis value of the controller in the official 1540 coordinate system
     */
    public double getRectifiedX(Hand hand) {
        return -super.getY(hand);
    }

    /**
     * Get the Y axis value of the controller in the official 1540 coordinate system
     *
     * @param hand Side of controller whose value should be returned.
     * @return The Y axis value of the controller in the official 1540 coordinate system
     */
    public double getRectifiedY(Hand hand) {
        return -super.getX(hand);
    }

    public Vector2D get2DJoystickVector(Hand hand) {
        return new Vector2D(getRectifiedX(hand), getRectifiedY(hand));
    }

    /**
     * Gets angle from a 2D joystick
     *
     * @param hand Left vs right joystick of the xbox this
     * @return Angle in radians counter-clockwise from 12 o'clock
     */
    public double get2DJoystickAngle(Hand hand) { // TODO: Migrate to ROOSTER
        return Math.atan2(getRectifiedY(hand), getRectifiedX(hand));
    }

    public double get2DJoystickMagnitude(Hand hand) { // TODO: Migrate to ROOSTER
        return Vector2D.ZERO.distance(get2DJoystickVector(hand));
    }

    public StrictDPadButton getButton(DPadAxis button) {
        return new StrictDPadButton(this, 0, button);
    }

    public JoystickButton getButton(XboxButton button) {
        return new JoystickButton(this, button.value);
    }

    public AxisButton getButton(XboxAxis axis, double threshold) {
        return new AxisButton(this, threshold, axis.value);
    }

    public MultiAxisButton getButton(double threshold, XboxAxis... axes) {
        int[] axesIds = new int[axes.length];
        for (int i = 0; i < axes.length; i++) {
            axesIds[i] = axes[i].value;
        }
        return new MultiAxisButton(this, threshold, axesIds);
    }
}
