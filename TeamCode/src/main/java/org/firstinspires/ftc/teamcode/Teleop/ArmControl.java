package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmControl {
    private Servo horArmControl;
    private Servo vertArmControl;
    private double armPosition = 0.5; // Default arm position

    // Constants for servo limits
    private static final double ARM_MIN_POS = 0.0; // Adjust based on your servo's range
    private static final double ARM_MAX_POS = 1.0;

    public ArmControl() {
        // Empty constructor
    }

    /**
     * Initializes the arm servo with the hardware map.
     * @param hardwareMap The hardware map to access hardware configuration.
     */
    public void init(HardwareMap hardwareMap) {
        horArmControl = hardwareMap.get(Servo.class, "horArmControl");
        horArmControl.setPosition(armPosition); // Initialize to default position

        vertArmControl = hardwareMap.get(Servo.class, "vertArmControl");
        vertArmControl.setPosition(ARM_MAX_POS); // Initialize to default position
    }

    /**
     * Sets the arm position directly, ensuring it stays within the allowed range.
     * @param position The desired position for the arm servo.
     */
    public void sethorArmControl(double position) {
        // Constrain the position to within valid limits
        armPosition = Math.max(ARM_MIN_POS, Math.min(ARM_MAX_POS, position));
        horArmControl.setPosition(armPosition);
    }
    public void setvertArmControl(double position) {
        // Constrain the position to within valid limits
        armPosition = Math.max(ARM_MIN_POS, Math.min(ARM_MAX_POS, position));
        vertArmControl.setPosition(armPosition);
    }

    /**
     * Increments the arm position by a specified amount.
     * @param increment The amount to change the arm position by.
     */
    public void adjustVertArmPosition(double increment) {
        sethorArmControl(armPosition + increment);
    }

    /**
     * Gets the current arm position.
     * @return The current arm servo position.
     */
    public double getArmPosition() {
        return armPosition;
    }
}