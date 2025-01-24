package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config // This is so the dashboard will pick up variables
public class Gripper {

    private Servo horzGripper = null;
    private Servo vertGripper = null;
    private Servo anglerHorizontal = null;
    private Servo anglerVertical = null;

    // Constants for gripper limits
    private static final double GRIPPER_MIN_POS = 0.0; // Minimum gripper position
    private static final double GRIPPER_MAX_POS = 1; // Maximum gripper position
    private double gripperPosition = 0.5; // Start at midpoint

    // Torque Servo for angler
    private static final double ANGLER_UP = 1.0;
    private static final double ANGLER_DOWN = 0.1;

    public Gripper() {
        // Empty constructor
    }

    public void init(HardwareMap hwMap) {
        // Initialize the gripper and angler servos
        horzGripper = hwMap.get(Servo.class, "horzGripper"); // Ctrl hub port 0
        anglerHorizontal = hwMap.get(Servo.class, "anglerHorizontal"); // Ctrl Hub port 1
        horzGripper.setDirection(Servo.Direction.REVERSE);
        horzGripper.setPosition(gripperPosition); // Start at a safe position

        vertGripper = hwMap.get(Servo.class, "vertGripper"); // Ctrl hub port 0
        anglerVertical = hwMap.get(Servo.class, "anglerVertical"); // Ctrl Hub port 1
        vertGripper.setDirection(Servo.Direction.REVERSE);
        horzGripper.setPosition(gripperPosition); // Start at a safe position
    }

    public void setGripperPosition(double increment) {
        // Calculate the new position and constrain it within the limits
        //double newPosition = gripperPosition + increment;
        gripperPosition = Math.max(GRIPPER_MIN_POS, Math.min(GRIPPER_MAX_POS, increment));
        horzGripper.setPosition(gripperPosition); // Update the servo
    }

    public void gripperStopped() {
        horzGripper.setPosition(gripperPosition); // Stop gripper movement
    }

    public void setGripperPositionDirect(double position) {
        // Constrain the position within limits
        gripperPosition = Math.max(GRIPPER_MIN_POS, Math.min(GRIPPER_MAX_POS, position));
        horzGripper.setPosition(gripperPosition);
    }

    public void setAnglerHorizontalUP() {
        anglerHorizontal.setPosition(ANGLER_UP); // Move angler to up position
    }

    public void setAnglerHorizontalDOWN() {
        anglerHorizontal.setPosition(ANGLER_DOWN); // Move angler to down position
    }
    public void setAnglerHorizontalMID() {
        anglerHorizontal.setPosition(ANGLER_UP/2);
    }

    public void setAnglerVerticalUP() {
        anglerVertical.setPosition(ANGLER_UP); // Move angler to up position
    }

    public void setAnglerVerticalDOWN() {
        anglerVertical.setPosition(ANGLER_DOWN); // Move angler to down position
    }
    public void setAnglerVerticalMID() {
        anglerVertical.setPosition(ANGLER_UP/2);
    }
    public double getGripperPosition() {
        return gripperPosition;
    }
    public void setAnglerInit() {
        anglerHorizontal.setPosition(0.0);
    }
}