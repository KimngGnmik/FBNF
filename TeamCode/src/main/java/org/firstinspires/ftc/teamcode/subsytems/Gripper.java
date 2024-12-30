package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config // This is so the dashboard will pick up variables
public class Gripper {

    private Servo gripper = null;
    private Servo angler = null;

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
        gripper = hwMap.get(Servo.class, "gripper"); // Ctrl hub port 0
        angler = hwMap.get(Servo.class, "angler"); // Ctrl Hub port 1
        gripper.setDirection(Servo.Direction.REVERSE);
        gripper.setPosition(gripperPosition); // Start at a safe position
    }

    public void setGripperPosition(double increment) {
        // Calculate the new position and constrain it within the limits
        //double newPosition = gripperPosition + increment;
        gripperPosition = Math.max(GRIPPER_MIN_POS, Math.min(GRIPPER_MAX_POS, increment));
        gripper.setPosition(gripperPosition); // Update the servo
    }

    public void gripperStopped() {
        gripper.setPosition(gripperPosition); // Stop gripper movement
    }

    public void setGripperPositionDirect(double position) {
        // Constrain the position within limits
        gripperPosition = Math.max(GRIPPER_MIN_POS, Math.min(GRIPPER_MAX_POS, position));
        gripper.setPosition(gripperPosition);
    }

    public void setAnglerUP() {
        angler.setPosition(ANGLER_UP); // Move angler to up position
    }

    public void setAnglerDown() {
        angler.setPosition(ANGLER_DOWN); // Move angler to down position
    }
    public void setAnglerMid() {
        angler.setPosition(ANGLER_UP/2);
    }
    public double getGripperPosition() {
        return gripperPosition;
    }
    public void setAnglerInit() {
        angler.setPosition(0.0);
    }
}