/*
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AdvancedAuto", group="Autonomous")
public class AdvancedAuto extends LinearOpMode {

    public static double MOTOR_POWER = 0.6; // adjust this if needed
    ElapsedTime timer = new ElapsedTime();
    private MecanumDriveBase drive; // Declare drive as a class member


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize Mecanum drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set motors to run-to-position mode

        // Wait for the start signal
        waitForStart();
    }

    private void moveForward(double distanceInches) {
        drive.setTargetPosition(new Pose2d(distanceInches, 0, 0));
        drive.followTrajectorySync();
    }

    private void moveSideways(double distanceInches) {
        drive.setTargetPosition(new Pose2d(0, distanceInches, 0));
        drive.followTrajectorySync();
    }

    private void turn(double angleDegrees) {
        drive.setTargetPosition(new Pose2d(0, 0, Math.toRadians(angleDegrees))); // Road Runner uses radians
        drive.followTrajectorySync();
    }
}
*/