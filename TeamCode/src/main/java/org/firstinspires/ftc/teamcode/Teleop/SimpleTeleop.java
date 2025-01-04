package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;
import org.firstinspires.ftc.teamcode.subsytems.SliderControl;
import org.firstinspires.ftc.teamcode.Teleop.ArmControl;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "Teleop")
public class SimpleTeleop extends LinearOpMode {

    private Gripper gripper;

    private Gripper vertGripper;
    private ArmControl armControl;
    private ArmControl vertArmControl;
    private SliderControl sliderControl;

    public static double MOTOR_POWER = 0.6;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the gripper
        gripper = new Gripper();
        gripper.init(hardwareMap);

        vertGripper = new Gripper();
        vertGripper.init(hardwareMap);

        // Initialize the arm control
        armControl = new ArmControl();
        armControl.init(hardwareMap);

        // Initialize the slider control
        //sliderControl = new SliderControl();
        //sliderControl.init(hardwareMap, "verticalSlider", "horizontalSlider");

        //vertArmControl = new ArmControl();
        //vertArmControl.init(hardwareMap);

        // Initialize the Mecanum drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double armMovement;

        MOTOR_POWER = 0.6;

        // Wait for the start signal
        waitForStart();

        while (!isStopRequested()) {

            // Mecanum drive control
            /*
            drive.setWeightedDrivePower(new Pose2d(
                    gamepad1.left_stick_y,  // strafe
                    gamepad1.left_stick_x,   // Forward/Backwards
                    gamepad1.right_stick_x  // Rotate
            ));
            drive.update();
            */

            // drive.setMotorPowers(frontLeft, rearLeft, rearRight, frontRight)
             if(gamepad1.right_stick_y<-0.2) { // forward
                 drive.setMotorPowers(MOTOR_POWER, MOTOR_POWER, MOTOR_POWER, MOTOR_POWER);

             } else if(gamepad1.right_stick_y>0.2) { // backwards
                 drive.setMotorPowers(-MOTOR_POWER, -MOTOR_POWER, -MOTOR_POWER, -MOTOR_POWER);
             } else if(gamepad1.right_stick_x>0.2) { //strafe right
                 drive.setMotorPowers(MOTOR_POWER, -MOTOR_POWER, MOTOR_POWER, -MOTOR_POWER);
             } else if(gamepad1.right_stick_x<-0.2) { //strafe left
                 drive.setMotorPowers(-MOTOR_POWER, MOTOR_POWER, -MOTOR_POWER, MOTOR_POWER);
             } else if(gamepad1.left_stick_x>0.2) { // rotate right
                 drive.setMotorPowers(MOTOR_POWER, MOTOR_POWER, -MOTOR_POWER, -MOTOR_POWER);
             } else if(gamepad1.left_stick_x<-0.2) { // rotate left
                 drive.setMotorPowers(-MOTOR_POWER, -MOTOR_POWER, MOTOR_POWER, MOTOR_POWER);
             } else {
                 drive.setMotorPowers(0,0,0,0);
             }
             if (gamepad1.a) {
                 MOTOR_POWER =0.4;
             } else if (gamepad1.b) {
                 MOTOR_POWER -=0.8;
             } else if (gamepad1.x) {
                 MOTOR_POWER = 0.6;
             }



                 // Gripper control
            if (gamepad2.right_stick_x > 0.2)
                gripper.setAnglerUP();
            else if (gamepad2.right_stick_x < -0.2) {
                gripper.setAnglerDown();
            } else {
                gripper.setAnglerMid();
            }

            if (gamepad2.dpad_left) {
                //open, which is 1
                gripper.setGripperPosition(0.6);
            } else if (gamepad2.dpad_right) {
                //close, which is 0
                gripper.setGripperPosition(0.9);
            }

            // Reset the timer for the next step



            // Arm control with left stick Y
            if(gamepad2.dpad_up == true) {
                armControl.setArmPosition(0);
            }
            if (gamepad2.dpad_down == true) { // Scale the stick input
                armControl.setArmPosition(1);
            }



           /* // Horizontal slider control
            if (gamepad1.dpad_left) {
                sliderControl.controlHorizontalSlider(-0.5); // Retract horizontal slider
            } else if (gamepad1.dpad_right) {
                sliderControl.controlHorizontalSlider(0.5); // Extend horizontal slider
            }
            else {
                sliderControl.controlHorizontalSlider(0); // Stop horizontal slider if no input
            }


            if (gamepad1.dpad_up) {
                sliderControl.controlVerticalSlider(-0.5); // Retract vertical slider
            } else if (gamepad1.dpad_down) {
                sliderControl.controlVerticalSlider(0.5); // Extend vertical slider
            }
            else{
                sliderControl.controlVerticalSlider(0); // Stop vertical slider if no input
            }
*/

            // Telemetry updates
            telemetry.addData("Gripper Position", gripper.getGripperPosition());
            telemetry.addData("Arm Position", armControl.getArmPosition());
            //telemetry.addData("Vertical Slider Length", sliderControl.getVerticalSliderLen());
            //telemetry.addData("Horizontal Slider Length", sliderControl.getHorizontalSliderLen());
            telemetry.addData("Motor Power", MOTOR_POWER);
            telemetry.update();
        }
    }
}