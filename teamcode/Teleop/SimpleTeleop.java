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
        sliderControl = new SliderControl();
        sliderControl.init(hardwareMap, "verticalSlider", "horizontalSlider");

        vertArmControl = new ArmControl();
        vertArmControl.init(hardwareMap);

        // Initialize the Mecanum drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double armMovement;

        // Wait for the start signal
        waitForStart();

        while (!isStopRequested()) {

            // Mecanum drive control
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,  // Forward/backward
                    -gamepad1.left_stick_x,  // Strafe
                    -gamepad1.right_stick_x  // Rotate
            ));
            drive.update();

            // Gripper control
            if (gamepad2.right_stick_x > 0.2) gripper.setAnglerUP();
            else if (gamepad2.right_stick_x < -0.2) {
                gripper.setAnglerDown();
            } else {
                gripper.setAnglerMid();
            }

            if (gamepad2.left_stick_x > 0.2) {
                gripper.setGripperPosition(0.01);
            } else if (gamepad2.left_stick_x < -0.2) {
                gripper.setGripperPosition(-0.01);
            } else {
                gripper.gripperStopped();
            }

            // Reset the timer for the next step

            if(gamepad2.b){ // To Move block from Hor Gripper to Vert Gripper (Automation)
                /*Bottom gripper holds on to the sample while the vertical gripper opens and
                moves to an optimal position for transfer */
                timer.reset();
                gripper.setGripperPosition(0);
                vertGripper.setGripperPosition(0);
                vertArmControl.setArmPosition(0);
                while (timer.seconds() <1 && opModeIsActive()){

                }
                //Angler is set to the middle and waits 0.5 seconds
                timer.reset();
                gripper.setAnglerMid();
                while (timer.seconds() <1 && opModeIsActive()){

                }
                //Arm goes up and waits 0.5 seconds
                timer.reset();
                armControl.setArmPosition(0.5);
                while(timer.seconds() <1 && opModeIsActive()){

                }
                //vertGripper holds onto sample and waits 0.5 seconds
                timer.reset();
                vertGripper.setGripperPosition(1);
                while(timer.seconds() <1 && opModeIsActive()){

                }
                //Gripper is set open and waits 0.5 seconds
                timer.reset();
                gripper.setGripperPosition(1);
                while(timer.seconds() <1 && opModeIsActive()) {

                }
                //Arm moves down and waits 0.5 seconds
                timer.reset();
                armControl.setArmPosition(0);
                while(timer.seconds()<1 && opModeIsActive()){

                }

                vertArmControl.setArmPosition(1);
            }

            // Arm control with left stick Y
            if(gamepad2.dpad_up == true) {
                armControl.setArmPosition(0);
            }
            if (gamepad2.dpad_down == true) { // Scale the stick input
                armControl.setArmPosition(1);
            }



            // Horizontal slider control
            if (gamepad1.dpad_left) {
                sliderControl.controlHorizontalSlider(-0.5); // Retract horizontal slider
            } else if (gamepad1.dpad_right) {
                sliderControl.controlHorizontalSlider(0.5); // Extend horizontal slider
            }
            else {
                sliderControl.controlHorizontalSlider(0); // Stop horizontal slider if no input
            }

            // Vertical slider control
            if (gamepad1.dpad_up) {
                sliderControl.controlVerticalSlider(-0.5); // Retract vertical slider
            } else if (gamepad1.dpad_down) {
                sliderControl.controlVerticalSlider(0.5); // Extend vertical slider
            }
            else{
                sliderControl.controlVerticalSlider(0); // Stop vertical slider if no input
            }


            // Telemetry updates
            telemetry.addData("Gripper Position", gripper.getGripperPosition());
            telemetry.addData("Arm Position", armControl.getArmPosition());
            //telemetry.addData("Vertical Slider Length", sliderControl.getVerticalSliderLen());
            //telemetry.addData("Horizontal Slider Length", sliderControl.getHorizontalSliderLen());
            telemetry.update();
        }
    }
}