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

    /*
     * This is the initialization section. Here you initialize everything the robot has and the type
     * Essentially your instantiating an object. Think of it as you creating something using a template
     * rather than having to start from scratch each time you want to create something of the same type
     *
     * For Example: Private Gripper ___ is basically saying "Hey I want a Gripper named ___ and it
     * should have the basic properties/functionalities that are defined in the gripper class"
     * This way you can quickly say: Private Gripper VerticalGripper; Private Gripper HorizontalGripper
     * If you look at the initialization of the Gripper class, you can see that it already defines stuff like
     * the angler, minimum and maximum position etc. These properties apply to both the VerticalGripper, and
     * HorizontalGripper. Thus saving you time and not having to write the same code twice
     */

    // Horizontal gripper and Horizontal Arm control of the type Gripper and Arm Control
    private Gripper horgripper;
    private ArmControl horArmControl;


    // Vertical gripper and Vertical Arm control of the type Gripper and Arm Control
    private Gripper vertGripper;
    private ArmControl vertArmControl;

    // Slider control
    private SliderControl sliderControl;


    /*
     * The default Motor power. Increasing it makes the motor run faster, decreasing it makes it slower
     * Be careful though, in theory the motor power goes from 0 to 1. The more power the use the hotter it gets
     * and the faster the battery drains. Its good to increase it temporarily for things like a boost.
     */

    public static double MOTOR_POWER;


    /*
     * This timer is used so that if you want the code to run until the timer has finished (meaning teleop has
     * ended). Its a good idea to use it, currently its not being used
     */

    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the gripper
        horgripper = new Gripper();
        horgripper.init(hardwareMap);

        vertGripper = new Gripper();
        vertGripper.init(hardwareMap);

        // Initialize the arm control
        horArmControl = new ArmControl();
        horArmControl.init(hardwareMap);

        vertArmControl = new ArmControl();
        vertArmControl.init(hardwareMap);

        // Initialize the slider control
        sliderControl = new SliderControl();
        sliderControl.init(hardwareMap, "verticalSlider", "horizontalSlider");



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
             }
             else if(gamepad1.left_stick_x>0.2) { //strafe right
                 drive.setMotorPowers(MOTOR_POWER, -MOTOR_POWER, MOTOR_POWER, -MOTOR_POWER);
             } else if(gamepad1.left_stick_x<-0.2) { //strafe left
                 drive.setMotorPowers(-MOTOR_POWER, MOTOR_POWER, -MOTOR_POWER, MOTOR_POWER);
             }
             else if(gamepad1.right_stick_x>0.2) { // rotate right
                 drive.setMotorPowers(MOTOR_POWER, MOTOR_POWER, -MOTOR_POWER, -MOTOR_POWER);
             } else if(gamepad1.right_stick_x<-0.2) { // rotate left
                 drive.setMotorPowers(-MOTOR_POWER, -MOTOR_POWER, MOTOR_POWER, MOTOR_POWER);
             } else {
                 drive.setMotorPowers(0,0,0,0);
             }
             if (gamepad1.a) {
                 MOTOR_POWER =0.6;
             } else if (gamepad1.y) {
                 MOTOR_POWER =0.8;
             } else if (gamepad1.b) {
                 MOTOR_POWER = 0.4;
             }


             // Horizontal Angler
            if (gamepad2.right_stick_x > 0.2)
                horgripper.setAnglerUP();
            else if (gamepad2.right_stick_x < -0.2) {
                horgripper.setAnglerDown();
            } else {
                horgripper.setAnglerMid();
            }

            // Vertical Angler
            if (gamepad2.right_stick_y > 0.2){
                vertGripper.setAnglerUP();
            }
            else if (gamepad2.right_stick_x < -0.2){
                vertGripper.setAnglerDown();
            }
            else{
                vertGripper.setAnglerMid();
            }


            // Horizontal Gripper
            if (gamepad2.dpad_left) {
                //open, which is 1
                horgripper.setGripperPosition(0.6);
            } else if (gamepad2.dpad_right) {
                //close, which is 0
                horgripper.setGripperPosition(0.9);
            }

            // Vertical Gripper
            if (gamepad2.dpad_up) {
                //open, which is 1
                vertGripper.setGripperPosition(0.6);
            } else if (gamepad2.dpad_down) {
                //close, which is 0
                vertGripper.setGripperPosition(0.9);
            }



            // Arm control with left stick X
            if(gamepad2.left_stick_x > 0.2) {
                horArmControl.setArmPosition(0);
            } else if (gamepad2.left_stick_x < -0.2) { // Scale the stick input
                horArmControl.setArmPosition(1);
            }
            else{
                horArmControl.setArmPosition(0.5);
            }

            // Arm control with left stick Y
            if(gamepad2.left_stick_y > 0.2) {
                vertArmControl.setArmPosition(0);
            } else if (gamepad2.left_stick_y < -0.2) { // Scale the stick input
                vertArmControl.setArmPosition(1);
            }
            else{
                vertArmControl.setArmPosition(0.5);
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
            telemetry.addData("Gripper Position", horgripper.getGripperPosition());
            telemetry.addData("Arm Position", horArmControl.getArmPosition());
            //telemetry.addData("Vertical Slider Length", sliderControl.getVerticalSliderLen());
            //telemetry.addData("Horizontal Slider Length", sliderControl.getHorizontalSliderLen());
            telemetry.addData("Motor Power", MOTOR_POWER);
            telemetry.update();
        }
    }
}