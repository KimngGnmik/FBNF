package org.firstinspires.ftc.teamcode.Autonomous;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry; // Import for telemetry

;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Teleop.ArmControl;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;
import org.firstinspires.ftc.teamcode.subsytems.SliderControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
//@Disabled
public class RedObserverSide extends LinearOpMode {
    private ElapsedTime AutoTimer = new ElapsedTime();
    private ArmControl armControl;
    private Gripper gripper;
    private SliderControl sliderControl;
    private double speedFactor = 0.65;

    private Gripper vertGripper;

    private ArmControl vertArmControl;




    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // - - - Setting up Arm motors - - - //
        armControl = new ArmControl();
        armControl.init(hardwareMap);
        armControl.setArmPosition(0);

        vertGripper = new Gripper();
        vertGripper.init(hardwareMap);
        vertGripper.gripperStopped();
        vertGripper.setAnglerDown();

        vertArmControl = new ArmControl();
        vertArmControl.init(hardwareMap);


        // - - - Setting up Slider motors - - - //
        sliderControl = new SliderControl();
        sliderControl.init(hardwareMap, "verticalSlider", "horizontalSlider");


        // - - - Initialize gripper to starting position - - - //
        gripper = new Gripper();
        gripper.init(hardwareMap);
        gripper.gripperStopped();
        gripper.setAnglerDown();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(30,8), Math.toRadians(0))
                .build();

        Trajectory splineLeft = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(30,-15), Math.toRadians(0))
                .build();

        Trajectory specimenSpline = drive.trajectoryBuilder(startPose, true)
                .splineToLinearHeading(new Pose2d(25, 10, Math.toRadians(180)), Math.toRadians(360))
                .build();
        // x is forward y is back

        TrajectorySequence trajSequence = drive.trajectorySequenceBuilder(startPose)
                .addTrajectory(traj1)
                //.splineTo(new Vector2d(30, -10), Math.toRadians(0))  // Add traj2 to the sequence
                .forward(0.02)
                //.strafeLeft(23)
                .strafeLeft(20)
                // spline to submersible
                .addTrajectory(splineLeft)
                // goes back 18
                .forward(18)
                // goes to the right 4
                .strafeLeft(4)
                // forward
                .back(20)
                //.strafeLeft(5)
                .forward(26)
                .back(23)
                .addTrajectory(specimenSpline)
                .forward(3)

                .build();

        waitForStart();



        drive.followTrajectorySequenceAsync(trajSequence);




        //while (!isStopRequested() && opModeIsActive()) ;




        /*
         Robot facing forward:
         30, 8, Math.toRadians(0) spline to the left
         30, -8, Math.toRadians(0) spline to the right
         // cannot have Negative: 30, 8, Math.toRadians(-90) splines to the left but rotates right ~90 degrees
         30, 8, Math.toRadians(90) splines to the left but rotates left ~90 degrees
         -30, 8, Math.toRadians(0) rotates left 180 degrees and moves forward

         Trajectory traj1 = drive.trajectoryBuilder(startPose, true) turns 180 degrees and then moves backwards
        */

        while (opModeIsActive()) {
            drive.update(); // VERY IMPORTANT!

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", Math.toRadians(poseEstimate.getHeading()));
            telemetry.update();
        }



    }
}