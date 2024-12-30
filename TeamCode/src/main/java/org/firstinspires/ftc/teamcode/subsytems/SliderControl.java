package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SliderControl {
    private DcMotorEx verticalSlider;
    private DcMotorEx horizontalSlider;

    // Slider configuration constants
    private static final double ManualControlPower = 0.2; // Power for manual control

    public void init(HardwareMap hardwareMap, String verticalMotorName, String horizontalMotorName) {
        verticalSlider = hardwareMap.get(DcMotorEx.class, verticalMotorName);
        horizontalSlider = hardwareMap.get(DcMotorEx.class, horizontalMotorName);

        verticalSlider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlider.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        verticalSlider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        horizontalSlider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlider.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        horizontalSlider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // Manual control for the vertical slider
    public void controlVerticalSlider(double verticalInput) {
        if (Math.abs(verticalInput) > 0.1) {
            verticalSlider.setPower(verticalInput * ManualControlPower);
        } else {
            verticalSlider.setPower(0); // Stop motor when no input
        }
    }

    // Manual control for the horizontal slider
    public void controlHorizontalSlider(double horizontalInput) {
        if (Math.abs(horizontalInput) > 0.1) {
            horizontalSlider.setPower(horizontalInput * ManualControlPower);
        } else {
            horizontalSlider.setPower(0); // Stop motor when no input
        }
    }

    // Telemetry for debugging (optional)
    public double getVerticalSliderPosition() {
        return verticalSlider.getCurrentPosition();
    }

    public double getHorizontalSliderPosition() {
        return horizontalSlider.getCurrentPosition();
    }
}


/*package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SliderControl {
    private DcMotorEx verticalSlider;
    private DcMotorEx horizontalSlider;

    // Slider configuration constants
    private double PitchDiameter = 38.2; // mm
    private double Inch_Per_Tick = PitchDiameter * Math.PI / 25.4 / 537.7;
    private double DepositLength = 17.5; // inches
    private double IntakeLength = 5; // inches
    private double RuntoPositionPower = 0.45;

    public void init(HardwareMap hardwareMap, String verticalMotorName, String horizontalMotorName) {
        // Initialize motors
        verticalSlider = hardwareMap.get(DcMotorEx.class, verticalMotorName);
        horizontalSlider = hardwareMap.get(DcMotorEx.class, horizontalMotorName);

        // Setup encoders and zero power behavior
        verticalSlider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlider.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        verticalSlider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        horizontalSlider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlider.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        horizontalSlider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // Vertical Slider Methods
    public double getVerticalSliderLen() {
        return verticalSlider.getCurrentPosition() * Inch_Per_Tick;
    }

    public void setVerticalSliderIntake() {
        setVerticalSliderLen(IntakeLength);
    }

    public void setVerticalSliderDeposit() {
        setVerticalSliderLen(DepositLength);
    }

    private void setVerticalSliderLen(double length) {
        int targetPosition = (int) (length / Inch_Per_Tick);
        verticalSlider.setTargetPosition(targetPosition);
        verticalSlider.setPower(RuntoPositionPower);
        verticalSlider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setVerticalSliderPower(double power) {
        verticalSlider.setPower(power);
    }

    // Horizontal Slider Methods
    public double getHorizontalSliderLen() {
        return horizontalSlider.getCurrentPosition() * Inch_Per_Tick;
    }

    public void setHorizontalSliderIntake() {
        setHorizontalSliderLen(IntakeLength);
    }

    public void setHorizontalSliderDeposit() {
        setHorizontalSliderLen(DepositLength);
    }

    private void setHorizontalSliderLen(double length) {
        int targetPosition = (int) (length / Inch_Per_Tick);
        horizontalSlider.setTargetPosition(targetPosition);
        horizontalSlider.setPower(RuntoPositionPower);
        horizontalSlider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setHorizontalSliderPower(double power) {
        horizontalSlider.setPower(power);
    }
}*/