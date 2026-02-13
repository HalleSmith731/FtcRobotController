package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="GOAL Line Auto", group="Linear Opmode")
public class Auto_Goal_Line extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // ---------------- DRIVE CONSTANTS ----------------
    static final double TICKS_PER_REV = 537.6;     // GoBILDA 312 RPM
    static final double WHEEL_DIAMETER_IN = 3.78;
    static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    @Override
    public void runOpMode() {

        // ---------------- HARDWARE MAP ----------------
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Reverse left side (mecanum standard)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Brake when stopped
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        waitForStart();
        if (!opModeIsActive()) return;

        // =====================================================
        // DRIVE FORWARD 16 INCHES
        // =====================================================
        driveEncoder(-17, 0.45);
    }

    // =====================================================
    // ENCODER DRIVE FUNCTION
    // =====================================================
    void resetEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void driveEncoder(double inches, double power) {

        int ticks = (int)(inches * TICKS_PER_INCH);

        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + ticks);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + ticks);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + ticks);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + ticks);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);

        while (opModeIsActive() &&
                (frontLeftDrive.isBusy() || frontRightDrive.isBusy()
                        || backLeftDrive.isBusy() || backRightDrive.isBusy())) {
            telemetry.update();
        }

        stopDrive();
    }

    void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
