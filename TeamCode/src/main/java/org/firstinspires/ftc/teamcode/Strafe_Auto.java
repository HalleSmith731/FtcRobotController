package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Forward 24 + Strafe 24", group="Linear Opmode")
public class Strafe_Auto extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Motor directions for mecanum
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        if (opModeIsActive()) {

            // -------------------------
            // Move forward 24 inches
            // -------------------------
            driveForwardTimed(0.5, 1500); // Tune 1500ms to match 24 inches

            // -------------------------
            // Strafe right 24 inches
            // -------------------------
            strafeRightTimed(0.5, 1500); // Tune 1500ms to match 24 inches
        }
    }

    // -------------------------
    // Drive straight forward/backward
    // -------------------------
    private void driveForwardTimed(double power, long timeMs) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);

        sleep(timeMs);
        stopDrive();
    }

    // -------------------------
    // Strafe right
    // -------------------------
    private void strafeRightTimed(double power, long timeMs) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);

        sleep(timeMs);
        stopDrive();
    }

    // -------------------------
    // Stop all motors
    // -------------------------
    private void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
