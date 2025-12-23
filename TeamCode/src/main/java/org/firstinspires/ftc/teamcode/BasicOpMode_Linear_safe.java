package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum TeleOp Smooth", group="Linear Opmode")
public class BasicOpMode_Linear_safe extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    // Launcher motor
    private DcMotor launcher;

    // CRservo feeders
    private CRServo rightFeeder;
    private CRServo leftFeeder;

    // intake
    private DcMotor intake;

    // Timer
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // hardware map
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        intake = hardwareMap.get(DcMotor.class, "intake");

        launcher = hardwareMap.get(DcMotor.class, "launcher");

        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");

        // Reverse left side motors for mecanum
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Launcher direction
        launcher.setDirection(DcMotor.Direction.FORWARD);

        // zero power brake
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Run without encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            double y = -gamepad1.left_stick_y; // Forward/back
            double x = gamepad1.left_stick_x * 1.1; // Strafe
            double turn = gamepad1.right_stick_x * 0.8; // turn

            // Calculate raw powers
            double flPower = y + x + turn;
            double frPower = y - x - turn;
            double blPower = y - x + turn;
            double brPower = y + x - turn;

            // Find the largest absolute value
            double max = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                    Math.max(Math.abs(blPower), Math.abs(brPower))));

            // Scale powers if any is above 1
            if (max > 1.0) {
                flPower /= max;
                frPower /= max;
                blPower /= max;
                brPower /= max;
            }

            // Apply powers
            frontLeftDrive.setPower(flPower);
            frontRightDrive.setPower(frPower);
            backLeftDrive.setPower(blPower);
            backRightDrive.setPower(brPower);


            // launcher control

            if (gamepad2.right_bumper) {
                launcher.setPower(0.52);
            } else if (gamepad2.left_bumper) {
                launcher.setPower(-0.50);

            } else {
                launcher.setPower(0);
            }

            // feeder control

            if (gamepad2.b) {
                rightFeeder.setPower(1);
                leftFeeder.setPower(-1);
            } else if (gamepad2.dpad_left) {
                rightFeeder.setPower(-1);
                leftFeeder.setPower(1);
            } else {
                rightFeeder.setPower(0);
                leftFeeder.setPower(0.0);
            }

            if (gamepad2.y) {
                intake.setPower(0.75);
            } else if (gamepad2.a) {
                intake.setPower(-0.75);
            } else {
                intake.setPower(0);

            }

            // telemetry

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Power", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", flPower, frPower, blPower, brPower);
            telemetry.addData("Launcher Power", launcher.getPower());
            telemetry.addData("Right Feeder", rightFeeder.getPower());
            telemetry.addData("Left Feeder", leftFeeder.getPower());
            telemetry.update();
        }
    }
}