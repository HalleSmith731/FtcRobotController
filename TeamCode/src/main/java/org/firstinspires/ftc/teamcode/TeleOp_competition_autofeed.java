package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Comp-Ready TeleOp AutoFeed", group="Linear Opmode")
public class TeleOp_competition_autofeed extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx launcher;
    private CRServo rightFeeder, leftFeeder;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime flywheelStableTimer = new ElapsedTime();

    static final double TARGET_VELOCITY = 1500; // flywheel ticks/sec
    double F = 13;
    double P = 34;
    static final double RPM_TOLERANCE = 70;
    static final double FLYWHEEL_STABLE_TIME = 0.25; // seconds

    boolean flywheelStable = false;

    @Override
    public void runOpMode() {

        // --- HARDWARE MAP ---
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");

        // --- MOTOR DIRECTIONS ---
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // --- MOTOR MODES ---
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // --- MAIN LOOP ---
        while (opModeIsActive()) {

            // --- DRIVE CONTROL ---
            double normalSpeed = 1.0;
            double slowSpeed = 0.35; // precision drive
            double speed = gamepad1.left_bumper ? slowSpeed : normalSpeed;

            // Cubic scaling for precision
            double y = Math.pow(-gamepad1.left_stick_y, 3) * speed;
            double x = Math.pow(gamepad1.left_stick_x, 3) * speed;
            double turn = Math.pow(gamepad1.right_stick_x, 3) * 0.6; // constant turn scaling

            double fl = y + x + turn;
            double fr = y - x - turn;
            double bl = y - x + turn;
            double br = y + x - turn;

            double max = Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr),
                            Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            frontLeftDrive.setPower(fl);
            frontRightDrive.setPower(fr);
            backLeftDrive.setPower(bl);
            backRightDrive.setPower(br);

            // --- LAUNCHER CONTROL ---
            if (gamepad1.right_bumper) {
                launcher.setVelocity(TARGET_VELOCITY);
            } else {
                launcher.setPower(0);
                flywheelStable = false; // reset stable flag if launcher stops
            }

            double velocity = launcher.getVelocity();
            boolean inRange = Math.abs(velocity - TARGET_VELOCITY) < RPM_TOLERANCE;

            if (inRange) {
                if (!flywheelStable) {
                    flywheelStable = true;
                    flywheelStableTimer.reset();
                }


                if (flywheelStableTimer.seconds() >= FLYWHEEL_STABLE_TIME) {
                    leftFeeder.setPower(-1);
                    rightFeeder.setPower(1);
                } else {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                }

            } else {
                flywheelStable = false;
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
            }

            // --- TELEMETRY ---
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("Launcher Velocity", "%.0f", launcher.getVelocity());
            telemetry.addData("Drive Speed", speed);
            telemetry.addData("Flywheel Stable", flywheelStable);
            telemetry.update();
        }
    }
}
