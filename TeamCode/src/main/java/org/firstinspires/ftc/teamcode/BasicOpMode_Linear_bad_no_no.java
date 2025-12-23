package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum TeleOp RPM + LED", group="Linear Opmode")
public class BasicOpMode_Linear_bad_no_no extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    private DcMotorEx launcher;

    private DigitalChannel ledRed, ledGreen;

    private CRServo rightFeeder, leftFeeder;
    private DcMotor intake;

    private ElapsedTime runtime = new ElapsedTime();

    static final double TICKS_PER_REV = 28.0;
    static final double TARGET_RPM = 2750;
    static final double TARGET_VELOCITY = (TARGET_RPM / 60.0) * TICKS_PER_REV; // ticks/sec
    static final double RPM_TOLERANCE = 150;

    private ElapsedTime flywheelStableTimer = new ElapsedTime();
    private boolean flywheelStable = false;
    private boolean feeding = false;
    static final double FEED_ON_TIME = 140;   // ms (short push)
    static final double FEED_OFF_TIME = 140;  // ms (recovery)


    @Override
    public void runOpMode() {

        // -------- HARDWARE MAP --------
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");

        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");

        ledRed   = hardwareMap.get(DigitalChannel.class, "led_red");
        ledGreen = hardwareMap.get(DigitalChannel.class, "led_green");

        // -------- LED SETUP --------
        ledRed.setMode(DigitalChannel.Mode.OUTPUT);
        ledGreen.setMode(DigitalChannel.Mode.OUTPUT);
        setLED(false, true); // RED by default

        // -------- MOTOR DIRECTIONS --------
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // -------- MOTOR MODES --------
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {

            // -------- MECANUM DRIVE --------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double turn = gamepad1.right_stick_x * 0.8;

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

            // -------- LAUNCHER CONTROL --------
            if (gamepad2          .right_bumper) {
                launcher.setVelocity(TARGET_VELOCITY);
                updateLauncherLED();
            } else {
                launcher.setPower(0);
                setLED(false, true); // RED
            }

            // intake
            if (gamepad2.y) {
                intake.setPower(0.75);
            } else if (gamepad1.a) {
                intake.setPower(-0.75);
            } else {
                intake.setPower(0);
            }

            // feeder
            if (gamepad2.b) {
                rightFeeder.setPower(1);
                leftFeeder.setPower(-1);
            } if (gamepad2.x) {
                rightFeeder.setPower(-1);
                leftFeeder.setPower(1);
            } else {
                rightFeeder.setPower(0);
                leftFeeder.setPower(0);
            }

            // -------- FLYWHEEL STABILITY & FEEDING --------
            double velocity = launcher.getVelocity();
            boolean inRange = Math.abs(velocity - TARGET_VELOCITY) < RPM_TOLERANCE;

            if (gamepad1.right_bumper && inRange) {

                // First time reaching stable speed
                if (!flywheelStable) {
                    flywheelStable = true;
                    flywheelStableTimer.reset();
                }

                // FEED ON
                if (!feeding && flywheelStableTimer.milliseconds() > FEED_OFF_TIME) {
                    feeding = true;
                    flywheelStableTimer.reset();
                    leftFeeder.setPower(-1);
                    rightFeeder.setPower(1);
                }

                // FEED OFF
                if (feeding && flywheelStableTimer.milliseconds() > FEED_ON_TIME) {
                    feeding = false;
                    flywheelStableTimer.reset();
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                }

            } else {
                flywheelStable = false;
                feeding = false;
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
            }

            // -------- TELEMETRY --------
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("Launcher RPM", "%.0f", getLauncherRPM());
            telemetry.addData("Launcher Velocity", "%.0f", launcher.getVelocity());
            telemetry.update();
        }
    }

    // ================= FUNCTIONS =================

    public double getLauncherRPM() {
        return (launcher.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    public void updateLauncherLED() {
        if (Math.abs(getLauncherRPM() - TARGET_RPM) <= RPM_TOLERANCE) {
            setLED(true, false);   // GREEN
        } else {
            setLED(false, true);  // RED
        }
    }

    // ACTIVE LOW LEDS
    // greenOn, redOn
    public void setLED(boolean green, boolean red) {
        ledGreen.setState(!green);
        ledRed.setState(!red);
    }
}
