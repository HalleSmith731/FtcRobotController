package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="GOOD TeleOp use this one", group="Linear Opmode")
public class TeleOp_good extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    private DcMotorEx launcher;

    private CRServo rightFeeder, leftFeeder;

    private ElapsedTime runtime = new ElapsedTime();
    static final double TARGET_VELOCITY = 1500; // ticks/sec

    double F = 13;
    double P = 34;
    static final double RPM_TOLERANCE = 70;



    @Override
    public void runOpMode() {

        // -------- HARDWARE MAP --------
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");


        // mecanum left
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // enoodcers
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

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

            // mecanum
            double normalSpeed = 1;
            double slowSpeed = .4;

// percision drive
            double speed = gamepad1.left_bumper ? slowSpeed : normalSpeed;

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

            // -launcher control
            if (gamepad1.right_bumper) {
                launcher.setVelocity(TARGET_VELOCITY);
            } else {
                launcher.setPower(0);
            }

            // intake

            if (gamepad1.b) {
                leftFeeder.setPower(-1);
                rightFeeder.setPower(1);
            } else if (gamepad1.dpad_left) {
                leftFeeder.setPower(1);
                rightFeeder.setPower(-1);
            } else {
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
            }





            // flywheel stability and feeding
            double velocity = launcher.getVelocity();
            boolean inRange = Math.abs(velocity - TARGET_VELOCITY) < RPM_TOLERANCE;
            telemetry.addData("Velocity", velocity);



            if (inRange) {//gamepad1.b &&

                // feeder
//                if (gamepad1.b) {
                    rightFeeder.setPower(1);
                    leftFeeder.setPower(-1);
//                } else {
//                    rightFeeder.setPower(0);
//                    leftFeeder.setPower(0);
//                }
//                // First time reaching stable speed
//                if (!flywheelStable) {
//                    flywheelStable = true;
//                    flywheelStableTimer.reset();
//                }
//
//                // FEED ON
//                if (!feeding && flywheelStableTimer.milliseconds() > FEED_OFF_TIME) {
//                    feeding = true;
//                    flywheelStableTimer.reset();
//                    leftFeeder.setPower(-1);
//                    rightFeeder.setPower(1);
//                }
//
//                // FEED OFF
//                if (feeding && flywheelStableTimer.milliseconds() > FEED_ON_TIME) {
//                    feeding = false;
//                    flywheelStableTimer.reset();
//                    leftFeeder.setPower(0);
//                    rightFeeder.setPower(0);
//                }

            } else {
//                flywheelStable = false;
//                feeding = false;
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
            }

            // -------- TELEMETRY --------
            telemetry.addData("Runtime", runtime.toString());
//            telemetry.addData("Launcher RPM", "%.0f", getLauncherRPM());
            telemetry.addData("Launcher Velocity", "%.0f", launcher.getVelocity());
            telemetry.update();
        }
    }



    public double getLauncherRPM() {
        return launcher.getVelocity() ;
    }

}
