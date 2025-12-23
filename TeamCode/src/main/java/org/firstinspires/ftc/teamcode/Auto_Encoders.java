package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Good auto use this one", group="Linear Opmode")
public class Auto_Encoders extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx launcher;
    private CRServo leftfeeder, rightfeeder;

    // ---------------- DRIVE CONSTANTS ----------------
    static final double TICKS_PER_REV = 537.6;
    static final double WHEEL_DIAMETER_IN = 3.78;
    static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    // ---------------- SHOOTER CONSTANTS ----------------
//    static final double RPM_TOLERANCE = 300;
    static final int SHOTS = 3;
    static final double TARGET_VELOCITY = 1300; // ticks/sec

    double F = 13;
    double P = 34;
    static final double RPM_TOLERANCE = 150;

    @Override
    public void runOpMode() {

        // ---------------- HARDWARE MAP ----------------
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftfeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightfeeder = hardwareMap.get(CRServo.class, "right_feeder");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        waitForStart();
        if (!opModeIsActive()) return;

        // =====================================================
        // 1️⃣ DRIVE FORWARD
        // =====================================================
        driveEncoder(-24, 0.6);

        // =====================================================
        // 2️⃣ SPIN UP + SHOOT 3 BALLS (ENCODER CONTROLLED)
        // =====================================================
        shootBallsEncoderBased();

        // =====================================================
        // 3️⃣ DRIVE BACK
        // =====================================================
        driveEncoder(24, 0.6);
    }

    // =====================================================
    // SHOOTER FUNCTION (NO SLEEP)
    // =====================================================
    void shootBallsEncoderBased() {

        launcher.setVelocity(TARGET_VELOCITY);

        ElapsedTime rpmTimer = new ElapsedTime();
        ElapsedTime feedTimer = new ElapsedTime();

        int lastPos = launcher.getCurrentPosition();
        double rpm = 0;

        int shotsFired = 0;
        boolean feeding = false;

        while (opModeIsActive() && shotsFired < SHOTS) {
            double velocity = launcher.getVelocity();
            boolean inRange = Math.abs(velocity - TARGET_VELOCITY) < RPM_TOLERANCE;
            if (inRange) {


                // ---- RPM CALCULATION ----
//            if (rpmTimer.seconds() >= RPM_SAMPLE_TIME) {
//                int currentPos = launcher.getCurrentPosition();
//                int deltaTicks = currentPos - lastPos;
//
//                double revolutions =
//                        deltaTicks / (FLYWHEEL_TICKS_PER_REV * FLYWHEEL_GEAR_RATIO);
//
//                rpm = (revolutions / rpmTimer.seconds()) * 60.0;
//
//                lastPos = currentPos;
//                rpmTimer.reset();
//            }
//
//            boolean rpmStable = Math.abs(rpm - TARGET_RPM) <= RPM_TOLERANCE;

                // ---- FEED ONE BALL ----
//            if (rpmStable && !feeding) {
                if (feedTimer.seconds()>1) {
                    leftfeeder.setPower(-1);
                    rightfeeder.setPower(1);
                    feedTimer.reset();
                    shotsFired+=1;
                }

//                feedTimer.reset();
//                feeding = true;
//            }
//
//            if (feeding && feedTimer.seconds() >= FEED_TIME) {
//                leftfeeder.setPower(0);
//                rightfeeder.setPower(0);
//                feeding = false;
//            }
            }

            telemetry.addData("RPM", (int) rpm);
            telemetry.addData("Shots Fired", shotsFired);
            telemetry.update();

        }

        launcher.setVelocity(0);
        leftfeeder.setPower(0);
        rightfeeder.setPower(0);
    }

    // =====================================================
    // DRIVE FUNCTIONS (UNCHANGED)
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

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
