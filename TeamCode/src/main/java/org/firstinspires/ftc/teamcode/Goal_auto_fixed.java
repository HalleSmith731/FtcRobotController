package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Autonomous(name="Auto: Back 4ft Shoot 3 Return (WORKING)", group="Auto")
public class Goal_auto_fixed extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Shooter
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    private DigitalChannel ledRed, ledGreen;

    // ---------------- CONSTANTS ----------------
    static final double WHEEL_DIAMETER_IN = 3.78;
    static final double TICKS_PER_REV = 537.6;   // GoBILDA 312 RPM
    static final double GEAR_RATIO = 1.0;

    static final double TICKS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);

    static final int DRIVE_4FT_TICKS = (int)(48 * TICKS_PER_INCH);

    static final double TARGET_RPM = 2400;
    static final double RPM_TOLERANCE = 150;

    @Override
    public void runOpMode() {

        // ---------------- HARDWARE MAP ----------------
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_drive");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        ledRed   = hardwareMap.get(DigitalChannel.class, "led_red");
        ledGreen = hardwareMap.get(DigitalChannel.class, "led_green");

        ledRed.setMode(DigitalChannel.Mode.OUTPUT);
        ledGreen.setMode(DigitalChannel.Mode.OUTPUT);

        // ---------------- DIRECTIONS ----------------
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // ---------------- BRAKE ----------------
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------------- ENCODERS ----------------
        resetDriveEncoders();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // =====================================================
        // 1️⃣ DRIVE BACKWARD 4 FT
        // =====================================================
        driveStraight(-DRIVE_4FT_TICKS, 0.6);

        // =====================================================
        // 2️⃣ SPIN UP SHOOTER
        // =====================================================
        launcher.setVelocity(rpmToTicks(TARGET_RPM));

        while (opModeIsActive() && !launcherAtSpeed()) {
            setLED(false, true);
            telemetry.addData("RPM", getLauncherRPM());
            telemetry.update();
        }

        setLED(true, false);

        // =====================================================
        // 3️⃣ SHOOT 3 BALLS
        // =====================================================
        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            feedOneBall();
            feedOneBall();
            feedOneBall();
            sleep(350);
        }

        launcher.setVelocity(0);
        setLED(false, true);

        // =====================================================
        // 4️⃣ DRIVE FORWARD BACK TO START
        // =====================================================
        driveStraight(DRIVE_4FT_TICKS, 0.6);
    }

    // =====================================================
    // FUNCTIONS
    // =====================================================

    void resetDriveEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void driveStraight(int ticks, double power) {

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy()
                        || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.update();
        }

        stopDrive();

        // IMPORTANT: return to normal mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    boolean launcherAtSpeed() {
        return Math.abs(getLauncherRPM() - TARGET_RPM) < RPM_TOLERANCE;
    }

    double getLauncherRPM() {
        return (launcher.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    double rpmToTicks(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    void feedOneBall() {
        leftFeeder.setPower(-1);
        rightFeeder.setPower(1);
        sleep(320);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    void setLED(boolean green, boolean red) {
        ledGreen.setState(!green);
        ledRed.setState(!red);
    }
}
