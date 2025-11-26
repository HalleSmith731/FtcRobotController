package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Blue Intake Score 3 Auto (No Encoders)", group="Linear Opmode")
public class BlueIntake_Score3_Auto extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotor launcher;
    private CRServo leftfeeder, rightfeeder;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        launcher = hardwareMap.get(DcMotor.class, "launcher");
        leftfeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightfeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // Reverse left side so robot drives straight
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Just run without encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (opModeIsActive()) {

            // Step 1: Drive forward for 1.5 seconds
            driveTimed(-0.52, 320);

            // Step 2: Spin up launcher
            launcher.setPower(.52);
            sleep(1500);

            // Step 3: Feed 3 balls
            sleep(1700);
            feedBall();
            sleep(1700);
            feedBall();
            sleep(1700);
            feedBall();
            sleep(1700);
            feedBall();
            sleep(1700);


            // Step 4: Stop launcher
            launcher.setPower(0);

            // Step 5: Back up for 1 second
            driveTimed(0.60, 350);
        }
    }

    private void driveTimed(double power, long timeMs) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
        sleep(timeMs);
        stopDrive();
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void feedBall() {
        leftfeeder.setPower(-1);
        rightfeeder.setPower(1);
        sleep(425);
        leftfeeder.setPower(0);
        rightfeeder.setPower(0);
        sleep(426
        );
    }
}