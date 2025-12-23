package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class Flywheel_tuning extends LinearOpMode {
    public DcMotorEx FlywheelMotor;
    public double highVelocity  = 2200;
    public double lowVelocity = 1100;
    double curTargetVelocity = highVelocity;

    double F = 13;
    double P = 34;


    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.00};

    int stepIndex = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        FlywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        FlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        FlywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) {
                if (curTargetVelocity == highVelocity) {
                    curTargetVelocity = lowVelocity;
                } else { curTargetVelocity = highVelocity; }
            }
            if (gamepad1.bWasPressed()){
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                F -= stepSizes[stepIndex];
            }
             if (gamepad1.dpadRightWasPressed()) {
                 F += stepSizes[stepIndex];

             }

             if (gamepad1.dpadUpWasPressed()) {
                 P += stepSizes[stepIndex];
             }
             if (gamepad1.dpadDownWasPressed()) {
                 P -= stepSizes[stepIndex];
             }

             PIDFCoefficients pidfCoefficient = new PIDFCoefficients(P, 0, 0, F);
             FlywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficient);

             FlywheelMotor.setVelocity(curTargetVelocity);

             double curVelocity = FlywheelMotor.getVelocity();
             double error = curTargetVelocity - curVelocity;

             telemetry.addData("Target Velocity", curTargetVelocity);
             telemetry.addData( "Current Velocity", curVelocity);
             telemetry.addData("Error", error);
             telemetry.addData("P", P);
             telemetry.addData("F", F);
             telemetry.addData("Step Size", stepSizes[stepIndex]);
             telemetry.update();

        }
    }
}
