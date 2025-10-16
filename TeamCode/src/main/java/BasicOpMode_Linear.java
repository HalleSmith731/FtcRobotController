
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Linear OpMode with Servo", group="Linear OpMode")
public class BasicOpMode_Linear extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armDrive = null;
    private CRServo contServo = null;
    private CRServo contServo2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armDrive   = hardwareMap.get(DcMotor.class, "arm_drive");

        contServo = hardwareMap.get(CRServo.class, "claw_servo");
        contServo2 = hardwareMap.get(CRServo.class, "claw_servo2");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);

            double armPower = 0.0;
            if (gamepad1.right_bumper) {
                armPower = 0.51;  // raise arm slowly (51% power)
            } else if (gamepad1.left_bumper) {
                armPower = -0.51; // lower arm slowly
            } else {
                armPower = 0.0; // stop

            }
            armDrive.setPower(armPower);

            // Continuous Rotation Servo control
            if (gamepad1.a) {
                contServo.setPower(1.0);   // spin forward
            } else if (gamepad1.b) {
                contServo.setPower(-1.0);  // spin backward
            } else {
                contServo.setPower(0.0);   // stop
            }

            if (gamepad1.a) {
                contServo2.setPower(-1.0);   // spin forward
            } else if (gamepad1.b) {
                contServo2.setPower(1.0);  // spin backward
            } else {
                contServo2.setPower(0.0);   // stop
            }

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), arm (%.2f)", leftPower, rightPower, armPower);
            telemetry.addData("Servo Power", "%.2f", contServo.getPower());
            telemetry.addData("Servo Power", "%.2f", contServo2.getPower());
            telemetry.update();
        }
    }
}

