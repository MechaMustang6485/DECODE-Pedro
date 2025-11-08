package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;


@TeleOp
public class J extends LinearOpMode {

    private DcMotor shooter;
    private CRServo servoOne;
    private Limelight3A limelight;

    public void runOpMode() throws InterruptedException{
        initHardware();
        while (!isStarted()){
            slotTelemetry();
        }
        waitForStart();
        while (opModeIsActive()){
            DriveTrain();
        }


    }

    public void DriveTrain () throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeft = hardwareMap.dcMotor.get("Fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("Bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("Fr");
        DcMotor backRight = hardwareMap.dcMotor.get("Br");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.back) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if(gamepad1.leftBumperWasPressed())  {
                servoOne.setPower(1);
            }

            if (gamepad1.left_trigger >= 1) {
                servoOne.setPower(-1);
            }

            if (gamepad1.dpad_up) {
                servoOne.setPower(0);
            }

            if(gamepad1.right_trigger >= 1) {
                shooter.setPower(0);
            }
            if(gamepad1.right_bumper) {
                shooter.setPower(1);
            }

        }
    }

    private void initHardware() {
        initServo();
        initShooter();
        initLimeLight();
    }

    private void initShooter() {
        shooter=hardwareMap.get(DcMotor.class,"shooter");
        shooter.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void initLimeLight(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    private void initServo() {
        servoOne=hardwareMap.get(CRServo.class,"intake");
        servoOne.setPower(0);
    }

    private void TeleOpControls(){

        if(gamepad1.leftBumperWasPressed())  {
            servoOne.setPower(1);
        }

        if (gamepad1.left_trigger >= 1) {
            servoOne.setPower(-1);
        }

        if (gamepad1.dpad_up) {
            servoOne.setPower(0);
        }

        if(gamepad1.right_bumper) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults)

                    if (botpose != null) {
                        double x = botpose.getPosition().x;
                        double y = botpose.getPosition().y;
                        double z = botpose.getPosition().z;

                        if (z >= 2.3){
                            shooter.setPower(0.5);
                        }

                        else {
                            shooter.setPower(1);
                        }




                            telemetry.addData("MT1 Location", "(" + x + ", " + y + ", " + z + ")");
                            telemetry.update();

                    }
            }
        }


        if(gamepad1.right_trigger >= 1) {
            shooter.setPower(0);
        }
        if(gamepad1.x) {
            shooter.setPower(1);
        }



}
    public void slotTelemetry(){
        telemetry.addLine("Left Bumper starts intake");
        telemetry.addLine("Left Trigger starts outtake");
        telemetry.addLine("Dpad_Up stops servo");
        telemetry.addLine("Right Bumber starts shooter");
        telemetry.addLine("Right Trigger stops shooer");
        telemetry.addData("Shooter:", shooter.getPower());
        telemetry.addData("Axon:", servoOne.getPower());
        telemetry.update();

    }
}
