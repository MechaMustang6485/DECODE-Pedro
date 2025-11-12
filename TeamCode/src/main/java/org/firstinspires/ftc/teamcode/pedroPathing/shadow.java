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
public class shadow extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor shooter;


    @Override
    public void runOpMode()throws InterruptedException{
        initHardware();
        while (!isStarted()){

        }
        waitForStart();
        while (opModeIsActive()){
            TeleOpControls();

        }


    }

    private void initHardware() {
        initlime();
        initShooter();
    }


    private void initShooter() {
        shooter=hardwareMap.get(DcMotor.class,"shooter");
        shooter.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
    }

    private void initlime() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.setPollRateHz(100);
        limelight.start();
    }


    private void TeleOpControls(){

        LLResult result = limelight.getLatestResult();
        // if (result.isValid()) {
        // Access general information
        //Pose3D botpose = result.getBotpose();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    int id = fr.getFiducialId();
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    double z = botpose.getPosition().z;
                    telemetry.addData("MT1 Location", "(" + x + ", " + y + ","+ z +")");

                    if (id == 24 && z <= 0.5) {
                        shooter.setPower(0.7);
                    }

                    if (id == 24 && z >= 4) {
                        shooter.setPower(1);
                    }

                }


            }
        }
        //  }


    }

}

