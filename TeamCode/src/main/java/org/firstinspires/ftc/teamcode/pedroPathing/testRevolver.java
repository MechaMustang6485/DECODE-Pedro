package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Config file
 */
@TeleOp(group="Primary")
public class testRevolver extends LinearOpMode{
    private Servo servoTouch;
    //    private CRServo servoIn;
    private DcMotorEx revolver;
    // This is how much the motor moves for each slot 96
    private double velocity=0;

    private DcMotor shooter;

    private TouchSensor touch;
    private boolean touchVal=false;

    private ColorSensor colorSens;
    private int purpleValue;
    private boolean purple;
    private int greenValue;
    private boolean green;

    private boolean emptySlot;

    private int alphaValue;

    private Limelight3A limelight;


    // if over shoot lower kp, increase kd, and experiment with PIDposition to tune
    private double topVelocity=750;
    //    private double currentVelocity=0.0;
//    private double maxVelocity=750;
    private double F=32767.0/ topVelocity;
    private double kP=F*0.2;
    private double kI=kP*0.01;
    private double kD=kI*0.001;
    private double PIDposition=4.0;



    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();
        while (!isStarted()){
            telemetry();
            getColors();
//            getTouchSensor();
            color();
        }
        waitForStart();
        while (opModeIsActive()){
            telemetry();
            getColors();
            teleOpControls();
//            getTouchSensor();
            color();
        }
    }

    public void initHardware() {
//        initServoTouch();
//        initServoIn();
        initRevolver(F,kP,kI,kD,PIDposition);
        initShooter();
//        initTouch();
        initColorSensor();
        initLimeLight();

    }

//    public void initServoTouch(){
//        servoTouch=hardwareMap.get(Servo.class,"touch");
//        servoTouch.setDirection(Servo.Direction.FORWARD);
//        servoTouch.setPosition(0);
//    }

//    public void initServoIn(){
//        servoIn=hardwareMap.get(CRServo.class,"servoIn");
//        servoIn.setPower(0);
//    }

    public void initRevolver(double F,double kP,double kI,double kD,double PIDposition){
        revolver=hardwareMap.get(DcMotorEx.class,"revolver");
        revolver.setDirection(DcMotorEx.Direction.FORWARD);
        revolver.setVelocity(velocity);
        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setPositionPIDFCoefficients(PIDposition);
        revolver.setVelocityPIDFCoefficients(kP,kI,kD,F);
        revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setTargetPosition(0);
        revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        revolver.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }

    public void initShooter(){
        shooter=hardwareMap.get(DcMotor.class,"shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
//    public void initTouch(){touch=hardwareMap.get(TouchSensor.class,"touchSensor");}
//    public void getTouchSensor(){
//        touchVal=touch.isPressed();
//    }

    public void initLimeLight(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(100);
        limelight.start();
    }



    public void initColorSensor(){
        colorSens=hardwareMap.get(ColorSensor.class,"colorSensor");
    }
    public void getColors(){
        purpleValue=colorSens.blue();
        greenValue=colorSens.green();
        alphaValue=colorSens.alpha();
    }

    public void color(){
        if (purpleValue<greenValue){
            green=true;
        }
        if (purpleValue>greenValue){
            purple=true;
        }
//        if(purpleValue<100 && greenValue<100 && !touch.isPressed()){
//            emptySlot=true;
//        }
    }

    public void teleOpControls(){

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                if (botpose != null) {
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    double z = botpose.getPosition().z;
                    int id = fr.getFiducialId();

                    if(id == 23) {
                        initSequenceA();
                        telemetry.addData("Apriltag", "("+id+")");
                    }

                    if(id == 21) {
                        initSequenceB();
                        telemetry.addData("Apriltag", "("+id+")");
                    }
                    if(id == 22) {
                        initSequenceC();
                        telemetry.addData("Apriltag", "("+id+")");
                    }



                    telemetry.addData("MT1 Location", "(" + x + ", " + y + ", " + z + ")");
                }
            }
        }

/*
        if(gamepad1.aWasPressed()){
            initSequenceA();
        }
        if(gamepad1.bWasPressed()){
            initSequenceB();
        }
        if(gamepad1.xWasPressed()){
            initSequenceC();
        }

 */
//        if (gamepad1.yWasPressed()){
//            initEmpty();
//        }
    }

//    private void initEmpty() {
//        servoTouch.setPosition(0.3);
//        if(emptySlot){
//            servoIn.setPower(1.0);
//            if(touch.isPressed()){
//                servoIn.setPower(0);
//                revolver.setTargetPosition(96);
//                servoIn.setPower(1.0);
//                if(revolver.getTargetPosition()==288 && touch.isPressed()){
//                    servoTouch.setPosition(0);
//                    servoIn.setPower(0);
//                    telemetry.addLine("All slots filled");
//                    telemetry.update();
//                }
//            }
//        }
//    }

    // revolver sequences
    // P P G
    public void initSequenceA(){
        initP();
        sleep(2000);
        shooter.setPower(0);
        sleep(2000);
        initPP();
        sleep(2000);
        shooter.setPower(0);
        sleep(2000);
        initPPG();
        sleep(2000);
        shooter.setPower(0);
    }
    //    P P G
    public void initP(){
        //P
        revolver.setTargetPosition(96);
        if (green) {
            revolver.setTargetPosition(96);
        }else if (!green) {
            shooter.setPower(0.5);
            telemetry.addLine("P");
            telemetry.update();
        }
    }

    public void initPP(){
        // P P
        revolver.setTargetPosition(96);
        if (green) {
            revolver.setTargetPosition(96);
        }
        if (!green) {
            shooter.setPower(0.5);
            telemetry.addLine("P");
            telemetry.addLine("P");
            telemetry.update();
        }
    }

    public void initPPG(){
        // P P G
        revolver.setTargetPosition(96);
        if (!green) {
            revolver.setTargetPosition(96);
        }
        if (green) {
            shooter.setPower(0.5);
            telemetry.addLine("P");
            telemetry.addLine("P");
            telemetry.addLine("G");
            telemetry.update();
        }
    }

    // G P P
    public void initSequenceB(){
        initG();
        sleep(2000);
        shooter.setPower(0);
        sleep(2000);
        initGP();
        sleep(2000);
        shooter.setPower(0);
        sleep(2000);
        initGPP();
        sleep(2000);
        shooter.setPower(0);
        sleep(2000);
    }

    public void initG(){
        // G
        revolver.setTargetPosition(96);
        if (!green) {
            revolver.setTargetPosition(96);
        }
        if (green) {
            shooter.setPower(0.5);
            telemetry.addLine("G");
            telemetry.update();
        }
    }

    public void initGP(){
        // G P
        revolver.setTargetPosition(96);
        if (green) {
            revolver.setTargetPosition(96);
        }
        if (!green) {
            shooter.setPower(0.5);
            telemetry.addLine("G");
            telemetry.addLine("P");
            telemetry.update();
        }
    }

    public void initGPP(){
        // G P P
        revolver.setTargetPosition(96);
        if (!green) {
            revolver.setTargetPosition(96);
        }
        if (green) {
            shooter.setPower(0.5);
            telemetry.addLine("G");
            telemetry.addLine("P");
            telemetry.addLine("P");
            telemetry.update();
        }
    }


    // P G P
    public void initSequenceC(){
        initP2();
        sleep(2000);
        shooter.setPower(0);
        sleep(2000);
        initPG();
        sleep(2000);
        shooter.setPower(0);
        sleep(2000);
        initPGP();
        sleep(2000);
        shooter.setPower(0);
        sleep(2000);

    }

    public void initP2(){
        // P
        revolver.setTargetPosition(96);
        if (green) {
            revolver.setTargetPosition(96);
        }
        if (!green) {
            shooter.setPower(0.5);
            telemetry.addLine("P");
            telemetry.update();
        }
    }

    public void initPG(){
        // P G
        revolver.setTargetPosition(96);
        if (!green) {
            revolver.setTargetPosition(96);
        }
        if (green) {
            shooter.setPower(0.5);
            telemetry.addLine("P");
            telemetry.addLine("G");
            telemetry.update();
        }
    }

    public void initPGP(){
        // P G P
        revolver.setTargetPosition(96);
        if (!green) {
            revolver.setTargetPosition(96);
        }
        if (green) {
            shooter.setPower(0.5);
            telemetry.addLine("P");
            telemetry.addLine("G");
            telemetry.addLine("P");
            telemetry.update();
        }
    }

    public void telemetry(){
        telemetry.addData("greenValue","%.2f",greenValue);
        telemetry.addData("purpleValue","%.2f", purpleValue);
        telemetry.addData("alphaValue","%.2f",alphaValue);
        telemetry.update();
    }


}