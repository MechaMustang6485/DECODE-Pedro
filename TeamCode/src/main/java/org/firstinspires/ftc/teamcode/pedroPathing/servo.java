package org.firstinspires.ftc.teamcode.TeamCode.BasicNotes_TeleOp.Revolver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Config file
 */
@TeleOp(group="Primary")
public class servo extends LinearOpMode{
    private Servo servoTouch;
    private CRServo servoIn;
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


    // if over shoot lower kp, increase kd, and experiment with PIDposition to tune
    private double topVelocity=750;
    //    private double currentVelocity=0.0;
//    private double maxVelocity=750;
    private double F=32767.0/ topVelocity;
    private double kP=F*0.2;
    private double kI=kP*0.01;
    private double kD=kI*0.008;
    private double PIDposition=4.0;



    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();
        while (!isStarted()){
            getColors();
            getTouchSensor();
            color();
        }
        waitForStart();
        while (opModeIsActive()){
            getColors();
            getTouchSensor();
            color();
            teleOpControls();
        }
    }

    public void initHardware() {
        initServoTouch();
        initServoIn();
        initRevolver(F,kP,kI,kD,PIDposition);
        initShooter();
        initTouch();
        initColorSensor();

    }

    public void initServoTouch(){
        servoTouch=hardwareMap.get(Servo.class,"stopper");
        servoTouch.setDirection(Servo.Direction.FORWARD);
        servoTouch.setPosition(0);
    }

    public void initServoIn(){
        servoIn=hardwareMap.get(CRServo.class,"intake");
        servoIn.setPower(0);
    }

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
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void initTouch(){touch=hardwareMap.get(TouchSensor.class,"touch");}
    public void getTouchSensor(){
        touchVal=touch.isPressed();
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
        if(purpleValue<100 && greenValue<100 && !touch.isPressed()){
            emptySlot=true;
        }
    }

    public void teleOpControls(){
        if(gamepad1.aWasPressed()){
            initSequenceA();
        }
        if(gamepad1.bWasPressed()){
            initSequenceB();
        }
        if(gamepad1.xWasPressed()){
            initSequenceC();
        }
        if (gamepad1.yWasPressed()){
            initEmpty();
        }
    }

    private void initEmpty() {
        servoTouch.setPosition(0.3);
        if(emptySlot){
            servoIn.setPower(1.0);
            if(touch.isPressed()){
                servoIn.setPower(0);
                revolver.setTargetPosition(96);
                servoIn.setPower(1.0);
                if(revolver.getTargetPosition()==288 && touch.isPressed()){
                    servoTouch.setPosition(0);
                    servoIn.setPower(0);
                    telemetry.addLine("All slots filled");
                    telemetry.update();
                }
            }
        }
    }

    // revolver sequences
    // P P G
    public void initSequenceA(){
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

        sleep(2000);
        shooter.setPower(0);
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

        sleep(2000);
        shooter.setPower(0);
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
        sleep(2000);
        shooter.setPower(0);

    }

    // G P P
    public void initSequenceB(){
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
        sleep(2000);
        shooter.setPower(0);
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

        sleep(2000);
        shooter.setPower(0);
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
        sleep(2000);
        shooter.setPower(0);
    }


    // P G P
    public void initSequenceC(){
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
        sleep(2000);
        shooter.setPower(0);
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

        sleep(2000);
        shooter.setPower(0);
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
        sleep(2000);
        shooter.setPower(0);

    }

}