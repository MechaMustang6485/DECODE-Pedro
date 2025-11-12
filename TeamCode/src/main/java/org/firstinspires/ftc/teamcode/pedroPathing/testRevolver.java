package org.firstinspires.ftc.teamcode.pedroPathing;

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
public class testRevolver extends LinearOpMode{
    private Servo arm;
    private CRServo intake;
    private Servo stopper ;
    private int pos=96;
    private DcMotorEx revolver;
    private DcMotor shooter;
    private TouchSensor touch;
    private boolean touchVal=false;
    private ColorSensor colorSens;
    private double purpleValue;
    private boolean purple;
    private double greenValue;
    private boolean green;
    private boolean emptySlot;
    private double alphaValue;

    // if over shoot experiment with PIDposition to tune
    private double PIDposition=13;



    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();
        while (!isStarted()){
            telemetry();
            getColors();
            color();
        }
        waitForStart();
        while (opModeIsActive()){
            teleOpControls();
            telemetry();
            getColors();
            color();
        }
    }

    public void initHardware() {
        initRevolver(PIDposition);
        initShooter();
        initColorSensor();
        initIntake();
        initArm();
        initStopper();
        initTouch();
        getTouchSensor();

    }
    public void initIntake(){
        intake=hardwareMap.get(CRServo.class,"intake");
        intake.setPower(0);
    }

    public void initArm(){
        arm=hardwareMap.get(Servo.class,"arm");
        arm.setDirection(Servo.Direction.FORWARD);
        arm.setPosition(0.6);
    }
    public void initStopper(){
        stopper=hardwareMap.get(Servo.class,"stopper");
        stopper.setPosition(0.3);
    }

    public void initTouch(){
        touch=hardwareMap.get(TouchSensor.class,"touch");
    }
    public void getTouchSensor(){
        touchVal=touch.isPressed();
    }

    public void initRevolver(double PIDposition){
        revolver=hardwareMap.get(DcMotorEx.class,"revolver");
        revolver.setDirection(DcMotorEx.Direction.FORWARD);
        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setPositionPIDFCoefficients(PIDposition);
        revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setPower(0);
        revolver.setTargetPosition(0);
        revolver.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    public void initShooter(){
        shooter=hardwareMap.get(DcMotor.class,"shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setPower(0);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        green = false;
        purple = false;

        if (purpleValue<greenValue){
            green=true;
        }
        if (purpleValue>greenValue){
            purple=true;
        }

    }



    public void teleOpControls(){
        initStart();
        if(gamepad1.a){
            initSequenceA();
        }
        if(gamepad1.b){
            initSequenceB();
        }
        if(gamepad1.x) {
            initSequenceC();
        }
        if(gamepad1.y){
            stopper.setPosition(-0.3);
            intake.setPower(1);
            if(gamepad1.right_bumper){
                intake.setPower(0);
            }
            if(touch.isPressed()){
                revolver.setTargetPosition(96);
                sleep(2000);
                revolver.setTargetPosition(192);
                sleep(2000);
                revolver.setTargetPosition(288);
                sleep(2000);
                if(revolver.getTargetPosition()==288 && touch.isPressed()){
                    arm.setPosition(0);
                    intake.setPower(0);
                    telemetry.addLine("All slots filled");
                    telemetry.update();
                }
            }
        }

    }


    // revolver sequences
    // P P G
    public void initSequenceA(){
        initP();
        sleep(2000);
        shooter.setPower(0);
        initPP();
        sleep(2000);
        shooter.setPower(0);
        initPPG();
        sleep(3000);
        shooter.setPower(0);
    }
    public void initStart(){
        revolver.setPower(0.2);
        if(greenValue>=710){
            revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            revolver.setPower(0);
            revolver.setTargetPosition(0);
        }
    }
    //    P P G
    public void initP(){
        revolver.setTargetPosition(96);
        revolver.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shooter.setPower(1);
        revolver.setPower(0.5);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("P");
        telemetry.update();
    }



    public void initPP(){
        // P P
        revolver.setTargetPosition(192);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("P");
        telemetry.addLine("P");
        telemetry.update();

    }

    public void initPPG(){
        // P P G
        revolver.setTargetPosition(288);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("P");
        telemetry.addLine("P");
        telemetry.addLine("G");
        telemetry.update();

    }

    // G P P
    public void initSequenceB(){
        initG();
        sleep(2000);
        arm.setPosition(0.6);
        shooter.setPower(0);
        initGP();
        sleep(2000);
        arm.setPosition(0.6);
        shooter.setPower(0);
        initGPP();
        sleep(3000);
        arm.setPosition(0.6);
        shooter.setPower(0);
    }

    public void initG(){
        // G
        revolver.setTargetPosition(0);
        revolver.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("G");
        telemetry.update();
    }

    public void initGP(){
        // G P
        revolver.setTargetPosition(96);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("G");
        telemetry.addLine("P");
        telemetry.update();
    }

    public void initGPP(){
       //G P P
        revolver.setTargetPosition(192);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("G");
        telemetry.addLine("P");
        telemetry.addLine("P");
        telemetry.update();
    }


    // P G P
    public void initSequenceC(){
        initP2();
        sleep(2000);
        arm.setPosition(0.6);
        shooter.setPower(0);
        initPG();
        sleep(2000);
        arm.setPosition(0.6);
        shooter.setPower(0);
        initPGP();
        sleep(3000);
        arm.setPosition(0.6);
        shooter.setPower(0);

    }

    public void initP2(){
        // P
        revolver.setTargetPosition(96);
        revolver.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("P");
        telemetry.update();
    }

    public void initPG(){
        // P G
        revolver.setTargetPosition(288);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("P");
        telemetry.addLine("G");
        telemetry.update();
    }

    public void initPGP(){
        // P G P
        revolver.setTargetPosition(192);
        revolver.setPower(0.5);
        shooter.setPower(1);
        sleep(2000);
        arm.setPosition(0);
        arm.setPosition(0.6);
        telemetry.addLine("P");
        telemetry.addLine("G");
        telemetry.addLine("P");
        telemetry.update();
    }




    public void telemetry(){
        telemetry.addLine("a PPG");
        telemetry.addLine("b GPP");
        telemetry.addLine("x PGP");
        telemetry.addData("greenValue","%.2f",greenValue);
        telemetry.addData("purpleValue","%.2f", purpleValue);
        telemetry.addData("alphaValue","%.2f",alphaValue);
        telemetry.update();
    }


}
