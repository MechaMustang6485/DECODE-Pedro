package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Config file
 */
@TeleOp(group="Primary")
public class Revolver2 extends LinearOpMode{
    private Servo servoTouch;
    private double servoPos=1.0;
    private DcMotorEx revolver;
    // This is how much the motor moves for each slot
    private int revolverPos=96;
    private double velocity=0;

    private TouchSensor touch;
    private boolean touchVal=false;

    private ColorSensor colorSens;
    private int purpleValue;
    public boolean purple;
    private int greenValue;
    public boolean green;
    private int alphaValue;


    // if over shoot lower kp, increase kd, and experiment with PIDposition to tune
    private double topVelocity=750;
    private double currentVelocity=0.0;
    private double maxVelocity=750;
    private double F=32767.0/ topVelocity;
    private double kP=F*0.3;
    private double kI=kP*0.01;
    private double kD=kI*0.001;
    private double PIDposition=5.0;



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
        }
    }

    public void initHardware() {
        initServoTouch();
        initRevolver(F,kP,kI,kD,PIDposition);
        initTouch();
        initColorSensor();
    }

    public void initServoTouch(){
        servoTouch=hardwareMap.get(Servo.class,"servoTouch");
        servoTouch.setDirection(Servo.Direction.FORWARD);
        servoTouch.setPosition(0);
    }

    public void initRevolver(double F,double kP,double kI,double kD,double PIDposition){
        revolver=hardwareMap.get(DcMotorEx.class,"motorOne");
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

    public void initTouch(){
        touch=hardwareMap.get(TouchSensor.class,"touchSensor");
    }
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

    public void teleOpControls(){
        if(gamepad1.a){
            initSequenceA();
        }
        if(gamepad1.b){
            initSequenceB();
        }
        if(gamepad1.x){
            initSequenceC();
        }
    }

    public void color(){
        if (purpleValue<greenValue){
            green=true;
        }
        if (purpleValue>greenValue){
            purple=true;
        }
    }

    // P P G
    public void initSequenceA(){
        // P
        revolver.setTargetPosition(revolverPos);
        servoTouch.setPosition(5);
        if(!touchVal){
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
        }
        servoTouch.setPosition(-5);
        revolver.setTargetPosition(revolverPos);
        if(!purple){
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
            if(!touchVal){
                servoTouch.setPosition(-5);
                revolver.setTargetPosition(revolverPos);
                servoTouch.setPosition(5);
            }
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
        }
        if(purple){
            telemetry.addLine("P");
            telemetry.update();
        }
        // 5 sec
        sleep(5000);

        // P P
        revolver.setTargetPosition(revolverPos);
        servoTouch.setPosition(5);
        if(!touchVal){
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
        }
        servoTouch.setPosition(-5);
        revolver.setTargetPosition(revolverPos);
        if(!purple){
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
            if(!touchVal){
                servoTouch.setPosition(-5);
                revolver.setTargetPosition(revolverPos);
                servoTouch.setPosition(5);
            }
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
        }
        if(purple){
            telemetry.addLine("P");
            telemetry.addLine("P");
            telemetry.update();
        }

        // 5 sec
        sleep(5000);

        // P P G
        revolver.setTargetPosition(revolverPos);
        servoTouch.setPosition(5);
        if(!touchVal){
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
        }
        servoTouch.setPosition(-5);
        revolver.setTargetPosition(revolverPos);
        if(!green){
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
            if(!touchVal){
                servoTouch.setPosition(-5);
                revolver.setTargetPosition(revolverPos);
                servoTouch.setPosition(5);
            }
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
        }
        if(green){
            telemetry.addLine("P");
            telemetry.addLine("P");
            telemetry.addLine("G");
            telemetry.update();
        }


    }

    // G P P
    public void initSequenceB(){
        // G
        revolver.setTargetPosition(revolverPos);
        servoTouch.setPosition(5);
        if(!touchVal){
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
        }
        servoTouch.setPosition(-5);
        revolver.setTargetPosition(revolverPos);
        if(!green){
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
            if(!touchVal){
                servoTouch.setPosition(-5);
                revolver.setTargetPosition(revolverPos);
                servoTouch.setPosition(5);
            }
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
        }
        if(green){
            telemetry.addLine("G");
            telemetry.update();
        }

        // 5 sec
        sleep(5000);

        // G P
        revolver.setTargetPosition(revolverPos);
        servoTouch.setPosition(5);
        if(!touchVal){
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
        }
        servoTouch.setPosition(-5);
        revolver.setTargetPosition(revolverPos);
        if(!purple){
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
            if(!touchVal){
                servoTouch.setPosition(-5);
                revolver.setTargetPosition(revolverPos);
                servoTouch.setPosition(5);
            }
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
        }
        if(purple){
            telemetry.addLine("G");
            telemetry.addLine("P");
            telemetry.update();
        }

        // 5 sec
        sleep(5000);


        // G P P
        revolver.setTargetPosition(revolverPos);
        servoTouch.setPosition(5);
        if(!touchVal){
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
        }
        servoTouch.setPosition(-5);
        revolver.setTargetPosition(revolverPos);
        if(!purple){
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
            if(!touchVal){
                servoTouch.setPosition(-5);
                revolver.setTargetPosition(revolverPos);
                servoTouch.setPosition(5);
            }
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
        }
        if(purple){
            telemetry.addLine("G");
            telemetry.addLine("P");
            telemetry.addLine("P");
            telemetry.update();
        }

    }


    // P G P
    public void initSequenceC(){
        // P
        revolver.setTargetPosition(revolverPos);
        servoTouch.setPosition(5);
        if(!touchVal){
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
        }
        servoTouch.setPosition(-5);
        revolver.setTargetPosition(revolverPos);
        if(!purple){
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
            if(!touchVal){
                servoTouch.setPosition(-5);
                revolver.setTargetPosition(revolverPos);
                servoTouch.setPosition(5);
            }
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
        }
        if(purple){
            telemetry.addLine("P");
            telemetry.update();
        }
        // 5 sec
        sleep(5000);

        // P G
        revolver.setTargetPosition(revolverPos);
        servoTouch.setPosition(5);
        if(!touchVal){
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
        }
        servoTouch.setPosition(-5);
        revolver.setTargetPosition(revolverPos);
        if(!green){
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
            if(!touchVal){
                servoTouch.setPosition(-5);
                revolver.setTargetPosition(revolverPos);
                servoTouch.setPosition(5);
            }
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
        }
        if(green){
            telemetry.addLine("P");
            telemetry.addLine("G");
            telemetry.update();
        }

        // 5 sec
        sleep(5000);

        // P G P
        revolver.setTargetPosition(revolverPos);
        servoTouch.setPosition(5);
        if(!touchVal){
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
        }
        servoTouch.setPosition(-5);
        revolver.setTargetPosition(revolverPos);
        if(!purple){
            revolver.setTargetPosition(revolverPos);
            servoTouch.setPosition(5);
            if(!touchVal){
                servoTouch.setPosition(-5);
                revolver.setTargetPosition(revolverPos);
                servoTouch.setPosition(5);
            }
            servoTouch.setPosition(-5);
            revolver.setTargetPosition(revolverPos);
        }
        if(purple){
            telemetry.addLine("P");
            telemetry.addLine("G");
            telemetry.addLine("P");
            telemetry.update();
        }

    }

}

