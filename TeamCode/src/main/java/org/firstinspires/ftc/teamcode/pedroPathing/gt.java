package org.firstinspires.ftc.teamcode.pedroPathing;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(group="Primary")
public class gt extends LinearOpMode {
    private Servo servo;
    private double servoPos=1.0;

    private TouchSensor touch;
    private boolean touchVal=false;

    @Override
    public void runOpMode() throws InterruptedException{
        initHardware();
        while (!isStarted()){
            getTouchSensor();
        }
        waitForStart();
        while (opModeIsActive()){
            getTouchSensor();
            teleOpControls();
        }
    }

    public void initHardware() {
        initServoTouch();
        initTouch();
    }
    public void initServoTouch(){
        servo=hardwareMap.get(Servo.class,"stopper");
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(0);
    }

    public void initTouch(){
        touch=hardwareMap.get(TouchSensor.class,"touchSensor");
    }
    public void getTouchSensor(){
        touchVal=touch.isPressed();
    }
    public void teleOpControls(){

        if (gamepad1.a) {
            servo.setPosition(0.3);
        }

        if (gamepad1.b) {
            servo.setPosition(-0.3);
        }

    }

}


