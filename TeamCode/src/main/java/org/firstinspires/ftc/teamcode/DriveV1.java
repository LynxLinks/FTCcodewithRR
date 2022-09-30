package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//name and class

@TeleOp(name = "DriveV3", group="Linear Opmode")


public class DriveV1 extends OpMode {

    //Define Motors
    double target;
    DcMotor M0;
    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D0;
    DigitalChannel D1;

    public void ServoClamp() {

        //if (D0.getState() == true) S0.setPosition(.63);

        if (gamepad1.left_bumper) S0.setPosition(.46);
        if (gamepad1.right_bumper) S0.setPosition(.63);
        if((D1.getState() == true) && (gamepad1.left_bumper == false)) S0.setPosition(.63);
    }
    //drive loop
    public void MoveDriveTrain(){
        //drive variables
        double yAxis;
        double xAxis;
        double Rotate;

        //input to change variables
        yAxis = gamepad1.left_stick_y;
        xAxis = gamepad1.left_stick_x;
        Rotate = -gamepad1.left_trigger+gamepad1.right_trigger;

        //apply variables to motor
        M0.setPower(-(Rotate + (-yAxis + xAxis)));
        M1.setPower(-(Rotate + (+yAxis + xAxis)));
        M2.setPower(-(Rotate + (yAxis - xAxis)));
        M3.setPower(-(Rotate + (-yAxis - xAxis)));

        //dowm
        if (gamepad1.a) {
            target = 0;
        }
        //up
        if(gamepad1.b) {
            target = 2225;
        }
        if(gamepad1.y) {
            target = 1600;
        }
        if(gamepad1.x) {
            target = 950;
        }
        if(D0.getState() && (target == 0)){
            M0_2.setDirection(DcMotor.Direction.FORWARD);
            M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        M0_2.setPower(-1 * ((1 - Math.pow(10,((target - M0_2.getCurrentPosition())/400)))/(1 + Math.pow( 10,((target - M0_2.getCurrentPosition())/400)))));
        telemetry.addData("current ",M0_2.getCurrentPosition());
        telemetry.addData("delta", target - M0_2.getCurrentPosition());
        telemetry.addData("target",target);
        telemetry.addData("equation",-.5 * ((1 - Math.pow( 10,((target - M0_2.getCurrentPosition())/100)))/(1 + Math.pow( 10,((target - M0_2.getCurrentPosition())/100)))));
        telemetry.addData("clamp ",D1.getState());
        telemetry.addData("slide ",D0.getState());
        telemetry.update();
        //dick


    }


    //init sequence
    @Override
    public void init() {
        //Add Motors
        M0 = hardwareMap.get(DcMotor.class,"M0");
        M1 = hardwareMap.get(DcMotor.class,"M1");
        M2 = hardwareMap.get(DcMotor.class,"M2");
        M3 = hardwareMap.get(DcMotor.class,"M3");
        M0_2 = hardwareMap.get(DcMotor.class,"M0_2");
        S0 = hardwareMap.get(Servo.class,"S0");
        D0 = hardwareMap.get(DigitalChannel.class,"D0");
        D1 = hardwareMap.get(DigitalChannel.class,"D1");

        //Set Motors
        M0.setDirection(DcMotor.Direction.FORWARD);
        M0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M1.setDirection(DcMotor.Direction.FORWARD);
        M1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M2.setDirection(DcMotor.Direction.FORWARD);
        M2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M3.setDirection(DcMotor.Direction.FORWARD);
        M3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    //loop while init
    @Override
    public void init_loop() {

    }

    //runs once after start is pressed
    @Override
    public void start(){
        target = 0;

    }

    //looping program after start
    @Override
    public void loop() {
        MoveDriveTrain();
        ServoClamp();

    }

}
