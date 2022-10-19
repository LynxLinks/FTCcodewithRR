package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Auto4.X1;
import static org.firstinspires.ftc.teamcode.Auto4.X2;
import static org.firstinspires.ftc.teamcode.Auto4.Y1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.gson.annotations.Until;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.lang.reflect.Array;

//name and class
@Config
@TeleOp(name = "DriveV3", group="Linear Opmode")

public class DriveV3 extends LinearOpMode {

    DcMotor M0;
    FtcDashboard dashboard;
    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D0;
    DistanceSensor D1;
    double yoffset = 5;  //constant added to all y positions
    int y = 0;   //y coordinate input
    int x = 0;   //x coordinate input
    double d = 12;  //diagonal distance forward and backward
    double target; //slide target position
    double vy = 0;  //vector roadrunner x value
    double vx = 0;  //vector roadrunner y value
    double vo = 0;  //target roadrunner theta
    double xi = 0;  //initial robot position against wall in coordinate system, either .5 or -.5
    int[] hdata = new int[]{200,1100,200,1100,200,1100, 1750, 2350, 1750, 1100, 200,1100,200,1100,200,
            1100, 1750, 2350, 1750, 1100,
            200,1100,200,1100,200};
    boolean atwall = true; //used to know whether to run to or from

    boolean start = true; //used to wit untill side chosen


    //public static double x1 = -28;
    //public static double y1 = 12;
    //public boolean initial = true;

    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        //Add Motors
        M0 = hardwareMap.get(DcMotor.class,"M0");
        M1 = hardwareMap.get(DcMotor.class,"M1");
        M2 = hardwareMap.get(DcMotor.class,"M2");
        M3 = hardwareMap.get(DcMotor.class,"M3");
        M0_2 = hardwareMap.get(DcMotor.class,"M0_2");
        S0 = hardwareMap.get(Servo.class,"S0");
        D0 = hardwareMap.get(DigitalChannel.class,"D0");
        D1 = hardwareMap.get(DistanceSensor.class,"D1");

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

        waitForStart();

        while(D0.getState() == true){
            M0_2.setPower(.3);
        }
        while(D0.getState() == false){
            M0_2.setPower(-0.05);
        }
        while (start){
            if (gamepad1.dpad_right){
                xi = .5;
                start = false;
            }
            if (gamepad1.dpad_right){
                xi = -.5;
                start = false;
            }
        }
        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        S0.setPosition(0.0);

        target = 200;

        while (opModeIsActive()) {
            MoveDriveTrain();
            ServoClamp();
            RoadRunner();
        }
    }
    public void Cords(){

    }
    public void Untilslide(){
        while (Math.abs(target - M0_2.getCurrentPosition()) > 10){
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));

        }
        return;
    }
    public void RoadRunner() {
    //change cordinates with gamepad 2
            if (gamepad2.dpad_up) y += 1;
            if (gamepad2.dpad_down) y -= 1;
            if (gamepad2.dpad_left) x -= 1;
            if (gamepad2.dpad_right) y += 1;
            telemetry.addData("x  ",x);
            telemetry.addData("y  ",y);
            telemetry.update();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //vy
        vy = -(yoffset+24*(y-1));
        //vx
        if (x > 0)  { vx = -24*Math.floor(Math.abs(x-xi));}
        else        { vx = 24*Math.floor(Math.abs(x-xi));}
        //vo
        if (x>xi)   {vo = 135;}
        else        {vo = -135;}

        //move to y position and theta
        Trajectory t1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(vy, 0, Math.toRadians(vo)))
                .build();
        //move to x position
        Trajectory t2 = drive.trajectoryBuilder(t1.end())
                .lineToLinearHeading(new Pose2d(vy, vx, Math.toRadians(vo)))
                .build();
        //move diagonal forwards to target junction
        Trajectory t3 = drive.trajectoryBuilder(t2.end())
                .forward(d)
                .build();
        //move diagonal backwards to center of tile
        Trajectory f1 = drive.trajectoryBuilder(t3.end())
                .back(d)
                .build();
        //move to 0 x and 0 theta
        Trajectory f2 = drive.trajectoryBuilder(f1.end())
                .lineToLinearHeading(new Pose2d(vy, 0, 0))  //might have to be Math.toRadians(vo)
                .build();
        //move to 0 y
        Trajectory f3 = drive.trajectoryBuilder(f2.end())
                .lineToLinearHeading(new Pose2d(0, 0, 0))
                .build();
        //move triggered by gamepad 1 a
        if (gamepad1.a) {
            //to junction
           if (atwall){
               //clamp already closed from distance sensor
               //move to y position and theta
               //move to x position
               //set slide height
               //move diagonal to junction
               //reset cordinates to (0,0)
               drive.followTrajectory(t1);
               drive.followTrajectory(t2);
               target = hdata[(x + 5*y + 2)];
               Untilslide();
               drive.followTrajectory(t3);

               atwall = false;
               y = 0;
               x = 0;

           }
           //from junction
           else{
               //drop cone
               //move diagonal back to center of tile
               //move to 0 x
               //lower slide
               //move to 0 y and 0 theta
               S0.setPosition(0.0);
               drive.followTrajectory(f1);
               drive.followTrajectory(f2);
               target = 200;
               Untilslide();
               drive.followTrajectory(f3);
                atwall = true;
           }
        }
    }
    public void ServoClamp() {
        //automatic clamping if within distance
        if((target == 200) && (D1.getDistance(DistanceUnit.METER) <= .033)){
            target = 5;
            Untilslide();
            S0.setPosition(0.3);
            while(D0.getState() == false){
                M0_2.setPower(-0.05);
            }
            M0_2.setDirection(DcMotor.Direction.FORWARD);
            M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void MoveDriveTrain(){
        double yAxis;
        double xAxis;
        double Rotate;
        yAxis = gamepad1.left_stick_y*.8 + gamepad1.right_stick_y/3;
        xAxis = gamepad1.left_stick_x*.8 + gamepad1.right_stick_x/3;
        Rotate = -gamepad1.left_trigger/2 + gamepad1.right_trigger/2;
        M0.setPower((Rotate + (-yAxis + xAxis)));
        M3.setPower((Rotate + (-yAxis - xAxis)));
        M1.setPower(-(Rotate + (yAxis + xAxis)));
        M2.setPower(-(Rotate + (yAxis - xAxis)));
    }
}
