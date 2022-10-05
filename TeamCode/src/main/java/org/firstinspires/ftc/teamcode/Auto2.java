package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Auto2", group="Linear Opmode")

public class Auto2 extends OpMode {

    //Define Motors
    double target;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D1;


    public void ServoClamp() {

        //if (D0.getState() == true) S0.setPosition(.63);

        if (gamepad1.left_bumper) S0.setPosition(0);
        if (gamepad1.right_bumper) S0.setPosition(.51);
        if((D1.getState() == true) && (gamepad1.left_bumper == false)) S0.setPosition(.51);
    }

    //init sequence
    @Override
    public void init() {
        S0 = hardwareMap.get(Servo.class,"S0");
        S0.setPosition(.51);

    }
    @Override
    public void init_loop() {

    }

    //runs once after start is pressed
    @Override
    public void start(){
        M0_2 = hardwareMap.get(DcMotor.class,"M0_2");

        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        S0 = hardwareMap.get(Servo.class,"S0");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory to = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(-32.5, 0),Math.toRadians(90))
                .splineTo(new Vector2d(-32.5, 11), Math.toRadians(90))
                .build();
       /* Trajectory from = drive.trajectoryBuilder(to.end())
                .splineToLinearHeading(new Pose2d(-18, 0, Math.toRadians(0)), Math.toRadians(135))
                .strafeTo(new Vector2d(0, 0))
                .build();

        */
       // S0.setPosition(.51);
        target = 2250;
        while(Math.abs(target - M0_2.getCurrentPosition()) >10) {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
        }
        M0_2.setPower(0);
        drive.followTrajectory(to);
        S0.setPosition(0.0);
            //open clamp
            //lower slide
          //  drive.followTrajectory(from);
            //while limit == false
                //drive forward
            //repeat



    }

    //looping program after start
    @Override
    public void loop() {
    }

}
