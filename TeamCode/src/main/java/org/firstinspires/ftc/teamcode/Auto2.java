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

        if (gamepad1.left_bumper) S0.setPosition(0);
        if (gamepad1.right_bumper) S0.setPosition(.51);
        if((D1.getState() == true) && (gamepad1.left_bumper == false)) S0.setPosition(.51);
    }

    //init sequence
    @Override
    public void init() {
    }
    @Override
    public void init_loop() {

    }

    //runs once after start is pressed
    @Override
    public void start(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory to = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-18, 0))
                .splineToLinearHeading(new Pose2d(-32, 8, Math.toRadians(135)), Math.toRadians(0))
                .build();
        Trajectory from = drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(-18, 0, Math.toRadians(0)), Math.toRadians(135))
                .strafeTo(new Vector2d(0, 0))
                .build();
        //while t <25
            //close clamp
            //raise slide
            drive.followTrajectory(to);
            //open clamp
            //lower slide
            drive.followTrajectory(from);
            //while limit == false
                //drive forward
            //repeat



    }

    //looping program after start
    @Override
    public void loop() {

    }

}
