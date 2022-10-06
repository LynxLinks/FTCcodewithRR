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
    public static double X1 = -40;
    public static double Y1 = 6;
    double loop = 0;

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

    }

    //looping program after start
    @Override
    public void loop() {
        if (loop == 0) {
            M0_2 = hardwareMap.get(DcMotor.class, "M0_2");

            M0_2.setDirection(DcMotor.Direction.FORWARD);
            M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Trajectory to1 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(X1, 0, Math.toRadians(90)))
                    .build();
            Trajectory to2 = drive.trajectoryBuilder(to1.end())
                    .forward(Y1)
                    .build();
            Trajectory loop1 = drive.trajectoryBuilder(to2.end())
                    .forward(8)
                    .build();
            Trajectory loop2 = drive.trajectoryBuilder(loop1.end())
                    .back(8)
                    .build();
            Trajectory from2 = drive.trajectoryBuilder(loop2.end())
                    .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                    .build();

            target = 2400;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }
            M0_2.setPower(0);
            drive.followTrajectory(to1);
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }
            M0_2.setPower(0);
            drive.followTrajectory(to2);
            //S0.setPosition(.10);

            target = 0;
            drive.followTrajectory(from2);
            //while limit == false
            //drive forward
            //repeat
            loop = 1;
        }
    }
}
