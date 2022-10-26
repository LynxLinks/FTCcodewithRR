package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled
@TeleOp(name="color test", group="Linear Opmode")

public class Auto1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        ColorSensor C1;
        C1 = hardwareMap.get(ColorSensor.class, "C1");
        C1.enableLed(true);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .back(19)
                .build();

        drive.followTrajectory(traj1);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("red",C1.red());
            telemetry.update();
        }
    }
}