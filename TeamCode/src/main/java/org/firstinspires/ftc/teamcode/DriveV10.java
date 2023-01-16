package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Config
@TeleOp(name = "DriveV10", group="Linear Opmode")
public class DriveV10 extends Statics {

    public static double d1 = 7;
    public static boolean usepreset = true;
    public static double slideoffset = 750;
    public static double reverseoffset = 3;
    public static double offset = 14;
    Pose2d autopose2;


    public void runOpMode() {
        StaticInit(false,d1,slideoffset,reverseoffset,offset);


        slidecalibrated = false;
        target = 800;
        waitForStart();


        rrinnit();
        autopose2 = (PoseStorage.autoPose);
        drive.setPoseEstimate(autopose2);
        if (usepreset) Init();
        while (opModeIsActive()) {
            Slide();
            UI();
            manual();
            ServoTrigger();
        }
    }
    public void ServoTrigger() {

        if (D5.getState() == false && D1.getDistance(DistanceUnit.INCH) < 1.75 && atwall){
            ServoClamp();
        }
        if (gamepad1.right_bumper ) {
            ServoClamp();
        }
        if (gamepad1.left_bumper) {
            drop();
        }
    }
    public void Init(){
        drive.setPoseEstimate(autopose2);
        if (autopose2.getX() > 0) {
            traj = drive.trajectorySequenceBuilder(autopose2)
                    .lineToLinearHeading(new Pose2d(65, -12, Math.toRadians(0)))
                    .build();
        }else{
            traj = drive.trajectorySequenceBuilder(autopose2)
                    .lineToLinearHeading(new Pose2d(-65, -12,Math.toRadians(180)))
                    .build();
        }
        drive.followTrajectorySequenceAsync(traj);
        while (Math.abs(gamepad1.left_stick_x) < .5
                && Math.abs(gamepad1.left_stick_y) < .5
                && Math.abs(gamepad1.right_stick_x) < .5
                && Math.abs(gamepad1.right_stick_y) < .5
                && drive.isBusy()
                && !isStopRequested()
                && Math.abs(gamepad1.right_trigger) < .5
                && Math.abs(gamepad1.left_trigger) < .5) {
            drivestack();
            Slide();
            UI();
        }

    }




}


