package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.Auto14.autopose;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Config
@TeleOp(name = "DriveV10", group="Linear Opmode")
public class DriveV10 extends Statics {

    public static double d1 = 9;
    public static boolean usepreset = true;
    public static double slideoffset = 750;
    public static double reverseoffset = 8.9;
    public static double offset = 14;

    public void runOpMode() {
        StaticInit(false,d1,slideoffset,reverseoffset,offset);


        if (position == 2){
            wcordset = 1;
            w = 1;
            xm = 1;
        } else {
            wcordset = 4;
            w = 4;
            xm = -1;
        }
        ycordset = ycord[0];
        xcordset = xm*xcord[0];
        target = 800;



        waitForStart();
        if (usepreset) Init();
        math(xcordset,ycordset,wcordset,false);
        math(xcordset,ycordset,wcordset,false);
        rrinnit();
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
    }
    public void Init(){
        drive.setPoseEstimate(autopose);
        if (autopose.getX() > 0) {
            traj = drive.trajectorySequenceBuilder(autopose)
                    .lineToLinearHeading(new Pose2d(65, -12, Math.toRadians(0)))
                    .build();
        }else{
            traj = drive.trajectorySequenceBuilder(autopose)
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


