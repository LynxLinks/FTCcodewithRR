package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.Auto14.autopose;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMax1;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMax2;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMin1;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMin2;
import static org.firstinspires.ftc.teamcode.TestServos.camBothClosed;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Config
@TeleOp(name = "DriveV10", group="Linear Opmode")
public class DriveV10 extends Statics {

    public static double d1 = 11;
    int[] xcord = new int[]{-1,0,-1,0,1,0};
    int [] ycord = new int[]{3,2,1,1,2,1,2};
    public static boolean useiteration = false;
    public static boolean usepreset =false;
    public static boolean savepos = true;
    public static double slideoffset = 750;
    public static double reverseoffset = 10;
    public static double offset = 14;

    public void runOpMode() {
        StaticInit(false,d1,xcord,ycord,useiteration,slideoffset,0,reverseoffset,offset);
        S0.setPosition(camBothClosed);
        S1.setPosition(UmbrellaMin1); //.7
        S2.setPosition(UmbrellaMax2); //.03

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
        if (usepreset) Init();


        waitForStart();
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
        if (gamepad1.right_bumper ) {
            if (atwall) {
                ServoClamp();
            } else {
                drop();
            }
        }
        if (D5.getState() == false && D1.getDistance(DistanceUnit.INCH) < 1.75 && atwall){
            ServoClamp();
        }
    }
    public void Init(){
        drive.setPoseEstimate(autopose);
        traj = drive.trajectorySequenceBuilder(autopose)
                .lineToLinearHeading(new Pose2d(-65, -12,vopark))
                .build();
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


