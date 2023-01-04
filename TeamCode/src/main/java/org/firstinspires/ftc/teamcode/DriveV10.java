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

    public static double d1 = 11.5;
    int[] xcord = new int[]{-1,0,-1,0,1,0};
    int [] ycord = new int[]{3,2,1,1,2,1,2};
    public static boolean useiteration = false;
    public static boolean usepreset =false;
    public static boolean savepos = true;
    public static double slideoffset = 750;

    public void runOpMode() {
        StaticInit(true,d1,xcord,ycord,useiteration,slideoffset);
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

        if (usepreset) Init();

        math(xcordset,ycordset,wcordset,savepos);
        math(xcordset,ycordset,wcordset,savepos);
        waitForStart();
rrinnit();
        while (opModeIsActive()) {
            Slide();
            UI();
            manual();
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
    public void manual(){

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y*.5 - 0.1*gamepad1.right_stick_y,
                        -gamepad1.left_stick_x*.5- 0.1*gamepad1.right_stick_x,
                        gamepad1.left_trigger - gamepad1.right_trigger
                )
        );
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

    public void UI() {


        //Manual Servo
        if (gamepad1.left_bumper) {
            S0.setPosition(camBothClosed);
        }

        //Manual Slide
        if (gamepad1.a) target = starget;
        if (gamepad1.b) target = 2275;
        if (gamepad1.y) target = 1750;
        if (gamepad1.x) target = 1350;

        //Mauanl Umbrella
        if (gamepad2.left_stick_button){ //down
            S1.setPosition(UmbrellaMax1); //.7
            S2.setPosition(UmbrellaMin2); //.03
        }if (gamepad2.right_stick_button){ // up
            S1.setPosition(UmbrellaMin1); //.02
            S2.setPosition(UmbrellaMax2); ;//.7
        }

        //coordinates
        if (!gamepad2.dpad_up) dup = true;
        if (!gamepad2.dpad_down) ddown = true;
        if (!gamepad2.dpad_left) dleft = true;
        if (!gamepad2.dpad_right) dright = true;
        if (!gamepad2.right_bumper) dbright = true;
        if (!gamepad2.left_bumper) dbleft = true;
        if (!gamepad1.dpad_right) dright2 = true;
        if (!gamepad1.dpad_left) dleft2 = true;

        if (gamepad2.x && preset < xcord.length) gx = true;
        if (gamepad2.y && preset > 1) gy = true;
        if (gamepad2.right_bumper && wcordset < 4 && dbright) {
            dbright = false;
            wcordset += 1;
        }
        if (gamepad2.left_bumper && wcordset > 1 && dbleft) {
            dbleft = false;
            wcordset -= 1;
        }
        if ((gamepad2.dpad_up) && dup && wcordset < 4) {
            dup = false;
            ycordset += 1;
        }
        if ((gamepad2.dpad_down) && ddown) {
            ddown = false;
            ycordset -= 1;
        }
        if ((gamepad2.dpad_right) && dright) {
            dright = false;
            xcordset += 1;
        }
        if ((gamepad2.dpad_left) && dleft) {
            dleft = false;
            xcordset -= 1;

        }
        if((gamepad2.x) && gx){
            preset += 1;
            xcordset = xm*xcord[preset-1];
            ycordset = ycord[preset-1];

            gx = false;
        }
        if((gamepad2.y) && gy){
            preset -= 1;
            xcordset = xm*xcord[preset-1];
            ycordset = ycord[preset-1];

            gy = false;
        }
        if(gamepad2.right_stick_y > .5){
            beacon = true;
        }
        if(gamepad2.right_stick_y < -.5){
            beacon = false;
        }
        if(gamepad2.a){
            xcordset = 0;
            ycordset = 2;
        }
        if((gamepad1.dpad_right) && dright2){
            dright2 = false;
            //usedistance = true;
            Drive(xcordset,ycordset,wcordset,savepos,true);

        }
        if((gamepad1.dpad_left) && dleft2){
            dleft2 = false;
            //usedistance = false;
            Drive(xcordset,ycordset,wcordset,savepos,false);

        }
        if(gamepad2.right_trigger > 0.6){
            slidecalibrated = false;
        }
        if(!gamepad2.b) Bbutton = true;
        if(gamepad2.b && Bbutton){
            Bbutton = false;
            math(xcordset,ycordset,wcordset,savepos);
        }


        telemetry.addData("x", xcordset);
        telemetry.addData("", "");
        telemetry.addData("y", ycordset);
        telemetry.addData("", "");
        telemetry.addData("w", wcordset);
        telemetry.addData("", "");
        telemetry.addData("atwall", atwall);
        telemetry.addData("", "");
        telemetry.addData("beacon", beacon);
        telemetry.update();

    }
}


