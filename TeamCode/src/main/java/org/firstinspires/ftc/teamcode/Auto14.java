package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

@Config
@Autonomous(name="Auto14", group="Linear Opmode")

public class Auto14 extends Statics {

    public static double d1 = 7;
    public static double reverseoffset = 8;
    public static double dwall = 17.5;
    public static double dwall2 = -8;
    public static double ywallblue = 64.5;
    public static double ywallred = 63.5;
    public static double slideoffset = 200;
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model2.tflite";
    private static final String[] LABELS = {"1", "2", "3"};
    private static final String VUFORIA_KEY = "AW9YHO7/////AAABmac9s9LxrUDQuPRw+qlK4+1ZyAszPO7ouIyCLjm98NVZSbtunzGw0u8sSmhuTWNKjpGUxGCkMqV1mVUxgl9h4/J0GxK6120V5SfAcPH2XO17MGzFAm421Lcixendmv2WpyNU3HqERp0Og+sWFVwQTMM5f9rPnzsGiOVSOJ3xgZ9vltV2yHIrYxq1X95szieyo3xGab+kyy4mP5gWgu4VYwqffKb+nhXQ28jTzYhkqTbmE1saub+9juGnkNbqolX3A82q6/jrpIq1a/Nx5Egebwv1ItuABv0lq0gQJ4MiAGOf6czB9FnreVCYxSA4bUvCEZYxUG9RPgbczmU6eW85/wDskT3+1vMYR+BoqqwSa0mr";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    String zone = "3";
    int [] xcord = new int[]{1,1,0};
    int [] ycord = new int[]{2,3,2};
    public void runOpMode() {
        StaticInit(true,d1,slideoffset,reverseoffset,offset);
        initVuforia();
        initTfod();
        if (D2.getDistance(DistanceUnit.INCH) > D4.getDistance(DistanceUnit.INCH)) position = 1;
        else position = 2;
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 12.0);
        }
        FtcDashboard.getInstance().startCameraStream(tfod, 0);
        rrinnit();
        Init();
        while (!isStarted()) {
            IdentifyVuforia();
            telemetry.addData("park",zone);
            telemetry.addData("start",position);
            telemetry.addData("right", Math.round(D2.getDistance(DistanceUnit.INCH)));
            telemetry.addData("left", Math.round(D4.getDistance(DistanceUnit.INCH)));
            if (!D0.getState()){telemetry.addData("", "//////////SLIDE FALSE///////////");}
            telemetry.update();

        }
        if (zone == "1") zonei = 1;
        if (zone == "2") zonei = 2;
        if (zone == "3") zonei = 3;
        drive.followTrajectorySequenceAsync(init1);
        drive.update();
        while (drive.isBusy()
                && !isStopRequested()) {
            drivestack();
            Slide();
        }
        Cycle();
    }
    public void Cycle() {

        for (int i = 0; i < xcord.length; i++) {
            ServoClamp();
            M0_2.setPower(.25);
            if (D5.getState()) {
                Drive(xm * xcord[i], ycord[i], wcordset, true);
                if (i < xcord.length - 1) {
                    Drive(xm * xcord[i], ycord[i], wcordset, true);
                } else {
                    drop();
                    target = 500;
                    Park(zonei, wcordset,false);
                }
            } else {

                Park(zonei, wcordset,true);
                break;
            }

        }
    }
    public void Park(int zone, int w,boolean broke){
        if (w == 4) {
            io = Math.toRadians(0);
            if (zone == 1) {
                ix = 12 ;
            }
            if (zone == 2) {
                ix = 36;
            }
            if (zone == 3) {
                ix = 58;
            }
            if (broke) {
                vopark = Math.toRadians(0);
                vopark2 = Math.toRadians(1);
            }else{
                vopark = Math.toRadians(180);
                vopark2 = Math.toRadians(180);
            }
        }else{
            io = Math.toRadians(180);
            if (zone == 1) {
                ix = -58 ;
            }
            if (zone == 2) {
                ix = -36 ;
            }
            if (zone == 3) {
                ix = -12 ;
            }
            if (broke) {
                vopark = Math.toRadians(180);
                vopark2 = Math.toRadians(1);
            }else{
                vopark = Math.toRadians(0);
                vopark2 = Math.toRadians(180);
            }
        }
        traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(ix,-12,vopark))
                .turn(vopark2)
                .build();
        drive.followTrajectorySequenceAsync(traj);
        drive.update();
        while (drive.isBusy()) {
            drivestack();
            Slide();
        }
        PoseStorage.autoPose = drive.getPoseEstimate();
        PoseStorage.initw = initw;
        PoseStorage.initx = initx;
        PoseStorage.inity = inity;
    }
    public void Init() {
        double distanceholder = 0;
        int count = 0;
        target = 800;
        if (position == 1) {
            drive.setPoseEstimate(new Pose2d(0,-ywallblue , Math.toRadians(-90)));

            initw = 4;
            initx = 2;
            inity = 2;
            for(int i = 0;i < 10; i++) {
                distanceholder = D4.getDistance(DistanceUnit.INCH);
                if (distanceholder < 55) {
                    distance += distanceholder;
                    count += 1;
                }
            }
            distance = distance/count;
            xm = 1;
            wcordset = 4;
            x1 = (distance - dwall);
            y1 = 2-ywallblue;
            o1 = Math.toRadians(-90);
            x2 = (distance - dwall2);
            y2 = -12;
            o2 = Math.toRadians(0);
            init1 = drive.trajectorySequenceBuilder(new Pose2d(0, -ywallblue, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(x1, y1, o1))
                    .splineToSplineHeading(new Pose2d(x2, y2, o2), o2)
                    .build();
        } else if (position == 2){
            drive.setPoseEstimate(new Pose2d(0,-ywallred , Math.toRadians(-90)));

            initw = 1;
            initx = -2;
            inity = 2;
            for(int i = 0;i < 10; i++) {
                distanceholder = D2.getDistance(DistanceUnit.INCH);
                if (distanceholder < 55) {
                    distance += distanceholder;
                    count += 1;
                }
            }
            distance = distance/count;
            xm = -1;
            wcordset = 1;
            x1 = -(distance - dwall);
            y1 = 2-ywallred;
            o1 = Math.toRadians(-90);
            x2 = -(distance - dwall2);
            y2 = -12;
            o2 = Math.toRadians(180);
            init1 = drive.trajectorySequenceBuilder(new Pose2d(0, -ywallred, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(x1, y1, o1))
                    .splineToSplineHeading(new Pose2d(x2, y2, o2), o2)
                    .build();
        }
        math2(1, 1, wcordset, true);
        math2(1, 1, wcordset, true);

    }
    public void IdentifyVuforia(){
        telemetry.clear();
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    zone = recognition.getLabel();
                }
            }
        }
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.1f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 330;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }

}