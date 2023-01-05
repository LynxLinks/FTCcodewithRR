package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMax1;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMax2;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMin1;
import static org.firstinspires.ftc.teamcode.TestServos.UmbrellaMin2;
import static org.firstinspires.ftc.teamcode.TestServos.camBothClosed;
import static org.firstinspires.ftc.teamcode.TestServos.camBothOpen;

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

    public static double d1 = 11.5;//11.2
    public static double stagger = 0;//1
    public static double parkyoffset = 1;//-2
    public static double dwall = 16.5;
    public static double dwall2 = -6;
    public static double dwall3blue = 32;
    public static double dwall3red = 26;
    public static double ywall = 50;
    public static double ywall2 = 50;//50
    public static double ywalloffsetred = 0;//4
    public static double ywalloffsetblue = 6;//4
    public static double slideoffset = 400;
    boolean useiteration = true;

    public static Pose2d pose = new Pose2d();
    public static Pose2d autopose = new Pose2d();
    boolean savepos;
    double distance;

    int i;
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model2.tflite";
    private static final String[] LABELS = {"1", "2", "3"};
    private static final String VUFORIA_KEY = "AW9YHO7/////AAABmac9s9LxrUDQuPRw+qlK4+1ZyAszPO7ouIyCLjm98NVZSbtunzGw0u8sSmhuTWNKjpGUxGCkMqV1mVUxgl9h4/J0GxK6120V5SfAcPH2XO17MGzFAm421Lcixendmv2WpyNU3HqERp0Og+sWFVwQTMM5f9rPnzsGiOVSOJ3xgZ9vltV2yHIrYxq1X95szieyo3xGab+kyy4mP5gWgu4VYwqffKb+nhXQ28jTzYhkqTbmE1saub+9juGnkNbqolX3A82q6/jrpIq1a/Nx5Egebwv1ItuABv0lq0gQJ4MiAGOf6czB9FnreVCYxSA4bUvCEZYxUG9RPgbczmU6eW85/wDskT3+1vMYR+BoqqwSa0mr";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    String zone = "3";

    int[] ycord;
    int[] xcord;


    public void runOpMode() {
        StaticInit(true,d1,xcord,ycord,useiteration,slideoffset);
        S0.setPosition(camBothClosed);
        S1.setPosition(UmbrellaMin1); //.7
        S2.setPosition(UmbrellaMax2); //.03
        initVuforia();
        initTfod();
        if (!D5.getState()) { // audience side
            if (D2.getDistance(DistanceUnit.INCH) > D4.getDistance(DistanceUnit.INCH)) position = 1;
            else position = 2;
            xcord = new int[]{1,1,0};
            ycord = new int[]{2,3,2};
        }else{ //non audience side
            if (D2.getDistance(DistanceUnit.INCH) < D4.getDistance(DistanceUnit.INCH)) position = 3;
            else position = 4;
            xcord = new int[]{1,2,0};
            ycord = new int[]{2,2,2};
        }
        StaticInit(true,d1,xcord,ycord,useiteration,slideoffset);
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
            if (position == 1 || position == 4){telemetry.addData("distance", D4.getDistance(DistanceUnit.INCH));}
            else{
                //telemetry.addData("distance", D2.getDistance(DistanceUnit.INCH));
                telemetry.addData("distance", distance);
            }
            if (D2.getDistance(DistanceUnit.INCH) < 20||D4.getDistance(DistanceUnit.INCH)<20||(D2.getDistance(DistanceUnit.INCH) > 36&&D4.getDistance(DistanceUnit.INCH) > 36)){
                telemetry.addData("", "CHECK DISTANCE SENSORS");}
            if (!D0.getState()){telemetry.addData("", "//////////SLIDE FALSE///////////");}
            telemetry.update();
        }
        drive.followTrajectorySequenceAsync(init1);
        drive.update();
        while (drive.isBusy()
                && !isStopRequested()) {
            drivestack();
            Slide();
        }
        Cycle();
        Park();
    }

    public void Init() {
        double distanceholder = 0;
        int count = 0;
        target = 800;
        if (position == 1 || position == 2) {
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(-90)));
            if (position == 1) {
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
                y1 = 2;
                o1 = Math.toRadians(-90);
                x2 = (distance - dwall2);
                y2 = ywall;
                o2 = Math.toRadians(0);

            } else {
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
                y1 = 2;
                o1 = Math.toRadians(-90);

                x2 = -(distance - dwall2);
                y2 = ywall;
                o2 = Math.toRadians(180);

            }
            init1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(x1, y1, o1))
                    .splineToSplineHeading(new Pose2d(x2, y2, o2), o2)
                    .build();
            /*drive.followTrajectorySequenceAsync(init1);
            drive.update();
            while (drive.isBusy()
                    && !isStopRequested()) {
                drivestack();
                Slide();
            }*/
        }


        if (position == 3 || position == 4) {
            S0.setPosition(camBothOpen);
            if (position == 4) {
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
                pose = (new Pose2d( dwall3blue - distance, 0, Math.toRadians(-90)));
                o2 = Math.toRadians(135);
                o4 = Math.toRadians(0);
                x4 = 32;
                y4 = ywall2+ywalloffsetblue;
            }if (position == 3) {
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
                pose = (new Pose2d(  distance - dwall3red, 0, Math.toRadians(-90)));
                o2 = Math.toRadians(45);
                o4 = Math.toRadians(180);
                x4 = -32;
                y4 = ywall2+ywalloffsetred;

            }
            y1 = ywall2 - 18;
            x2 = 12.5 * Math.cos(o2);
            y2 = ywall2 + (12.5 * Math.sin(o2));
            y3 = 4;


            drive.setPoseEstimate(pose);
            init1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(-90)))
                    .back(y1)
                    .addDisplacementMarker(() -> {
                        target = hdata[7];
                        S1.setPosition(UmbrellaMax1); //.7
                        S2.setPosition(UmbrellaMin2); //.03
                    })
                    .splineToSplineHeading(new Pose2d(x2, y2, o2), o2)
                    //.build();
            //init2 = drive.trajectorySequenceBuilder(init1.end())
                    .addDisplacementMarker(() -> {
                        drop();
                    })
                    .back(y3)
                    .addDisplacementMarker(() -> {
                        target = 650;
                    })
                    .splineToSplineHeading(new Pose2d(x4, y4, o4), o4)
                    .build();

            /*drive.followTrajectorySequenceAsync(init1);
            drive.update();
            while (drive.isBusy()
                    && !isStopRequested()) {
                drivestack();
                Slide();
            }
            drop();
            drive.followTrajectorySequenceAsync(init2);
            drive.update();
            while (drive.isBusy()
                    && !isStopRequested()) {
                drivestack();
                Slide();
            }*/
        }



    }


    public void Park() {

        if (position == 1 || position == 4){
            if (zone == "1") park = 12;
            if (zone == "2") park = 36;
            if (zone == "3") park = 58;
            //vopark = Math.toRadians(180);
            vopark = Math.toRadians(-135);
        }if (position == 2 || position == 3) {
            if (zone == "1") park = -58;
            if (zone == "2") park = -36;
            if (zone == "3") park = -12;
            //vopark = Math.toRadians(0);
            vopark = Math.toRadians(-45);
        }
        drive.setPoseEstimate(prevpose);
        parktraj = drive.trajectorySequenceBuilder(prevpose)
                //.back(2)
                .back(8)
                .addDisplacementMarker(() ->{target = 500;})
                //.splineToSplineHeading(new Pose2d(park,vy+parkyoffset,vopark),vopark)
                .lineToLinearHeading(new Pose2d(park,vy+parkyoffset,vopark))
                .build();
        drive.followTrajectorySequenceAsync(parktraj);
        while( drive.isBusy()){
            drivestack();
            Slide();
        }
        autopose = drive.getPoseEstimate();
    }
    public void Cycle(){
        math(xcordset,ycordset,wcordset,true);
        math(xcordset,ycordset,wcordset,true);
        boolean center = false;
        for(int i = 0;i < xcord.length; i++){
            if (i ==1) {
                center = true;
            }else{
                center = false;
            }

            xcordset = xm * xcord[i];
            ycordset = ycord[i];
            if (!center){
                ServoClamp();
            }
            Drive(xcordset,ycordset,wcordset,true,center);
            if (i < xcord.length -1) {
                Drive(xcordset,ycordset,wcordset,true,center);
                //Slam();
                }
            else {drop();}
        }
    }
    /*public void Slam(){
        drive.setPoseEstimate(new Pose2d());
        traj = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(dslam)
                .build();
        drive.followTrajectorySequenceAsync(traj);
        drive.update();
        while( drive.isBusy()){
            drivestack();

            Slide();}
    }*/
    public void IdentifyVuforia(){
        telemetry.clear();
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                //telemetry.addData("# Objects Detected", updatedRecognitions.size());
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    //telemetry.addData("GoTo", recognition.getLabel(), recognition.getConfidence() * 100 );
                    zone = recognition.getLabel();
                }

            }
        }
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
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

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }


}




