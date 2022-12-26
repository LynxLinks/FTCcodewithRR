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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

@Config
@Autonomous(name="Auto13", group="Linear Opmode")

public class Auto13 extends Statics {

    public static double d1 = 10.5;
    public static double stagger = .8;
    public static double parkyoffset = 2;
    public static double dwall = 16;
    public static double dwall2 = -2;
    public static double ywall = 50;
    public static double dslam = 5;
    public static int[] xcord = new int[]{1,1,0};
    public static int[] ycord = new int[]{2,3,2};
    boolean useiteration = true;

    public static Pose2d autopose = new Pose2d();
    boolean savepos;

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model2.tflite";
    private static final String[] LABELS = {"1", "2", "3"};
    private static final String VUFORIA_KEY = "AW9YHO7/////AAABmac9s9LxrUDQuPRw+qlK4+1ZyAszPO7ouIyCLjm98NVZSbtunzGw0u8sSmhuTWNKjpGUxGCkMqV1mVUxgl9h4/J0GxK6120V5SfAcPH2XO17MGzFAm421Lcixendmv2WpyNU3HqERp0Og+sWFVwQTMM5f9rPnzsGiOVSOJ3xgZ9vltV2yHIrYxq1X95szieyo3xGab+kyy4mP5gWgu4VYwqffKb+nhXQ28jTzYhkqTbmE1saub+9juGnkNbqolX3A82q6/jrpIq1a/Nx5Egebwv1ItuABv0lq0gQJ4MiAGOf6czB9FnreVCYxSA4bUvCEZYxUG9RPgbczmU6eW85/wDskT3+1vMYR+BoqqwSa0mr";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    String zone = "3";


    public void runOpMode() {
        StaticInit(true,d1,xcord,ycord,useiteration);
        S0.setPosition(camBothClosed);
        S1.setPosition(UmbrellaMin1); //.7
        S2.setPosition(UmbrellaMax2); //.03
        initVuforia();
        initTfod();
        if (D1.getDistance(DistanceUnit.INCH)>2) { // audience side
            if (D2.getDistance(DistanceUnit.INCH) > D4.getDistance(DistanceUnit.INCH)) position = 1;
            else position = 2;
        }else{ //non audience side
            if (D2.getDistance(DistanceUnit.INCH) < D4.getDistance(DistanceUnit.INCH)) position = 3;
            else position = 4;
        }
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 12.0);
        }
        FtcDashboard.getInstance().startCameraStream(tfod, 0);
        while (!isStarted()) {
            IdentifyVuforia();
        }
        rrinnit();
        Init();
        Cycle();
        Park();
    }

    public void Init() {

        target = 900;
        if (position == 1 || position == 2) {
            if (position == 1) {
                xm = 1;
                wcordset = 4;
                x1 = (D4.getDistance(DistanceUnit.INCH) - dwall);
                y1 = 2;
                o1 = Math.toRadians(-90);
                x2 = (D4.getDistance(DistanceUnit.INCH) - dwall2);
                y2 = ywall;
                o2 = Math.toRadians(0);

            } else{
                xm = -1;
                wcordset = 1;
                x1 = -(D2.getDistance(DistanceUnit.INCH) - dwall);
                y1 = 2;
                o1 = Math.toRadians(-90);

                x2 = -(D2.getDistance(DistanceUnit.INCH) - dwall2);
                y2 = ywall;
                o2 = Math.toRadians(180);

            }
            drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(-90)));
            init1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(x1,y1,o1))
                    .splineToSplineHeading(new Pose2d(x2, y2, o2), o2)
                    .build();
            drive.followTrajectorySequenceAsync(init1);
            drive.update();
            while( drive.isBusy()
                    && !isStopRequested()){
                drive.update();
                Slide();
            }


        }
        if (position == 3) {
            savepos = false;
            xm = -1;
            xcordset = 1;
            ycordset = 3;
            wcordset = 3;
            Drive(xcordset,ycordset,wcordset,savepos);
            xcordset = -1;
            ycordset = 3;
            wcordset = 1;
            math(xcordset,ycordset,wcordset,savepos);
            math(xcordset,ycordset,wcordset,savepos);
            Drive(xcordset,ycordset,wcordset,savepos);

        } if (position == 4){
            savepos = false;
            xm = 1;
            xcordset = -1;
            ycordset = 3;
            wcordset = 2;
            Drive(xcordset,ycordset,wcordset,savepos);
            xcordset = 1;
            ycordset = 3;
            wcordset = 4;
            math(xcordset,ycordset,wcordset,savepos);
            math(xcordset,ycordset,wcordset,savepos);
            Drive(xcordset,ycordset,wcordset,savepos);
        }

    }
    public void Park() {

        if (position == 1){
            if (zone == "1") park = 12;
            if (zone == "2") park = 36;
            if (zone == "3") park = 58;
            vopark = Math.toRadians(0);
        }if (position == 2) {
            if (zone == "1") park = -58;
            if (zone == "2") park = -36;
            if (zone == "3") park = -12;
            vopark = Math.toRadians(180);
        }if (position == 3){

        }if (position == 4){

        }
        drive.setPoseEstimate(prevpose);
        parktraj = drive.trajectorySequenceBuilder(prevpose)
                .back(2)
                .addDisplacementMarker(() ->{target = 800;})
                .splineToSplineHeading(new Pose2d(park,vy+parkyoffset,vopark),vopark)
                .build();
        drive.followTrajectorySequenceAsync(parktraj);
        while( drive.isBusy()){
            drive.update();
            Slide();
        }
        autopose = drive.getPoseEstimate();
    }
    public void Cycle(){
        savepos = true;
        math(xcordset,ycordset,wcordset,savepos);
        math(xcordset,ycordset,wcordset,savepos);
        for(int i = 0;i < xcord.length; i++){

            xcordset = xm * xcord[i];
            ycordset = ycord[i];
           ServoClamp();
            Drive(xcordset,ycordset,wcordset,savepos);
            if (i < xcord.length -1) {Drive(xcordset,ycordset,wcordset,savepos);Slam();}
            else {drop();}
        }
    }
public void Slam(){
    drive.setPoseEstimate(new Pose2d());
    traj = drive.trajectorySequenceBuilder(new Pose2d()).forward(dslam).build();
    drive.followTrajectorySequenceAsync(traj);
    drive.update();
    while( drive.isBusy()){drive.update();Slide();}
}
    public void IdentifyVuforia(){
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData("GoTo", recognition.getLabel(), recognition.getConfidence() * 100 );
                    zone = recognition.getLabel();
                }
                telemetry.addData("zone",zone);
                //telemetry.addLine("working");
                telemetry.addData("position",position);
                telemetry.addData("", "");
                telemetry.addData("right", D2.getDistance(DistanceUnit.INCH));
                telemetry.addData("left", D4.getDistance(DistanceUnit.INCH));


                //telemetry.addData("xm",xm);
                telemetry.update();
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




