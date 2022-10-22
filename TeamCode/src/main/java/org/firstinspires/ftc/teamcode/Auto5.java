package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone.LABELS;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone.TFOD_MODEL_ASSET;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Config
@Autonomous(name="Auto5", group="Linear Opmode")

public class Auto5 extends LinearOpMode{
    double target;
    DcMotor M0_2;
    Servo S0;
    DigitalChannel D1;
    double loop = 0;
    String zone = null;
    FtcDashboard dashboard;
    public static double sensedistance = -19;
    public static double X1 = -51;
    public static double Y1 = -11;
    public static double X2 = -58.5;
    public static double Y2 = 27;
    public static double turncoef = 1;
    public static double slide1 = .27;
    public static double slide2 = -.3;
    public static double slide3 = -.4;
    public static double slide4 = .35;


    public static double slidei = 460;
    public static double slided = 70;

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "AW9YHO7/////AAABmac9s9LxrUDQuPRw+qlK4+1ZyAszPO7ouIyCLjm98NVZSbtunzGw0u8sSmhuTWNKjpGUxGCkMqV1mVUxgl9h4/J0GxK6120V5SfAcPH2XO17MGzFAm421Lcixendmv2WpyNU3HqERp0Og+sWFVwQTMM5f9rPnzsGiOVSOJ3xgZ9vltV2yHIrYxq1X95szieyo3xGab+kyy4mP5gWgu4VYwqffKb+nhXQ28jTzYhkqTbmE1saub+9juGnkNbqolX3A82q6/jrpIq1a/Nx5Egebwv1ItuABv0lq0gQJ4MiAGOf6czB9FnreVCYxSA4bUvCEZYxUG9RPgbczmU6eW85/wDskT3+1vMYR+BoqqwSa0mr";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        S0 = hardwareMap.get(Servo.class,"S0");
        M0_2 = hardwareMap.get(DcMotor.class, "M0_2");
        D1 = hardwareMap.get(DigitalChannel.class,"D1");

        S0.setPosition(.3);

        M0_2.setDirection(DcMotor.Direction.FORWARD);
        M0_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M0_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M0_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Trajectory start1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(X1, 0, Math.toRadians(
                        180)))
                .build();
        Trajectory start2 = drive.trajectoryBuilder(start1.end())
                .strafeLeft(-(Y1))
                .build();
        Trajectory loop1 = drive.trajectoryBuilder(start2.end())
                .forward(Math.abs(X2-X1))
                .build();
        Trajectory loop2 = drive.trajectoryBuilder(loop1.end())
                .back(Math.abs(X2-X1))

                .build();
        Trajectory loop3 = drive.trajectoryBuilder(loop2.end())
                .lineToLinearHeading(new Pose2d(X1, Y2, Math.toRadians(turncoef*    90)))
                .build();
        Trajectory loop35 = drive.trajectoryBuilder(loop3.end())
                .forward(1)
                .build();
        //
        Trajectory loop4 = drive.trajectoryBuilder(loop35.end())
                .lineToLinearHeading(new Pose2d(X1, Y1, Math.toRadians(180)))
                .build();

        Trajectory loopexit = drive.trajectoryBuilder(loop2.end())
                .strafeRight(9.5)
                .build();
        Trajectory loopexit2 = drive.trajectoryBuilder(loopexit.end())
                .back(24)
                .build();
        Trajectory park1 = drive.trajectoryBuilder(loopexit2.end())
                .strafeLeft(24)
                .build();
        Trajectory park3 = drive.trajectoryBuilder(loopexit2.end())
                .strafeRight(24)
                .build();
        while(isStarted()==false){
            FtcDashboard.getInstance().startCameraStream(vuforia, 5);
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
                    telemetry.update();
                }
            }

        }

        // run until the end of the match (driver presses STOP)
        if(isStopRequested()) return;

        telemetry.addData("zone",zone);
        telemetry.update();
        sleep(10000);
        M0_2.setPower(slide1);

        drive.followTrajectory(start1);
        drive.followTrajectory(start2);

        target = 2500;

        while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
        }

        M0_2.setPower(0);
        drive.followTrajectory(loop1);
        S0.setPosition(0.0);
        M0_2.setPower(slide2);
        drive.followTrajectory(loop2);
        while(loop < 2) {
            drive.followTrajectory(loop3);
            drive.followTrajectory(loop35);
            // set target before drop
            /*target = slidei - (slided*loop);
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }*/

            target = slidei - (slided*loop)-160;
            while (Math.abs((target - M0_2.getCurrentPosition())) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }/*
            target = M0_2.getCurrentPosition() - 150;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }*/
            S0.setPosition(0.51);
            M0_2.setPower(slide4);
            while (M0_2.getCurrentPosition() < slidei - (slided*loop)){

            }
            drive.followTrajectory(loop4);

            target = 2500;
            while (Math.abs(target - M0_2.getCurrentPosition()) > 10) {
                M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
            }
            M0_2.setPower(0);
            drive.followTrajectory(loop1);
            S0.setPosition(0.0);
            M0_2.setPower(-.15);
            drive.followTrajectory(loop2);

            loop++;
        }
        target = 0;
        while (target - M0_2.getCurrentPosition() < 1) {
            M0_2.setPower(-1 * ((1 - Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250))) / (1 + Math.pow(10, ((target - M0_2.getCurrentPosition()) / 250)))));
        }
        M0_2.setPower(0);
        drive.followTrajectory(loopexit);
        drive.followTrajectory(loopexit2);

        if (zone.equals("1 Bolt")){
            drive.followTrajectory(park1);
        }
        else if (zone.equals("2 Bulb")){
            //nothing
        }
        else if (zone.equals("3 Panel")){
            drive.followTrajectory(park3);
        }
        //wile limit == false
        //drive forward
        //repeat

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
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
