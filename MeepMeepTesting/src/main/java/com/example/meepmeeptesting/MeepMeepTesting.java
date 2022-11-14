package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);
       double d = 11.5;
        double Sdrop = 200;

        int y = 2;
        int x = 0;
        int w = 4;
        double target;
        double vy = 0;
        double vx = 0;
        double vo = 0;
        double ix = 0;
        double iy = 0;
        double io = 0;
        double x1;
        double x2;
        double x3;
        double y1;
        double y2;
        double y3;
        double o1;
        boolean atwall = true; //used to know whether to run to or from
        boolean yfirst = false;
        boolean dup;
        boolean ddown;
        boolean dright;
        boolean dleft;
        boolean dbright;
        boolean dbleft;
        boolean dslide;
        boolean slidecalibrated;
        boolean beenoff;
        Pose2d currentpose;
        if(w == 1){
            ix = -65;
            iy = -12;
            io = Math.toRadians(180);
        }
        if(w == 2){
            ix = -12;
            iy = -65;
            io = Math.toRadians(-90);
        }
        if(w == 3){
            ix = 12;
            iy = -65;
            io = Math.toRadians(-90);
        }
        if(w == 4){
            ix = 65;
            iy = -12;
            io = 0;
        }
        int[] hdata = new int[]{400, 1300, 400, 1300, 400,
                1300, 1950, 2550, 1950, 1300,
                400, 1300, 400, 1300, 400,
                1300, 1950, 2550, 1950, 1300,
                400, 1300, 400, 1300, 400};
        if (atwall) {
            if(w == 1){
                vx = 24 * x - 12;
                if (y >= 3){
                    vo = Math.toRadians(135);
                    vy = 24 * (y - 3) -12;
                }
                else{
                    vo = Math.toRadians(-135);
                    vy = 24 * (y - 2) -12;
                }
            }
            if(w == 2){
                vy = 24 * (y -3) - 12;
                if (x >= 0){
                    vo = Math.toRadians(45);
                    vx = 24 * x -12;
                }
                else{
                    vo = Math.toRadians(135);
                    vx = 24 * (x + 1) -12;
                }
            }
            if(w == 3){
                vy = 24 * (y -3) - 12;
                if (x >= 1){
                    vo = Math.toRadians(45);
                    vx = 24 * (x - 1) + 12;
                }
                else{
                    vo = Math.toRadians(135);
                    vx = 24 * x  + 12;
                }
            }
            if(w == 4){
                vx = 24 * x + 12;
                if (y >= 3){
                    vo = Math.toRadians(45);
                    vy = 24 * (y - 3) - 12;
                }
                else{
                    vo = Math.toRadians(-45);
                    vy = 24 * (y - 2) -12;
                }
            }
            if(yfirst){
                x1 = ix;
                y1 = vy;
                x2 = vx;
                y2 = vy;
                x3 = vx + d * Math.cos(vo);
                y3 = iy + d * Math.sin(vo) ;
                o1 = vo;
            }
            else{
                x1 = vx;
                y1 = iy;
                x2 = vx;
                y2 = vy;
                x3 = vx + d * Math.cos(vo);
                y3 = iy + d * Math.sin(vo) ;
                o1 = vo;
            }
            currentpose = new Pose2d(ix,iy,io);
        }
        else {
            if(w == 1){
                ix = -65;
                iy = -12;
                io = Math.toRadians(180);
            }
            if(w == 2){
                ix = -12;
                iy = -65;
                io = Math.toRadians(-90);
            }
            if(w == 3){
                ix = 12;
                iy = -65;
                io = Math.toRadians(-90);
            }
            if(w == 4){
                ix = 65;
                iy = -12;
                io = 0;
            }
            if(yfirst){
                x2 = vx;
                y2 = iy;

            }
            else{
                x2 = ix;
                y2 = vy;
            }

            x1 = vx;
            y1 = vy;
            x3 = ix;
            y3 = iy;
            o1 = io;

            currentpose = new Pose2d(vx + d*Math.cos(vo),vy + d*Math.sin(vo),vo);
        }





        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(30, 20, Math.toRadians(100), Math.toRadians(100), 6.7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(currentpose)
                                .lineToLinearHeading(new Pose2d(x1,y1,o1))
                                .lineToLinearHeading(new Pose2d(x2,y2,o1))
                                .lineToLinearHeading(new Pose2d(x3,y3,o1))
                                .build());



                        /*.forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .build()*/



        // Declare out second bot


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)

                .start();
    }
}