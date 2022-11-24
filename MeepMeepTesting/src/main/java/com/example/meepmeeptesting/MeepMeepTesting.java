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
        MeepMeep meepMeep = new MeepMeep(1000, 120);

        int y = 5;
        int x = 2;
        int w = 1;

        boolean atwall = true;
        double d = 14;
        double d1 = 11;
        double d2 = 3;
        double vy = 0;
        double vx = 0;
        double vo = 0;
        double starget;
        double target;
        double x1 = 0;
        double x2 = 0;
        double x3 = 0;
        double x4;
        double y1 = 0;
        double y2 = 0;
        double y3 = 0;
        double y4;
        double o1 = 0;
        double o2 = 0;
        double o3 = 0;
        double o4;
        double iy = 0;
        double ix = 0;
        double io = 0;
        double xm = 1;
        Pose2d currentpose;
        double offset = 16;
        double doffset = 8;


        if (w == 1) {
            ix = -65;
            iy = -12;
            io = Math.toRadians(180);
            target = 850;
        }
        if (w == 2) {
            ix = -12;
            iy = -65;
            io = Math.toRadians(-90);
            target = 300;


        }
        if (w == 3) {
            ix = 12;
            iy = -65;
            io = Math.toRadians(-90);
            target = 300;

        }
        if (w == 4) {
            ix = 65;
            iy = -12;
            io = 0;
            target = 850;


        }
        if (y == 6) {
            d = d2;
        } else {
            d = d1;
        }
        if (w == 1) {

            if (y >= 3) {


                vy = 24 * (y - 3) - 12;
                if (x <= -2) {
                    vx = 24 * (x + 1) - 12;
                    vo = Math.toRadians(135);
                } else {


                    vx = 24 * x - 12;
                    vo = Math.toRadians(45);
                }
            } else {
                vy = 24 * (y - 2) - 12;
                if (x <= -2) {
                    vx = 24 * (x + 1) - 12;
                    vo = Math.toRadians(-135);
                } else {
                    vx = 24 * x - 12;
                    vo = Math.toRadians(-45);
                }
            }
            x1 = vx - offset;
            y1 = iy;
            o1 = io;
        }
        if (w == 2) {

            if (x >= 0) {

                vx = 24 * x - 12;
                if (y <= 1) {
                    vy = 24 * (y - 2) - 12;
                    vo = Math.toRadians(-45);
                } else {
                    vy = 24 * (y - 3) - 12;
                    vo = Math.toRadians(45);
                }
            } else {

                vx = 24 * (x + 1) - 12;
                if (y <= 1) {
                    vy = 24 * (y - 2) - 12;
                    vo = Math.toRadians(-135);
                } else {
                    vy = 24 * (y - 3) - 12;
                    vo = Math.toRadians(135);
                }
            }
            x1 = ix;
            y1 = vy - offset;
            o1 = io;
        }
        if (w == 3) {
            if (x >= 1) {
                vx = 24 * (x - 1) + 12;
                if (y <= 1) {
                    vy = 24 * (y - 2) - 12;
                    vo = Math.toRadians(-45);
                } else {
                    vy = 24 * (y - 3) - 12;
                    vo = Math.toRadians(45);
                }

            } else {


                vx = 24 * x + 12;
                if (y <= 1) {
                    vy = 24 * (y - 2) - 12;
                    vo = Math.toRadians(-135);
                } else {
                    vy = 24 * (y - 3) - 12;
                    vo = Math.toRadians(135);
                }
            }
            x1 = ix;
            y1 = vy - offset;
            o1 = io;

        }
        if (w == 4) {

            if (y >= 3) {
                vy = 24 * (y - 3) - 12;
                if (x >= 2) {
                    vx = 24 * (x - 1) + 12;
                    vo = Math.toRadians(45);
                } else {
                    vx = 24 * x + 12;
                    vo = Math.toRadians(135);
                }
            } else {
                vy = 24 * (y - 2) - 12;
                if (x >= 2) {
                    vx = 24 * (x - 1) + 12;
                    vo = Math.toRadians(-45);
                } else {
                    vx = 24 * x + 12;
                    vo = Math.toRadians(-135);
                }


            }
            x1 = vx + offset;
            y1 = iy;
            o1 = io;


        }
        if (y == 6) {
            vx = vx - 2 * xm;
        }


        if (atwall) {
            if (y ==6){
                d = d2;
            }
            else{
                d = d1;
            }
            if(w == 1){

                if (y >= 3){


                    vy = 24 * (y - 3) -12;
                    if (x <= -2){
                        vx = 24 * (x+1) - 12;
                        vo = Math.toRadians(135);
                    }
                    else{


                        vx = 24 * x - 12;
                        vo = Math.toRadians(45);
                    }
                }
                else{
                    vy = 24 * (y - 2) -12;
                    if (x <= -2){
                        vx = 24 * (x+1) - 12;
                        vo = Math.toRadians(-135);
                    }
                    else{
                        vx = 24 * x - 12;
                        vo = Math.toRadians(-45);
                    }
                }
                x1 = vx - offset;
                y1 = iy;
                o1 = io;
            }
            if(w == 2){

                if (x >= 0){

                    vx = 24 * x -12;
                    if (y <= 1){
                        vy = 24 * (y -2) - 12;
                        vo = Math.toRadians(-45);
                    }else{
                        vy = 24 * (y -3) - 12;
                        vo = Math.toRadians(45);
                    }
                }
                else{

                    vx = 24 * (x + 1) -12;
                    if (y <= 1){
                        vy = 24 * (y -2) - 12;
                        vo = Math.toRadians(-135);
                    }else{
                        vy = 24 * (y -3) - 12;
                        vo = Math.toRadians(135);
                    }
                }
                x1 = ix;
                y1 = vy - offset;
                o1 = io;
            }
            if(w == 3) {
                if (x >= 1) {
                    vx = 24 * (x - 1) + 12;
                    if (y <= 1){
                        vy = 24 * (y -2) - 12;
                        vo = Math.toRadians(-45);
                    }else{
                        vy = 24 * (y -3) - 12;
                        vo = Math.toRadians(45);
                    }

                } else {


                    vx = 24 * x + 12;
                    if (y <= 1) {
                        vy = 24 * (y - 2) - 12;
                        vo = Math.toRadians(-135);
                    } else {
                        vy = 24 * (y - 3) - 12;
                        vo = Math.toRadians(135);
                    }
                }
                x1 = ix;
                y1 = vy - offset;
                o1 = io;

            }
            if(w == 4){

                if (y >= 3){
                    vy = 24 * (y - 3) - 12;
                    if (x >= 2){
                        vx = 24 * (x-1) + 12;
                        vo = Math.toRadians(45);
                    }
                    else{
                        vx = 24 * x + 12;
                        vo = Math.toRadians(135);
                    }
                }
                else{
                    vy = 24 * (y - 2) -12;
                    if (x >= 2){
                        vx = 24 * (x-1) + 12;
                        vo = Math.toRadians(-45);
                    }
                    else{
                        vx = 24 * x + 12;
                        vo = Math.toRadians(-135);
                    }


                }
                x1 = vx + offset;
                y1 = iy;
                o1 = io;


            }
            if (y == 6){
                vx = vx -2*xm;
            }


            x2 = vx + d * Math.cos(vo);
            y2 = vy + d * Math.sin(vo) ;
            o2 = vo;
            x3 = vx + d * Math.cos(vo);
            y3 = vy + d * Math.sin(vo) ;
            o3 = vo;

            currentpose = new Pose2d(ix,iy,io);
            atwall = false;
        }
        else {
            if(w == 1){
                ix = -65;
                iy = -12;
                io = Math.toRadians(180);
                starget = 850;
                y2 = iy;
                x2 = vx-offset;
            }
            if(w == 2){
                ix = -12;
                iy = -65;
                io = Math.toRadians(-90);
                starget = 500;
                y2 = vy-offset;
                x2 = ix;
            }
            if(w == 3){
                ix = 12;
                iy = -65;
                io = Math.toRadians(-90);
                starget = 500;
                y2 = vy-offset;
                x2 = ix;

            }
            if(w == 4){
                ix = 65;
                iy = -12;
                io = 0;
                starget = 850;
                y2 = iy;
                x2 = vx+offset;
            }




            x1 = vx + doffset*Math.cos(vo);
            y1 = vy + doffset*Math.sin(vo);
            o1 = vo;

            o2 = io;

            x3 = ix;
            y3 = iy;
            o3 = io;



            currentpose = new Pose2d(vx + d*Math.cos(vo),vy + d*Math.sin(vo),vo);
            atwall = true;
        }














        double finalX = x1;
        double finalY = y1;
        double finalO = o1;
        double finalIo = io;
        double finalX1 = x2;
        double finalY1 = y2;
        double finalO1 = o2;
        Pose2d finalCurrentpose = currentpose;
        double finalX2 = x3;
        double finalY2 = y3;
        double finalO2 = o3;
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints( 62,50, Math.toRadians(320), Math.toRadians(210), 6.7)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(finalCurrentpose)

                               //.lineToLinearHeading(new Pose2d(-12,y2,Math.toRadians(-90)))
                                //.lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(-90)))
                                //
                                //.splineToSplineHeading(new Pose2d(x1,y3,Math.toRadians(45)),Math.toRadians(45))
                                //.build());*/



                                .lineToLinearHeading(new Pose2d(finalX, finalY,finalO))
                                .splineToSplineHeading(new Pose2d(finalX1, finalY1, finalO1), finalO1)
                                .splineToSplineHeading(new Pose2d(finalX2-.01, finalY2-.01, finalO2 +.01), finalO2)
                                //.lineToLinearHeading(new Pose2d(x2 + .01,y2 + .01,o2))
                               // .lineToLinearHeading(new Pose2d(x3 + .01,y3 + .03,o3))
                                //.lineToLinearHeading(new Pose2d(x4,y4,o4))
                                .build());






        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)


                .start();


    }

}