package frc.robot.lib;

import java.lang.Math;

public class MotionUtil {

    public static double deg2rad(double deg){
        return deg * Math.PI / 180;
    }

    public static double rad2deg(double rad){
        return rad * 180 / Math.PI;
    }

    public static double getSignedMinAngleDiff(double angleOne, double angleTwo){
        double rawDiff = angleTwo - angleOne;
        return Math.atan2(Math.cos(rawDiff), Math.sin(rawDiff));
    }
}