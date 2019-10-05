package frc.robot;

import java.lang.Math;

public class Util {

    public static double getSignedAngleDist(double angleOne, double angleTwo){
        return Math.atan2(Math.sin(angleOne - angleTwo), Math.cos(angleOne - angleTwo));
    }

    public static double deg2rad(double deg){
        return deg * Math.PI / 180;
    }

    public static double rad2deg(double rad){
        return rad * 180 / Math.PI;
    }

}