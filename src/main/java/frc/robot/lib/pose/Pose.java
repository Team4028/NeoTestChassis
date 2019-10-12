package frc.robot.lib.pose;

import frc.robot.Constants;
import frc.robot.lib.MotionUtil;

public class Pose{

    public double x; //x position of the COM of the chassis in inches
    public double y; //y position of the COM of the chassis in inches
    public double phi; //angle of the chassis in degrees.

    public Pose(double x_val, double y_val, double phi_val){
        this.x = x_val;
        this.y = y_val;
        this.phi  = phi_val;
    }

    public Pose(double x_val, double y_val, double phi_val, boolean isRadians){
        this.x = x_val;
        this.y = y_val;
        if (isRadians){
            this.phi = phi_val;
        } else {
            this.phi = MotionUtil.deg2rad(phi_val);
        }
    }

    public vec2 getCOMVec(){
        return new vec2(this.x, this.y);
    }

    public static Pose id(){
        return new Pose(0, 0, 0);
    }

    public static final vec2 toFrontRight = new vec2(.5 * Constants.TRACK_LENGTH, -.5 * Constants.TRACK_LENGTH);
    public static final vec2 toFrontLeft = new vec2(.5 * Constants.TRACK_LENGTH, .5 * Constants.TRACK_LENGTH);
    public static final vec2 toBackRight = new vec2(-.5 * Constants.TRACK_LENGTH, -.5 * Constants.TRACK_LENGTH);
    public static final vec2 toBackLeft = new vec2(-.5 * Constants.TRACK_LENGTH, .5 * Constants.TRACK_LENGTH);

    public vec2 getFrontRightVec(){
        return this.getCOMVec().add(toFrontRight.rotated(this.phi));
    }

    public vec2 getFrontLeftVec(){
        return this.getCOMVec().add(toFrontLeft.rotated(this.phi));
    }

    public vec2 getBackRight(){
        return getCOMVec().add(toBackRight.rotated(this.phi));
    }

    public vec2 getBackLeft(){
        return getCOMVec().add(toBackLeft.rotated(this.phi));
    }
}