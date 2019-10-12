package frc.robot.lib.pose;


public class vec2{
    public double x;
    public double y;

    public vec2(double x_val, double y_val){
        this.x = x_val;
        this.y = y_val;
    }

    public vec2 add(vec2 other){
        return new vec2(this.x + other.x, this.y + other.y);
    }
 
    //radians counterclockwise
    public vec2 rotated(double rotation){
        double cos = Math.cos(rotation);
        double sin = Math.sin(rotation);
        double x_val = x * cos - y * sin;
        double y_val = x * sin + y * cos;
        return new vec2(x_val, y_val);
    }
}