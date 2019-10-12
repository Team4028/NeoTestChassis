package frc.robot.lib;

import jaci.pathfinder.Trajectory;

public class PathFollower {
    
    double kp, ki, kd, kv, ka;
    double last_error, heading;
    int segment;
    Trajectory trajectory;

    public PathFollower(Trajectory trag){
        trajectory = trag;
    }

    public PathFollower(){}

    public void setTrajectory(Trajectory traj){
        trajectory = traj;
        reset();
    }

    public void configurePIDVA(double kp_new, double ki_new, double kd_new, double kv_new, double ka_new){
        kp = kp_new;
        ki = ki_new;
        kd = kd_new;
        kv = kv_new;
        ka = ka_new;
    }

    public double getKp(){
        return kp;
    }

    public void setKp(double kp_new){
        kp = kp_new;
    }

    public double getKi(){
        return ki;
    }

    public void setKi(double ki_new){
        ki = ki_new;
    }

    public double getKd(){
        return kd;
    }

    public void setKd(double kd_new){
        kd = kd_new;
    }

    public double getKv(){
        return kv;
    }

    public void setKv(double kv_new){
        kv = kv_new;
    }

    public double getKa(){
        return ka;
    }

    public void setKa(double ka_new){
        ka = ka_new;
    }


    public void reset(){
        last_error = 0;
        segment = 0;
    }

    public double calculate(double distance_covered){
        if(segment < trajectory.length()){
            Trajectory.Segment seg = trajectory.get(segment);
            double error = seg.position - distance_covered;
            double calculated_val = kp * error + kd * ((error - last_error)/(seg.dt) - seg.velocity) + kv * seg.velocity + ka * seg.acceleration;
            last_error = error;
            heading = seg.heading;
            segment++;
            return calculated_val;
        } else {
            return 0;
        }
    }

    public double getHeading(){
        return heading;
    }

    public double getError(){
        return last_error;
    }

    public Trajectory.Segment getSegment(){
        Trajectory.Segment seg = new Trajectory.Segment(0, 0, 0, 0, 0, 0, 0, 0);
        try {
            seg = trajectory.get(segment);
        } catch (ArrayIndexOutOfBoundsException e){
            //What's happening?
        }
        return seg;
    }

    public boolean isFinished(){
        return segment >= trajectory.length();
    }

    
}