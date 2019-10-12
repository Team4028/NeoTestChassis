package frc.robot.lib.pose;

public class PoseEstimator{
    private PoseEstimator _instance = new PoseEstimator();

    Pose curPose;

    private PoseEstimator(){      
        curPose = Pose.id();  
    }

    public void setPose(Pose p){
        curPose = p;
    }

    public Pose getCurPose(){
        return curPose;
    }

    /**
     * EXPLANATION:
     * We can determine uniquely the new pose of the robot by knowing the distance the robot moved forward and the heading change of the robot.
     * Supposing locally constant velocities for each wheel. Kinematics tells us that left/right wheel rotation kinematics are a linear transformation
     * away from forward/rotation kinematics. In particular, solving for that transformation, we get that v_tau = .5 * (v_r + v_l) and that
     * v_omega = (v_r - v_l) / l, where l is the track width. In particular, we make the local assumption then that angular velocity is constnat.
     * This gives us phi(t) = phi(t0) + v_omega * t, where v_omega is from NavX just phi_t+1 - phi_t / deltaT. Integrating the projection of the
     * constant magnitude velocity vector v_tau =(determinable as above from encoder values) gives us the equation of motion below. 
     */
    public Pose update(double x_dist, double y_dist, double new_heading){
        double old_heading = curPose.phi;
        double dphi = new_heading - old_heading;
        double fwd_dist = .5 * (x_dist + y_dist);
        double dx = fwd_dist * (Math.sin(new_heading) - Math.sin(old_heading)) / dphi;
        double dy = fwd_dist * (Math.cos(old_heading) - Math.cos(new_heading)) / dphi;
        curPose.x += dx;
        curPose.y += dy;
        curPose.phi += dphi;
        return curPose;
    }
}