package frc.robot.commands.chassis;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import jaci.pathfiner.Pathfinder;
import sun.jvm.hotspot.code.DebugInfoReadStream;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.modifiers.TankModifier;

import frc.robot.subsystems.Chassis;

import frc.robot.lib.PathFollower;
import frc.robot.lib.ReflectingCSVWriter;


public class FollowPath extends Command implements Sendable {
    PathFollower mLeftFollower;
    PathFollower mRightFollower;
    Supplier<Trajectory> trajectorySupplier;
    Trajectory trajectory;
    TankModifier modifier;
    double angleDiff; 
    boolean isBackwards;
    double startingAngle;

    double leftOutput;
    double rightOutput;

    double followerLoopTime;
    double followerDeltaT;

    double lastTime;
    private Notifier notifier;

    public class DebugInfo {
        public double leftPosition; 
        public double leftVelocity;
        public double leftPositionError;
        public double leftVelocityError;
        public double leftOutput;
        public double leftTargetVelocity;
        public double leftTargetAcceleration;

        public double rightPosition; 
        public double rightVelocity;
        public double rightPositionError;
        public double rightVelocityError;
        public double rightOutput;
        public double rightTargetVelocity;
        public double rightTargetAcceleration;   
    }

    private ReflectingCSVWriter<DebugInfo> writer;
    private DebugInfo debugInfo;
    

	public FollowPath(Supplier<Trajectory> trajectorySupplier) {
		this(trajectorySupplier, false, 0);
	}
	
	public FollowPath(Trajectory trajectory) {
		this(() -> trajectory, false, 0);
	}
	
	public FollowPath(Trajectory trajectory, boolean backwards) {
		this(() -> trajectory, backwards, 0);
	}
	
	public FollowPath(Supplier<Trajectory> trajectorySupplier, double startingAngle) {
		this(trajectorySupplier, false, startingAngle);
	}
	
	public FollowPath(Trajectory trajectory, double startingAngle) {
		this(() -> trajectory, false, startingAngle);
	}
	
	public FollowPath(Trajectory trajectory, boolean backwards, double startingAngle) {
		this(() -> trajectory, backwards, startingAngle);
	}
	 
	public FollowPath(Supplier<Trajectory> trajectorySupplier, boolean backwards, double startingAngle) {
		super("Follow Trajectory");
		this.trajectorySupplier = trajectorySupplier;
		this.isBackwards = backwards;
		this.startingAngle = startingAngle;
		requires(Chassis.getInstance());
    }
    
    @Override
    protected void initialize() {
        notifier = new Notifier(this::notifierExecute);
        this.trajectory = trajectorySupplier.get();
        this.modifier = new TankModifier(trajectory).modify(Constants.kDrive_Motion_)
    }
}


