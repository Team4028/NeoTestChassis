package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Chassis extends Subsystem {

    private CANSparkMax _rightMaster = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax _leftMaster = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax _rightSlave = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax _leftSlave = new CANSparkMax(4, MotorType.kBrushless);

    private CANEncoder _rightEncoder = new CANEncoder(_rightMaster);
    private CANEncoder _leftEncoder = new CANEncoder(_leftMaster);

    private CANPIDController _rightpidController = new CANPIDController(_rightMaster);
    private CANPIDController _leftpidController = new CANPIDController(_leftMaster);
    // Constants
    private static final int kTimeoutMilliseconds = 5;
    private static final double INCHES_PER_ENCODER_COUNT = 120/54.77; //Empirical
    private static final double kVelocityConversionFactor = 0.0399605247193; //empirical


    private static final Chassis _instance = new Chassis();

    private Chassis(){
        ConfigMotors();
    };

    public Chassis getInstance(){
        return _instance;
    }

    public void ConfigMotors()
    {
        int currentLimit = 60;
        _leftSlave.follow(_leftMaster);
        _rightSlave.follow(_rightMaster);
        _rightMaster.setInverted(true);
        _rightSlave.setInverted(true);
        _leftMaster.setInverted(false);
        _leftSlave.setInverted(false);
        _leftMaster.setCANTimeout(kTimeoutMilliseconds);
        _leftSlave.setCANTimeout(kTimeoutMilliseconds);
        _rightMaster.setCANTimeout(kTimeoutMilliseconds);
        _rightSlave.setCANTimeout(kTimeoutMilliseconds);
        setIsBrakeMode(true);
        setRampRate(1.2);
        _leftMaster.setSmartCurrentLimit(currentLimit);
        _rightMaster.setSmartCurrentLimit(currentLimit);
        _leftSlave.setSmartCurrentLimit(currentLimit);
        _rightSlave.setSmartCurrentLimit(currentLimit);
        _rightMaster.burnFlash();
        _leftMaster.burnFlash();
        _leftSlave.burnFlash();
        _rightSlave.burnFlash();
        _leftEncoder.setPositionConversionFactor(INCHES_PER_ENCODER_COUNT);
        _rightEncoder.setPositionConversionFactor(INCHES_PER_ENCODER_COUNT);
        _leftEncoder.setVelocityConversionFactor(kVelocityConversionFactor);
        _rightEncoder.setVelocityConversionFactor(kVelocityConversionFactor);
        zeroEncoders();
    }

    public void setIsBrakeMode(boolean isBrakeMode){
        if(isBrakeMode){
            _leftMaster.setIdleMode(IdleMode.kBrake);
            _leftSlave.setIdleMode(IdleMode.kBrake);
            _rightMaster.setIdleMode(IdleMode.kBrake);
            _rightSlave.setIdleMode(IdleMode.kBrake);
        } else {
            _leftMaster.setIdleMode(IdleMode.kCoast);
            _leftSlave.setIdleMode(IdleMode.kCoast);
            _rightMaster.setIdleMode(IdleMode.kCoast);
            _rightSlave.setIdleMode(IdleMode.kCoast);
        }
    }

    public void setRampRate(double rr, boolean isOpen){
        if (isOpen){
            _leftMaster.setOpenLoopRampRate(rr);
            _rightMaster.setOpenLoopRampRate(rr);
            _leftSlave.setOpenLoopRampRate(rr);
            _rightSlave.setOpenLoopRampRate(rr);
        } else {
            _leftMaster.setClosedLoopRampRate(rr);
            _rightMaster.setClosedLoopRampRate(rr);
            _leftSlave.setClosedLoopRampRate(rr);
            _rightSlave.setClosedLoopRampRate(rr);
        }
    }

    public void setRampRate(double rampRate){
        _leftMaster.setOpenLoopRampRate(rampRate);
        _leftSlave.setOpenLoopRampRate(rampRate);
        _rightMaster.setOpenLoopRampRate(rampRate);
        _rightSlave.setOpenLoopRampRate(rampRate);
    }

    public void zeroEncoders(){
        _leftEncoder.setPosition(0);
        _rightEncoder.setPosition(0);
    }

    public void setLeftRight(double left, double right){
        _leftMaster.set(left);
        _rightMaster.set(right);
    }

    public void arcadeDrive(double throttle, double turn){
        setLeftRight(.7 * throttle - .3 * turn, .7 * throttle + .3 * turn);
    }

    public double getLeftPos(){
        return _leftEncoder.getPosition();
    }

    public double getRightPos(){
        return _rightEncoder.getPosition();
    }

    public double getLeftVelo(){
        return _leftEncoder.getVelocity();
    }

    public double getRightVelo(){
        return _rightEncoder.getVelocity();
    }

    public double getFwdVelo(){
        return .5 * (getLeftVelo() + getRightVelo());
    }

    @Override
    public void initDefaultCommand(){}
}
