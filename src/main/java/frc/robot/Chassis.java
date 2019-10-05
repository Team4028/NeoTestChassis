package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.Timer;

public class Chassis{
    /*
    private static final Chassis _instance = new Chassis();

    public enum autonState {DRIVE_SET_DISTANCE, DO_NOTHING, FIXED_VBUS, GET_MAX_VELO};

    private double _fixedAutoVBUS = 0;
    public autonState _mAutonState = autonState.DO_NOTHING;
    public double _targetPosition = 0;
    private double _autonStartTime = -1;
    private double _fixedVBusMaxTime = -1;

    private boolean flag = false;

    private final double kDriveSetDistanceErrorEpsilon = .5; //inches

    private CANSparkMax _leftMaster = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax _leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax _rightMaster = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax _rightSlave = new  CANSparkMax(4, MotorType.kBrushless);

    final double ENCODER_COUNTS_PER_MOTOR_ROTATION = 224; //encoder counts per rotation
    final double GEARBOX_RATIO = 8.45; //unitless //Ratio of Wheel rotations per gear rotations
    final double WHEEL_DIAMETER = 6; //inches
    final double INCHES_PER_ENCODER_COUNT = 120/54.77; //WHEEL_DIAMETER * Math.PI * GEARBOX_RATIO / ENCODER_COUNTS_PER_MOTOR_ROTATION; //This is an Empirical Value
    final double kVelocityConversionFactor = 0.0399605247193; //empirical
    final int kTimeoutMilliseconds = 5;
    final double kMaxVelo = 190.0; //Empirical //191.2 exactly measured but this constant is unused as of right now. 
    final double kNativeAccelConversionFactor = GEARBOX_RATIO * Math.PI * WHEEL_DIAMETER / 60; // Rotation seconds per minute inch

    final double kStraightMaxSpeedRampTime = 10;
    final double kStraightMaxAccelRPMPS = (kMaxVelo / kStraightMaxSpeedRampTime) / kNativeAccelConversionFactor;
    final double kMult_straight =  1;
    final double kP_straight =  .07 * kMult_straight;
    final double kI_straight = 0. * kMult_straight;
    final double kD_straight = 0. * kMult_straight;
    final double kFF_straight = 0. * kMult_straight;

    final double kMaxStraightErrorMotorRot = kDriveSetDistanceErrorEpsilon / (Math.PI * WHEEL_DIAMETER * GEARBOX_RATIO);
    final double kMaxVelocityRPM = .6 * kMaxVelo * 60 / (Math.PI * WHEEL_DIAMETER * GEARBOX_RATIO);
    CANEncoder _leftEncoder = new CANEncoder(_leftMaster);
    CANEncoder _rightEncoder = new CANEncoder(_rightMaster);
  
    CANPIDController _straightLeftPIDController = new CANPIDController(_leftMaster);
    CANPIDController _straightRightPIDController = new CANPIDController(_rightMaster);

    private Chassis(){}

    public static Chassis getInstance(){
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

    public void setRampRate(double rampRate){
        _leftMaster.setOpenLoopRampRate(rampRate);
        _leftSlave.setOpenLoopRampRate(rampRate);
        _rightMaster.setOpenLoopRampRate(rampRate);
        _rightSlave.setOpenLoopRampRate(rampRate);
    }

    public double getLeftCmd(double mthrottle, double mturn){
      return .7 * mthrottle - .25 * mturn;
    }

    public double getRightCmd(double mthrottle, double mturn){
      return .7 * mthrottle + .25 * mturn;
    }

    public void setLeftRightCmd(double leftCmd, double rightCmd){
      _leftMaster.set(leftCmd);
      _rightMaster.set(rightCmd);
    }

    public void arcadeDriveCmd(double throttleCmd, double turnCmd){
        setLeftRightCmd(getLeftCmd(throttleCmd, turnCmd), getRightCmd(throttleCmd, turnCmd));
      }
  
      public void zeroEncoders(){
        _leftEncoder.setPosition(0);
        _rightEncoder.setPosition(0);
      }
  
      public double getLeftEncoderPos(){
        return _leftEncoder.getPosition();
      }
  
      public double getRightEncoderPos(){
        return _rightEncoder.getPosition();
      }

      public void configDoNothing(){
        _mAutonState = autonState.DO_NOTHING;
      }

      public void configDriveFixedVBus(double vbus, double time){
          _fixedAutoVBUS = vbus;
          _fixedVBusMaxTime = time;
          _mAutonState = autonState.FIXED_VBUS;
      }

      public void updateDriveFixedVBus(){
          setLeftRightCmd(_fixedAutoVBUS, _fixedAutoVBUS);
      }

      public void setStraightPIDFF(double[] pidff){
          _straightLeftPIDController.setP(pidff[0]);
          _straightRightPIDController.setP(pidff[0]);
          _straightLeftPIDController.setI(pidff[1]);
          _straightRightPIDController.setI(pidff[1]);
          _straightLeftPIDController.setD(pidff[2]);
          _straightRightPIDController.setD(pidff[2]);
          _straightLeftPIDController.setFF(pidff[3]);
          _straightRightPIDController.setFF(pidff[3]);
      }

      public void configDriveSetDistance(double targetPos){
          double[] pidffConstants = {kP_straight, kI_straight, kD_straight, kFF_straight};
          setStraightPIDFF(pidffConstants);
          _straightLeftPIDController.setSmartMotionMaxAccel(kStraightMaxAccelRPMPS, 0);
          _straightRightPIDController.setSmartMotionMaxAccel(kStraightMaxAccelRPMPS, 0);
          _straightLeftPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
          _straightRightPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
          _straightLeftPIDController.setSmartMotionAllowedClosedLoopError(kMaxStraightErrorMotorRot, 0); //rot
          _straightRightPIDController.setSmartMotionAllowedClosedLoopError(kMaxStraightErrorMotorRot, 0); //rot
          _straightLeftPIDController.setSmartMotionMinOutputVelocity(0, 0); //should be defaulted to 0 anyway. Should be nonnegative
          _straightRightPIDController.setSmartMotionMinOutputVelocity(0, 0); //should be defaulted to 0 anyway, should be nonnegative
          _straightLeftPIDController.setSmartMotionMaxVelocity(kMaxVelocityRPM, 0); //this isn't for limiting--this is cruise velocity
          _straightRightPIDController.setSmartMotionMaxVelocity(kMaxVelocityRPM, 0); //this isn't for limiting--this is cruise velocity
          _targetPosition = targetPos;
          _mAutonState = autonState.DRIVE_SET_DISTANCE;
      }

      public void updateDriveSetDistance(){
        _straightLeftPIDController.setReference(_targetPosition, ControlType.kSmartMotion, 0);
        _straightRightPIDController.setReference(_targetPosition, ControlType.kSmartMotion, 0);
        System.out.println("ERROR: " + Double.toString(_targetPosition - .5 * (_leftEncoder.getPosition() + _rightEncoder.getPosition())));
        System.out.println("MOTOR OUTPUT: " + Double.toString(.5 * (_leftMaster.getAppliedOutput() + _rightMaster.getAppliedOutput())));
    }

      public void initAutonStartTime(){
        _autonStartTime = Timer.getFPGATimestamp();
      }

      public boolean isDriveFixedVBusFinished(){
          if (_autonStartTime == -1 || _fixedVBusMaxTime == -1){
              return true;
          } else {
            return Timer.getFPGATimestamp() - _autonStartTime > _fixedVBusMaxTime;    
        }
      }

      public boolean isDriveSetDistanceFinished(){
          if ((Math.abs(_leftEncoder.getPosition() - _targetPosition) < kDriveSetDistanceErrorEpsilon) 
                    && (Math.abs(_rightEncoder.getPosition() - _targetPosition) < kDriveSetDistanceErrorEpsilon)){
                        System.out.println("PID LOOP TERMINATING");
                        return true;
          } else {
              return false;
          }
      }

      private void stop(){
          setLeftRightCmd(0, 0);
          setIsBrakeMode(true);
      }

      public void updateAuton(){
          switch (_mAutonState) {
            case DO_NOTHING:
                stop();
            case DRIVE_SET_DISTANCE:
                if (!isDriveSetDistanceFinished()){
                    updateDriveSetDistance();
                } else {
                    stop();
                    _mAutonState = autonState.DO_NOTHING;
                }
            case FIXED_VBUS:
                if (! isDriveFixedVBusFinished()){
                    updateDriveFixedVBus();
                } else {
                    flag = true;
                    stop();
                    _mAutonState = autonState.DO_NOTHING;
                }
          }
      }

      public double getFwdVelo(){
          return .5 * (_leftEncoder.getVelocity() + _rightEncoder.getVelocity());
      }

      public boolean getFlag(){
          return flag;
      }
    */
}