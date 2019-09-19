/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  CANSparkMax _leftMaster = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax _leftSlave = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax _rightMaster = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax _rightSlave = new  CANSparkMax(4, MotorType.kBrushless);
  XboxController controller = new XboxController(0);

  CANEncoder _leftEncoder = new CANEncoder(_leftMaster);
  CANEncoder _rightEncoder = new CANEncoder(_rightMaster);

  CANPIDController _straightLeftPIDController = new CANPIDController(_leftMaster);
  CANPIDController _straightRightPIDController = new CANPIDController(_rightMaster);
  // Constants
  final double ENCODER_COUNTS_PER_MOTOR_ROTATION = 224; //encoder counts per rotation
  final double GEARBOX_RATIO = 8.45; //unitless //Ratio of Wheel rotations per gear rotations
  final double WHEEL_DIAMETER = 6; //inches
  final double INCHES_PER_ENCODER_COUNT = 120/54.77; //WHEEL_DIAMETER * Math.PI * GEARBOX_RATIO / ENCODER_COUNTS_PER_MOTOR_ROTATION;
  
  final int kTimeoutMilliseconds = 5;

  final int kSraightPIDSlot = 0;
  final double kPStraight = .1;
  final double kIStraight = 0.;
  final double kDStraight = 0.;
  final double kFFStraight = 0;


  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    ConfigMotors();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    teleopChassis();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
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
        System.out.println("IPEC: " + INCHES_PER_ENCODER_COUNT);
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

    public void teleopChassis(){
      double throttle = -controller.getY(Hand.kLeft);
      double turn = -controller.getX(Hand.kRight);
      arcadeDriveCmd(throttle, turn);
      if (controller.getAButtonReleased()){
        zeroEncoders();
      }
      if (controller.getBButtonReleased()){
        printEncoderPositions();
      }
      if (controller.getYButtonReleased()){
        System.out.println("Left Conv Fac: " + _leftEncoder.getPositionConversionFactor());
        System.out.println("Right Conv Fac: " + _rightEncoder.getPositionConversionFactor());
      }
      outputEncoderPositions();
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

    public void outputEncoderPositions(){
      SmartDashboard.putNumber("LEFT POS:  ", getLeftEncoderPos());
      SmartDashboard.putNumber("RIGHT POS: ", getRightEncoderPos());
    }

    public void printEncoderPositions(){
      System.out.println("LEFT POS:  " + getLeftEncoderPos());
      System.out.println("RIGHT POS: " + getRightEncoderPos());
    }
}
