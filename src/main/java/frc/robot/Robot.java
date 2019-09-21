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

import frc.robot.Chassis;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  Chassis _chassis = Chassis.getInstance();
  XboxController controller = new XboxController(0);
  double autoDistance = 0;
  double fixedVBus = .3;
  double fixedVBusTime = 2.5;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    _chassis.ConfigMotors();
  }

  @Override
  public void autonomousInit() {
    autonInitChassis();
  }

  @Override
  public void autonomousPeriodic() {
    _chassis.updateAuton();
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

    public void autonInitChassis(){
      _chassis.zeroEncoders();
      _chassis.initAutonStartTime();
      if (autoDistance > 0){
        _chassis.configDriveSetDistance(autoDistance);
      } else if (fixedVBus != 0) {
        _chassis.configDriveFixedVBus(fixedVBus, fixedVBusTime);
      } else {
        _chassis.configDoNothing();
      }
    }

    public void teleopChassis(){
      double throttle = -controller.getY(Hand.kLeft);
      double turn = -controller.getX(Hand.kRight);
      _chassis.arcadeDriveCmd(throttle, turn);
      if (controller.getAButtonReleased()){
        _chassis.zeroEncoders();
      }
      if (controller.getBButtonReleased()){
        printEncoderPositions();
      }
    }

    public void printEncoderPositions(){
      System.out.println("LEFT POS:  " + _chassis.getLeftEncoderPos());
      System.out.println("RIGHT POS: " + _chassis.getRightEncoderPos());
    }
}
