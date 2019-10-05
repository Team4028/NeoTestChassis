package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

/**
 * This class exposes the OnBoard Navigation Sensor Lead Student:
 */
public class GyroNavX {
    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	// define class level working variables
	private AHRS _navXSensor;
		
	private static GyroNavX _instance = new GyroNavX();
	
	public static GyroNavX getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private GyroNavX() {	
		try {          
			_navXSensor = new AHRS(SPI.Port.kMXP); // Communication via RoboRIO MXP (SPI) 
		  } catch (RuntimeException ex ) {
			  DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		  }
	}
	
    public double getYaw() { 
	return _navXSensor.getYaw();
	}
	
	public void zeroYaw() { 
		_navXSensor.zeroYaw(); 
	}
	
	public double getPitch() { //Axis Perpendicular to the Front/Back of the robot
	return _navXSensor.getPitch();
	}
}
