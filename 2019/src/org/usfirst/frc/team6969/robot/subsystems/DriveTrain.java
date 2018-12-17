/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.subsystems;

import org.usfirst.frc.team6969.robot.OI;
import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.commands.TeleOpDrive;
import org.usfirst.frc.team6969.robot.commands.TestMovementCommand;

//import org.usfirst.frc.team6969.robot.GyroItg3200;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrain extends Subsystem {
	private static DifferentialDrive robotDrive;
	private static boolean goHalfSpeed;
	private static boolean goFullSpeed;
	private static int leftYAxis;
	private static final boolean invertRight = false;// use these if one side of robot is moving backwards
	private static final boolean invertLeft = false;//see above
	private static final boolean stopWhenHitTargetAngle = false;//if true, isFinished returns true when the robot reaches the target angle.
	private static final boolean keepCommandWhenFinished = true;//keep trying to reach target angle even if you reached it
	//if (absolute value) |targetAngle - angle| < angleErrorThreshold, robot will stop turning (finichedMoving = true)
	private static final double angleErrorThreshold = 1;
	private static final double movementErrorThreshold = 1;//!\ no units specified yet, when movement is implemented add units.

	//if (absolute value) |targetAngle - angle| < maxSpeedAtWhatError, the speed is multiplied by|targetAngle - angle| < maxSpeedAtWhatError
	private static final double maxSpeedAtWhatError = 45; 
	private static final double maxSpeedAtWhatDist = 2;//for movement (not turning, moving forward/backward)
	//a multiplier for speed. Probably should be within the range [0, 1]
	private static final double turnPower = 1;

	public static double targetAngle = 0;// change to rotate robot. (the robot tries to reach this angle, relative to gyro angle)
	public static boolean finishedMoving = false;// is the robot currently turning? (see angleErrotThreshold)
	private static DifferentialDrive drive;// reference needed to drive

	//these variables will be used when the robot automatically moves forward a target distance. Not in use now. No units specified
	private double targetDist;
	private double coveredDist;
	private static int rightYAxis;
//	private static TestMovementCommand test;//cheese solution
	public static AnalogGyro gyro;
	//public static ADXL345_I2C accelerometer; 
	
    public void initDefaultCommand() {
    	robotDrive =  RobotMap.drive;
    	gyro = RobotMap.gyro;
    	//accelerometer = RobotMap.accelerometer;
        goHalfSpeed = false;
        goFullSpeed = false;
        leftYAxis = Robot.m_oi.leftYAxis;
        rightYAxis = Robot.m_oi.rightYAxis;
//        test = new TestMovementCommand();
//        test.start();//cheese solution
        setDefaultCommand(new TeleOpDrive());
    }
    
    public void takeJoystickInputs(OI oi) {

    	if(robotDrive == null ) // prevents robotDrive from being null
    	{
    		this.initDefaultCommand();
    	}
    	//Speed Controls
    	if(oi.leftBumper.get())
    			goHalfSpeed = true;
    	if(!oi.leftBumper.get())
    			goHalfSpeed = false;
    	if(oi.rightBumper.get())
    			goFullSpeed = true;    		
    	if(!oi.rightBumper.get())
    			goFullSpeed = false;
    	
    	
    	//Sets motor speeds
    	if(!goHalfSpeed && !goFullSpeed) { //going 0.75 speed (NORMAL)
	    	robotDrive.tankDrive(oi.getController().getRawAxis(leftYAxis) * -1 * 0.75, oi.getController().getRawAxis(rightYAxis) * -1 *  0.75);
    	}
    	if(goHalfSpeed) {
    		robotDrive.tankDrive(oi.getController().getRawAxis(leftYAxis) * -1 * 0.5, oi.getController().getRawAxis(rightYAxis) * -1 * 0.5);
    	}
    	if(goFullSpeed) {
    		robotDrive.tankDrive(oi.getController().getRawAxis(leftYAxis) * -1 , oi.getController().getRawAxis(rightYAxis) * -1);
    	}
    	
    	if (oi.circleButton.get())
    	{
    		//insert any code here
    		
//    		turn(90);
    	}
    	RobotMap.clawMotor.set(1);
		RobotMap.forkliftMotor.set(1);
    }
    
//    private double findShortestAngleBetween(double target, double value) {
//		double temp = target - value;//get angle between
//		//find shortest angle between (e.g., if target is 0 and value is 360, make sure it returns 0, not -360)
//		temp += (temp > 180) ? -360 : (temp < -180) ? 360 : 0;
//		return temp;
//	}
//    
//    public void turn(double power) {
//		//calculate the relative angle, the angle given by gyro compared to target angle
//		double readAngle = gyro.getAngle()%360;//don't allow angle to exceed 360 by dividing by 360 and using remainder
//		double error = findShortestAngleBetween(targetAngle, readAngle);
//		//check if robot has reached target position (if so, set finishedMoving = true)
//		if (error > -angleErrorThreshold && error < angleErrorThreshold) {
//			finishedMoving = true;
//			return;
//			//if robot is not at target rotation, check if finishedMoving needs to be set to false (only if keepCommandWhenFinished = true)
//		} else if (keepCommandWhenFinished) {
//			finishedMoving = false;
//		}
//
//		if (finishedMoving)
//			return;//if finishedMoving, you don't need to execute any movement code
//		//set left and right speeds to power. Useful if robot needs to move while correcting its rotation
//		double leftSpeed = power;
//		double rightSpeed = power;
//
//		//turn the robot (case 1, robot needs to turn clockwise)
//		if (error > 0) {
//			double absError = error;
//
//			// Turn left by setting the left track to high speed and right to negative
//			// speed. The speed is the max speed, unless the error is smaller than
//			// maxSpeedAtWhatError, then the speed is error/maxSpeedAtWhatError
//			leftSpeed += absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
//			rightSpeed -= absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
//		}
//		//turn the robot (case 1, robot needs to turn counter-clockwise)
//		if (error < 0) {
//			double absError = error * -1;
//			// Turn right by setting the right track to high speed and left to negative
//			// speed. The speed is the max speed, unless the error is smaller than
//			// maxSpeedAtWhatError, then the speed is error/maxSpeedAtWhatError
//
//			leftSpeed -= absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
//			rightSpeed += absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
//		}
//
//		rightSpeed = invertRight ? -rightSpeed : rightSpeed;
//		leftSpeed = invertLeft ? -leftSpeed : leftSpeed;
//
//		// clamp
//		if (rightSpeed > 1)
//			rightSpeed = 1;
//		if (rightSpeed < -1)
//			rightSpeed = -1;
//		if (leftSpeed > 1)
//			leftSpeed = 1;
//		if (leftSpeed < -1)
//			leftSpeed = -1;
//
//		// drive the robot using the values calculated.
//		// either this or the drive.tankDrive below can be used.
//
//		// set motor speeds
//		//		    RobotMap.driveTrainLeftFront.set(leftSpeed);
//		//		    RobotMap.driveTrainLeftBack.set(leftSpeed);
//		//		    
//		//		    RobotMap.driveTrainRightFront.set(rightSpeed);
//		//		    RobotMap.driveTrainRightBack.set(rightSpeed);
//
//		drive.tankDrive(leftSpeed, rightSpeed);// drive the robot using the values calculated
//
//	}
    
    //Arcade drive rather than take drive
    public void takeArcadeInputs(double speed, double zRotation) {
    	robotDrive.arcadeDrive(speed, zRotation);
    }
    
    public void stop() {
    	robotDrive.stopMotor();
    }
}