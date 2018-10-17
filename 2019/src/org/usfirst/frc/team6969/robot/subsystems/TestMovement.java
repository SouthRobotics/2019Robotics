
/**
 * Best way to test this script would be to disable all other movement.
 * Expected calls to methods:
 * Initialize when robot turns on
 * Execute when robot is on and not finished
 * 
 * ||||If i misunderstood when and which methods are called (above is wrong), this script won't work!||||

 * 
 * isFinished will return true if robot is within angleErrorThreshold if and only if stopWhenHitTargetAngle = true
 * 
 **/

package org.usfirst.frc.team6969.robot.subsystems;

import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.commands.TestMovementCommand;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//All imports below are the default imports that come with the FRC package
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class TestMovement extends Subsystem {

	private static final boolean invertRight = false;// use these if one side of robot is moving backwards
	private static final boolean invertLeft = false;
	private static final boolean stopWhenHitTargetAngle = false;
	private static final boolean keepCommandWhenFinished = true;
	private static final double angleErrorThreshold = 1;
	private static final double maxSpeedAtWhatError = 45;
	private static final double turnPower = 1;

	public static double targetAngle = 0;// change to rotate robot.
	public static boolean finishedMoving = false;// is the robot currently executing a movement or turn?
	private static Gyro gyro;// reference needed to sense rotation
	private static DifferentialDrive drive;// reference needed to drive

	private double targetDist;
	private double coveredDist;
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new TestMovementCommand());
	}

	// Called just before this Command runs the first time
//	protected void initialize() {
//		gyro = RobotMap.gyro;
//		gyro.reset();
//		gyro.calibrate();//unsure if needed, might reset gyro angle
//		drive = RobotMap.drive;
//	}
//
//	// Called repeatedly when this Command is scheduled to run
//	protected void execute() {
//		turn(0);// change 0 to speed of robot (-1, 1) if robot is moving forwards/backwards
//		// during rotation
//
//	}

	// power -- the desired speed of the robot (forward-backward) before turning.
	// Should be 0 unless robot is turning while moving.
	// gyro & drive -- references
//	public void turn(double power) {
//		double error = targetAngle - gyro.getAngle();
//		if (error > -angleErrorThreshold && error < angleErrorThreshold) {
//			finishedMoving = true;
//			return;
//		} else if (keepCommandWhenFinished) {
//			finishedMoving = false;
//		}
//
//		if (finishedMoving)
//			return;
//		
//		double leftSpeed = power;
//		double rightSpeed = power;
//
//		if (error > 0) {
//			double absError = error;
//
//			// Turn left by setting the left track to high speed and right to negative
//			// speed. The speed is the max speed, unless the error is smaller than
//			// maxSpeedAtWhatError, then the speed is error/maxSpeedAtWhatError
//			leftSpeed += absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
//			rightSpeed -= absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
//		}
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
//
//	@Override
//	protected boolean isFinished() {
//		// TODO Auto-generated method stub
//		return finishedMoving && stopWhenHitTargetAngle;
//	}

}

//old code, not deleted in case i want to take parts of it.

///*----------------------------------------------------------------------------*/
///* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
///* Open Source Software - may be modified and shared by FRC teams. The code   */
///* must be accompanied by the FIRST BSD license file in the root directory of */
///* the project.                                                               */
///*----------------------------------------------------------------------------*/
//
//package org.usfirst.frc.team6969.robot.commands;
//
//import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//
//import org.usfirst.frc.team6969.robot.Robot;
//
///**
// * An example command.  You can replace me with your own command.
// */
//public class TestMovement extends Command {
//	class movementObject{//class to store commands for movement (both turning and movement)
//		movementEnum type;//type (see movementEnum)
//		double amount;//amount, if movement in (maybe ft, cm), if turning in degrees
//		
//		public movementObject (movementEnum type, double amount) {//constructor
//			this.type = type;
//			this.amount = amount;
//		}
//	}
//	public enum movementEnum{move, turn};//enum to determine if a movementObject is for moving or turning.
//	
//	public int currentCommandNumber;//the index in movementCommands that the robot is currently exicuting
//	
//	public double howMuchWasMovedSoFar;//keep track of how far was moved
//	
//	public double targetRotation;//the target rotation;
//	
//	public double deltaTargetRotation;//how far from target rotation the robot is
//	
//	private DifferentialDrive drive;//drive to move robot
//	
//	public movementObject[] movementCommands = new movementObject[] {//array to store commands
//			new movementObject(movementEnum.move, 10),
//			new movementObject(movementEnum.turn, 90),
//			new movementObject(movementEnum.move, 2),
//			new movementObject(movementEnum.turn, -90),
//			new movementObject(movementEnum.move, 2),
//	};
//	
//	public TestMovement() {
//		// Use requires() here to declare subsystem dependencies
//		//requires(Robot.m_subsystem);
//		
//		
////		requires(Robot.driveTrain);
//	}
//
//	// Called just before this Command runs the first time
//	@Override
//	protected void initialize() {
//		//Initialization
//		drive = Robot.robotDrive;
//	}
//
//	// Called repeatedly when this Command is scheduled to run
//	@Override
//	protected void execute() {
//		//movement, etc.
//		if(movementCommands[currentCommandNumber].type == movementEnum.move) {
//			move();
//		}
//		if(movementCommands[currentCommandNumber].type == movementEnum.turn) {
//			turn();
//		}
//	}
//	
//	protected void move() {
//		//TODO: move robot
//	}
//	
//	protected void turn() {
//		//TODO: turn robot
//	}
//
//	// Make this return true when this Command no longer needs to run execute()
//	@Override
//	protected boolean isFinished() {
//		return false;
//	}
//
//	// Called once after isFinished returns true
//	@Override
//	protected void end() {
//	}
//
//	// Called when another command which requires one or more of the same
//	// subsystems is scheduled to run
//	@Override
//	protected void interrupted() {
//	}
//}
