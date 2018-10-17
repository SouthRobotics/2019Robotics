
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

package org.usfirst.frc.team6969.robot.commands;

import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.subsystems.TestMovement;

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

public class TestMovementCommand extends Command {

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
	
	//need to make constructor
	
	public TestMovement sub;//TODO: assign this, might need to make one in robot.java
	
	public TestMovementCommand() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.testMovement);
		
    	this.initialize(); // after the TeleopDrive object is declared
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		gyro = RobotMap.gyro;
		gyro.reset();
		gyro.calibrate();//unsure if needed, might reset gyro angle
		drive = RobotMap.drive;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		turn(0);// change 0 to speed of robot (-1, 1) if robot is moving forwards/backwards
		// during rotation

	}

	// power -- the desired speed of the robot (forward-backward) before turning.
	// Should be 0 unless robot is turning while moving.
	// gyro & drive -- references
	public void turn(double power) {
		double error = targetAngle - gyro.getAngle();
		if (error > -angleErrorThreshold && error < angleErrorThreshold) {
			finishedMoving = true;
			return;
		} else if (keepCommandWhenFinished) {
			finishedMoving = false;
		}

		if (finishedMoving)
			return;
		
		double leftSpeed = power;
		double rightSpeed = power;

		if (error > 0) {
			double absError = error;

			// Turn left by setting the left track to high speed and right to negative
			// speed. The speed is the max speed, unless the error is smaller than
			// maxSpeedAtWhatError, then the speed is error/maxSpeedAtWhatError
			leftSpeed += absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
			rightSpeed -= absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
		}
		if (error < 0) {
			double absError = error * -1;
			// Turn right by setting the right track to high speed and left to negative
			// speed. The speed is the max speed, unless the error is smaller than
			// maxSpeedAtWhatError, then the speed is error/maxSpeedAtWhatError

			leftSpeed -= absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
			rightSpeed += absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
		}

		rightSpeed = invertRight ? -rightSpeed : rightSpeed;
		leftSpeed = invertLeft ? -leftSpeed : leftSpeed;

		// clamp
		if (rightSpeed > 1)
			rightSpeed = 1;
		if (rightSpeed < -1)
			rightSpeed = -1;
		if (leftSpeed > 1)
			leftSpeed = 1;
		if (leftSpeed < -1)
			leftSpeed = -1;

		// drive the robot using the values calculated.
		// either this or the drive.tankDrive below can be used.

		// set motor speeds
		//		    RobotMap.driveTrainLeftFront.set(leftSpeed);
		//		    RobotMap.driveTrainLeftBack.set(leftSpeed);
		//		    
		//		    RobotMap.driveTrainRightFront.set(rightSpeed);
		//		    RobotMap.driveTrainRightBack.set(rightSpeed);

		drive.tankDrive(leftSpeed, rightSpeed);// drive the robot using the values calculated

	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return finishedMoving && stopWhenHitTargetAngle;
	}
	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}

}
