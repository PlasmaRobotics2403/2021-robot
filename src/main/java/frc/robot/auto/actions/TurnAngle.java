package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive;
import frc.robot.auto.util.Action;
/**
 *
 */
public class TurnAngle implements Action {

	double speed;
	double angle;
	double angleDiff;
	
	double speedPValue;
	double minSpeed;
	double fullSpeedTime;
	
	double fullStartTime;
	
	Drive drive;
	
	public TurnAngle(double speed, double angle, Drive drive){
		if(angle >=0){
			this.speed = Math.abs(speed); //speed is default left
		}
		else{
			this.speed = -Math.abs(speed);
		}
		this.angle = angle;
		this.drive = drive;
		speedPValue = .005;
		minSpeed = .3 * this.speed / Math.abs(this.speed);
		fullSpeedTime = .12;
	}
	
	@Override
	public boolean isFinished() {
		SmartDashboard.putNumber("angleDiff", angleDiff);
		SmartDashboard.putNumber("Turn Angle" , 1);
		SmartDashboard.putNumber("gyro", drive.getGyroAngle());
		return Math.abs(angleDiff) < 2;
	}

	@Override
	public void start() {
		drive.resetEncoders();
		while(Math.abs(drive.getDistance()) > 1){
			drive.resetEncoders();
			DriverStation.reportWarning("broke", false);
			}
		drive.zeroGyro();
		angleDiff = angle;
		fullStartTime = Timer.getFPGATimestamp();
	}

	@Override
	public void update() {
		angleDiff = angle + drive.getGyroAngle();
		if(Timer.getFPGATimestamp() - fullStartTime < fullSpeedTime){
			drive.autonTankDrive(speed, -speed);
			DriverStation.reportWarning("forced full", false);
		}
		else if(Math.abs(angleDiff) * speedPValue < Math.abs(minSpeed)){ //Prop slowdown speeds would be slower than min speed
			drive.autonTankDrive(minSpeed, -minSpeed);
			DriverStation.reportWarning("min", false);
		}
		else if(Math.abs(angleDiff) * speedPValue < Math.abs(speed)){ //Prop slowdown speeds would be slower than max speed
			drive.autonTankDrive(speedPValue * angleDiff, speedPValue * -angleDiff);
			DriverStation.reportWarning("slow", false);
		}
		else{
			drive.autonTankDrive(speed, -speed);
			DriverStation.reportWarning("full", false);
		}
	}
	

	//@Override
	public void end() {
		drive.stopDrive();
		DriverStation.reportWarning("done", false);
	}

}