package frc.robot.auto.modes;

import frc.robot.auto.actions.IntakeRoller;
import frc.robot.auto.actions.SetTurretPosition;
import frc.robot.auto.actions.Shoot;
import frc.robot.auto.actions.SpinUp;
import frc.robot.auto.actions.Straight;
import frc.robot.auto.actions.Tracking;
import frc.robot.auto.actions.extendIntake;
import frc.robot.auto.actions.followTrajectory;
import frc.robot.auto.actions.gyroAngle;
import frc.robot.auto.util.AutoMode;
import frc.robot.auto.util.AutoModeEndedException;

import javax.sound.midi.Track;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

//import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Drive;
import frc.robot.Intake;
import frc.robot.Shooter;
import frc.robot.Turret;



/**
 *
 */
public class Bounce extends AutoMode {
	Drive driveTrain;
	Turret turret;
	Shooter shooter;
	Intake intake;
	NetworkTable table;

    public Bounce(Drive driveTrain, Turret turret, Shooter shooter, Intake intake, NetworkTable table) {
		this.driveTrain = driveTrain;
		this.turret = turret;
		this.shooter = shooter;
		this.intake = intake;
		this.table = table;
    }
	/*
	 * (non-Javadoc)
	 * 
	 * @see org.usfirst.frc.team2403.robot.auto.util.AutoMode#routine()
	 */
	@Override
	protected void routine() throws AutoModeEndedException {
		DriverStation.reportWarning("started Action", false);
		runAction(new Tracking(turret, false, 80));
        runActionsParallel(new followTrajectory(13, driveTrain, intake), new extendIntake(intake, true));
        runAction(new followTrajectory(14, driveTrain, intake));
        runAction(new followTrajectory(15, driveTrain, intake));
        runActionsParallel(new followTrajectory(16, driveTrain, intake), new extendIntake(intake, false));
		
		DriverStation.reportWarning("Finished Action", false);
	}

}