package frc.robot.auto.modes;

import frc.robot.auto.actions.IntakeRoller;
import frc.robot.auto.actions.SetTurretPosition;
import frc.robot.auto.actions.Shoot;
import frc.robot.auto.actions.SpinUp;
import frc.robot.auto.actions.Straight;
import frc.robot.auto.actions.Tracking;
import frc.robot.auto.actions.Wait;
import frc.robot.auto.actions.followTrajectory;
import frc.robot.auto.actions.gyroAngle;
import frc.robot.auto.util.AutoMode;
import frc.robot.auto.util.AutoModeEndedException;
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
public class TenBallAuto extends AutoMode {
	Drive driveTrain;
	Turret turret;
	Shooter shooter;
	Intake intake;
	NetworkTable table;

    public TenBallAuto(Drive driveTrain, Turret turret, Shooter shooter, Intake intake, NetworkTable table) {
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
		runActionsParallel(new Tracking(turret, false, 100), new SpinUp(shooter, 16000));
		runActionsParallel(new IntakeRoller(intake, true), new followTrajectory(8, driveTrain, intake));
		runAction(new Tracking(turret, true, 0));
		runAction(new Shoot(turret, shooter, intake, table, 4, 16000));
		runActionsParallel(new Tracking(turret, false, 160), new followTrajectory(9, driveTrain, intake));
		runAction(new followTrajectory(10, driveTrain, intake));
		runAction(new Wait(.05));
		//runActionsParallel(new IntakeRoller(intake, false), new Tracking(turret, true, 0));
		runAction(new Tracking(turret, true, 0));
		//runActionsParallel((new followTrajectory(11, driveTrain, intake), new Shoot(turret, shooter, intake, table, 4, 16000));
		runAction(new followTrajectory(11, driveTrain, intake));
		runActionsParallel(new IntakeRoller(intake, false), new Shoot(turret, shooter, intake, table, 4, 16000));


		//runAction(new Tracking(turret, false, 100));
        //runAction(new SpinUp(shooter, 16000));
        //runAction(new IntakeRoller(intake, true));
        //runAction(new followTrajectory(8, driveTrain, intake));
        //runAction(new Tracking(turret, true, 0));
		//runAction(new Shoot(turret, shooter, intake, table, 4, 16000));
		//runAction(new Tracking(turret, false, 160));
		//runAction(new followTrajectory(9, driveTrain, intake));
		//runAction(new followTrajectory(10, driveTrain, intake));
		//runAction(new IntakeRoller(intake, false));
		//runAction(new Tracking(turret, true, 0));
		//runAction(new followTrajectory(11, driveTrain, intake));
		//runAction(new Shoot(turret, shooter, intake, table, 4, 16000));
		
		
		DriverStation.reportWarning("Finished Action", false);
	}

}