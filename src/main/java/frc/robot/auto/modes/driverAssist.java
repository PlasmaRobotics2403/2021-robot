package frc.robot.auto.modes;

import frc.robot.auto.actions.IntakeRoller;
import frc.robot.auto.actions.SetTurretPosition;
import frc.robot.auto.actions.Shoot;
import frc.robot.auto.actions.SpinUp;
import frc.robot.auto.actions.Straight;
import frc.robot.auto.actions.Tracking;
import frc.robot.auto.actions.Wait;
import frc.robot.auto.actions.driverAssistRunning;
import frc.robot.auto.actions.extendIntake;
import frc.robot.auto.actions.followTrajectory;
import frc.robot.auto.util.AutoMode;
import frc.robot.auto.util.AutoModeEndedException;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

//import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Drive;
import frc.robot.Intake;
import frc.robot.Shooter;
import frc.robot.Turret;



/**
 *
 */
public class driverAssist extends AutoMode {
	Drive driveTrain;
	Turret turret;
	Shooter shooter;
	Intake intake;
	NetworkTable table;

    public driverAssist(Drive drivetrain, Turret turret, Shooter shooter, Intake intake, NetworkTable table) {
		this.driveTrain = drivetrain;
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
		//runAction(new driverAssistRunning(true));
		
        runAction(new IntakeRoller(intake, false));
        runActionsParallel(new followTrajectory(22, driveTrain, intake), new Tracking(turret, true, 180));
        runAction(new Shoot(turret, shooter, intake, table, 2, 18000));
        runActionsParallel(new followTrajectory(23, driveTrain, intake), new extendIntake(intake, true));
		runAction(new IntakeRoller(intake, true));
		

        //runAction(new driverAssistRunning(false));
		DriverStation.reportWarning("Finished Action", false);
	}

}