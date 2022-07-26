package frc.robot.auto.modes;

import frc.robot.auto.actions.Tracking;
import frc.robot.auto.actions.followTrajectory;
import frc.robot.auto.util.AutoMode;
import frc.robot.auto.util.AutoModeEndedException;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;

//import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Drive;
import frc.robot.Intake;
import frc.robot.Shooter;
import frc.robot.Turret;



/**
 *
 */
public class LeaveTarmac extends AutoMode {
	Drive driveTrain;
	Turret turret;
	Shooter shooter;
	Intake intake;
	NetworkTable table;

    public LeaveTarmac(Drive drivetrain) {
		this.driveTrain = drivetrain;
    }
	/*
	 * (non-Javadoc)
	 * 
	 * @see org.usfirst.frc.team2403.robot.auto.util.AutoMode#routine()
	 */
	@Override
	protected void routine() throws AutoModeEndedException {
		DriverStation.reportWarning("started Action", false);
		
		runAction(new Tracking(turret, false, 160));
		runAction(new followTrajectory(25, driveTrain, intake));

		
		DriverStation.reportWarning("Finished Action", false);
	}

}