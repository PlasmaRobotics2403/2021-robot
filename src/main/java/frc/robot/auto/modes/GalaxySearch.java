package frc.robot.auto.modes;

import frc.robot.auto.actions.IntakeRoller;
import frc.robot.auto.actions.SetTurretPosition;
import frc.robot.auto.actions.Shoot;
import frc.robot.auto.actions.SpinUp;
import frc.robot.auto.actions.Straight;
import frc.robot.auto.actions.Tracking;
import frc.robot.auto.actions.determinePathGS;
import frc.robot.auto.actions.extendIntake;
import frc.robot.auto.actions.followTrajectory;
import frc.robot.auto.actions.gyroAngle;
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
public class GalaxySearch extends AutoMode {
	Drive driveTrain;
	Intake intake;
    NetworkTable table;
    Turret turret;

    public GalaxySearch(Drive driveTrain, Intake intake, NetworkTable table, Turret turret) {
		this.driveTrain = driveTrain;
		this.intake = intake;
        this.table = table;
        this.turret = turret;
    }
	/*
	 * (non-Javadoc)
	 * 
	 * @see org.usfirst.frc.team2403.robot.auto.util.AutoMode#routine()
	 */

     /*
        Paths:
        Red A = 1.0
        Red B = 2.0
        Blue A = 3.0
        Blue B = 4.0
     */

	@Override
	protected void routine() throws AutoModeEndedException {
        DriverStation.reportWarning("started Action", false);
        runAction(new Tracking(turret, false, 80));
        runAction(new determinePathGS(table));
        //runAction(new extendIntake(intake, true));
        runAction(new IntakeRoller(intake, true));

        //runAction(new followTrajectory(18, driveTrain, intake));

        if(SmartDashboard.getNumber("Galaxy Search Path", 0.0) == 1.0){
            runAction(new followTrajectory(18, driveTrain, intake));
        }
        else if(SmartDashboard.getNumber("Galaxy Search Path", 0.0) == 2.0){
            runAction(new followTrajectory(19, driveTrain, intake));
        }
        else if(SmartDashboard.getNumber("Galaxy Search Path", 0.0) == 3.0){
            runAction(new followTrajectory(20, driveTrain, intake));
        }
        else if(SmartDashboard.getNumber("Galaxy Search Path", 0.0) == 4.0){
            runAction(new followTrajectory(21, driveTrain, intake));
        }
        else {
            DriverStation.reportWarning("No Path Detected", false);
        }
        
        
        runAction(new IntakeRoller(intake, false));
        //runAction(new extendIntake(intake, false));
		DriverStation.reportWarning("Finished Action", false);
	}

}
