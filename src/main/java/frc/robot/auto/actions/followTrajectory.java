package frc.robot.auto.actions;

import java.io.File;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.spline.PoseWithCurvature;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Drive;
import frc.robot.Intake;
import frc.robot.auto.util.Action;
import frc.robot.auto.util.GenerateTrajectory;

public class followTrajectory implements Action {

	Drive drive;
	Intake intake;
    GenerateTrajectory generateTrajectory;

	RamseteCommand ramsete;
	Trajectory trajectory0;
	Trajectory trajectory1;
	Trajectory trajectory2;
	Trajectory trajectory3;
	Trajectory trajectory4;
	Trajectory trajectory5;
	Trajectory trajectory6;
	Trajectory trajectory7;
	Trajectory trajectory8;
	Trajectory trajectory9;
	Trajectory trajectory10;
	Trajectory trajectory11;
	TrajectoryConfig config0;
	TrajectoryConfig config1;
	TrajectoryConfig config2;
	TrajectoryConfig config3;
	TrajectoryConfig config4;
	TrajectoryConfig config5;
	TrajectoryConfig config6;
	TrajectoryConfig config7;
	TrajectoryConfig config8;
	TrajectoryConfig config9;
	TrajectoryConfig config10;
	TrajectoryConfig config11;

	Trajectory slalom0;
	TrajectoryConfig slalomConfig0;

	Trajectory bounce0;
	TrajectoryConfig bounceConfig0;
	Trajectory bounce1;
	TrajectoryConfig bounceConfig1;
	Trajectory bounce2;
	TrajectoryConfig bounceConfig2;
	Trajectory bounce3;
	TrajectoryConfig bounceConfig3;
	
	Trajectory barrel;
	TrajectoryConfig barrelConfig;

	Trajectory galaxySearch1;
	Trajectory galaxySearch2;
	Trajectory galaxySearch3;
	Trajectory galaxySearch4;
	TrajectoryConfig galaxySearchConfig;
	TrajectoryConfig galaxySearchConfig2;
	TrajectoryConfig galaxySearchConfig3;
	TrajectoryConfig galaxySearchConfig4;

	Trajectory teleop1;
	Trajectory teleop2;
	TrajectoryConfig teleopConfig1;
	TrajectoryConfig teleopConfig2;

	Trajectory[] trajectoryArray;
	int trajectoryNumber;

	Pose2d test;
	
	int i = 0;
	
	public followTrajectory(int trajectoryNumber, final Drive drive, final Intake intake) {
		this.trajectoryNumber = trajectoryNumber;
		this.drive = drive;  
		this.intake = intake; 

		test = new Pose2d(6.7, 0, new Rotation2d(Math.toRadians(0)));
		//test.exp(new Twist2d(0, 1.7, Math.toRadians(180)));

		DriverStation.reportWarning("getting trajectory", false);
		config0 = new TrajectoryConfig(1.5, 1.5)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11));
		trajectory0 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(2, 1.2)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3.5, 1.2, new Rotation2d(0)),
            // Pass config
            config0
		);
		
		config1 = new TrajectoryConfig(1.5, 1.5)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11));
		trajectory1 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
				new Translation2d(2, 1.2)
            ),
            // End 3 meters straight ahead of where we started, facing forward
			new Pose2d(5.8, 1.2, new Rotation2d(0)),
            // Pass config
            config1
		);
		
		config2 = new TrajectoryConfig(3, 1.5)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.setReversed(true);
		trajectory2 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
				new Translation2d(-2.5, 0)
            ),
            // End 3 meters straight ahead of where we started, facing forward
			new Pose2d(-5.0, -1.2, new Rotation2d(0)),
            // Pass config
            config2
		);
		
		config3 = new TrajectoryConfig(2.5, 1.75)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.setReversed(true);
		trajectory3 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
				new Translation2d(-2, -1.2)
            ),
            // End 3 meters straight ahead of where we started, facing forward
			new Pose2d(-3.5, -1.2, new Rotation2d(0)),
            // Pass config
            config3
		);
		
		config4 = new TrajectoryConfig(2.5, 1.75)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11));
		trajectory4 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(2, 1.2)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(4.6, 1.2, new Rotation2d(0)),
            // Pass config
            config4
		);

		config5 = new TrajectoryConfig(2.5, 1.75)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.setReversed(true);
		trajectory5 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
				new Translation2d(-2.25, -0.6)
            ),
            // End 3 meters straight ahead of where we started, facing forward
			new Pose2d(-4.5, -1.2, new Rotation2d(0)),
            // Pass config
            config5
		);

		config6 = new TrajectoryConfig(1.5, 1.5)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11));
		trajectory6 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(2, 1.2)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(4.5, 1.2, new Rotation2d(0)),
            // Pass config
            config6
		);

		config7 = new TrajectoryConfig(1.5, 1.5)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11));
		trajectory7 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 1.2) //1.3, 1.8
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2.5, 1.7, new Rotation2d(Math.toRadians(20))),
            // Pass config
            config7
		);

		config8 = new TrajectoryConfig(1.5, 1.5)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11));
		trajectory8 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1.4, 0) //1.3, 1.8
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1.9, -0.25, new Rotation2d(Math.toRadians(-62))),
            // Pass config
            config8
		);

		config9 = new TrajectoryConfig(1.5, 1.5)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.setReversed(true);
		trajectory9 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(-1, 0) //1.3, 1.8
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(-1.7, 0, new Rotation2d(Math.toRadians(62))),
            // Pass config
            config9
		);

		config10 = new TrajectoryConfig(1.0, 1.5)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11));
								
		trajectory10 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(2.3, 0) //1.3, 1.8
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(4.6, 0, new Rotation2d(Math.toRadians(0))),
            // Pass config
            config10
		);

		config11 = new TrajectoryConfig(3, 1.5)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.setReversed(true);
		trajectory11 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(-3.5, 0)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(-5.5, -1, new Rotation2d(Math.toRadians(0))),
            // Pass config
            config11
		);

		slalomConfig0 = new TrajectoryConfig(7.0, 0.8)// (3.5, 0.75) good values --try in coast mode
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.addConstraint(new CentripetalAccelerationConstraint(4.45)); //4.4
		slalom0 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(1.3, 0.8, new Rotation2d(Math.toRadians(60))), //D3 between cones
                new Pose2d(2.5, 1.6, new Rotation2d(0)),//After D3
				new Pose2d(4.5, 1.6, new Rotation2d(0)),//Before D9 Straight strech
				new Pose2d(5.8, 0.8, new Rotation2d(Math.toRadians(-60))),//D9
				new Pose2d(6.7, -0.1, new Rotation2d(Math.toRadians(0))),//After D9
				//new Pose2d().exp(new Twist2d(0,1.7, Math.toRadians(180)))
				new Pose2d(7.6,0.8, new Rotation2d(Math.toRadians(90))),//D11
				new Pose2d(6.7, 1.5, new Rotation2d(Math.toRadians(180))),//Before D9 the second time //1.6
				new Pose2d(5.9, 0.8, new Rotation2d(Math.toRadians(240))),//D9 the second time //5.8
				new Pose2d(4.5, -0.0, new Rotation2d(Math.toRadians(180))),//After D9 second time  Brings it in a little to avoid moving
				new Pose2d(2.6, -0.0, new Rotation2d(Math.toRadians(180))),
				new Pose2d(1.3, 0.8, new Rotation2d(Math.toRadians(120))),
				new Pose2d(-0.2, 1.6, new Rotation2d(Math.toRadians(180))) //120
            ),
            // Pass config
            slalomConfig0
		);

		bounceConfig0 = new TrajectoryConfig(10, 1.75) //2.5, 1.5
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.addConstraint(new CentripetalAccelerationConstraint(4.3));
		bounce0 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
				new Pose2d(1.3, 1.1, new Rotation2d(Math.toRadians(90)))
				
            ),
            // Pass config
            bounceConfig0
		);

		bounceConfig1 = new TrajectoryConfig(10, 1.25) //2.25, 1.25
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.setReversed(true)
								.addConstraint(new CentripetalAccelerationConstraint(2.5)); //3
		bounce1 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
				new Pose2d(-1.5, -0.6, new Rotation2d(Math.toRadians(30))),
				new Pose2d(-2.7, -1.0, new Rotation2d(Math.toRadians(90))), //-1.4
				new Pose2d(-1.5, -2.3, new Rotation2d(Math.toRadians(180))),
				new Pose2d(0.0, -2.3, new Rotation2d(Math.toRadians(180))) //0.1
				
            ),
            // Pass config
            bounceConfig1
		);

		bounceConfig2 = new TrajectoryConfig(10, 1.5) //2.25, 1.25
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.addConstraint(new CentripetalAccelerationConstraint(4.3));
		bounce2 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
				new Pose2d(1.8, 0, new Rotation2d(Math.toRadians(0))),
				new Pose2d(2.6, 1, new Rotation2d(Math.toRadians(90))), //2.5
				new Pose2d(1.9, 2.3, new Rotation2d(Math.toRadians(180))), //1.8
				new Pose2d(-0.1, 2.5, new Rotation2d(Math.toRadians(180))) //-0.2, 2.3
				
            ),
            // Pass config
            bounceConfig2
		);

		bounceConfig3 = new TrajectoryConfig(10, 1.75) //2.5, 1.5
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.setReversed(true)
								.addConstraint(new CentripetalAccelerationConstraint(4.3));
		bounce3 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
				new Pose2d(-1, -1.9, new Rotation2d(Math.toRadians(90)))
				
            ),
            // Pass config
            bounceConfig3
		);

		barrelConfig = new TrajectoryConfig(10.0, 2.6) // -- loves coast mode
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.addConstraint(new CentripetalAccelerationConstraint(4.3));
		barrel = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
				new Pose2d(2.7, 0, new Rotation2d(0)),
				new Pose2d(3.5, -0.8, new Rotation2d(Math.toRadians(-90))),
				new Pose2d(2.7, -1.6, new Rotation2d(Math.toRadians(-180))),
				new Pose2d(1.9, -0.8, new Rotation2d(Math.toRadians(-270))),
				new Pose2d(2.7, 0, new Rotation2d(0)),
				new Pose2d(5.1, 0, new Rotation2d(Math.toRadians(-360))),
				new Pose2d(5.9, 0.8, new Rotation2d(Math.toRadians(-270))),
				new Pose2d(5.1, 1.6, new Rotation2d(Math.toRadians(-180))),
				new Pose2d(4.2, 0.8, new Rotation2d(Math.toRadians(-90))),
				new Pose2d(6.7, -1.5, new Rotation2d(Math.toRadians(0))), 
				new Pose2d(7.4, -0.7, new Rotation2d(Math.toRadians(90))),
				new Pose2d(6.7, 0.4, new Rotation2d(Math.toRadians(180))),
				new Pose2d(0, 0.7, new Rotation2d(Math.toRadians(180)))
            ),
            // Pass config
            barrelConfig
		);

		galaxySearchConfig = new TrajectoryConfig(7, 2.5) //2.5, 2
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.addConstraint(new CentripetalAccelerationConstraint(4.3));
		galaxySearch1 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
				new Pose2d(2.5, -0.8, new Rotation2d(Math.toRadians(-40))),
				new Pose2d(4.1, -0.8, new Rotation2d(Math.toRadians(80))),
				new Pose2d(3.5, 2.1, new Rotation2d(Math.toRadians(45))), //1.9
				new Pose2d(7.1, 3.8, new Rotation2d(Math.toRadians(35)))
				
            ),
            // Pass config
            galaxySearchConfig
		);

		galaxySearchConfig2 = new TrajectoryConfig(2.5, 0.75) //2.5, 0.75
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.addConstraint(new CentripetalAccelerationConstraint(4.3));
		galaxySearch2 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
				new Pose2d(1.6, 0.1, new Rotation2d(Math.toRadians(10))),
				new Pose2d(3.8, -0.5, new Rotation2d(Math.toRadians(-10))),
				new Pose2d(4.2, 1.5, new Rotation2d(Math.toRadians(50))),
				new Pose2d(7.5, 4.1, new Rotation2d(Math.toRadians(35)))
				
            ),
            // Pass config
            galaxySearchConfig2
		);

		galaxySearchConfig3 = new TrajectoryConfig(7, 0.75) //2.5, 0.75
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.addConstraint(new CentripetalAccelerationConstraint(3.8));
		galaxySearch3 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
				new Pose2d(3, -1.0, new Rotation2d(Math.toRadians(-10))),
				new Pose2d(4.9, -0.45, new Rotation2d(Math.toRadians(87))), //4.9
				new Pose2d(5.0, 2.2, new Rotation2d(Math.toRadians(20))), 
				new Pose2d(6.2, 2.0, new Rotation2d(Math.toRadians(0))),
				new Pose2d(8.5, 2.0, new Rotation2d(Math.toRadians(0)))			
            ),
            // Pass config
            galaxySearchConfig3
		);

		galaxySearchConfig4 = new TrajectoryConfig(2.5, 0.75) //2.5, 0.75
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.addConstraint(new CentripetalAccelerationConstraint(4.3));
		galaxySearch4 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
				new Pose2d(4.6, 0.0, new Rotation2d(Math.toRadians(70))),
				new Pose2d(4.6, 2.1, new Rotation2d(Math.toRadians(20))),
				new Pose2d(8.2, 1.4, new Rotation2d(Math.toRadians(0))) 
            ),
            // Pass config
            galaxySearchConfig4
		);

		teleopConfig1 = new TrajectoryConfig(2.5, 0.75)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11));
		teleop1 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
				new Pose2d(2.5, 0, new Rotation2d(Math.toRadians(0)))			
            ),
            // Pass config
            teleopConfig1
		);

		teleopConfig2 = new TrajectoryConfig(2.5, 0.75)
								.setKinematics(new DifferentialDriveKinematics(Constants.WHEEL_BASE))
								.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(.277, 1.78, .275), new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
								.setReversed(true);
		teleop2 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, new Rotation2d(0)),
				new Pose2d(-2.5, 0, new Rotation2d(Math.toRadians(0)))
            ),
            // Pass config
            teleopConfig2
		);
		
		trajectoryArray = new Trajectory[25];
		trajectoryArray[0] = trajectory0;
		trajectoryArray[1] = trajectory1;
		trajectoryArray[2] = trajectory2;
		trajectoryArray[3] = trajectory3;
		trajectoryArray[4] = trajectory4;
		trajectoryArray[5] = trajectory5;
		trajectoryArray[6] = trajectory6;
		trajectoryArray[7] = trajectory7;
		trajectoryArray[8] = trajectory8;
		trajectoryArray[9] = trajectory9;
		trajectoryArray[10] = trajectory10;
		trajectoryArray[11] = trajectory11;
		trajectoryArray[12] = slalom0;
		trajectoryArray[13] = bounce0;
		trajectoryArray[14] = bounce1;
		trajectoryArray[15] = bounce2;
		trajectoryArray[16] = bounce3;
		trajectoryArray[17] = barrel;
		trajectoryArray[18] = galaxySearch1;
		trajectoryArray[19] = galaxySearch2;
		trajectoryArray[20] = galaxySearch3;
		trajectoryArray[21] = galaxySearch4;
		trajectoryArray[22] = teleop1;
		trajectoryArray[23] = teleop2;
		DriverStation.reportWarning("got Trajectory", false);
	}

	@Override
	public boolean isFinished() {
		return ramsete.isFinished();
	}
	
	@Override
	public void start() {
		DriverStation.reportWarning("before ramsete", false);
		ramsete = new RamseteCommand(trajectoryArray[trajectoryNumber],
									 drive::getPose,
									 new RamseteController(), 
									 new SimpleMotorFeedforward(.277, 1.78, .275), //.257, 1.82, .274
									 new DifferentialDriveKinematics(Constants.WHEEL_BASE), 
									 drive::getWheelSpeeds, 
									 new PIDController(3, 0.0, 0.0), 
									 new PIDController(3, 0.0, 0.0),
									 drive::setOutput,
									 drive);
		
		ramsete.initialize();
		DriverStation.reportWarning("finished creating ramsete", false);
	}

	@Override
	public void update() {
		SmartDashboard.putNumber("Left Error", drive.leftDrive.getClosedLoopError(0));
		SmartDashboard.putNumber("Right Error", drive.rightDrive.getClosedLoopError(0));
		//SmartDashboard.putNumber("leftPosition Error", leftFollower.getSegment().position - (drive.leftDrive.getSelectedSensorPosition(0) * Constants.DRIVE_ENCODER_CONVERSION));
		ramsete.execute(); 
		DriverStation.reportWarning("updated", false);
	}

	@Override
	public void end() {
		//drive.leftDrive.setSelectedSensorPosition(0, 0, Constants.TALON_TIMEOUT);
		//drive.rightDrive.setSelectedSensorPosition(0, 0, Constants.TALON_TIMEOUT);
		drive.zeroGyro();
		drive.leftDrive.set(ControlMode.PercentOutput, 0);
		drive.rightDrive.set(ControlMode.PercentOutput, 0);
		drive.leftDriveSlave.set(ControlMode.PercentOutput, 0);
		drive.rightDriveSlave.set(ControlMode.PercentOutput, 0);
		drive.resetOdometry();
	}

}
