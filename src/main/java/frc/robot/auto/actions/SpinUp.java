package frc.robot.auto.actions;

import frc.robot.auto.util.Action;
import frc.robot.Constants;
import frc.robot.Intake;
import frc.robot.Shooter;
import edu.wpi.first.wpilibj.DriverStation;

public class SpinUp implements Action{
    Shooter shooter;
    double speed;

    public SpinUp(Shooter shooter, double speed){
        this.shooter = shooter;
        this.speed = speed;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start() {
        shooter.spinToRPM(speed);
    }

    @Override
    public void update(){
    }

    public void end() {
        DriverStation.reportWarning("done", false);
    }
}