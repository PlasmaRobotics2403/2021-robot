package frc.robot.auto.actions;

import frc.robot.auto.util.Action;
import frc.robot.Constants;
import frc.robot.Intake;
import frc.robot.Turret;
import edu.wpi.first.wpilibj.DriverStation;

public class SetTurretOffSet implements Action{
    Turret turret;
    double angle;

    public SetTurretOffSet(Turret turret, double angle){
        this.turret = turret;
        this.angle = angle;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start() {
        turret.setTurretOffSet(angle);
    }

    @Override
    public void update(){
    }

    public void end() {
        DriverStation.reportWarning("done", false);
    }
}