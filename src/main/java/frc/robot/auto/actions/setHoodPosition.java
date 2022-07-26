package frc.robot.auto.actions;

import frc.robot.auto.util.Action;
import frc.robot.Shooter;
import edu.wpi.first.wpilibj.DriverStation;

public class setHoodPosition implements Action {
    Shooter shooter;
    double position;

    public setHoodPosition(Shooter shooter, double hoodPosition) {
        this.shooter = shooter;
        position = hoodPosition;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start() {
        shooter.setHoodPosition(position);
    }

    @Override
    public void update(){
    }

    public void end() {
        DriverStation.reportWarning("done", false);
    }
}