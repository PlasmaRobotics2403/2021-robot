package frc.robot.auto.actions;

import frc.robot.auto.util.Action;
import frc.robot.Constants;
import frc.robot.Intake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class driverAssistRunning implements Action{
    boolean running;

    public driverAssistRunning(Boolean running){
        this.running = running;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start() {
        SmartDashboard.putBoolean("Driver Assist Running", running);
    }

    @Override
    public void update(){
    }

    public void end() {
        DriverStation.reportWarning("done", false);
    }
}