package frc.robot.auto.actions;

import frc.robot.auto.util.Action;
import frc.robot.Constants;
import frc.robot.Intake;
import edu.wpi.first.wpilibj.DriverStation;

public class extendIntake implements Action{
    Intake intake;
    boolean engaged;

    public extendIntake(Intake intake, boolean engaged){
        this.intake = intake;
        this.engaged = engaged;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start() {
        if(engaged == true){
            intake.extendForeBar();
            intake.roller(0.0);
        }
        else {
            intake.roller(0.0);
            intake.retractForeBar();
        }
    }

    @Override
    public void update(){
    }

    public void end() {
        DriverStation.reportWarning("done", false);
    }
}