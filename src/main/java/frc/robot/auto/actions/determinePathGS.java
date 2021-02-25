package frc.robot.auto.actions;

import frc.robot.auto.util.Action;
import frc.robot.Constants;
import frc.robot.Intake;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class determinePathGS implements Action{
    NetworkTable table;
    
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    double vision_X;
    double vision_Y;

    public determinePathGS(NetworkTable table){
        this.table = table;

        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.vision_X = tx.getDouble(0.0);
        this.vision_Y = ty.getDouble(0.0);
    }

    @Override
    public boolean isFinished() {
        if(SmartDashboard.getNumber("Galaxy Search Path", 0.0) != 0.0) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void start() {
        table.getEntry("pipeline").setNumber(2);
        table.getEntry("ledMode").setNumber(1);
        if(vision_X > 13.0 && vision_X < 19.0 && vision_Y < -13.0){
            SmartDashboard.putNumber("Galaxy Search Path", 1.0);
        }
        else if(vision_X > -6.0 && vision_X < 0.0 && vision_Y < -16.0){
            SmartDashboard.putNumber("Galaxy Search Path", 2.0);
        }
        else if(vision_X > 7.0 && vision_X < 13.0 && vision_Y > -10.0){
            SmartDashboard.putNumber("Galaxy Search Path", 3.0);
        }
        else if(vision_X > -1.0 && vision_X < 5.0 && vision_Y > -10.0){
            SmartDashboard.putNumber("Galaxy Search Path", 4.0);
        }
        else {
            SmartDashboard.putNumber("Galaxy Search Path", 0.1);
        }
        
    }

    @Override
    public void update(){
    }

    public void end() {
        DriverStation.reportWarning("done", false);
    }
}