package org.firstinspires.ftc.teamcode.helpers;

import org.firstinspires.ftc.teamcode.mechanisms.CSensor;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Lights;


public class States {

    Drivetrain dt;
    Launcher la;
    CSensor cs;
    enum RobotState{
        IDLE,
        REPOSITIONING,
        SHOOTING,
        INTAKING,
        PARKING
    }
    public States(Launcher la, CSensor cs){
        this.la = la;
        this.cs = cs;
    }
    private RobotState currentState = RobotState.IDLE;

    public void transition(boolean intakeButton){
        switch(currentState){

            case IDLE:
                if (intakeButton) currentState = RobotState.INTAKING;
                break;

            case INTAKING:
                la.intake(1);
                la.transfer(-1);
                if (cs.getColor() == Lights.Color.GREEN || cs.getColor() == Lights.Color.PURPLE) {
                    la.intake(0);
                    la.transfer(0);
                }

        }
    }
    public RobotState getState() { return currentState; }

}
