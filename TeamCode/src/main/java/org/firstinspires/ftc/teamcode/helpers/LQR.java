package org.firstinspires.ftc.teamcode.helpers;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// NOT DONE I MESSED IT UP BAD BAD
//STUPID STUPID STUPID
public class LQR {
    public SimpleMatrix state = new SimpleMatrix(new double[6][1]);
    public SimpleMatrix target = new SimpleMatrix(new double[3][1]);
    public static final double[][] K_GAIN = {
            {15.811, -15.811, -11.180, 1.581, -1.581, -1.567}, // Front Left
            {15.811,  15.811,  11.180, 1.581,  1.581,  1.567}, // Front Right
            {15.811,  15.811, -11.180, 1.581,  1.581, -1.567}, // Back Left
            {15.811, -15.811,  11.180, 1.581, -1.581,  1.567}  // Back Right
            //I lowk have no idea if theyre in the right order
    };
    public SimpleMatrix K = new SimpleMatrix(K_GAIN);

    public SimpleMatrix calculate(SimpleMatrix x){

        SimpleMatrix error = target.minus(x);

        double angleError = AngleUnit.normalizeRadians(error.get(2, 0));
        error.set(2, 0, angleError);

        double ex = error.get(0, 0);
        double ey = error.get(1, 0);
        double evx = error.get(3, 0);
        double evy = error.get(4, 0);

        double currentHeading = x.get(2, 0);

        //need to set error back to robot specific drive
        error.set(0, 0, ex * Math.cos(-currentHeading) - ey * Math.sin(-currentHeading));
        error.set(1, 0, ex * Math.sin(-currentHeading) + ey * Math.cos(-currentHeading));

        error.set(3, 0, evx * Math.cos(-currentHeading) - evy * Math.sin(-currentHeading));
        error.set(4, 0, evx * Math.sin(-currentHeading) + evy * Math.cos(-currentHeading));


        return K.mult(error);
    }//returns matrix powers
    public SimpleMatrix getState(){
        return state;
    }
    public void updateLQR(double xpos, double ypos, double theta, double xvel, double yvel, double vtheta){
        state.set(0,0,xpos);
        state.set(1,0,ypos);
        state.set(2,0,theta);
        state.set(3,0,xvel);
        state.set(4,0,yvel);
        state.set(5,0,vtheta);
    }

    public void setTarget(double xpos, double ypos, double theta){
        target.set(0,0,xpos);
        target.set(1,0,ypos);
        target.set(2,0,theta);
    }



}
