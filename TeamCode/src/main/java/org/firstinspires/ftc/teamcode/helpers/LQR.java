package org.firstinspires.ftc.teamcode.helpers;

import org.ejml.simple.SimpleMatrix;
// NOT DONE I MESSED IT UP BAD BAD
//STUPID STUPID STUPID
public class LQR {
    public SimpleMatrix state = new SimpleMatrix(new double[6][1]);
    public static final double[][] K_DATA = {
            {15.811, -15.811, -11.180, 1.581, -1.581, -1.567}, // Front Left
            {15.811,  15.811,  11.180, 1.581,  1.581,  1.567}, // Front Right
            {15.811,  15.811, -11.180, 1.581,  1.581, -1.567}, // Back Left
            {15.811, -15.811,  11.180, 1.581, -1.581,  1.567}  // Back Right
            //I lowk have no idea if theyre in the right order
    };
    public SimpleMatrix K = new SimpleMatrix(K_DATA);

    public SimpleMatrix calculate(SimpleMatrix x, SimpleMatrix target){
        SimpleMatrix error = x.minus(target);

        double angleError = error.get(2, 0);
        while (angleError > Math.PI) angleError -= 2 * Math.PI;
        while (angleError < -Math.PI) angleError += 2 * Math.PI;
        error.set(2, 0, angleError);

        return K.mult(error).scale(-1);
    }//returns matrix u
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

}
