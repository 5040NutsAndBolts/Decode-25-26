package org.firstinspires.ftc.teamcode.helpers.thelly;

public class KalmanLocal {

    // State: [x, y, theta, vx, vy, omega]
    private double[] state = new double[6];

    // Covariance matrix P (6x6)
    private double[][] P = new double[6][6];

    // State transition A (6x6)
    private double[][] A = new double[6][6];

    // Measurement matrix H (3x6)
    private double[][] H = {
            {1, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0},
            {0, 0, 1, 0, 0, 0}
    };

    // Process noise Q (6x6, diagonal)
    private final double[] Q_diag = {0.01, 0.01, 0.001, 0.1, 0.1, 0.05};

    // Measurement noise R (3x3, diagonal)
    private final double[] R_diag = {0.001, 0.001, 0.0005};


    private double lastTime;

    public KalmanLocal(double[] initialPose) {
        state[0] = initialPose[0]; // x
        state[1] = initialPose[1]; // y
        state[2] = initialPose[2]; // theta

        P[0][0] = 0.01;
        P[1][1] = 0.01;
        P[2][2] = 0.001;
        P[3][3] = 1.0;
        P[4][4] = 1.0;
        P[5][5] = 0.5;

        lastTime = System.nanoTime() / 1e9;
    }

    public double[] update(double[] pinpointPose) {
        double now = System.nanoTime() / 1e9;
        double dt = now - lastTime;
        lastTime = now;
        dt = Math.min(dt, 0.05);
        buildA(dt);

        //vector
        double[] xpred= matrixVecMul(A,state);

        double[][] ppred= matrixAddDiag(matrixMul(matrixMul(A,P), transpose(A)), Q_diag);

        //residual = measure - H * xpred
        double[] hx= matrixVecMul(H,xpred);
        double[] residual = new double[3];
        residual[0] = pinpointPose[0] - hx[0];
        residual[1] = pinpointPose[1] - hx[1];
        residual[2] = normalizeAngle(pinpointPose[2] - hx[2]);

        //total uncertainty = H * Ppred * H^T + R
        double[][] S = matrixAddDiag(matrixMul(matrixMul(H,ppred),transpose(H)),R_diag);

        //K gain matrix = Ppred * H^T * S^-1
        double[][] K = matrixMul(matrixMul(ppred, transpose(H)), invert3x3(S));

        //X estimate (updates the state) = xpred + (K * residual)
        double[] Kres = matrixVecMul(K, residual);
        for (int i = 0; i < 6; i++) {
            state[i] = xpred[i] + Kres[i];
        }

        //P estimate (update P) = (I - K * H) * ppred
        P = matrixMul(matrixSubFromIdentity(matrixMul(K,H)), ppred);

        state[2] = normalizeAngle(state[2]);

        return state.clone();
    }

    public double[] getState() { return state.clone(); }

    public double[] getPosition() {
        return new double[]{state[0], state[1], state[2]};
    }

    public double[] getVelocity() {
        return new double[]{state[3], state[4], state[5]};
    }

    // UTILITY STUFF BELOW
    private void buildA(double dt) {
        for (int i = 0; i < 6; i++) A[i][i] = 1.0;
        A[0][3] = dt;
        A[1][4] = dt;
        A[2][5] = dt;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double[][] matrixMul(double[][] A, double[][] B) {
        int r = A.length, c = B[0].length, inner = B.length;
        double[][] C = new double[r][c];
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                for (int k = 0; k < inner; k++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    private double[] matrixVecMul(double[][] A, double[] v) {
        double[] result = new double[A.length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < v.length; j++)
                result[i] += A[i][j] * v[j];
        return result;
    }

    private double[][] transpose(double[][] A) {
        double[][] T = new double[A[0].length][A.length];
        for (int i = 0; i < A.length; i++)
            for (int j = 0; j < A[0].length; j++)
                T[j][i] = A[i][j];

        
        return T;
    }

    private double[][] matrixAddDiag(double[][] A, double[] diag) {
        double[][] result = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) {
            System.arraycopy(A[i], 0, result[i], 0, A[0].length);
            result[i][i] += diag[i];
        }
        return result;
    }

    private double[][] matrixSubFromIdentity(double[][] A) {
        double[][] result = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++)
                result[i][j] = (i == j ? 1.0 : 0.0) - A[i][j];
        }
        return result;
    }

    private double[][] invert3x3(double[][] m) {
        double det = m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
                - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
                + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
        double[][] inv = new double[3][3];
        inv[0][0] =  (m[1][1]*m[2][2]-m[1][2]*m[2][1])/det;
        inv[0][1] = -(m[0][1]*m[2][2]-m[0][2]*m[2][1])/det;
        inv[0][2] =  (m[0][1]*m[1][2]-m[0][2]*m[1][1])/det;
        inv[1][0] = -(m[1][0]*m[2][2]-m[1][2]*m[2][0])/det;
        inv[1][1] =  (m[0][0]*m[2][2]-m[0][2]*m[2][0])/det;
        inv[1][2] = -(m[0][0]*m[1][2]-m[0][2]*m[1][0])/det;
        inv[2][0] =  (m[1][0]*m[2][1]-m[1][1]*m[2][0])/det;
        inv[2][1] = -(m[0][0]*m[2][1]-m[0][1]*m[2][0])/det;
        inv[2][2] =  (m[0][0]*m[1][1]-m[0][1]*m[1][0])/det;
        return inv;
    }
}