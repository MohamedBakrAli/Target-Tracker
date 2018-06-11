#include "kalman.h"


Kalman::Kalman(){
        A[0][0] = A[1][1] = A[2][2] = A[3][3] = 1;
        H[0][0] = H[1][2] = 1;
}

double
Kalman::getFilteredValue(double x, double y, double deltaT, double *filteredValues){

        this->deltaT = deltaT;
        A[0][1] = A[2][3] = deltaT;
        calculate_priori_estimate ();
        estimate_error_covariance ();
        calculate_innovation (x, y);
        calculate_innovation_covariance ();
        calculate_kalman_gain ();
        calculate_posteriori_estimate ();
        calculate_posteriori_error_covariance ();

        *filteredValues = X[0][0];
        filteredValues++;
        *filteredValues = X[2][0];

}
void
Kalman::calculate_priori_estimate(void){
        // X^(k|k-1) = A*X^(k-1|k-1)
        double dummy[4][1];
        for (int i=0; i<4; i++)
                for (int j=0; j<1; j++) {
                        double sum = 0;
                        for (int k=0; k<4; k++)
                                sum += A[i][k]*X[k][j];
                        dummy[i][j] = sum;
                }

        for (int i=0; i<4; i++)
                for (int j=0; j<1; j++)
                        X[i][j] = dummy[i][j];
}

void
Kalman::estimate_error_covariance(void){
        // P^(k|k-1) = A*P^(k-1|k-1)*(A**T) + Qk
        double temp[4][4];
        for (int i=0; i<4; i++)
                for (int j=0; j<4; j++) {
                        double sum = 0;
                        for (int k=0; k<4; k++)
                                sum += A[i][k]*P[k][j];
                        temp[i][j] = sum;
                }

        // Finding the transpose matrix.
        double A_transpose[4][4];
        for(int i=0; i<4; i++)
                for(int j=0; j<4; j++)
                        A_transpose[i][j] = A[j][i];

        for (int i=0; i<4; i++)
                for (int j=0; j<4; j++) {
                        double sum = 0;
                        for (int k=0; k<4; k++)
                                sum += temp[i][k]*A_transpose[k][j];
                        P[i][j] = sum;
                }

        for (int i=0; i<4; i++)
                for (int j=0; j<4; j++)
                        P[i][j] += (Q[i][j]*deltaT);
}

void
Kalman::calculate_innovation (double x, double y){
        //I = Z - H*X^(k|k-1)
        double Z[2][1];
        Z[0][0] = x, Z[1][0] = y;

        double temp[2][1];
        for (int i=0; i<2; i++)
                for (int j=0; j<1; j++) {
                        double sum = 0;
                        for (int k=0; k<4; k++)
                                sum += H[i][k]*X[k][j];
                        temp[i][j] = sum;
                }

        for (int i=0; i<2; i++)
                for (int j=0; j<1; j++)
                        I[i][j] = Z[i][j] - temp[i][j];
}

void
Kalman::calculate_innovation_covariance (void){
        // S = H*P^(k|k-1)*(H**T) + R
        double temp[2][4];
        for (int i=0; i<2; i++)
                for (int j=0; j<4; j++) {
                        double sum = 0;
                        for (int k=0; k<4; k++)
                                sum += H[i][k]*P[k][j];
                        temp[i][j] = sum;
                }

        // Finding the transpose matrix.
        double H_transpose[4][2];
        for(int i=0; i<4; i++)
                for(int j=0; j<2; j++)
                        H_transpose[i][j] = H[j][i];

        for (int i=0; i<2; i++)
                for (int j=0; j<2; j++) {
                        double sum = 0;
                        for (int k=0; k<4; k++)
                                sum += temp[i][k]*H_transpose[k][j];
                        S[i][j] = sum;
                }

        for (int i=0; i<2; i++)
                for (int j=0; j<2; j++)
                        S[i][j] += R[i][j];
}

void
Kalman::calculate_kalman_gain (void){
        //K = P*(H**T)*(S**-1)
        double temp[4][2];

        // Finding the transpose matrix.
        double H_transpose[4][2];
        for(int i=0; i<4; i++)
                for(int j=0; j<2; j++)
                        H_transpose[i][j] = H[j][i];

        for (int i=0; i<4; i++)
                for (int j=0; j<2; j++) {
                        double sum = 0;
                        for (int k=0; k<4; k++)
                                sum += P[i][k]*H_transpose[k][j];
                        temp[i][j] = sum;
                }

        // Finding the inverse matrix.
        double S_inverse[2][2], deter = 1/(S[0][0]*S[1][1] - S[0][1]*S[1][0]);
        S_inverse[0][0] = S[1][1] * deter, S_inverse[0][1] = (-deter)*S[0][1];
        S_inverse[1][0] = (-deter)*S[1][0], S_inverse[1][1] = S[0][0]*deter;

        for (int i=0; i<4; i++)
                for (int j=0; j<2; j++) {
                        double sum = 0;
                        for (int k=0; k<2; k++)
                                sum += temp[i][k]*S_inverse[k][j];
                        K[i][j] = sum;
                }
}

void
Kalman::calculate_posteriori_estimate (void){
        //X^(k|k) = X^(k|k-1) + K * I
        double temp[4][1];
        for (int i=0; i<4; i++)
                for (int j=0; j<1; j++) {
                        double sum = 0;
                        for (int k=0; k<2; k++)
                                sum += K[i][k]*I[k][j];
                        temp[i][j] = sum;
                }

        for (int i=0; i<4; i++)
                for (int j=0; j<1; j++)
                        X[i][j] += temp[i][j];
}

void
Kalman::calculate_posteriori_error_covariance (void){
        //P(k|k) = (I - K*H)*P(k|k-1)
        double temp[4][4], dummy[4][4];
        for (int i=0; i<4; i++)
                for (int j=0; j<4; j++) {
                        double sum = 0;
                        for (int k=0; k<2; k++)
                                sum += K[i][k]*H[k][j];
                        temp[i][j] = sum;
                }

        // Construct the Identity Matrix.
        double ident[4][4] = {{0}};
        ident[0][0] = ident[1][1] = ident[2][2] = ident[3][3] = 1;

        for (int i=0; i<4; i++)
                for (int j=0; j<4; j++)
                        temp[i][j] = ident[i][j] - temp[i][j];

        for (int i=0; i<4; i++)
                for (int j=0; j<4; j++) {
                        double sum = 0;
                        for (int k=0; k<4; k++)
                                sum += temp[i][k]*P[k][j];
                        dummy[i][j] = sum;
                }
        for (int i=0; i<4; i++)
                for (int j=0; j<4; j++)
                        P[i][j] = dummy[i][j];
}
