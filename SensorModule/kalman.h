#ifndef _Kalman_h
#define _Kalman_h

class Kalman {
  private:
    /* Kalman filter variables */


    void calculate_priori_estimate (void);
    void estimate_error_covariance (void);
    void calculate_innovation (double, double);
    void calculate_innovation_covariance (void);
    void calculate_kalman_gain (void);
    void calculate_posteriori_estimate (void);
    void calculate_posteriori_error_covariance (void);

  public:
	Kalman ();
      /* The variables are x for the filtered value, q for the process noise,
         r for the sensor noise, p for the estimated error and k for the Kalman Gain.
         The state of the filter is defined by the values of these variables.

         The initial values for p is not very important since it is adjusted
         during the process. It must be just high enough to narrow down.
         The initial value for the readout is also not very important, since
         it is updated during the process.
         But tweaking the values for the process noise and sensor noise
         is essential to get clear readouts.

         For large noise reduction, you can try to start from: (see http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/ )
         q = 0.125
         r = 32
         p = 1023 //"large enough to narrow down"
         e.g.
         myVar = Kalman(0.125,32,1023,0);
      */

      double Q[4][4] = {{1, 0, 0, 0},{0, 1, 0, 0},{0, 0, 1, 0},{0, 0, 0, 1}}; //process noise covariance
      double P[4][4] = {{1, 0, 0, 0},{0, 1, 0, 0},{0, 0, 1, 0},{0, 0, 0, 1}}; //estimation error covariance
      double R[2][2] = {{1, 0}, {0, 1}}; //measurement covariance
      double A[4][4] = {{0}}; //state
      double H[2][4] = {{0}}; //observation model
      double X[4][1] = {{0}}; //value
      double K[4][2] = {{0}}; //kalman gain
      double I[2][1] = {{0}}; //Innovation
      double S[2][2] = {{0}}; //Innovation covariance
      double deltaT;


    double getFilteredValue(double, double, double, double *);
};

#endif
