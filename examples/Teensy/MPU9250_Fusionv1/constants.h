#define cout SerialUSB1

//elapsedMillis dump;

float SAMPLE_RATE; // replace this with actual sample RATE in Hz
float g = 9.80665;
float rads2degs = 57.296;


float fHeading = 0;
 
float val[10];
int16_t raw_val[10];

uint64_t timestamp;
uint8_t raw_values_on = 0;
uint8_t x_values_on = 0;
uint8_t serial_values_on = 0;
uint8_t telem_values_on = 0;
uint8_t fusion_on = 0;


// gyro bias estimation
float gyrox_off, gyroy_off, gyroz_off;


float mag_norm;

double dt; long t_old;

//Freeimu calibration
int cal_acc_off[] = {13, 43, 74};
int cal_acc_scale[]  = {2030, 2069, 2090};
int cal_magn_off[]  = {73, 168, -190};
int cal_magn_scale[]  = {226, 250, 248};

//Min - max
float acc_off[] = {0.198315, 0.198315, -0.331219};
float acc_scale[]  = {0.998495, 0.999163,  0.986459};
float magn_off[]  = {11.415598, 29.230030, -37.452061};
float magn_scale[]  = {1.037051, 1.005165, 0.960739};

//Magneto 1.2 Calibration 
float MagOffset[3] = {13.090573, 30.546150, -32.757856}; 
float mCal[3][3] = 
{
  {1.016476, -0.011290, 0.00759},
  {-0.011290, 1.004871, 0.062744},
  {0.00759, 0.062744, 1.032701}
};

//Motion cal
float MagOffset1[3] = { 13.278,30.538,-33.388 }; 
float mCal1[3][3] = 
{
  {0.9895,-0.0176,-0.0027},
  {-0.0176,0.9847,0.0544},
  {-0.0027,0.0544,1.0296}
};
