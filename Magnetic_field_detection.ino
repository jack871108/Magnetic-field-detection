#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"

MPU9250 accelgyro;
I2Cdev   I2C_M;

unsigned long time;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

float MF;
float total_mf;
float average_mf;
float threshold_mf;

int count=0;
int stat_p = 0;
int stat_c = 0;

int count_vac = 0;
int count_occu = 0;

float averagex =0;
float averagey =0;
float averagez =0;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];


#define sample_num_mdate  5000      

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max =0;
volatile int my_max =0;
volatile int mz_max =0;

volatile int mx_min =0;
volatile int my_min =0;
volatile int mz_min =0;


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  int stat_p = 0;
  int stat_c = 0;
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
	
	delay(1000);
	Serial.println("     ");
 
	//Mxyz_init_calibrated ();
  
}

void loop() 
{   
  getAccel_Data();
  getGyro_Data();
  getCompassDate_calibrated(); // compass data has been calibrated here 
 
  MF = sqrt(pow(Mxyz[0],2)+pow(Mxyz[1],2)+pow(Mxyz[2],2));
	
	Serial.println("X,Y,Z,MF:");
	Serial.print(Mxyz[0]); 
	Serial.print(",");
	Serial.print(Mxyz[1]); 
	Serial.print(",");
	Serial.print(Mxyz[2]);
  Serial.print(",");
  Serial.println(MF);
  count++;
  
  getAverage_MF();




  if(threshold_mf >0 && average_mf > 0){ 
   if(stat_p == 0){
    if(MF>threshold_mf)
    {
      count_occu ++;
      if(count_occu>=10)
      {
        Serial.println("目前有車停放");
        stat_c =1;
        Red();
        }else
        {
          Serial.println("正在確認是否有車");
          stat_c = 0;
          Green();
          }
      }else
      {
        Serial.println("目前無車停放");
        stat_c = 0;
        count_occu = 0;
        Green();
        }
    }else
      {
       if(MF<threshold_mf){ 
        count_vac ++;
        if(count_vac>10){
          Serial.print("現在無車停放");
          Green();
          stat_c = 0;
          }else
          {
            Serial.println("正在確認是否有車");
            stat_c = 1;
             Red();
            }
          }else{
            Serial.println("現在有車停放");
            stat_c = 1;
             Red();
            count_vac = 0;
            }
        }
  
      
   }
   stat_p = stat_c;

   
   
	delay(500);
}


void getHeading(void)
{
  heading=180*atan2(Mxyz[1],Mxyz[0])/PI;
  if(heading <0) heading +=360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1]/cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh)/PI;
  if(yh<0)    tiltheading +=360;
}



void Mxyz_init_calibrated ()
{
	
	Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
	Serial.print("  ");
	Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
	Serial.print("  ");
	Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
	while(!Serial.find("ready"));	
	Serial.println("  ");
	Serial.println("ready");
	Serial.println("Sample starting......");
	Serial.println("waiting ......");
	
	get_calibration_Data ();
	
	Serial.println("     ");
	Serial.println("compass calibration parameter ");
	Serial.print(mx_centre);
	Serial.print("     ");
	Serial.print(my_centre);
	Serial.print("     ");
	Serial.println(mz_centre);
	Serial.println("    ");
}


void get_calibration_Data ()
{
		for (int i=0; i<sample_num_mdate;i++)
			{
			get_one_sample_date_mxyz();
			
			Serial.print(mx_sample[2]);
			Serial.print(" ");
			Serial.print(my_sample[2]);                            //you can see the sample data here .
			Serial.print(" ");
			Serial.println(mz_sample[2]);
			


			
			if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];			
			if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //find max value			
			if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];		
			
			if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
			if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//find min value
			if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];
						
			}
			
			mx_max = mx_sample[1];
			my_max = my_sample[1];
			mz_max = mz_sample[1];			
					
			mx_min = mx_sample[0];
			my_min = my_sample[0];
			mz_min = mz_sample[0];
	

	
			mx_centre = (mx_max + mx_min)/2;
			my_centre = (my_max + my_min)/2;
			mz_centre = (mz_max + mz_min)/2;	
	
}






void get_one_sample_date_mxyz()
{		
		getCompass_Data();
		mx_sample[2] = Mxyz[0];
		my_sample[2] = Mxyz[1];
		mz_sample[2] = Mxyz[2];
}	


void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;//16384  LSB/g
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384; 
}

void getGyro_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;//131 LSB(��/s)
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
	I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
	delay(10);
	I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);
	
    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
	my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
	mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;	
	
	//Mxyz[0] = (double) mx * 1200 / 4096;
	//Mxyz[1] = (double) my * 1200 / 4096;
	//Mxyz[2] = (double) mz * 1200 / 4096;
	Mxyz[0] = (double) mx * 4800 / 8192;
	Mxyz[1] = (double) my * 4800 / 8192;
	Mxyz[2] = (double) mz * 4800 / 8192;
}

void getCompassDate_calibrated ()
{
	getCompass_Data();
	Mxyz[0] = Mxyz[0] - mx_centre;
	Mxyz[1] = Mxyz[1] - my_centre;
	Mxyz[2] = Mxyz[2] - mz_centre;	
}

void getAverage_MF()
{
  if(count>=20) 
  {
   average_mf = total_mf/20 ;
   threshold_mf = average_mf + 6 ;
   Serial.print("average MF:");
   Serial.println(average_mf);
   Serial.print("TH:");
   Serial.println(threshold_mf);
    }else
    {
      total_mf = total_mf+MF;
     
      }
  }

void Green()
 {
  digitalWrite(15, LOW);
  digitalWrite(16, HIGH);
  }

void Red()
  {
  digitalWrite(15, HIGH);
  digitalWrite(16, LOW);
    }
  
