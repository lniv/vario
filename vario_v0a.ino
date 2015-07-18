// Simple presure based variometer, using a bmp180 to drive an audio and a stepper needle.
// Copyright Niv Levy July 11th 2015 and later
// released under GPL v2

// originally based on http://www.rcgroups.com/forums/showthread.php?t=1749208  ( Rolf R Bakke, Oct 2012), but zero remains.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Stepper.h>
#include <TimerOne.h>
#include <TinyGPS.h>
#include <ADC.h>

#define speaker_PIN 10

#define DPducer_PIN 14
ADC *adc = new ADC(); // adc object;

// change this to the number of steps on your motor
#define STEPS 100
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(STEPS, 4, 5, 6, 7);
    
// how many steps per m/sec of climb rate (or something - units are still uncertain at this point in time. assuming we want a 20kt range (+-10kt) = 10m/sec range over 600 steps -> 1m/sec is 60 steps
#define STEP_MULT 60.0 // 

// i'm going to do the filtering not so nicely, by just implementing a ring buffer, storing deltas and going from there. Since bmp180 seems to get about 30Hz, and i'd like at most 0.5sec, we'll start with 20 samples.
// we should at least use a non square window, TODO
#define N_samples 75 // 25sampls/sec and a 1.5sec time constant : 37.5 samples x 2  = 75

const int dT = 40; // time between samples, msec

const float ddsAcc_limit = dT * 2;
const float sound_threshold = ddsAcc_limit / 2;

// in m/sec (or something - the time unit is less certain here!
const float neg_dead_band = -0.15F;
const float pos_dead_band = 0.15F;

float H[N_samples];
float sample_times[N_samples];
float delta_times[N_samples];
float lrCoef[2];

byte counter = 0;
bool grab_data = false;
long since_last;

elapsedMillis t = 0;

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(BMP085_MODE_ULTRAHIGHRES);

TinyGPS gps;
#define Uart Serial1


void time_isr(void) {
    since_last = long(t);
    t = 0;
    grab_data = true;
}



unsigned long time = 0, last_time = 0;

float toneFreq, pressure, altitude, old_altitude;

float ddsAcc; // int is 4 bytes on the later arduino and teensy!

short position, old_position;

// borrowed from http://jwbrooks.blogspot.com/2014/02/arduino-linear-regression-function.html
void simpLinReg(float* x, float* y, float* lrCoef, int n){
    // pass x and y arrays (pointers), lrCoef pointer, and n.  The lrCoef array is comprised of the slope=lrCoef[0] and intercept=lrCoef[1].  n is length of the x and y arrays.
    // http://en.wikipedia.org/wiki/Simple_linear_regression

    // initialize variables
    float xbar=0;
    float ybar=0;
    float xybar=0;
    float xsqbar=0;
    
    // calculations required for linear regression
    for (int i=0; i<n; i++){
	xbar=xbar+x[i];
	ybar=ybar+y[i];
	xybar=xybar+x[i]*y[i];
	xsqbar=xsqbar+x[i]*x[i];
    }
    xbar=xbar/n;
    ybar=ybar/n;
    xybar=xybar/n;
    xsqbar=xsqbar/n;
    
    // simple linear regression algorithm
    lrCoef[0]=(xybar-xbar*ybar)/(xsqbar-xbar*xbar);
    lrCoef[1]=ybar-lrCoef[0]*xbar;
}

/*
//lowpass filter bessel, 2nd order, corner = 0.04 (normalized - 1Hz with a 25Hz sampling), as calculated using http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=be&passmode=lp&order=2&usesr=usesr&sr=25&frequencyLow=1&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send
// replaced with http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=be&passmode=lp&order=5&usesr=usesr&sr=25&frequencyLow=0.6&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send


//Low pass bessel filter order=5 alpha1=0.024 
class filter
{
	public:
		filter()
		{
			for(int i=0; i <= 5; i++)
				v[i]=0.0;
		}
	private:
		float v[6];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = v[3];
			v[3] = v[4];
			v[4] = v[5];
			v[5] = (1.743482055851e-5 * x)
				 + (  0.3938443413 * v[0])
				 + ( -2.3459351909 * v[1])
				 + (  5.6191364702 * v[2])
				 + ( -6.7677177341 * v[3])
				 + (  4.1001141993 * v[4]);
			return 
				 (v[0] + v[5])
				+5 * (v[1] + v[4])
				+10 * (v[2] + v[3]);
		}
};*/


// http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=be&passmode=lp&order=1&usesr=usesr&sr=25&frequencyLow=0.6&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send
//Low pass bessel filter order=1 alpha1=0.024 
class filter
{
	public:
		filter()
		{
			v[0]=0.0;
		}
	private:
		float v[2];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = (7.023571980621e-2 * x)
				 + (  0.8595285604 * v[0]);
			return 
				 (v[0] + v[1]);
		}
};



filter lowpass;

void setup()
{
    int i;
    
    //handle the adc measuring the diff pressure (airspeed sensor)
    pinMode(DPducer_PIN, INPUT);
    adc->setReference(ADC_REF_3V3, ADC_0); // might make sense to use the more stable 1.2V, but need to change scaling
    adc->setAveraging(1); // set number of averages
    adc->setResolution(16); // set bits of resolution
    adc->setConversionSpeed(ADC_HIGH_SPEED); // change the conversion speed
    adc->setSamplingSpeed(ADC_HIGH_SPEED); // change the sampling speed
    
    /* Initialise the sensor */
    if(!bmp.begin())
    {
	/* There was a problem detecting the BMP085 ... check your connections */
	Serial.print("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
	while(1);
    }
    Serial.begin(115200); // note that number i BS for teensy.
    // just a place to start - should be actual pressure, but whatever
    //initialize pressure
    bmp.getPressure(&pressure);
    old_altitude = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure / 100.0F);
    
    for (i =0; i < N_samples; i++) {
	H[i] = old_altitude;
	sample_times[i] = 0.0;
    }
    
    stepper.setSpeed(300);
    // try to get it to center
    // uncomment this if necessary to set - but it does bang the motor against the stop!
    stepper.step(-600);
    stepper.step(300);
    old_position = position = 300;
    
    Timer1.initialize(dT * 1000);
    Timer1.attachInterrupt(time_isr);
    
    Uart.begin(57600);
    
    Serial.println("start");
}


void loop()
{
    int i, value;
    long since_last_copy;
    float climb_rate, climb_rate_filter;

    if (Uart.available()) {
	char c = Uart.read();
// 	    Serial.print(c);  // uncomment to see raw GPS data
	if (gps.encode(c)) {
	    long lat, lon;
	    unsigned long age;
	    gps.get_position(&lat, &lon, &age);
	    Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
	    Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
	}
    }
    if (grab_data) {
	
	noInterrupts();
	since_last_copy = since_last;
	grab_data = false;
	interrupts();
	
	bmp.getPressure(&pressure);
	H[counter % N_samples] = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure / 100.0F);
	sample_times[counter % N_samples] = float(millis());
	for (i=0; i<N_samples ; i++)
	    delta_times[i] = sample_times[i] - sample_times[0];
	// KLUDGE - i should probably allocate mem properly etc
	simpLinReg(&(delta_times[0]), &(H[0]), &(lrCoef[0]), N_samples);
	
	// NOTE : the filter here is for a fixed 25Hz sampling rate!
	// FIXME - should be 25, but seems a mess
	if (counter > 0)
	    climb_rate_filter =  lowpass.step(H[counter % N_samples] - H[(counter-1) % N_samples]);
	else // KLUDGE
	    climb_rate_filter =  lowpass.step(H[0] - H[N_samples-1]);
	
	//climb_rate = lrCoef[0] * 1000; // in m/sec
	climb_rate = climb_rate_filter;
	
	old_position = position;
	position = constrain(300 - int(climb_rate * STEP_MULT), 0, 600);
	stepper.step(position - old_position);

	// NOTE: can do non linear function here, e.g. logarithmic
	// scaled so 5/msec is full range
	toneFreq = constrain(climb_rate * 100, -500, 500);
	
	// copied from my mkiv audio code
	if (toneFreq > 0)
	    toneFreq += 150 ; 

	ddsAcc += climb_rate * 100.0;
	if (ddsAcc > ddsAcc_limit) 
	    ddsAcc = 0;
	if (ddsAcc < 0)
	    ddsAcc = 0;

	Serial.print("i= ");
	Serial.print(counter);
	Serial.print(", dt (ms)= ");
	Serial.print(since_last_copy);
    //     Serial.print(", P= ");
    //     Serial.print(pressure);
	Serial.print(", Alt= ");
	Serial.print(H[counter % N_samples]);
	Serial.print("m , lrCoef: ");
	Serial.print(lrCoef[0] * 1000);
	Serial.print(" m/sec,");
	Serial.print(lrCoef[1]);
	Serial.print(" m, f= ");
	Serial.print(toneFreq + 510);
	Serial.print(", ddsAcc = ");
	Serial.print(ddsAcc);
	Serial.print(", climb_rate_filter = ");
	Serial.print(climb_rate_filter);
	Serial.print(", positions = ");
	Serial.print(position);
	Serial.print(",");
	Serial.print(old_position);
	value = adc->analogRead(DPducer_PIN);
	Serial.print(", DP= ");
	Serial.println(value*3.3/adc->getMaxValue(ADC_0), DEC);
	
	// using the linear regression instead of my stupidity
	//if (climb_rate < neg_dead_band || ( climb_rate > pos_dead_band && ddsAcc > sound_threshold))
	if (climb_rate < neg_dead_band || ( climb_rate > pos_dead_band && counter % 12 > 6))
	    tone(speaker_PIN, toneFreq + 510);
	else
	    noTone(speaker_PIN);
	counter++;
    }
}
