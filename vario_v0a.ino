// Simple presure based variometer, using a bmp180 to drive an audio and a stepper needle.
// Copyright Niv Levy July 11th 2015 and later
// released under GPL v2


// Using openvario protocol - follow https://github.com/Turbo87/openvario-protocol (which tophat beta doesn't seem to actually get? may be worthwhile to implement something else

// originally based on http://www.rcgroups.com/forums/showthread.php?t=1749208  ( Rolf R Bakke, Oct 2012), but zero remains.

#define USE_STEPPER 1


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


#ifdef USE_STEPPER
// change this to the number of steps on your motor
#define STEPS 100
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
// was 4,5,6,7 moving to 3,4,5,6 to free 7 and 8 for serial port
Stepper stepper(STEPS, 3, 4, 5, 6);
    
// how many steps per m/sec of climb rate (or something - units are still uncertain at this point in time. assuming we want a 20kt range (+-10kt) = 10m/sec range over 600 steps -> 1m/sec is 60 steps
#define STEP_MULT 60.0 // 

#endif //USE_STEPPER

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

// gps connected to Serial1
TinyGPS gps;
#define Uart Serial1

// FC connected on pins 7,8; pin8 is tx 
#define FC Serial3

void time_isr(void) {
    since_last = long(t);
    t = 0;
    grab_data = true;
}



unsigned long time = 0, last_time = 0;

float toneFreq, pressure, altitude, old_altitude;

float ddsAcc; // int is 4 bytes on the later arduino and teensy!

short position, old_position;

float dp_offset;

// keep track of gps message and just copy it when it's done. we don't actually need the parser, but i'll use it to see when a message is finished!
String gps_s = String();

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

// from b, a = signal.iirfilter(4, f0/f, btype = 'lowpass', ftype = 'butterworth')
double b[5] = {1.32937289e-05,   5.31749156e-05,   7.97623734e-05, 5.31749156e-05,   1.32937289e-05};
double a[5] = {1.        , -3.67172909,  5.06799839, -3.11596693,  0.71991033};

class filter {
	public:
		filter() {
		    for (int i=0; i < 5; i++) {
			x[i] = 0.0;
			y[i] = 0.0;
		    }
		}
	private:
		double x[5], y[5];
	public:
		double step(double new_val) 
		{
		    int i;
		    for (i=0; i< 4; i++) { 
			x[i] = x[i+1];
			y[i] = y[i+1];
		    }
		    x[4] = new_val;
		    y[4]  = 0.0;
		    for (i=0; i< 5 ; i++)
			y[4] += b[i] * x[4-i];
		    for (i=1; i < 5; i++)
			y[4] -= a[i] * y[4-i];
		    //y[4] = y[4] / a[0]; // not needed since a[0] =1.0;
			return y[4];
		}
};
    

filter Vz_lowpass, Vx_lowpass;

const float R =  8.31432; // N·m/(mol·K)
const float Tb = 288.15; //K
const float L_b = -0.0065;  // K/m
const float g = 9.80665; // m/s2
const float M =  0.0289644; // kg/mol
const float roe_0 = 1.225; // kg/m^3
const float Vcc = 5.0;//(NOTE: we could measure this using an a2d channel and a divider

// calculate true airspeed based on pressure and differential pressure.
// following densiy model to 11Km
float TAS(float altitude) {
    float EAS, volts, q, roe, TAS;
    int value;
    // from wikipedia 
    // TAS = EAS * sqrt(roe_0 / roe)
    // EAS = sqrt(2*q/ roe_0)
    //where ro_0 = see level density 1.225 kg/m^3, q is dynamic pressure
    
    // transfer function for a MPXV7007 is Vout = Vs *(0.057*P + 0.5) + error
    // we have a 2:1 voltage divider on the output to stay below 3.3V
    // slope is fixed within 1% and we can ignore it, offset we'll take at startup, for what that's worth - ot maybe calibrate once?
    
    //read diff presssure directly (nothing else needs it)
    //dp_offset : signal at P=0 -vs/4
    value = adc->analogRead(DPducer_PIN); // = Vout / 2 = Vs/2 * (0.057*P + 0.5) + error/2
    volts = value * 3.3 / adc->getMaxValue(ADC_0);
    //Serial.println(volts);
    q = ((volts - dp_offset) * 2.0/ Vcc  - 0.5) / 0.057; //kPa
    //Serial.println(q);
    // TODO: i'm not sure what is a legit thing to do - i can think of cases where pitot is lower than static, but whether to treat them as real airspeed or just throw them out is unclear to me. (i.e. it's essentially something/one sucking on the pitot - but i can think of a transient during e.g. a tail slide that would do this)
    if (q>0)
	EAS = sqrt(2*q * 1000.0 / roe_0); // m/sec
    else
	EAS = -sqrt(2*(-q) * 1000.0 / roe_0); //m/sec
    //https://en.wikipedia.org/wiki/Barometric_formula
    
    roe = roe_0 * pow((1- L_b * altitude/Tb), 1 + g * M / (R -L_b));
    TAS = EAS * sqrt(roe_0/roe);
    return TAS;
    //Serial.println(EAS);
    //return EAS;
}    


// send a single POV sentence
void send_POV(float x, char t) {
    byte checksum = 0;
    unsigned int i;
    int N, frac;
    float abs_x = abs(x);
    String ex_zero = String(), s = String(), ps = String();
    
    N = floor(abs_x);
    frac = floor((abs_x - N) * 100); // need two decimal places
    if (frac < 10)
	ex_zero += "0";

    if (x >=0)
	ps = ps + "POV," + t + "," + N + "." + ex_zero + frac;
	//s = s + "$POV," + t + "," + N + "." + ex_zero + frac + "*" + checksum;
    else
	//s = s + "$POV," + t + ",-" + N + "." + ex_zero + frac + "*" + checksum;
	ps = ps + "POV," + t + ",-" + N + "." + ex_zero + frac;
    // checksum calculation from https://rietman.wordpress.com/2008/09/25/how-to-calculate-the-nmea-checksum/
    for (i=0; i< ps.length(); i++)
	checksum ^= ps[i]; 
    s = s + "$" + ps + "*";
    Serial.print(s);
    Serial.println(checksum, HEX);
    FC.print(s);
    FC.println(checksum, HEX);
    return;
}

// send information to flight computer (tophat, xcsoar..) using openvario spec, as defined at
// https://github.com/Turbo87/openvario-protocol
//$POV,<type>,<value>,<type>,<value>,...*<checksum>
void info2FC(float airspeed, float climb_rate, float pressure) {
    send_POV(airspeed * 3.6, 'S'); // convert to km/h
    send_POV(climb_rate, 'E');  // already in m/sec
    send_POV(pressure / 100.0, 'P');  //already in hPa
    //TODO send gps! we want to send RMC, GGA, GSA and GSV (and probbaly less?)
    return;
}


void setup()
{
    int i, value;
    float volts;
    
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
    
    //calculate diff pressure offset:
    dp_offset = 0.0;
    //dp_offset : signal at P=0 -vs/4
    // Vs = 5.0 (NOTE: we could measure this using an a2d channel and a divider
    for (i=0; i< 200 ;i++) {
	value = adc->analogRead(DPducer_PIN);    
	volts = value * 3.3 / adc->getMaxValue(ADC_0);
	dp_offset += volts - Vcc / 4.0;
    }
    dp_offset = dp_offset / 200.0;
    
#ifdef USE_STEPPER
    stepper.setSpeed(300);
    // try to get it to center
    // uncomment this if necessary to set - but it does bang the motor against the stop!
    stepper.step(-600);
    stepper.step(300);
    old_position = position = 300;
#endif //USE_STEPPER
    
    Timer1.initialize(dT * 1000);
    Timer1.attachInterrupt(time_isr);
    
    Uart.begin(57600);
    
    FC.begin(4800);
    
    Serial.println("start");
    Serial.print("dp offset = ");
    Serial.print(dp_offset);
    Serial.println("V");
}

int gps_messages = 0;
String last_gps = String();

void loop()
{
    int i;
    long since_last_copy;
    float climb_rate, climb_rate_filter, airspeed;
    

    if (Uart.available()) {
	char c = Uart.read();
 	 //   Serial.print(c);  // uncomment to see raw GPS data
	if (c == '$')
	    gps_s = String() + '$';
	else
	    gps_s = gps_s + c;
	if (gps.encode(c)) {
	    long lat, lon;
	    unsigned long age;
	    gps.get_position(&lat, &lon, &age);
	    //Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
	    //Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
	    /*
	    if (gps_s.startsWith("$GPGGA")) {		  
		    Serial.println(gps_s);
	    }*/
	    //Serial.println(gps_s);
	    if (gps_s.startsWith("$GPRMC"))
// 	    if (gps_s.startsWith("$GPGGA"))
		last_gps = gps_s;
	    gps_s = String();
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
	    climb_rate_filter =  Vz_lowpass.step(H[counter % N_samples] - H[(counter-1) % N_samples]);
	
	else // KLUDGE
	    climb_rate_filter =  Vz_lowpass.step(H[0] - H[N_samples-1]);
	
	airspeed = Vx_lowpass.step(TAS(H[counter % N_samples]));
	
	//climb_rate = lrCoef[0] * 1000; // in m/sec
	climb_rate = climb_rate_filter;

#ifdef USE_STEPPER
	old_position = position;
	position = constrain(300 - int(climb_rate * STEP_MULT), 0, 600);
	stepper.step(position - old_position);
#endif //USE_STEPPER
	
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
/*
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
	//value = adc->analogRead(DPducer_PIN);
	//Serial.print(", DP= ");
	//Serial.println(value*3.3/adc->getMaxValue(ADC_0), DEC);
	Serial.print(", V= ");
	Serial.print(airspeed);
	Serial.println(" m/sec");
*/
	// using the linear regression instead of my stupidity
	//if (climb_rate < neg_dead_band || ( climb_rate > pos_dead_band && ddsAcc > sound_threshold))
	if (climb_rate < neg_dead_band || ( climb_rate > pos_dead_band && counter % 12 > 6))
	    tone(speaker_PIN, toneFreq + 510);
	else
	    noTone(speaker_PIN);
	
	// send info every 1sec
	if (counter % 25 == 0) {
	    info2FC(airspeed, climb_rate_filter, pressure);
	    Serial.println(last_gps);
	    FC.println(last_gps);
	}
	counter++;
    }
}
