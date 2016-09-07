// Simple presure based variometer, using a bmp180 / ma5611 to drive an audio and a stepper needle.
// also has a gps and a diff pressure measuring airspeed, but those are used for sending to a FC, not for vario functionality at prsent.
// Copyright Niv Levy July 11th 2015 and later
// released under GPL v2

// TODO : clean up, get rid of the buffer and unused bits of code e.g. the regression fit.
// TODO: separate the lowpass filters for driving the stepper / audio (which can be fast - 0.5sec or so - from the info sent to the computer, which must not have information above 0.5Hz (to be conservative, 0.8 * f_Nyquist = 0.4Hz for a 1Hz message rate)

// Using openvario protocol - follow https://github.com/Turbo87/openvario-protocol

// originally based on http://www.rcgroups.com/forums/showthread.php?t=1749208  ( Rolf R Bakke, Oct 2012), but zero remains.
//ms5611 code adapated from http://forum.arduino.cc/index.php?topic=103377.0

// Total Energy source - if defined, use a MS5611 module (GY-63) on i2c, address 76; if not, use a BMP180. for now i'm including both and letting the compiler deal with it - i am using the altitude calculation from the bmp, and i'm not short of resources.
#define TE_MS5611 1

#define MS5611_CSB_PIN 16 // CSB pin (address)
#define MS5611_ADDRESS 0x76 

#define USE_STEPPER 1
#define MAKE_NOISE 1

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Stepper.h>
#include <TimerOne.h>
#include <TinyGPS.h>
#include <ADC.h>
#include <RunningAverage.h>

#define speaker_PIN 10

#define DPducer_PIN 14
ADC *adc = new ADC(); // adc object;


#ifdef USE_STEPPER
// change this to the number of steps on your motor
#define STEPS 100
#define STEPPER_OFFSET 22 // kludge - offset from mid point on a specific one
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
// was 4,5,6,7 moving to 3,4,5,6 to free 7 and 8 for serial port
Stepper stepper(STEPS, 3, 4, 5, 6);
    
// how many steps per m/sec of climb rate (or something - units are still uncertain at this point in time. assuming we want a 20kt range (+-10kt) = 10m/sec range over 600 steps -> 1m/sec is 60 steps
// either i got something above wrong, but it seemed that 0.5m/sec move the needle 1/3 scale!
#define STEP_MULT 60.0 // 

#endif //USE_STEPPER

// i'm going to do the filtering not so nicely, by just implementing a ring buffer, storing deltas and going from there. Since bmp180 seems to get about 30Hz, and i'd like at most 0.5sec, we'll start with 20 samples.
// we should at least use a non square window, TODO
#define N_samples 25 // 25sampls/sec and a 1.5sec time constant : 37.5 samples x 2  = 75

const int dT = 40; // time between samples, msec

const float Fs = 1000.0F / dT; // sampling rate [Hz]

const float ddsAcc_limit = dT * 2;
const float sound_threshold = ddsAcc_limit / 2;

// audio dead band [m/sec]
const float neg_dead_band = -0.5F;
const float pos_dead_band = 0.5F;

RunningAverage Vz(N_samples), Vx(N_samples), H(N_samples), P_buffer(N_samples);

float sample_times[N_samples];
float delta_times[N_samples];
float lrCoef[2];

byte counter = 0;
bool grab_data = false;
long since_last;
// use the first N packets to estimate speed bias
#define N_bias_packets 1000
//could be moved to setup
int bias_packets = N_bias_packets; 
float speed_bias = 0.0f; // m/sec

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

float toneFreq, pressure;

double altitude, old_altitude;

float ddsAcc; // int is 4 bytes on the later arduino and teensy!

short position, old_position;

float dp_offset;

// keep track of gps message and just copy it when it's done. we don't actually need the parser, but i'll use it to see when a message is finished!
String gps_s = String();

#ifdef TE_MS5611

uint16_t ms5611_C[7]; // compensation coefficients for the ms5611

//initialize ms5611, read coefficients
void ms5611_initial() {

    Serial.println();
    Serial.println("ms5611 PROM COEFFICIENTS");

    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(0x1E); // reset
    Wire.endTransmission();
    delay(10);


    for (int i=0; i<6  ; i++) {

        Wire.beginTransmission(MS5611_ADDRESS);
        Wire.write(0xA2 + (i * 2));
        Wire.endTransmission();

        Wire.beginTransmission(MS5611_ADDRESS);
        Wire.requestFrom(MS5611_ADDRESS, (uint8_t) 6);
        delay(1);
        if(Wire.available()) {
            ms5611_C[i+1] = Wire.read() << 8 | Wire.read();
        }
        else {
            Serial.println("ms5611 - Error reading PROM 1"); // error reading the PROM or communicating with the device
        }
        Serial.println(ms5611_C[i+1]);
    }
    Serial.println();
}

//get ms5611 value for a given code
long ms5611_getVal(byte code) {
    unsigned long ret = 0;
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(code);
    Wire.endTransmission();
    delay(10);
    // start read sequence
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write((byte) 0x00);
    Wire.endTransmission();
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.requestFrom(MS5611_ADDRESS, (int)3);
    if (Wire.available() >= 3) {
        ret = Wire.read() * (unsigned long)65536 + Wire.read() * (unsigned long)256 + Wire.read();
    }
    else {
        ret = -1;
    }
    Wire.endTransmission();
    return ret;
}

//read ms5611 and return pressure in hPA
float ms5611_pressure() {
    uint32_t D1 = 0;
    uint32_t D2 = 0;
    int64_t dT = 0;
    int32_t TEMP = 0;
    int64_t OFF = 0;
    int64_t SENS = 0;
    int32_t P = 0;
    float Temperature;
    float Pressure;
    D1 = ms5611_getVal(0x48); // Pressure raw
    D2 = ms5611_getVal(0x58);// Temperature raw

    dT   = D2 - ((uint32_t)ms5611_C[5] << 8);
    OFF  = ((int64_t)ms5611_C[2] << 16) + ((dT * ms5611_C[4]) >> 7);
    SENS = ((int32_t)ms5611_C[1] << 15) + ((dT * ms5611_C[3]) >> 8);

    TEMP = (int64_t)dT * (int64_t)ms5611_C[6] / 8388608 + 2000;

    Temperature = (float)TEMP / 100;
    
    P  = ((int64_t)D1 * SENS / 2097152 - OFF) / 32768;

    Pressure = (float)P / 100;
    return Pressure;
}

#endif // TE_MS5611

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
    
    //read diff pressure directly (nothing else needs it)
    //dp_offset : signal at P=0 -vs/4
    value = adc->analogRead(DPducer_PIN); // = Vout / 2 = Vs/2 * (0.057*P + 0.5) + error/2
    q = (((float) value)  / 65535.0f - 0.5) / 0.057; //kPa
    /*
    volts = value * 2.5 / adc->getMaxValue(ADC_0);
    //Serial.println(volts - dp_offset);
    // notes above are wrong now - the offset is part of the dp_offset recorded at start
    q = (volts - dp_offset) * 2.0/ (Vcc * 0.057); //kPa
    */
    //Serial.println(q);
    // TODO: i'm not sure what is a legit thing to do - i can think of cases where pitot is lower than static, but whether to treat them as real airspeed or just throw them out is unclear to me. (i.e. it's essentially something/one sucking on the pitot - but i can think of a transient during e.g. a tail slide that would do this)
    if (q>0) {
	EAS = sqrt(2*q * 1000.0 / roe_0); // m/sec
    }
    else {
	EAS = -sqrt(2*(-q) * 1000.0 / roe_0); //m/sec
    }
    //https://en.wikipedia.org/wiki/Barometric_formula
    
    roe = roe_0 * pow((1- L_b * altitude/Tb), 1 + g * M / (R -L_b));
    TAS = EAS * sqrt(roe_0/roe);
    return TAS;
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

    if (x >=0) {
	ps = ps + "POV," + t + "," + N + "." + ex_zero + frac;
    }
    else {
	ps = ps + "POV," + t + ",-" + N + "." + ex_zero + frac;
    }
    // checksum calculation from https://rietman.wordpress.com/2008/09/25/how-to-calculate-the-nmea-checksum/
    for (i=0; i< ps.length(); i++) {
	checksum ^= ps[i]; 
    }
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
#ifndef TE_MS5611
    send_POV(pressure / 100.0, 'P');  //already in hPa (wtf? i guess it's actually in Pa)
#else
    send_POV(pressure, 'P');  //hPa
#endif TE_MS5611
    return;
}


void setup()
{
    int i, value;
    float volts;
    
    //setup the MS5611 for correct i2c address
    // put pin 16 output, pulled high - CSB on ms5611, select 76 or 77
    pinMode(MS5611_CSB_PIN, OUTPUT);
    digitalWrite(MS5611_CSB_PIN, HIGH);
    
#ifdef TE_MS5611 // no idea if this is really needed, handled somewhere else etc
    // Disable internal pullups, 10Kohms are on the breakout
    PORTC |= (1 << 4);
    PORTC |= (1 << 5);
#endif // TE_MS5611
    
    //handle the adc measuring the diff pressure (airspeed sensor)
    pinMode(DPducer_PIN, INPUT);
    adc->setReference(ADC_REF_EXT, ADC_0); // might make sense to use the more stable 1.2V, but need to change scaling
    adc->setAveraging(64); // set number of averages
    adc->setResolution(14); // set bits of resolution
    adc->setConversionSpeed(ADC_HIGH_SPEED_16BITS); // change the conversion speed
    adc->setSamplingSpeed(ADC_HIGH_SPEED_16BITS); // change the sampling speed
    
    /* Initialise the sensor */
    if(!bmp.begin())
    {
	/* There was a problem detecting the BMP085 ... check your connections */
	Serial.print("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
	while(1);
    }
    Serial.begin(115200); // note that number i BS for teensy.
    delay(2000); // debug only, for seeing serial start on arduino console.  FIXME - comment out. 
    /*
    //calculate diff pressure offset:
    dp_offset = 0.0;
    //dp_offset : signal at P=0 -vs/4
    // Vs = 5.0 (NOTE: we could measure this using an a2d channel and a divider
    for (i=0; i< 200 ;i++) {
	value = adc->analogRead(DPducer_PIN);
	volts = value * 3.3 / adc->getMaxValue(ADC_0);
        Serial.println(volts);
	dp_offset += volts;// - Vcc / 4.0;
    }
    dp_offset = dp_offset / 200.0;
    */
#ifdef USE_STEPPER
    stepper.setSpeed(300);
    // try to get it to center
    // uncomment this if necessary to set - but it does bang the motor against the stop!
    stepper.step(-600);
    stepper.step(300 + STEPPER_OFFSET);
    old_position = position = 30;
#endif //USE_STEPPER

#ifdef TE_MS5611
    ms5611_initial();
    delay(1000);
    pressure = ms5611_pressure();
#else
    bmp.getPressure(&pressure);
    pressure /= 100.0F;
#endif //TE_MS5611
    
     // just a place to start - should be actual pressure, but whatever
    //initialize pressure altitude
    old_altitude = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure);
    
    P_buffer.clear();
    H.clear();
    Vz.clear();
    Vx.clear();
    
    Timer1.initialize(dT * 1000);
    Timer1.attachInterrupt(time_isr);
    
    Uart.begin(57600);
    
    FC.begin(115200);
    
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
    double Vz_filter_input;
    

    if (Uart.available()) {
	char c = Uart.read();
 	 //   Serial.print(c);  // uncomment to see raw GPS data
	if (c == '$') {
	    gps_s = String() + '$';
        }
	else {
	    gps_s = gps_s + c;
        }
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
	    if (gps_s.startsWith("$GPRMC")) {
// 	    if (gps_s.startsWith("$GPGGA"))
		last_gps = gps_s;
            }
	    gps_s = String();
	}
    }
    if (grab_data) {
	
	noInterrupts();
	since_last_copy = since_last;
	grab_data = false;
	interrupts();
	
#ifndef TE_MS5611
	bmp.getPressure(&pressure);
        pressure /= 100.0F;
#else
        pressure = ms5611_pressure();
#endif // TE_MS5611
        
        P_buffer.addValue(pressure);
        
	altitude = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure);
	H.addValue(altitude);
        
        Vz_filter_input = Fs * (altitude - old_altitude);
        old_altitude = altitude;
        
        /*
        Serial.print("Vz ");
        Serial.println(Vz_filter_input);
        */
        Vz.addValue(Vz_filter_input);
        climb_rate = Vz.getAverage();
        
        // TODO same 2, 4 decimation structure as above
        Vx.addValue(TAS(altitude));
        airspeed = Vx.getAverage();
        
	if (bias_packets) { 
            bias_packets--;
            speed_bias += TAS(altitude) / N_bias_packets;
            if (bias_packets % 10 == 0 && bias_packets) {
                Serial.print("speed bias = ");
                Serial.print(speed_bias);
                Serial.print(" N_packets = ");
                Serial.println(bias_packets);
            }
            else if (bias_packets == 0) {
                Serial.print("final speed bias = ");
                Serial.println(speed_bias);
            }
        }

#ifdef USE_STEPPER
//         if (counter % 5 == 0) {
//             old_position = position;
//             position = constrain(300 - int(climb_rate * STEP_MULT), 0, 600);
//             stepper.step(position - old_position);
//             Serial.print("Stepper position ");
//             Serial.println(position);
//         }
#endif //USE_STEPPER
        
	// NOTE: can do non linear function here, e.g. logarithmic
	// scaled so 5/msec is full range
	toneFreq = constrain(climb_rate * 100, -500, 500);
	
	// copied from my mkiv audio code
	if (toneFreq > 0) {
	    toneFreq += 150 ; 
        }
	ddsAcc += climb_rate * 100.0;
	if (ddsAcc > ddsAcc_limit) {
	    ddsAcc = 0;
        }
	if (ddsAcc < 0) {
	    ddsAcc = 0;
        }
/*
	Serial.print("i= ");
	Serial.print(counter);
	Serial.print(", dt (ms)= ");
	Serial.print(since_last_copy);
    //     Serial.print(", P= ");
    //     Serial.print(pressure);
	Serial.print(", Alt= ");
	Serial.print(H[counter]);
	Serial.print("m , lrCoef: ");
	Serial.print(lrCoef[0] * 1000);
	Serial.print(" m/sec,");
	Serial.print(lrCoef[1]);
	Serial.print(" m, f= ");
	Serial.print(toneFreq + 510);
	Serial.print(", ddsAcc = ");
	Serial.print(ddsAcc);
	Serial.print(", positions = ");
	
	//value = adc->analogRead(DPducer_PIN);
	//Serial.print(", DP= ");
	//Serial.println(value*3.3/adc->getMaxValue(ADC_0), DEC);
	Serial.print(", V= ");
	Serial.print(airspeed);
	Serial.println(" m/sec");
*/

#ifdef MAKE_NOISE
	// using the linear regression instead of my stupidity
	//if (climb_rate < neg_dead_band || ( climb_rate > pos_dead_band && ddsAcc > sound_threshold))
	if (climb_rate < neg_dead_band || ( climb_rate > pos_dead_band && counter % 12 > 6)) {
	    tone(speaker_PIN, toneFreq + 510);
        }
	else {
	    noTone(speaker_PIN);
        }
#endif // MAKE_NOISE
	// send info every 1sec
	if (counter % 25 == 0) {
	    info2FC(airspeed - speed_bias, climb_rate, P_buffer.getAverage());
	    Serial.println(last_gps.trim());
	    FC.println(last_gps.trim());
	}
	if (counter < N_samples) {
            counter++;
        }
        else {
            counter = 0;
        }
    }
}
