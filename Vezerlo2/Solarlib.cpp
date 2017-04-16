/* Solarlib.cpp		Luke Miller December 2012
 * Released into the public domain (originally based on U.S. Govt. products)
 * No warranty given or implied.
 * 
 * A library of functions for Arduino to calculate aspects of solar position
 * in the sky using a time value, latitude, longitude, and time zone.
 * Output includes estimates of current sun elevation and azimuth (position in 
 * the sky), sunrise, solar noon, and sunset times for the current day, and 
 * various other statistics for the sun at the given time. Results should be
 * accurate for the years 1901 to 2099, for locations between +/- 72 latitude.
 * Calculations are based on spreadsheet and information found at:
 * http://www.esrl.noaa.gov/gmd/grad/solcalc/calcdetails.html
 * 
 * Initialize the solar calculator using the initSolarCalc() function, which
 * lets you specify:
 * tzOffset 	- time zone Offset from Greenwich Mean Time (UTC). Time zones 
 * 				west of GMT should be given negative values. 
 * 				For example, Pacific Standard Time is -8 
 * 				
 * lat			- Latitude of the location you want to use. Values north of the
 * 				equator are positive, given in decimal degrees.
 * 				 				
 * lon			- Longitude of the location you want to use. Values in the 
 * 				western hemisphere have negative values (0 to -180), given in 
 * 				decimal degrees. 
 * 				For example: Monterey, California has lat/lon (36.62, -121.904)
 * 				
 * After running initSolarCalc() in the setup loop, you can call any of the 
 * extractor functions to get the position of the sun and sunrise/sunset times.
 * Each extractor function requires a Time value as input, from the Time 
 * library. The Time is given as SEConds since 1970-1-1 00:00 (unix epoch).
 * Useful functions, supplied with a time value t as the sole argument:
 * 		getSAA(t)		- Solar Azimuth (degrees clockwise from North)
 * 		getSEC_Corr(t)	- Solar Elevation, corrected for diffraction (degrees)
 * 		getSZA(t)		- Solar Zenith angle (degrees below straight up)
 * 		getSunriseTime(t)	- Sunrise Time for the current day 	(Time object)
 * 		getSunsetTime(t)	- Sunset Time for the current day	(Time object)
 * 		getSolarNoonTime(t)	- Solar Noon for the current day	(Time object)
 * 		getSunDuration(t)	- Minutes of Sunlight for the current day	
 * 		Many more found below...			 			
 *
 * **WARNING**
 * Only tested on 32-bit ARM Teensy 3.0. This library will probably fail 
 * horribly on 8-bit AVR products due to limits in precision of double/float 
 * values. Developed on Arduino software version 1.0.2 
 * 
 * You can check your results against the NOAA calculator:
 * http://www.esrl.noaa.gov/gmd/grad/solcalc/

*/
#include "Arduino.h"
#include "math.h"
#include "Time.h"
#include "Solarlib.h"

static SolarElements se; // cache that holds all calculated values

// initSolar function
void initSolarCalc(int tzOffset, double lat, double lon){
	se.tzOffset = tzOffset; // set time zone offset
	se.lat = lat;	// set current site latitude
	se.lon = lon;	// set current site longitude
}
// Return time zone offset when user asks for it. 
// Zones west of GMT are negative.
int gettzOffset(){
	return se.tzOffset;
}
// Return latitude when user asks for it.
double getlat(){
	return se.lat;
}
// Return longitude when user asks for it.
double getlon(){
	return se.lon;
}
double gettimeFracDay(time_t t){
	calcSolar(t, se);
	return se.timeFracDay;
}
long getunixDays(time_t t){
	calcSolar(t, se);
	return se.unixDays;
}
double getJDN(time_t t){
	calcSolar(t, se);
	return se.JDN;
}
// Extract Julian Century
double getJCN(time_t t){
	calcSolar(t, se);
	return se.JCN;
}
// Extract GMLS
double getGMLS(time_t t){
	calcSolar(t, se);
	return se.GMLS;
}
double getGMAS(time_t t){
	calcSolar(t, se);
	return se.GMAS;
}
// Extract Eccentricity of Earth Orbit
double getEEO(time_t t){
	calcSolar(t, se);
	return se.EEO;
}
// Extract Sun Equation of Center
double getSEC(time_t t){
	calcSolar(t, se);
	return se.SEC;
}
// Extract Sun True Longitude (degrees)  
double getSTL(time_t t){
	calcSolar(t, se);
	return se.STL;
}
// Extract Sun True Anomaly (degrees)
double getSTA(time_t t){
	calcSolar(t, se);
	return se.STA;
}
// Extract Sun Radian Vector
double getSRV(time_t t){
	calcSolar(t, se);
	return se.SRV;
}
// Extract Sun Apparent Longitude (degrees)
double getSAL(time_t t){
	calcSolar(t, se);
	return se.SAL;
}
// Extract Mean Oblique Ecliptic (degrees)
double getMOE(time_t t){
	calcSolar(t, se);
	return se.MOE;
}
// Extract Oblique correction (degrees)
double getOC(time_t t){
	calcSolar(t, se);
	return se.MOE;
}
// Extract Sun Right Ascension (degrees)
double getSRA(time_t t){
	calcSolar(t, se);
	return se.SRA;
}
// Extract Sun Declination (degrees)
double getSDec(time_t t){
	calcSolar(t, se);
	return se.SDec;
}
// Extract var y
double getvy(time_t t){
	calcSolar(t, se);
	return se.vy;
}
// Extract Equation of Time
double getEOT(time_t t){
	calcSolar(t, se);
	return se.EOT;
}
// Extract Hour Angle Sunrise (degrees)
double getHAS(time_t t){
	calcSolar(t, se);
	return se.HAS;
}
// Extract Solar Noon (fraction of a day)
double getSolarNoonfrac(time_t t){
	calcSolar(t,se);
	return se.SolarNoonfrac;
}
// Extract Solar Noon Days (days since 1970-1-1, local time zone)
double getSolarNoonDays(time_t t){
	calcSolar(t, se);
	return se.SolarNoonDays;
}
// Extract Solar Noon Time (Time object, SEConds since 1970-1-1)
time_t getSolarNoonTime(time_t t){
	calcSolar(t, se);
	return se.SolarNoonTime;
}
// Extract Sunrise (SEConds since 1970-1-1, local time zone)
double getSunrise(time_t t){
	calcSolar(t, se);
	return se.Sunrise;
}
// Extract Sunrise as Time object (SEConds since 1970-1-1, local time zone)
time_t getSunriseTime(time_t t){
	calcSolar(t, se);
	return se.SunriseTime;
}
// Extract Sunset (SEConds since 1970-1-1, local time zone)
double getSunset(time_t t){
	calcSolar(t, se);
	return se.Sunset;
}
// Extract Sunset as Time object (SEConds since 1970-1-1, local time zone)
time_t getSunsetTime(time_t t){
	calcSolar(t, se);
	return se.SunsetTime;
}
// Extract Sunlight Duration (day length, minutes)
double getSunDuration(time_t t){
	calcSolar(t, se);
	return se.SunDuration;
}
// Extract True Solar Time (minutes)
double getTST(time_t t){
	calcSolar(t, se);
	return se.TST;
}
// Extract Hour Angle (degrees)
double getHA(time_t t){
	calcSolar(t, se);
	return se.HA;
}
// Extract Solar Zenith Angle (degrees)
double getSZA(time_t t){
	calcSolar(t, se);
	return se.SZA;
}
// Solar Elevation Angle (degrees above horizontal)
double getSEA(time_t t){
	calcSolar(t, se);
	return se.SEA;
}
// Approximate Atmospheric Refraction (degrees)
double getAAR(time_t t){
	calcSolar(t, se);
	return se.AAR;
}
// Solar Elevation Corrected for Atmospheric refraction (degrees)
double getSEC_Corr(time_t t){
	calcSolar(t, se);
	return se.SEC_Corr;
}
// Extract Solar Azimuth Angle (degrees clockwise from North)
double getSAA(time_t t){
	calcSolar(t, se);
	return se.SAA;
}

// Main function to calculate solar values. Requires a time value (SEConds since
// 1970-1-1) as input. 
void calcSolar(time_t t, SolarElements &se){
    // Calculate the time past midnight, as a fractional day value
	// e.g. if it's noon, the result should be 0.5.
	se.timeFracDay = ((((double)(second(t)/60) + minute(t))/60) +
                   hour(t))/24;
    // unixDays is the number of whole days since the start
    // of the Unix epoch. The division sign will truncate any remainder
    // since this will be done as integer division.
    se.unixDays = t / 86400;
    // calculate Julian Day Number
    se.JDN = julianUnixEpoch + se.unixDays;
    // Add the fractional day value to the Julian Day number. If the
    // input value was in the GMT time zone, we could proceed directly
    // with this value. 
    se.JDN = se.JDN + se.timeFracDay;
    // Adjust JDN to GMT time zone
    se.JDN = se.JDN - ((double)se.tzOffset / 24);
    // Calculate Julian Century Number
    se.JCN = (se.JDN - 2451545) / 36525;
    // Geometric Mean Longitude of Sun (degrees)
    se.GMLS = (280.46646 + se.JCN * (36000.76983 + se.JCN * 0.0003032));
    // Finish GMLS calculation by calculating modolu(GMLS,360) as
    // it's done in R or Excel. C's fmod doesn't work in the same
    // way. The floor() function is from the math.h library.
    se.GMLS = se.GMLS - (360 * (floor(se.GMLS/360)) );
    // Geometric Mean Anomaly of Sun (degrees)
    se.GMAS = 357.52911 + (se.JCN * (35999.05029 - 0.0001537 * se.JCN));
    
    // Eccentricity of Earth Orbit
    se.EEO = 0.016708634 - (se.JCN * (0.000042037 + 0.0000001267 * se.JCN));
    // Sun Equation of Center
    se.SEC = sin(se.GMAS * DEG_TO_RAD) * (1.914602 -
                                    (se.JCN * (0.004817 + 0.000014 * se.JCN))) +
    sin((2*se.GMAS)* DEG_TO_RAD)*(0.019993-0.000101*se.JCN) +
    sin((3*se.GMAS)* DEG_TO_RAD) * 0.000289;
    // Sun True Longitude (degrees)
    se.STL = se.GMLS + se.SEC;
    // Sun True Anomaly (degrees)
    se.STA = se.GMAS + se.SEC;
    // Sun Radian Vector (Astronomical Units)
    se.SRV = (1.000001018 * (1- se.EEO * se.EEO))/(1 + se.EEO *
                                          cos(se.STA * DEG_TO_RAD));
    // Sun Apparent Longitude (degrees)
    se.SAL = se.STL - 0.00569 - (0.00478 *
                           sin((125.04 - 1934.136 * se.JCN) * DEG_TO_RAD));
    // Mean Oblique Ecliptic (degrees)
    se.MOE = 23 + (26 + (21.448-se.JCN * ( 46.815 + se.JCN *
                                    (0.00059 - se.JCN * 0.001813)))/60)/60;
    // Oblique correction (degrees)
    se.OC = se.MOE + 0.00256 * cos((125.04-1934.136*se.JCN)*DEG_TO_RAD);
    // Sun Right Ascension (degrees)
    se.SRA = (atan2(cos(se.OC * DEG_TO_RAD) * sin(se.SAL * DEG_TO_RAD),
                 cos(se.SAL * DEG_TO_RAD))) * RAD_TO_DEG;
    // Sun Declination (degrees)
    se.SDec = (asin(sin(se.OC * DEG_TO_RAD) *
                 sin(se.SAL * DEG_TO_RAD))) * RAD_TO_DEG;
    // var y
    se.vy = tan((se.OC/2) * DEG_TO_RAD) * tan((se.OC/2) * DEG_TO_RAD);
    
    // Equation of Time (minutes)
    se.EOT = 4 * ((se.vy * sin(2 * (se.GMLS * DEG_TO_RAD)) -
                2 * se.EEO * sin(se.GMAS * DEG_TO_RAD) +
                4 * se.EEO * se.vy * sin(se.GMAS * DEG_TO_RAD) * 
                cos(2*(se.GMLS*DEG_TO_RAD)) -
                0.5 * se.vy * se.vy * sin(4*(se.GMLS * DEG_TO_RAD)) -
                1.25 * se.EEO * se.EEO * sin(2*(se.GMAS* DEG_TO_RAD))) * 
                RAD_TO_DEG);
    // Hour Angle Sunrise (degrees)
    se.HAS = acos((cos(90.833*DEG_TO_RAD)/
                (cos(se.lat*DEG_TO_RAD) * cos(se.SDec*DEG_TO_RAD))) -
               tan(se.lat * DEG_TO_RAD) * tan(se.SDec * DEG_TO_RAD)) * 
               RAD_TO_DEG ;
    // Solar Noon - result is given as fraction of a day
    // Time value is in GMT time zone
    se.SolarNoonfrac = (720 - 4 * se.lon - se.EOT) / 1440 ;
    // SolarNoon is given as a fraction of a day. Add this
    // to the unixDays value, which currently holds the
    // whole days since 1970-1-1 00:00
    se.SolarNoonDays = se.unixDays + se.SolarNoonfrac;
    // SolarNoonDays is in GMT time zone, correct it to
    // the input time zone
    se.SolarNoonDays = se.SolarNoonDays + ((double)se.tzOffset / 24);
    // Then convert SolarNoonDays to SEConds
    se.SolarNoonTime = se.SolarNoonDays * 86400;
    // Sunrise Time, given as fraction of a day
    se.Sunrise = se.SolarNoonfrac - se.HAS * 4/1440;
    // Convert Sunrise to days since 1970-1-1
    se.Sunrise = se.unixDays + se.Sunrise;
    // Correct Sunrise to local time zone from GMT
    se.Sunrise = se.Sunrise + ((double)se.tzOffset / 24);
    // Convert Sunrise to SEConds since 1970-1-1
    se.Sunrise = se.Sunrise * 86400;
    // Convert Sunrise to a time_t object (Time library)
    se.SunriseTime = (time_t)se.Sunrise;
    // Sunset Time
    se.Sunset = se.SolarNoonfrac + se.HAS * 4/1440;
    // Convert Sunset to days since 1970-1-1
    se.Sunset = se.unixDays + se.Sunset;
    // Correct Sunset to local time zone from GMT
    se.Sunset = se.Sunset + ((double)se.tzOffset / 24);
    // Convert Sunset to SEConds since 1970-1-1
    se.Sunset = se.Sunset * 86400;
    // Convert Sunset to a time_t object (Time library)
    se.SunsetTime = (time_t)se.Sunset;
    // Sunlight Duration (day length, minutes)
    se.SunDuration = 8 * se.HAS;
    // True Solar Time (minutes)
    se.TST = (se.timeFracDay * 1440 +
           se.EOT + 4 * se.lon - 60 * se.tzOffset);
    // Finish TST calculation by calculating modolu(TST,360) as
    // it's done in R or Excel. C's fmod doesn't work in the same
    // way. The floor() function is from the math.h library.
    se.TST = se.TST - (1440 * (floor(se.TST/1440)) );
    // Hour Angle (degrees)
    if (se.TST/4 < 0) {
        se.HA = se.TST/4 + 180;
    } else if (se.TST/4 >= 0) {
        se.HA = se.TST/4 - 180;
    }
    // Solar Zenith Angle (degrees)
    se.SZA = (acos(sin(se.lat * DEG_TO_RAD) *
                sin(se.SDec* DEG_TO_RAD) +
                cos(se.lat * DEG_TO_RAD) *
                cos(se.SDec * DEG_TO_RAD) *
                cos(se.HA * DEG_TO_RAD))) * RAD_TO_DEG;
    // Solar Elevation Angle (degrees above horizontal)
    se.SEA = 90 - se.SZA;
    // Approximate Atmospheric Refraction (degrees)
    if (se.SEA > 85) {
        se.AAR = 0;
    } else if (se.SEA > 5) {
        se.AAR = (58.1 / tan(se.SEA * DEG_TO_RAD)) -
        0.07 / (pow(tan(se.SEA * DEG_TO_RAD),3)) +
        0.000086 / (pow(tan(se.SEA * DEG_TO_RAD),5));
    } else if (se.SEA > -0.575) {
        se.AAR = 1735 + se.SEA * (-581.2 * se.SEA *
                            (103.4 + se.SEA * (-12.79 + se.SEA * 0.711)));
    } else {
        se.AAR = -20.772 / tan(se.SEA * DEG_TO_RAD);
    }
    se.AAR = se.AAR / 3600.0;
    // Solar Elevation Corrected for Atmospheric
    // refraction (degrees)
    se.SEC_Corr = se.SEA + se.AAR;
    // Solar Azimuth Angle (degrees clockwise from North)
    if (se.HA > 0) {
        se.SAA = (((acos((sin(se.lat * DEG_TO_RAD) *
                       cos(se.SZA * DEG_TO_RAD) -
                       sin(se.SDec * DEG_TO_RAD)) /
                      (cos(se.lat * DEG_TO_RAD) *
                       sin(se.SZA * DEG_TO_RAD))) ) *
                RAD_TO_DEG) + 180);
        se.SAA = se.SAA - (360 * (floor(se.SAA/360)));
    } else {
        se.SAA = (540 - (acos((((sin(se.lat * DEG_TO_RAD) *
                              cos(se.SZA * DEG_TO_RAD))) -
                            sin(se.SDec * DEG_TO_RAD)) /
                           (cos(se.lat * DEG_TO_RAD) *
                            sin(se.SZA * DEG_TO_RAD)))) *
               RAD_TO_DEG);
        se.SAA = se.SAA - (360 * (floor(se.SAA/360)));
    }
}
