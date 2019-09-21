#include <Arduino.h>
#include <unistd.h>
#include "sgp4ext.h"
#include "sgp4unit.h"
#include "sgp4io.h"
#include "sgp4coord.h"
using namespace std;

//SET UP SOME VARIABLES
  double ro[3];
  double vo[3];
  double recef[3];
  double vecef[3];
  char typerun, typeinput, opsmode;
  gravconsttype  whichconst;
  double sec, secC, jd, jdC, tsince;


  double startmfe, stopmfe, deltamin;//cho nay hoi lai ch chac
  double tumin, mu, radiusearthkm, xke, j2, j3, j4, j3oj2;
  double latlongh[3]; //lat, long in rad, h in km above ellipsoid
  double siteLat, siteLon, siteAlt, siteLatRad, siteLonRad;
  double razel[3];
  double razelrates[3];
  int  year; int mon; int day; int hr; int min;
  int yearC; int monC; int dayC; int hrC; int minC;
  typedef char str3[4];
  str3 monstr[13];

  elsetrec satrec;
  double steps_per_degree = 1.38889; //Stepper motor steps per degree azimuth
  float elevation;

//VARIABLES FOR STEPPER CALCULATIONS
  float azimuth; //-180 to 0 to 180
  float prevAzimuth;
  float cAzimuth; //From 0 to 359.99
  float prevcAzimuth;
  bool stepperRelative = 0; //Has the stepper direction been initialized?
  float azimuthDatum = 0;
  int stepsFromDatum = 0;
  int stepsNext = 0;
  int dirNext = 1;
  int totalSteps = 0;
  int prevDir = 3; //Initialize at 3 to indicate that there is no previous direction yet (you can't have a "previous direction" until the third step)
  double azError = 0;

//SET VARIABLES
  //opsmode = 'i';
  //typerun = 'c';
  //typeinput = 'e';
  //whichconst = wgs72;
  // getgravconst( whichconst, tumin, mu, radiusearthkm, xke, j2, j3, j4, j3oj2 );
  // strcpy(monstr[1], "Jan");
  // strcpy(monstr[2], "Feb");
  // strcpy(monstr[3], "Mar");
  // strcpy(monstr[4], "Apr");
  // strcpy(monstr[5], "May");
  // strcpy(monstr[6], "Jun");
  // strcpy(monstr[7], "Jul");
  // strcpy(monstr[8], "Aug");
  // strcpy(monstr[9], "Sep");
  // strcpy(monstr[10], "Oct");
  // strcpy(monstr[11], "Nov");
  // strcpy(monstr[12], "Dec");

//ENTER TWO-LINE ELEMENT HERE
char longstr1[] = "1 25544U 98067A   19097.23063721 -.00000469  00000-0  00000+0 0  9999";
char longstr2[] = "2 25544  51.6449 353.9503 0002279 151.1697 290.4275 15.52495932164239";


 //ENTER SITE DETAILS HERE
  // siteLat = 11.05; //+North (Austin)
  // siteLon = 106.66; //+East (Austin)
  // siteAlt = 0.05; //km (Austin)
  // siteLatRad = siteLat * pi / 180.0;
  // siteLonRad = siteLon * pi / 180.0;

// time variable
String n = "";
long rtime=100 ,y;

void setup() {
    Serial.begin(9600);
    siteLat = 11.05; //+North (Austin)
    siteLon = 106.66; //+East (Austin)
    siteAlt = 0.05; //km (Austin)
    siteLatRad = siteLat * pi / 180.0;
    siteLonRad = siteLon * pi / 180.0;
    opsmode = 'i';
    typerun = 'c';
    typeinput = 'e';
    whichconst = wgs72;
    getgravconst( whichconst, tumin, mu, radiusearthkm, xke, j2, j3, j4, j3oj2 );
    strcpy(monstr[1], "Jan");
    strcpy(monstr[2], "Feb");
    strcpy(monstr[3], "Mar");
    strcpy(monstr[4], "Apr");
    strcpy(monstr[5], "May");
    strcpy(monstr[6], "Jun");
    strcpy(monstr[7], "Jul");
    strcpy(monstr[8], "Aug");
    strcpy(monstr[9], "Sep");
    strcpy(monstr[10], "Oct");
    strcpy(monstr[11], "Nov");
    strcpy(monstr[12], "Dec");
    Serial.println("DONE SETUP 1");
    //INITIALIZE SATELLITE TRACKING
    //pc.printf("Initializing satellite orbit...\n");
    twoline2rv(longstr1, longstr2, typerun, typeinput, opsmode, whichconst, startmfe, stopmfe, deltamin, satrec );

    Serial.println("DONE SETUP 2");
    //pc.printf("twoline2rv function complete...\n");
    //Call propogator to get initial state vector value
    sgp4(whichconst, satrec, 0.0, ro, vo);
    Serial.println("DONE SETUP 3");
    Serial.print("satrec:");
    Serial.println(satrec.no);
    //pc.printf("SGP4 at t = 0 to get initial state vector complete...\n");
    jd = satrec.jdsatepoch;

    invjday(jd, year, mon, day, hr, min, sec);
    //pc.printf("Scenario Epoch   %3i %3s%5i%3i:%2i:%12.9f \n", day, monstr[mon], year, hr, min, sec);

    //get realtime function


    jdC = getJulianFromUnix(rtime);
    invjday( jdC, yearC, monC, dayC, hrC, minC, secC);
    // Serial.println(dayC);//khuc nay sua thanh serial print
    // Serial.println(monstr[monC]);
    // Serial.println(yearC);
    // Serial.println(hr);
    // Serial.println(minC);
    // Serial.println(secC);


}// ngoac cua void loop
void loop() {
  //   if(Serial.available() > 0){
  //   while(Serial.available() > 0){
  //     n += char(Serial.read());
  //   }
  //   rtime = n.toInt();
  //   n = "";
  // }
  // else
  // {
  //   rtime ++;
  //   Serial.println(rtime);
  // }

      delay(1000) ;
       //RUN SGP4 AND COORDINATE TRANSFORMATION COMPUTATIONS
       jdC = getJulianFromUnix(rtime);
       Serial.print("jdc : ");
       Serial.println(jdC);
       tsince = (jdC - jd) * 24.0 * 60.0;
       sgp4(whichconst, satrec, tsince, ro, vo);
       teme2ecef(ro, vo, jdC, recef, vecef);
       ijk2ll(recef, latlongh);
       rv2azel(ro, vo, siteLatRad, siteLonRad, siteAlt, jdC, razel, razelrates);

       //CHECK FOR ERRORS
       if (satrec.error > 0)
       {
           //pc.printf("# *** error: t:= %f *** code = %3d\n", satrec.t, satrec.error);
          Serial.print("Check_T : ");
          Serial.println(satrec.t);// sua thanh serial print
          Serial.print("Check_Error1 :");
          Serial.println(satrec.error);
          Serial.println("Check_Error2");

           // cout<<satrec.t<<endl;
           // cout<<satrec.error<<endl;
       }
       else
       {
           azimuth = razel[1]*180/pi;
           if (azimuth < 0) {
               cAzimuth = 360.0 + azimuth;
           }
           else {
               cAzimuth = azimuth;
           }
           elevation = razel[2]*180/pi;

           //pc.printf("%16.8f%16.8f%16.8f%16.8f%16.8f%16.8f%16.8f\n", satrec.t, recef[0], recef[1], recef[2], vecef[0], vecef[1], vecef[2]);
           //pc.printf("%16.8f%16.8f%16.8f%16.8f%16.8f%16.8f%16.8f\n", satrec.t, latlongh[0]*180/pi, latlongh[1]*180/pi, latlongh[2], razel[0], razel[1]*180/pi, razel[2]*180/pi);

           //For first step, initialize the stepper direction assuming its initial position is true north
//            if (stepperRelative == 0){
//                stepsNext = int(cAzimuth * steps_per_degree);
//                dirNext = 2;
//                myMotor->step(stepsNext, dirNext, MICROSTEP); //Turn stepper clockwise to approximate initial azimuth
//                stepperRelative = 1;
//                azimuthDatum = stepsNext / steps_per_degree;
//                prevAzimuth = azimuth;
//                prevcAzimuth = cAzimuth;


             //  pc.printf("             Azimuth       Azimuth Datum    Steps from Datum         Total Steps          Steps Next           Direction          Az. Error\n");
              // cout<<"             Azimuth       Azimuth Datum    Steps from Datum         Total Steps          Steps Next           Direction          Az. Error"<<endl;



       }
//            else {
//
//                //Determine direction of rotation (note this will be incorrect if azimuth has crossed true north since previous step - this is dealt with later)
//                if ( cAzimuth < prevcAzimuth ) {
//                    dirNext = 1; //CCW
//                }
//                else {
//                    dirNext = 2; //CW
//                }


               //Check if azimuth has crossed from 360 to 0 degrees or vice versa
               if (abs( (azimuth - prevAzimuth) - (cAzimuth - prevcAzimuth) ) > 0.0001) {

                   //Recalculate direction of rotation
                   if ( cAzimuth > prevcAzimuth ) {
                       dirNext = 1; //CCW
                   }
                   else {
                       dirNext = 2; //CW
                   }

                   //Reset the azimuth datum
                   if (dirNext == 1) {
                       azimuthDatum = cAzimuth + azError + prevcAzimuth;
                   }
                   else {
                       azimuthDatum = cAzimuth - azError + (prevcAzimuth - 360);
                   }

                   //Reset totalSteps
                   totalSteps = 0;
               }


               //Check if azimuth rate has changed directions
               if (prevDir != 3) { //prevDir of 3 means there is no previous direction yet

                   if (prevDir != dirNext) {

                       //Reset totalSteps
                       totalSteps = 0;

                       //Reset azimuth datum
                       if (dirNext == 1) {
                           azimuthDatum = prevcAzimuth + azError;
                       }
                       else {
                           azimuthDatum = prevcAzimuth - azError;
                       }

                   }

               }
               stepsFromDatum = int( abs(cAzimuth - azimuthDatum) * steps_per_degree );
               stepsNext = stepsFromDatum - totalSteps;
               totalSteps += stepsNext;
               azError = abs(cAzimuth - azimuthDatum) - (totalSteps / steps_per_degree);

               //pc.printf("%20.2f%20.2f%20d%20d%20d%20d%20.2f\n", cAzimuth, azimuthDatum, stepsFromDatum, totalSteps, stepsNext, dirNext, azError);
//                cout<<cAzimuth<<endl;
//                cout<<azimuthDatum<<endl;
//                cout<<stepsFromDatum<<endl;
//                cout<<totalSteps<<endl;
//                cout<<stepsNext<<endl;
//                cout<<dirNext<<endl;
//                cout<<azError<<endl;
     //  cout << fixed << setprecision(6)<<endl;
Serial.print("phần này la check lỗi :");
Serial.print("epochyr :"); Serial.println(satrec.epochyr);
Serial.print("azimuth:    "); Serial.println(cAzimuth);
Serial.print("elevation:  "); Serial.println(elevation);

       // cout<<"azimuth:    "<<cAzimuth<<endl;
       // cout<<"elevation: "<<elevation<<endl;
//        cout<<" |\t"<<"azimuth:"<<setw(12)<<" |\t"<<"elevation:"<<setw(10)<<"|\t"<<endl;
//        cout<<" |\t"<<cAzimuth<<setw(10)<<"   |\t"<<elevation<<setw(10)<<"   |\t"<<endl;


//                if (stepsNext > 250) {
//
//                    pc.printf("something's probably wrong... too many steps\n\n\n\n");
//                    while(1){} // pause
//
//                }

 //              myMotor->step(stepsNext, dirNext, MICROSTEP);

//            EL_SERVO = Convert_El_to_Servo(elevation);
//            prevAzimuth = azimuth;
//            prevcAzimuth = cAzimuth;
//            prevDir = dirNext;
       }

 //      wait(1);

   //} //indefinite loop

//}

float Convert_El_to_Servo(float elevation) {

   float servo_90     = 0.71;     //Calibrated to servo
   float servo_0      = 0.445;    //Calibrated to servo
   float servo_min    = 0.186;
   float servo_ratio;

   //Interpolate servo ratio
   servo_ratio = servo_0 + (servo_90 - servo_0) * (elevation) / (90);
   if (servo_ratio < servo_min) {
       return servo_min;
   }
   else {
       return servo_ratio;
   }
}
  // put your main code here, to run repeatedly:
