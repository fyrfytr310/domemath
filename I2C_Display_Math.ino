
#include <BasicLinearAlgebra.h>
#include <LiquidCrystal_I2C.h> //library for i2c control of lcd
#include <Wire.h>
#include <Math.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); //LCD Address and column/row designation

//using namespace BLA;

float input = 0;
float azi = 0;
float alt = 0;
float lat = -84.6203;
int SoP = 0;
float r = 0;  //dome radius
float HourAngle = 0;
String stringSOP;
double xm, ym, zm;

BLA::Matrix <3> c ={0,0,0};  //Origin always 0,0,0
BLA::Matrix <3> cog = {0,0,0};  //Offset from origin in x,y,z dimensions
BLA::Matrix <3> cogtoO = {0,0,0};  //Actual pointint position, math completed in the void
BLA::Matrix <3> o = {0,0,0};  //For intersection of optical axis and dec axis
BLA::Matrix <3> l = {0,0,0};  //For actual pointint position of scope, math done in void
BLA::Matrix <3> underRoot = {0,0,0}; //For intermittant math steps
BLA::Matrix <3> d = {0,0,0}; //For intermittant math steps
BLA::Matrix <3> s = {0,0,0}; //For intermittant math steps

void setup() {
  lcd.init();                   // initialize the lcd
  lcd.clear();                  // start with a blank screen
  lcd.backlight();              // ensure backlight is on
  lcd.setCursor(0, 0);          // puts cursor at top left of display
  lcd.print("Azimuth:  ");
  lcd.setCursor(0, 1);
  lcd.print("Altitude: ");
  lcd.setCursor(0, 2);
  lcd.print("Side of Pier: ");
  lcd.setCursor(0, 3);
  lcd.print("Hour Angle: ");
  Serial.begin(9600);

}

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

void loop() {
  recvWithEndMarker();
  doMath();
  showNewData();
  //domeAzimuth();
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void doMath() {
 input = atof(receivedChars);
  if (input > 5999 && input <7000){
    azi = input - 6000;
  }
  if (input >1900 && input <2100){
    alt = input - 2000;
  }
  if (input == 2099 || input == 3000 || input == 3001){
    SoP = input-3000;
  }
  if (SoP == -1){
    stringSOP = "???";
  }
  if (SoP == 0) {
    stringSOP = "EAST";
  }
  if (SoP == 1) {
    stringSOP = "WEST";    
  }  
if (input > 9500 && input < 10500){
  HourAngle = input-10000;
}
  /*Do matrix math for dome azimuth
  if (input >xxx && input < XXX){
    
  }*/

  
//Matrix <3> Doi = {SoP*r*sin(han)*cos(lat), SoP*-r*sin(han)*sin(lat), Sop*r*cos(han)};  //Interwsection of optical path against dome
//Matrix <3> o = Cog + Doi;
//Matrix <3> l = {};
}

void showNewData() {
  if (newData == true) {
    Serial.print("Azimuth ");
    Serial.print(azi);
    Serial.print("Altitude ");
    Serial.println(alt);

    lcd.setCursor(13, 0);
    lcd.print("    ");
    lcd.setCursor(13, 0);
    lcd.print(azi);
    lcd.setCursor(14, 1);
    lcd.print("    ");
    lcd.setCursor(14, 1);
    lcd.print(alt);
    lcd.setCursor(14, 2);
    lcd.print("    ");
    lcd.setCursor(14, 2);
    lcd.print(stringSOP);
    lcd.setCursor(14, 3);
    lcd.print("      ");
    lcd.setCursor(14, 3);
    lcd.print(HourAngle,2);

    newData = false;
  }
}

void domeAzimuth(){
       {
            double inc = PI / 2 - alt;
            double az = PI / 2 - azi;

            //now work out the cogo point, intersection of optical axis and dec axis, i.e. the bit that moves
            cogtoO = {SoP*r*cos(HourAngle),SoP*-r*sin(lat)*sin(HourAngle), SoP*r*cos(lat)*sin(HourAngle)}; //x,y,z
            o = cog + cogtoO; 
            
            //3d cartesian coordinates  or unit vector for the actual OTA pointing direction
            xm = sin(inc) * cos(az);
            ym = sin(inc) * sin(az);
            zm = cos(inc);
            l =  { xm, ym, zm };  
            //l = l.Normalize(2);  


            //do the actual equation 
            underRoot = sqrt((o - c))- sqrt((o - c)/*.Norm(2)*/) + sqrt(r);                   
            d = -l*(o - c) + sqrt(underRoot);

            s = c + o + (l * d);

            /*var domealtaz = new DomeAltAz();

            domealtaz.DomeAzimuth = M_PI / 2 - Math.Atan2(s[1], s[0]);
            if (domealtaz.DomeAzimuth < 0) domealtaz.DomeAzimuth += 2 * M_PI;

            domealtaz.DomeAltitude = M_PI / 2 - Math.Acos(s[2] / domeR);
            domealtaz.cogtoO = cogtoO;

            return domealtaz;*/
}









////////////////////////////

/*Original Code
  public DomeAltAz Get(double domeR, double HourAngle,  double pierside, double ALT, double AZ)
        {
            double inc = M_PI / 2 - ALT;
            double az = M_PI / 2 - AZ;

            //origin of dome, 0,0,0
            var c = new DenseVector(new double[] { 0, 0, 0 });  

            //setup the intersection of RA And DEC axis which is constant.
            var cog = new DenseVector(new double[] { offsetEW, offsetNS, offsetUP });

            //now work out the cogo point, intersection of optical axis and dec axis, i.e. the bit that moves
            var cogtoO = new DenseVector(new double[] { 
                    pierside*r*cos(HourAngle), //x
                    pierside*-r*sin(latitude)*sin(HourAngle),  //y
                    pierside*r*cos(latitude)*sin(HourAngle)  //z
            });
            var o = cog + cogtoO; 

            double xm, ym, zm;
            //3d cartesian coordinates  or unit vector for the actual OTA pointing direction
            xm = Math.Sin(inc) * Math.Cos(az);
            ym = Math.Sin(inc) * Math.Sin(az);
            zm = Math.Cos(inc);
            var l = new DenseVector(new double[] { xm, ym, zm });  
            l = l.Normalize(2);  


            //do the actual equation 
            var underRoot = sqr(l.DotProduct(o - c))- sqr((o - c).Norm(2)) + sqr(domeR);                   
            var d = -l.DotProduct(o - c) + Math.Sqrt(underRoot);

            var s = c + o + (l * d);

            var domealtaz = new DomeAltAz();

            domealtaz.DomeAzimuth = M_PI / 2 - Math.Atan2(s[1], s[0]);
            if (domealtaz.DomeAzimuth < 0) domealtaz.DomeAzimuth += 2 * M_PI;

            domealtaz.DomeAltitude = M_PI / 2 - Math.Acos(s[2] / domeR);
            domealtaz.cogtoO = cogtoO;

            return domealtaz;

        }*/
