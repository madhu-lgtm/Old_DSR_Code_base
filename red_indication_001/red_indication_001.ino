    //rmcs.Brake_Motor(n1_id,0);
    // Serial.println("Re enable one");
    // rmcs.Disable_Digital_Mode(n1_id,0);
    // rmcs.Enable_Digital_Mode(n1_id,0);
    //rmcs.Restart(n1_id);
    //rmcs.SET_HOME(n1_id);

#include<RMCS2303drive.h>
RMCS2303 rmcs;  //object for class RMCS2303

int INP_CONTROL_MODE=260;// 257-> 260 ->256
int PP_gain=32;//32
int PI_gain=16;//16
int VF_gain=32;//32

int LPR = 11;
int acceleration=5000;//5000
int speed=5000; //8000

int min_pwm = 1051;  //**********  NEED TO CHECK *****************
int max_pwm = 1951;  //**********  NEED TO CHECK *****************
int min_rpm = 500;     //**********  NEED TO CHECK *****************
int max_rpm = 5000; //(JGB 37 - 520 - 12V - 66RPM (66 RPM)) 6500 -> 5000

double GR = 0.0103; // 66RPM = 0.0111 ;  60RPM = 0.0103 
double CurrentRPM;

int gain_rpm = 1000;// Need to tune (500 to 1000) //1000 is good


int dwell = 500; // 500 to 1000 //500 working good on bench for one seerder
int dwell_2 = 1000;
byte n1_id=7; 

void setup() {
  // put your setup code here, to run once:
  rmcs.Serial_selection(0);       //0-Hardware serial,1-Software serial
   rmcs.Serial0(9600); // Set Serial0 Baud    
   rmcs.begin(&Serial3,9600);    //mega2560:Serial1,Serial2,Serial3 and set baudrate.
   rmcs.WRITE_PARAMETER(n1_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed); //Write parameters to drive
   rmcs.Enable_Digital_Mode(n1_id, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
   int read_input_pwm = 2500; //for test bench
   speed = map(read_input_pwm,min_pwm,max_pwm,min_rpm,max_rpm);
   speed = constrain(speed, min_rpm, max_rpm); // Sets Minimum and maximum limits

  //  speed = 1000;
   Serial.print("RPM_SET : ");
   Serial.println(speed);
   rmcs.Speed(n1_id,speed);

    int n1_feed_back = rmcs.Speed_Feedback(n1_id);
   Serial.println(n1_feed_back);
  
  //  if (n1_feed_back == 0)
  //  {

  //   Serial.println("Re enable one************");
  //   rmcs.Disable_Digital_Mode(n1_id,0);
  //   delay(dwell);
  //   rmcs.Enable_Digital_Mode(n1_id,0);
    
  //  }
  // else
   if (n1_feed_back > (speed+gain_rpm))
   {
    Serial.println("Re enable two************");
    rmcs.Disable_Digital_Mode(n1_id,0);
    delay(dwell*2);
    rmcs.Enable_Digital_Mode(n1_id,0);
   // delay(dwell);
   }
  

   //rmcs.SET_HOME(n1_id); 
}
