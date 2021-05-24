#include <SPI.h>
#include "LS7366.h"
#include "SRampGenerator.h"
#include "PIDController.h"


// ========================== Motor ==============================
#define Motor_enable 18
#define Motor_Brake 19

#define MotorA_PWM 9
#define MotorA_dir 8
#define MotorB_PWM 7
#define MotorB_dir 6
#define MotorC_PWM 5
#define MotorC_dir 4
#define MotorD_PWM 3
#define MotorD_dir 2

// ========================== SPI =================================
#define CS1 10   //right Encoder
#define CS3 22   //left  Encoder

SRampGenerator rampGenerator;
LS7366 rightEncoder(CS1);
LS7366 leftEncoder(CS3);
PIDContorller pidLeft(1000, 60, 10, 4095, -4095);  //Kp, Ki, Kd, max, min
PIDContorller pidRight(1000, 60, 10, 4095, -4095); //Kp, Ki, Kd, max, min


long count_CSLeft = 0;
long count_CSRight = 0;
long count_CSLeft_old = 0;
long count_CSRight_old = 0;


double theta = 0;
double x_world = 0;
double y_world = 0;
bool   init_angle = false;
double error_left_I = 0;
double error_right_I = 0;
double old_want_left = 0;
double old_want_right = 0;
// ========================== SPI =================================

// ==========================  PID ================================
float vl = 0, vr = 0;
IntervalTimer speed_timer;
float _want_vl = 0, _want_vr = 0;
float _now_vl = 0 , _now_vr = 0;
float pre_vl_error = 0, pre_vr_error = 0;



// ========================== Parameter ===========================
double Two_Wheel_Length = 0.53;             // 兩輪間距 (m) < 寫ROM >
const float Wheel_R = 0.145;                 // 輪半徑  (m)       
const float ADJUST_R_WHEEL_RATE = 1.0;      // 左排輪子校正係數
const float ADJUST_L_WHEEL_RATE = 1.0;      // 右排輪子校正係數


//V & W
double v_right = 0;
double v_left = 0;

//encoder
//int st_v = 0,nd_v = 0,rd_v = 0,th_v = 0;


// ========================== ReceivePackage ===========================
unsigned char databuf[20];     // 用來儲存收進來的 data byte
int count = 0;
int connection = -1;//判斷連線
int con_check = -1;//判斷連線
int mode = 0;
float vx;//上層給速度x
float vy;//上層給速度y(不用)
float w;//上層給w
// ========================== ReceivePackage ===========================


unsigned long lastCheckTime = 0; //判斷有無連線用 (儲存上次判斷的當下時間)
bool speedTimerFlag = false;

void speedTimerISR() 
{    //speed_timer 的 Interupt Sub-Routine
  encoder_receive();
  setWheelCar(vx,w);
  speedTimerFlag = true;
}

//mapping
// uint16_t vToPulseWidth(float V)
// { // m/s => Pulse Width      //max RPM = 2500/19 = 131.5789 //PWM resolution 12bits = 4095
//   float RPM = V / (2 * PI * Wheel_R) * 60;
//   uint16_t p = map(abs(RPM), 0, 131.5789, 0, 4095);
//   return p;
// }


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  SPI.begin();

  analogWriteResolution(12);   //ADC解析度調整為12bit 0.00122
  //PWM
  pinMode(MotorA_PWM, OUTPUT);
  pinMode(MotorB_PWM, OUTPUT);
  pinMode(MotorC_PWM, OUTPUT);
  pinMode(MotorD_PWM, OUTPUT);
  //Direction
  pinMode(MotorA_dir, OUTPUT);
  pinMode(MotorB_dir, OUTPUT);
  pinMode(MotorC_dir, OUTPUT);
  pinMode(MotorD_dir, OUTPUT);
  //enable break
  pinMode(Motor_enable, OUTPUT);
  pinMode(Motor_Brake, OUTPUT);
  //SPI
  pinMode(CS1, OUTPUT);
  pinMode(CS3, OUTPUT);
  digitalWrite(CS1, LOW);
  digitalWrite(CS3, LOW);

  leftEncoder.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX1);
  leftEncoder.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4);
  leftEncoder.clear_counter();
  leftEncoder.clear_status_register();

  rightEncoder.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX1);
  rightEncoder.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4);
  rightEncoder.clear_counter();
  rightEncoder.clear_status_register();
  rightEncoder.clear_counter();



  //BK & EN
  digitalWrite(Motor_enable, LOW);
  digitalWrite(Motor_Brake, LOW);

  //速度PID_timer
  speed_timer.begin(speedTimerISR, 5000);   //5ms 進一次 speedTimerISR
  

  //  Serial.println("Start gen");
  //  rampGenereator.generateVelocityProfile(500,100);
  //  for(int i=0;i<rampGenereator.getTotalTimeFrames();i++){
  //    Serial.println(rampGenereator.getV());
  //  }
}


void loop() {
  if (millis() - lastCheckTime > 100) {
    if (connection == con_check) //判斷有無斷線
    {
      static int con_count = 0;
      con_count++;
      if (con_count > 10) //超過10次沒更新斷線
      {
        vx = 0;
        vy = 0;
        w = 0;
        con_count = 0;
      }
    }
    con_check = connection;
    lastCheckTime = millis();
  }
  if (speedTimerFlag) {
    speedTimerFlag = false;
  }

}



void setWheelCar(float v, float w)  // V[m/s]  W[rad/s]
{
  //Two_Wheel_Kinematics
   vr = v + (w * Two_Wheel_Length / 2.0f);  //Vr = Right PID Target
   vl = v - (w * Two_Wheel_Length / 2.0f);  //Vl = Left PID Target
  _want_vr = vr * ADJUST_R_WHEEL_RATE;  
  _want_vl = vl * ADJUST_L_WHEEL_RATE;  

  float pid_output_Left  = pidLeft.calculate(_want_vl,_now_vl);   //PID control
  float pid_output_Right = pidRight.calculate(_want_vr,_now_vr);  //PID control

  new_VtoPwm_Left(pid_output_Left);
  new_VtoPwm_Right(pid_output_Right);



}




/**************************SetPWM************************************/
uint16_t  new_VtoPwm_Left(float V) {
  if(V >= 0){
    digitalWrite(MotorA_dir,LOW);
    digitalWrite(MotorB_dir,LOW);
  }
  else{
    digitalWrite(MotorA_dir,HIGH);
    digitalWrite(MotorB_dir,HIGH);
  }

  analogWrite(MotorA_PWM, abs(V));
  analogWrite(MotorB_PWM, abs(V)); 
}


uint16_t  new_VtoPwm_Right(float V) {
  if(V >= 0){
    digitalWrite(MotorC_dir,HIGH);
    digitalWrite(MotorD_dir,HIGH);
  }
  else{
    digitalWrite(MotorC_dir,LOW);
    digitalWrite(MotorD_dir,LOW);
  }
  analogWrite(MotorC_PWM, abs(V));
  analogWrite(MotorD_PWM, abs(V)); 
}






/**************************SetPWM************************************/


/**************************EncoderReceive************************************/
void encoder_receive() { //里程計累積function 10ms running
  count_CSRight =(long)rightEncoder.read_counter(); 
  count_CSLeft = (long)leftEncoder.read_counter();
  //左右輪Encoder讀回來 Pluse數
  long count_CSL_dis = count_CSLeft - count_CSLeft_old; 
  long count_CSR_dis = count_CSRight - count_CSRight_old;
  double ressolution = ( 2 * PI * Wheel_R) / 4000.0; // 輪胎圓周(m)/一圈轉幾格pulse //4000 encoder 

  //回授的速度
  float vl = -(count_CSL_dis  * ressolution / 0.005); //單位 m/s(速度)
  float vr =  count_CSR_dis  * ressolution / 0.005; //單位 m/s(速度)

  _now_vl = vl;
  _now_vr = vr;

  count_CSLeft_old = count_CSLeft;
  count_CSRight_old = count_CSRight;

  Serial1.print("real Vl = ");Serial1.println(_now_vl);
  Serial1.print("real Vr = ");Serial1.println(_now_vr); 
}


/**************************ReceivePackage************************************/
void serialEvent()
{
  if (Serial.available() > 0)
  {
    unsigned char data = Serial.read();
    if (data == 'S' && count == 0)
    {
      databuf[count] = data;
      count++;
    }
    else if (data == 'T' && count == 1)
    {
      databuf[count] = data;
      count++;
    }
    else if (count > 1 && count < 17)
    {
      databuf[count] = data;
      count++;
    }
    else if (data == 'E' && count == 17)
    {
      databuf[count] = data;
      count++;
    }
    else if (data == 'N' && count == 18)
    {
      databuf[count] = data;
      count++;
    }
    else if (data == 'D' && count == 19)
    {
      databuf[count] = data;
      count = 0;

      char checksum = databuf[2] + databuf[3] + databuf[4] + databuf[5] + databuf[6] + databuf[7] + databuf[8] + databuf[9] + databuf[10] + databuf[11] + databuf[12] + databuf[13];
      if (databuf[0] == 'S' && databuf[1] == 'T' && databuf[17] == 'E' && databuf[18] == 'N' && databuf[19] == 'D' && databuf[14] == checksum)
      {
        connection = databuf[15];
        mode = databuf[16];
        int HighByte_integer_vx = databuf[2];
        int LowByte_integer_vx = databuf[3];
        int HighByte_float_vx = databuf[4];
        int LowByte_float_vx = databuf[5];

        int HighByte_integer_vy = databuf[6];
        int LowByte_integer_vy = databuf[7];
        int HighByte_float_vy = databuf[8];
        int LowByte_float_vy = databuf[9];

        int HighByte_integer_w = databuf[10];
        int LowByte_integer_w = databuf[11];
        int HighByte_float_w = databuf[12];
        int LowByte_float_w = databuf[13];

        int interger_vx = HighByte_integer_vx * 256 + LowByte_integer_vx;
        int float_vx = HighByte_float_vx * 256 + LowByte_float_vx;

        int interger_vy = HighByte_integer_vy * 256 + LowByte_integer_vy;
        int float_vy = HighByte_float_vy * 256 + LowByte_float_vy;

        int interger_w = HighByte_integer_w * 256 + LowByte_integer_w;
        int float_w = HighByte_float_w * 256 + LowByte_float_w;

        double re_vx = interger_vx + float_vx / 1000.0;
        double re_vy = interger_vy + float_vy / 1000.0;
        double re_w = interger_w + float_w / 1000.0;

        vx = re_vx - 100;  // V[m/s]  
        vy = re_vy - 100;  //不使用
        w = re_w - 100;    //W[rad/s]
        Serial1.print(vx);
        Serial1.print(" ");
        Serial1.println(w);
      }
    }
    else
    {
      memset(databuf, 0, sizeof(databuf));
      count = 0;
    }

  }
}

