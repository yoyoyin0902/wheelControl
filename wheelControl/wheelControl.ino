#include <SPI.h>
#include <EEPROM.h>

//Encoder
#define CLR B00000000
#define RD B01000000
#define WR B10000000
#define LOAD B11000000
#define MDR0 B00001000
#define MDR1 B00010000
#define DTR B00011000
#define CNTR B00100000
#define OTR B00101000
#define STR B00110000


// ========================== Motor ===========================
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
// ========================== Motor ===========================

// ========================== SPI =============================
#define CS1 10
#define CS3 22

long count_CSLeft = 0;
long count_CSRight = 0;
long count_CSLeft_old = 0;
long count_CSRight_old = 0;


double theta = 0;
double x_world = 0;
double y_world = 0;
bool init_angle = false;
double error_left_I = 0;
double error_right_I = 0;
double old_want_left = 0;
double old_want_right = 0;
// ========================== SPI =============================

// ========================== 里程計 PID ===========================
double vl = 0, vr = 0;
IntervalTimer myTimer , speed_timer;
double _want_vl = 0, _want_vr = 0;
double _now_vl = 0 , _now_vr = 0;
double pre_vl_error = 0, pre_vr_error = 0;
// ========================== 里程計 PID ===========================


// ========================== Parameter ===========================
double Two_Wheel_Length = 0.8;             // 兩輪間距 (m) < 寫ROM >
const float Wheel_R = 0.145;                 // 輪半徑  (m)
const float Unit_Pulse = 2000;              // 一圈Pulse總數
const float ADJUST_R_WHEEL_RATE = 1.0;      // 左排輪子校正係數
const float ADJUST_L_WHEEL_RATE = 1.0;      // 右排輪子校正係數
// ========================== Parameter ===========================

//long Per_Sec_Pulse_list[4] = {0, 0, 0, 0};

//break
bool isBreaking = false;

int max_rpm = 2500;

//V & W
//int V_now = 0, W_now = 0, V_last = 0, W_last = 0;
//int now_Vl = 0,now_Vr;
double v_right = 0;
double v_left = 0;

//encoder
//int st_v = 0,nd_v = 0,rd_v = 0,th_v = 0;

//PID
double KP = 1;
double Kd = 1;
double KI = 1;



// ========================== ReceivePackage ===========================
unsigned char databuf[20];     // 用來儲存收進來的 data byte
int count = 0;
int connection = -1;//判斷連線
int con_check = -1;//判斷連線
int mode = 0;
double vx;//上層給速度x
double vy;//上層給速度y(不用)
double w;//上層給w

// ========================== ReceivePackage ===========================

int input;
char rxBuffer[128];
int rxIndex = 0;
float VelocityLeft,VelocityRight;


void setup() {
  Serial.begin(115200);
  Serial.println("Start...");
  SPI.begin();
  analogWriteResolution(12); //ADC解析度調整為12bit 0.00122
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
  
  digitalWrite(Motor_enable, LOW);
  digitalWrite(Motor_Brake, LOW);


  //速度PID_timer
  speed_timer.begin(PID_function, 1000);
  //Encoder_timer
  myTimer.begin(encoder_receive,1000);  
  digitalWrite(CS1, HIGH);
  digitalWrite(CS3, HIGH);
  SPI.transfer(CLR | CNTR);
  digitalWrite(CS1, LOW);
  digitalWrite(CS3, LOW);
}

void loop() {
  receiveConnect(); 
  //motor_Control(VelocityLeft, VelocityRight);
} 



//void MakeUpVelocity() {
//  //isBreaking
//  if(isBreaking){ 
//     V_now  = 0; 
//     W_now  = 0; 
//     V_last = 0; 
//     W_last = 0; 
//     st_v = 0;
//     nd_v = 0;
//     rd_v = 0;
//     th_v = 0; 
//  }  
//  else{  //做加減速
//    if(V_now - V_last) != 0){
//       int V_range = V_now - V_last;
//       if(V_range != 0){
//        if(V_range > 0)
//          V_last +=1;
//        else
//          V_last -=1;
//       }   
//    }
//    if(W_now != W_last){
//      int W_range = W_now - W_last; 
//      if (W_range != 0){
//        if (W_range > 0)
//          W_last += 1;
//        else
//          W_last -= 1;
//      }
//    }
//    Two_Wheel_Kinematics(V_last, W_last); //差速輪
//    if(V_left > 0){
//      digitalWrite(MotorA_dir,HIGH);
//      digitalWrite(MotorD_dir,LOW);
//      st_v = V_left;
//      th_v = V_left; 
//    }
//    else{
//      digitalWrite(MotorA_dir,LOW);
//      digitalWrite(MotorD_dir,HIGH);
//      st_v = -1 * V_left;
//      th_v = -1 * V_left;
//    }
//    if(V_right > 0){
//      digitalWrite(MotorC_dir,LOW);
//      digitalWrite(MotorB_dir,LOW); 
//      nd_v = V_right;
//      rd_v = V_right;     
//    }
//    else{
//      digitalWrite(MotorC_dir,HIGH);
//      digitalWrite(MotorB_dir,HIGH);
//      st_v = -1 * V_right;
//      th_v = -1 * V_right;
//    }
//  }
//}



//void cycle_function(){
////    float _v = 0.5; //encoder
////    float Circle_Time = 2.0 * Wheel_R * PI / _v; // 一圈週期(s)
////    float Per_Sec_Pulse = Unit_Pulse / Circle_Time; // 1秒pulse 
//  Per_Sec_Pulse_list[0] = (st_v == ? 0 : floor(Unit_Pulse / (2.0 * Wheel_R * 3.14 / st_v)) / 100 / 100);
//  Per_Sec_Pulse_list[1] = (nd_v == ? 0 : floor(Unit_Pulse / (2.0 * Wheel_R * 3.14 / nd_v)) / 100 / 100);
//  Per_Sec_Pulse_list[2] = (rd_v == ? 0 : floor(Unit_Pulse / (2.0 * Wheel_R * 3.14 / rd_v)) / 100 / 100);
//  Per_Sec_Pulse_list[3] = (th_v == ? 0 : floor(Unit_Pulse / (2.0 * Wheel_R * 3.14 / th_v)) / 100 / 100);
//
//  for(int k = 0;k <4 ;++k)增量式编码器0
//}




void Two_Wheel_Kinematics(double v,double w)/*float &v_left, float &v_right*/ 
{ 
  //w強制在同為正或同為負時 避免車子正負狀態時自旋
  if(w>=0){
    v_right = v + ceil((double)(w * Two_Wheel_Length))  / 2.0;  //ceil傳回不小於x的最小整數
    v_left = v - ceil((double)(w * Two_Wheel_Length)) / 2.0;
  }
  else{
    v_right = v + (-1 * ceil((double)(abs(w) * Two_Wheel_Length) / 2.0));
    v_left = v - (-1 * ceil((double)(abs(w) * Two_Wheel_Length) / 2.0));
  }
  _want_vr = v_right * ADJUST_R_WHEEL_RATE; //運動學後值
  _want_vl  = v_left * ADJUST_L_WHEEL_RATE;  //運動學後值
}

/**************************PID************************************/
void PID_function(){
  double error_Left = _want_vl - _now_vl; //左車與左encoder回傳值差
  double error_Right = _want_vr - _now_vr; //右車與右encoder回傳值差
  error_left_I += error_Left * 0.001; //未來的偏差量
  error_right_I += error_Right * 0.001; //未來的偏差量
  //馬達旋轉量(Turn) = Kp * (現在的偏差量) + Ki * (過去的偏差量) + Kd * (未來的偏差量)
  double Ul = KP * error_Left + Kd * (error_Left - pre_vl_error) /*+ error_left_I * KI*/;
  double Ur = KP * error_Right + Kd * (error_Left - pre_vr_error)/* + KI * error_right_I*/;
  pre_vl_error = error_Left;
  pre_vr_error = error_Right;

  if (_want_vl == 0 && _want_vr == 0 && fabs(Ur) < 150 && fabs(Ul) < 150 ) { //_want_v 運動學後
    digitalWrite(Motor_Brake, HIGH); 
    error_left_I = 0;
    error_right_I = 0;
  } else {
    digitalWrite(Motor_Brake, LOW);
  }
  new_VtoPwm( Ul, MotorA_dir, MotorB_dir, MotorA_PWM,MotorB_PWM); //送出新的速度
  new_VtoPwm( Ur, MotorC_dir, MotorC_dir, MotorC_PWM,MotorD_PWM); //送出新的速度
  
}
/**************************PID************************************/

/**************************SetPWM************************************/
void  new_VtoPwm(double V, int pin_direction1, int pin_direction2, int pin_PWM1,int pin_PWM2){
 int  max_pwm = 4000;
 if(pin_direction1 == MotorA_dir && pin_direction2 == MotorB_dir){
   if(V > 0){
     digitalWrite(pin_direction1, LOW);
     digitalWrite(pin_direction2, LOW);
     int _pwm = (V > max_pwm) ? max_pwm : V;
     analogWrite(pin_PWM1,_pwm);
     analogWrite(pin_PWM2,_pwm);
   }
   else{
    digitalWrite(pin_direction1, HIGH);
    digitalWrite(pin_direction2, HIGH);
    V= V * -1;
    int _pwm = (V > max_pwm) ? max_pwm : V;
    analogWrite(pin_PWM1,_pwm);
    analogWrite(pin_PWM2,_pwm);    
   }
 }else{
   if(V > 0){
      digitalWrite(pin_direction1, HIGH);
      digitalWrite(pin_direction2, HIGH);
      int _pwm = (V > max_pwm) ? max_pwm : V;
      analogWrite(pin_PWM1,_pwm);
      analogWrite(pin_PWM2,_pwm);  
   }else{
      digitalWrite(pin_direction1, LOW);
      digitalWrite(pin_direction2, LOW);
      V = V * -1;
      int _pwm = (V > max_pwm) ? max_pwm : V;
      analogWrite(pin_PWM1,_pwm);
      analogWrite(pin_PWM2,_pwm);
    }  
  }
}


/**************************SetPWM************************************/


/**************************EncoderReceive************************************/
double ressolution = 0.3 * PI / 4000.0; // 輪胎圓周(m)/一圈轉幾格pulse //原本4000但encoder 怪怪的
void encoder_receive(){ //里程計累積function 1ms running
   count_CSRight = CS(CS3);
   count_CSLeft = CS(CS1);
   long count_CSL_dis = count_CSLeft - count_CSLeft_old;
   long count_CSR_dis = count_CSRight - count_CSRight_old;

   //回授的速度
   double vl = count_CSL_dis * 100.0 * ressolution / 0.001; //單位 cm/s(速度) 
   double vr = count_CSR_dis * 100.0 * ressolution / 0.001; //單位 cm/s(速度)
//   double rpmLeft = vl * 60.0 / PI * 
//   double rpmRight = 
   _now_vl = vl;
   _now_vr = vr;
   
   double dtheta = (count_CSR_dis - count_CSL_dis) * ressolution / 0.565; //沒有透過IMU 直接透過雙輪里程計去累積計算。
   x_world = x_world + ((count_CSL_dis + count_CSR_dis) / 2.0) * ressolution * 100.0 * cos(theta); //單位 cm
   y_world = y_world + ((count_CSL_dis + count_CSR_dis) / 2.0) * ressolution * 100.0 * sin(theta);
   theta = theta - dtheta;
   send_data(x_world, y_world, theta, vl, vr);
   count_CSLeft_old = count_CSLeft;
   count_CSRight_old = count_CSRight;   
}

long CS(int pin)
{
  long count = 0;
  digitalWrite(pin, HIGH);
  byte b = SPI.transfer((byte) RD | CNTR);
  count = SPI.transfer(0x00);
  count <<= 8;
  count |= SPI.transfer(0x00);
  count <<= 8;
  count |= SPI.transfer(0x00);
  count <<= 8;
  count |= SPI.transfer(0x00);
  digitalWrite(pin, LOW);
  return count;
}
/**************************Send里程************************************/
void send_data(double X_w, double Y_w, double T, double _vl, double _vr){
  int X_world_int = (int)X_w;
  int Y_world_int = (int)Y_w;
  int _vl_int = (int)_vl;
  int _vr_int = (int)_vr;
  byte _vl_float = ((long)(_vl * 100.0) % 100);
  byte _vr_float = ((long)(_vr * 100.0) % 100);
  byte X_world_float = ((long)(X_w * 100.0) % 100);
  byte Y_world_float = ((long)(Y_w * 100.0) % 100);
  byte theta_float = ((long)(T * 100.0) % 100);
  byte a[19];
  a[0] = 'A';
  a[1] = 'B';
  a[2] = (byte)((X_world_int & 0xFF00) >> 8);
  a[3] = (byte)((X_world_int & 0x00FF));
  a[4] = (byte)(X_world_float);
  a[5] = (byte)((Y_world_int & 0xFF00) >> 8);
  a[6] = (byte)(Y_world_int & 0x00FF);
  a[7] = (byte)(Y_world_float);
  a[8] =  (byte)(((int)T & 0xFF00) >> 8);
  a[9] =  (byte)((int)T & 0x00FF);
  a[10] =  (byte)(theta_float);
  a[11] = (byte)((_vl_int & 0xFF00) >> 8);
  a[12] = (byte)((_vl_int & 0x00FF));
  a[13] = (byte)(_vl_float);
  a[14] = (byte)((_vr_int & 0xFF00) >> 8);
  a[15] = (byte)(_vr_int & 0x00FF);
  a[16] = (byte)(_vr_float);
  a[17] =  'C';
  a[18] =  'D';
  for (int i = 0; i < 19; i++) {
    //Serial.write(a[i]);
  }    
}
/**************************Send里程************************************/

/**************************EncoderReceive************************************/
//void motor_Control(const float v_left, const float v_right) 
//{
//   PWM_Right_Control = (v_right /max_rpm)*4095;
//   PWM_Left_Control = (v_left /max_rpm)*4095;
//   analogWrite(MotorA_PWM,PWM_Right_Control);
//   analogWrite(MotorB_PWM,PWM_Left_Control);
//   analogWrite(MotorC_PWM,PWM_Right_Control);
//   analogWrite(MotorD_PWM,PWM_Left_Control);
//}

//void serialEvent() 
//{
//  receive_package();
//  if (Serial.available() > 0) {
//    rxBuffer[rxIndex++] = Serial.read();
//    if (rxBuffer[rxIndex - 1] == '\n') {
//      Serial.println("received!!!");
//      rxBuffer[rxIndex] = '\0';
//      Speed = atoi(rxBuffer);
//      Serial.println(Speed);
//      rxIndex = 0;
//    }
//  }
//}

/**************************ReceivePackage************************************/
void receiveConnect()
{
  if(connection == con_check)//判斷有無斷線
  {
    static int con_count = 0;
    con_count++;
    if(con_count > 10) //超過10次沒更新斷線
    {
      vx = 0;
      vy = 0;
       w = 0;
      con_count = 0;
    }
    con_check = connection;
    delay(100);
  }   
}

void serialEvent()
{
    if(Serial.available() > 0)
    {
        unsigned char data = Serial.read();
        //Serial.println(data);
        if(data == 'S' && count == 0)
        {
            databuf[count] = data;
            count++;
        }
        else if(data == 'T' && count == 1)
        {
            databuf[count] = data;
            count++;
        }
        else if(count>1 && count < 17)
        {
            databuf[count] = data;
            count++;
        }
        else if(data == 'E' && count == 17)
        {
            databuf[count] = data;
            count++;
        }
        else if(data == 'N' && count == 18)
        {
            databuf[count] = data;
            count++;
        }
        else if(data == 'D' && count == 19)
        {
            databuf[count] = data;
            count=0;

            char checksum = databuf[2] + databuf[3] + databuf[4] + databuf[5] + databuf[6] + databuf[7] + databuf[8] + databuf[9] + databuf[10] + databuf[11] + databuf[12] + databuf[13];
            if(databuf[0] == 'S' && databuf[1] == 'T' && databuf[17] == 'E' && databuf[18] == 'N' && databuf[19] == 'D' && databuf[14] == checksum)
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

                vx = re_vx - 100;//拿去用
                vy = re_vy - 100;//拿去用
                w = re_w - 100;//拿去用
                Serial.print(vx);
                Serial.print(" ");
                Serial.println(vy);
                Two_Wheel_Kinematics(vx, vy); //進差速輪運動學
            }
        }
        else
        {
            memset(databuf, 0, sizeof(databuf));
            count = 0;
        }

    }
}
/**************************ReceivePackage************************************/
