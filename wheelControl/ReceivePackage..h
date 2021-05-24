unsigned char databuf[20];     // 用來儲存收進來的 data byte
int count = 0;
int connection = -1;//判斷連線
int con_check = -1;//判斷連線
int mode = 0;

void receive_package()
{
  if (Serial.available() > 0)
  {
    unsigned char data = Serial.read();
    //Serial.println(data);
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

        vx = re_vx - 100;//拿去用
        vy = re_vy - 100;//拿去用
        w = re_w - 100;//拿去用
        Serial1.print(vx);
        Serial1.print(" ");
        Serial1.println(w);
        //rampGenerator.generateVelocityProfile(vx, 100);
        Two_Wheel_Kinematics(vx, w);
      }
    }
    else
    {
      memset(databuf, 0, sizeof(databuf));
      count = 0;
    }

  }
}