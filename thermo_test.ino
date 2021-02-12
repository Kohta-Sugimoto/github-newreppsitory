#include <M5Stack.h>
#include <Wire.h>
#include <WiFi.h>

#define FPSC 0x02
#define INTC 0x03
#define AVE 0x07
#define T01L 0x80

/*for displaying color*/
float gain = 10.0;
float offset_x = 0.2;
float offset_green = 0.6;

float sigmoid(float x, float g, float o){
    return (tanh((x + o) * g / 2) + 1) / 2;
}

uint16_t heat(float x){  // 0.0〜1.0の値を青から赤の色に変換する
    x = x * 2 - 1;  // -1 <= x < 1 に変換

    float r = sigmoid(x, gain, -1 * offset_x);
    float b = 1.0 - sigmoid(x, gain, offset_x);
    float g = sigmoid(x, gain, offset_green) + (1.0 - sigmoid(x, gain, -1 * offset_green)) - 1;

    return (((int)(r * 255)>>3)<<11) | (((int)(g * 255)>>2)<<5) | ((int)(b * 255)>>3);
}


/*for i2c*/
void write8(int id, int reg, int data){
    Wire.beginTransmission(id);
    Wire.write(reg);
    Wire.write(data);
    uint8_t result = Wire.endTransmission();
    Serial.printf("reg: 0x%02x, result: 0x%02x\r\n", reg, result);
}

void dataread(int id,int reg,int *data,int datasize){
    Wire.beginTransmission(id);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(id, datasize);
    int i = 0;
    while (Wire.available() && i < datasize){
        data[i++] = Wire.read();
    }
}


#define AMG88_ADDR 0x68 // in 7bit
#define WIDTH (320 / 5)
#define HEIGHT (240 / 5)
#define DATASIZE 64
#define THERMOCNT 4

int mode_number = 0;
float tempPrevious[DATASIZE + 1];
int detectSensity = 1;
bool thermoCheck = false;
int thermoCnt = 0;
float thermoAve[DATASIZE];
float standardThermo = -1;

void setup(){
    M5.begin();
    Serial.begin(115200);
    pinMode(21, INPUT_PULLUP);
    pinMode(22, INPUT_PULLUP);
    Wire.begin();
    M5.Speaker.begin();
    M5.Speaker.mute();              //noise removal
    WiFi.mode(WIFI_OFF);

    tempPrevious[0] = 0;
    
    write8(AMG88_ADDR, FPSC, 0x00);  // 10fps
    write8(AMG88_ADDR, INTC, 0x00);  // INT出力無効
    write8(AMG88_ADDR, 0x1F, 0x50);  // 移動平均出力モード有効
    write8(AMG88_ADDR, 0x1F, 0x45);
    write8(AMG88_ADDR, 0x1F, 0x57);
    write8(AMG88_ADDR, AVE, 0x20);
    write8(AMG88_ADDR, 0x1F, 0x00);
}




float maxTemp(float *temp){
    float max = 0;
    int i;
    
    for(i = 0; i < DATASIZE; i++){
        if(temp[i] > max){
            max = temp[i];
        }
      }
    
    return max;
}

float minTemp(float *temp){
    float min = 100;
    int i;

    for(i = 0; i < DATASIZE; i++){
        if(temp[i] < min){
          min = temp[i];
        }
    }

    return min;
}

void DrawMinMax(float *temp){
    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(2 * WIDTH + 10, 1 * HEIGHT);
    M5.Lcd.setTextColor(0xF000, 0xFFFF);
    M5.Lcd.printf("max:%.1f", maxTemp(temp));
    M5.Lcd.setTextSize(1);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(2 * WIDTH + 10, 10);
    M5.Lcd.setTextColor(0xF000, 0xFFFF);
    M5.Lcd.printf("min:%.1f", minTemp(temp));
    M5.Lcd.setTextSize(1);
}



bool isInvade(float *temp){
    unsigned int over_cnt = 0;
    int i;
    for(i = 0; i < DATASIZE; i++){
        if(temp[i] > 20){
            over_cnt++;
        }
    }

    if(over_cnt >= 10){
        double tempChange_sum = 0;
        for(i = 0; i < DATASIZE; i++){
            tempChange_sum += abs(temp[i] - tempPrevious[i + 1]);
        }
        if(tempChange_sum >= 2.5 * DATASIZE / detectSensity){
            return true;
        }
    }
        
    return false;
}



float TempMeasure(float *temp){
    int i;
    float checkThermo = 0;
    thermoCnt++;

    for(i = 0; i < DATASIZE; i++){
        thermoAve[i] += temp[i];
    }

    if(thermoCnt >= THERMOCNT){
        for(i = 0; i < DATASIZE; i++){
            thermoAve[i] /= THERMOCNT;
          //  Serial.print(thermoAve[i]);
           // Serial.print("\n");
        }
        if(standardThermo < 0){            //set normal temp
          standardThermo = thermoAve[0];
          for(i = 1; i < DATASIZE; i++){
              standardThermo += thermoAve[i];
          }
          standardThermo /= DATASIZE;
          Serial.print("standardThermo = ");
          Serial.print(standardThermo);
          Serial.print("\n");
        }else{                            //check increase temp
            checkThermo = thermoAve[0];
            for(i = 1; i < DATASIZE; i++){
                checkThermo += thermoAve[i];
            }
            checkThermo /= DATASIZE;
           
            Serial.print("checkThermo = ");
            Serial.print(checkThermo);
            Serial.print("\n");
            if(checkThermo > standardThermo){
                Serial.print("hot");
                M5.Lcd.setTextSize(3);
                M5.Lcd.setCursor(50, 3 * HEIGHT - 10);
                M5.Lcd.setTextColor(BLACK, 0xFFFF);
                M5.Lcd.printf("Be Careful");    //mode display
                M5.Lcd.setTextSize(1);
            }
        }

        thermoCheck = false;
        thermoCnt = 0;
    }

    return checkThermo;
}


void DrawThermo_mode0(float *temp){
    M5.Lcd.fillScreen(0xFFFF);
    int x, y;
    for(y = 0; y < 4; y++){
        for(x = 0; x < 4; x++){
            float t = temp[(8 - y - 1) * 8 + 8 - x - 1];
            uint16_t color = heat(map(constrain((int)t, 0, 60), 0, 60, 0, 100) / 100.0);
            M5.Lcd.fillRect(x * WIDTH, y * HEIGHT, WIDTH, HEIGHT, color);
            M5.Lcd.setCursor(x * WIDTH + WIDTH / 2, y * HEIGHT + HEIGHT / 2);
            M5.Lcd.setTextColor(BLACK, color);
            M5.Lcd.printf("%d", (int)t);
        }
    }
}

void DrawThermo_mode1(float *temp, bool detect_flag){
    M5.Lcd.fillScreen(0xFFFF);
    int x, y;
    for(y = 0; y < 8; y++){
        for(x = 0; x < 8; x++){
            float t = temp[(8 - y - 1) * 8 + 8 - x - 1];
            uint16_t color = heat(map(constrain((int)t, 0, 60), 0, 60, 0, 100) / 100.0);
            M5.Lcd.fillRect(x * WIDTH * 1/4, y * HEIGHT * 1/4, WIDTH * 1/4, HEIGHT * 1/4, color);
       }
    }

    DrawMinMax(temp);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(25, 3 * HEIGHT + 25);
    M5.Lcd.setTextColor(BLACK, 0xFFFF);
    M5.Lcd.printf("Security Camera");    //mode display
    M5.Lcd.setTextSize(1);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(25, 3 * HEIGHT - 10);
    M5.Lcd.setTextColor(BLACK, 0xFFFF);
    M5.Lcd.printf("Sensity:%d", detectSensity);    //mode display
    M5.Lcd.setTextSize(1);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(2 * WIDTH + 10, 2 * HEIGHT);
    M5.Lcd.setTextColor(0xF000, 0xFFFF);
    if(detect_flag == true){
        M5.Lcd.printf("detect");
    }else{
        M5.Lcd.printf("non detect");
    }
    M5.Lcd.setTextSize(1);
  
}


void DrawThermo_mode2(float *temp){
    M5.Lcd.fillScreen(0xFFFF);
    int x, y;
    for(y = 0; y < 8; y++){
        for(x = 0; x < 8; x++){
            float t = temp[(8 - y - 1) * 8 + 8 - x - 1];
            uint16_t color = heat(map(constrain((int)t, 0, 60), 0, 60, 0, 100) / 100.0);
            M5.Lcd.fillRect(x * WIDTH * 1/4, y * HEIGHT * 1/4, WIDTH * 1/4, HEIGHT * 1/4, color);
       }
    }

    DrawMinMax(temp);

    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(45, 3 * HEIGHT + 25);
    M5.Lcd.setTextColor(BLACK, 0xFFFF);
    M5.Lcd.printf("thermometer");    //mode display
    M5.Lcd.setTextSize(1);


    float checkThermo;
    
    if(thermoCheck == true){
        checkThermo = TempMeasure(temp);
        if(checkThermo == 0){                 //基準計測中
            M5.Lcd.setTextSize(3);
            M5.Lcd.setCursor(25, 3 * HEIGHT - 10);
            M5.Lcd.setTextColor(BLACK, 0xFFFF);
            M5.Lcd.printf("Wait");    //mode display
            M5.Lcd.setTextSize(1);
        }else{

        }
    }else{
    }
}







void TranslateSensorData(int *sensorData, float *temp){
    for (int i = 0 ; i < DATASIZE; i++){
        int16_t temporaryData = sensorData[i * 2 + 1] * 256 + sensorData[i * 2];
        if(temporaryData > 0x200){
            temp[i] = (-temporaryData +  0xfff) * -0.25;
        }else{
            temp[i] = temporaryData * 0.25;
        }
    }
}


void loop(){
    int i;
    M5.update();
    
    if(M5.BtnA.wasPressed()){
        mode_number = 0;
    }
    if(M5.BtnB.wasPressed()){
        if(mode_number == 1){
          detectSensity++;
          if(detectSensity > 5){
            detectSensity = 1;
          }
        }
        mode_number = 1;
    }
    if(M5.BtnC.wasPressed()){
        if(mode_number == 2){
          thermoCheck = true;
          for(i = 0; i < DATASIZE; i++){
              thermoAve[i] = 0;
          }
        }
        mode_number = 2;
        
    }
    
    float temp[DATASIZE];
    Wire.requestFrom(AMG88_ADDR, 1);
    
    if(mode_number == 0){           //mode:0
        while (Wire.available()) {
            byte val = Wire.read();
        //  Serial.print(val);
        }
        int sensorData[128];
        dataread(AMG88_ADDR, T01L, sensorData, 128);
        TranslateSensorData(sensorData, temp);
        
        DrawThermo_mode0(temp);
    }
  
    if(mode_number == 1){           //mode:1 security camera
        while (Wire.available()) {
           byte val = Wire.read();
     //     Serial.print(val);
        }
        int sensorData[128];
        dataread(AMG88_ADDR, T01L, sensorData, 128);
    
        TranslateSensorData(sensorData, temp);

        if(isInvade(temp) == true){
            DrawThermo_mode1(temp, true);
        }else{
            DrawThermo_mode1(temp, false);
        }
        
        tempPrevious[0] = 1;
        for(i = 0; i < DATASIZE; i++){
          tempPrevious[i] = temp[i];     //以前の値を記録
        }
    }
  
    if(mode_number == 2){           //mode:2 Non-contact thermometer
        while (Wire.available()) {
           byte val = Wire.read();
     //     Serial.print(val);
        }
        int sensorData[128];
        dataread(AMG88_ADDR, T01L, sensorData, 128);
    
        TranslateSensorData(sensorData, temp);
        DrawThermo_mode2(temp);
    }

    
  
    M5.Lcd.setTextSize(5);
    M5.Lcd.setCursor(1 * WIDTH, 4 * HEIGHT);
    M5.Lcd.setTextColor(BLACK, 0xFFFF);
    M5.Lcd.printf("mode:%d", mode_number);    //mode display
    M5.Lcd.setTextSize(1);
  
    delay(1000);
   
}
