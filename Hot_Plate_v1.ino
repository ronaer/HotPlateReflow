/*******************************************************************************
 

   Sıcaklık °C 
       |         
   180-|--------------------------------------------------------  x   x
       |                                                   x   x          x     
       |                                                x                    x  
   150-|--------------------------------------------- x |                   |  x 
       |                                            x   |                   |     x
       |                                        x       |                   |       x
   125-|----------------------------------x  x          |                   |          x
       |                 x  x  x  x  x  x               |                   |
       |               x                |               |                   |
       |             x |                |               |                   |
       |           x   |                |               |                   |
       |         x     |                |               |                   |
       |       x       |                |               |                   |
       |     x         |                |               |                   |
       |   x           |                |               |                   |
   25 -| x             |                |               |                   |
       |<     ~60 s   >|<    ~120 s    >|<    ~30 s    >|<      ~60 s      >|
       |    Ön Isıtma  |   Sıvılaşma    |     Isıtma    |  Lehimleme İşlemi |   Soğuma
    0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _| _ _ _ _ _ _ _
                                                                  Zaman Saniye


-HotPlate Profil Set & PWM & PID örneği;
--Dr.TRonik YouTube/Mart 2024/İzmir/Türkiye.
---https://forum.arduino.cc/t/about-pid-controller/176991/13
---https://www.arduino.cc/reference/en/language/functions/advanced-io/tone/
*******************************************************************************/

/********************************************************************
  GLOBALS___GLOBALS___GLOBALS___GLOBALS___GLOBALS___GLOBALS___
*********************************************************************/
// Kütüphaneler
#include <PID_v1.h>  //PID Kontrol
//#include <math.h>    // NTC sıcaklık Logaritmik heaplamaları için / Yeni versiyon IDE'lerde eklenmeyebilir...

//ST7735
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7735.h>  // Hardware-specific library for ST7735
#include <SPI.h>              // SPI comminication

//Nesneler, değişkenler, Tanımlar...
double Setpoint, Input, Output;
/*
PID kontrol parametreleri sisteme özgü belirlenmelidir:
Kp: Değer arttıkça, hedefe daha hızlı ulaşır, ancak hedefi geçme riski de artar. Hatanın ANLIK değerini oluşturur. 
Ki: Değer azaldıkça (0 kapatır), kontrolör o kadar hızlı tepki verir, ancak salınım riski o kadar artar.Hatanın GEÇMİŞ değerlerini hesaba katar.
Kd: Değer arttıkça, hedefte tutabilmek için daha sert davranır, salınım artar. Mevcut değişim oranına göre, hatanın GELECEKTEKİ değerleri için hesaplama yapar.
*/
double Kp = 15, Ki = 0.2, Kd = 4.5;
PID hotPlate_PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Arduino nano/uno için st7735 spi pin bağlantıları - Pin Connections for nano/uno
#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 8
//SCL D13
//SDA D11
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

#define ntcPin A0      // NTC sensör pin...
#define ssrOutPin 3    // SSR Röle kontrol pini...
#define btnPin 7       // BUTTON PIN NUMBER
#define buzzer 6       // Buzzer PIN NUMBER
bool btnState = HIGH;  // buttonState_btnPin
static unsigned long timer, timer1;
int state = 6;
double sicaklik;
char Output_c[4];  //out() için

/********************************************************************
  SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___
*********************************************************************/
void setup() {
  // Open Serial communication
  Serial.begin(9600);

  tft.initR(INITR_BLACKTAB);     // initialize a ST7735S chip, black tab
  tft.setTextWrap(false);        // Allow text to run off right edge
  tft.fillScreen(ST77XX_BLACK);  // Ekranı temizlemek VE canvası siyah yapabilmek için
  tft.setRotation(1);            // 0_0°, 1_90°, 2_180°, 3_270° rotasyon...

  digitalWrite(ssrOutPin, LOW);
  pinMode(ssrOutPin, OUTPUT);
  pinMode(btnPin, INPUT_PULLUP);
  digitalWrite(btnPin, HIGH);
  Input = sicaklik;

  hotPlate_PID.SetMode(AUTOMATIC);  //turn the PID on

  tft.fillScreen(0);
  tft.setCursor(20, 16);
  tft.setTextColor(0xFFFF, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...
  tft.setTextSize(2);
  tft.print("SICAK PLAKA");
  tft.setCursor(30, 36);
  tft.print("LEHiMLEME");
  tft.setTextColor(0xFFE0, 0x0000);
  tft.setCursor(10, 70);
  tft.print("Dr.TRonik");
  tft.setCursor(69, 93);
  tft.print("YouTube");
  tft.setCursor(38, 121);
  tft.setTextColor(0xFFFF, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...
  tft.setTextSize(1);
  tft.print("HotPlate Reflow");

  tft.fillRoundRect(33, 88, 33, 24, 8, ST77XX_RED);
  tft.fillTriangle(45, 105, 45, 93, 56, 99, ST77XX_WHITE);
  tone(buzzer, 900, 200);

  delay(3000);
  tft.fillScreen(0);
  tone(buzzer, 900, 100);
}

/********************************************************************
  LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__
*********************************************************************/
void loop() {

  if (state >= 0 && state < 5) {
    out();
  }

  /////STATE 0 PWM kontrol PreHeat
  if (state == 0) {

    readTemp();

    if (sicaklik < 102) {
      Output = 42;
      analogWrite(ssrOutPin, Output);
    }
    if (sicaklik >= 100 && sicaklik < 126) {
      Output = 4;
      analogWrite(ssrOutPin, Output);
    }

    tft.setTextSize(2);

    tft.setCursor(0, 0);
    tft.setTextSize(2);
    tft.print("DURUM 1");
    tft.setCursor(90, 0);

    tft.print("ISITMA");

    tft.setCursor(0, 20);
    tft.setTextColor(0x07E0, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...
    tft.print("HEDEF: 125");

    tft.setCursor(0, 22);
    tft.drawCircle(126, 22, 3, ST77XX_GREEN);

    tft.setCursor(133, 20);
    tft.print("C");
    tft.setTextColor(0xFFE0, 0x0000);  //SARI
    tft.setCursor(0, 73);
    tft.print("KONTROL:PWM");

    tft.setTextColor(0xFFFF, 0x0000);  //BEYAZ
    tft.setCursor(0, 94);
    tft.print("OUT VAL:");

    tft.setCursor(100, 94);
    tft.print(Output_c);

    if (sicaklik >= 126) {
      //tft.fillScreen(ST77XX_BLACK);
      state = 1;
      timer = millis();
    }
  }

  /////STATE 1 125° PID kontrol Islatme/Soaking/Erime
  if (state == 1) {
    readTemp();

    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.setTextSize(2);
    tft.setTextColor(0xFFE0, 0x0000);
    tft.print("DURUM 2");
    tft.setCursor(90, 0);
    tft.print("ERiME ");

    tft.setCursor(0, 20);
    tft.setTextColor(0x07E0, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...
    tft.print("120sn.");

    tft.setCursor(0, 22);
    tft.drawCircle(126, 22, 3, ST77XX_GREEN);

    tft.setCursor(133, 20);
    tft.print("C");

    tft.setCursor(0, 73);
    tft.print("KONTROL:PID");
    tft.setTextColor(0xFFFF, 0x0000);  //BEYAZ
    tft.setCursor(0, 94);
    tft.print("OUT VAL:");

    tft.setCursor(100, 94);
    tft.print(Output_c);

    Output = constrain(Output, 0, 6);
    Input = sicaklik;
    Setpoint = 125;

    PID hotPlate_PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
    hotPlate_PID.SetMode(AUTOMATIC);
    hotPlate_PID.Compute();
    analogWrite(ssrOutPin, Output);


    if (millis() - timer > 120000) {
      state = 2;
      tft.fillScreen(ST77XX_BLACK);
    }
  }

  /////STATE 2 RAMP TO PEAK  PWM Kontrol
  if (state == 2) {

    readTemp();

    tft.setTextSize(2);
    tft.setTextColor(0x07FF, 0x0000);
    tft.setCursor(0, 0);
    tft.setTextSize(2);
    tft.print("DURUM 3");
    tft.setCursor(90, 0);
    tft.print("RAMPTO");

    tft.setCursor(0, 20);
    tft.setTextColor(0x07E0, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...
    tft.print("HEDEF: 180");

    tft.setCursor(0, 22);
    tft.drawCircle(126, 22, 3, ST77XX_GREEN);

    tft.setCursor(133, 20);
    tft.print("C");

    tft.setCursor(0, 73);
    tft.setTextColor(0xFFE0, 0x0000);  //SARI
    tft.print("KONTROL:PWM");
    tft.setTextColor(0xFFFF, 0x0000);  //BEYAZ
    tft.setCursor(0, 94);
    tft.print("OUT VAL:");

    tft.setCursor(100, 94);
    tft.print(Output_c);



    if (sicaklik < 165) {
      Output = 44;
      analogWrite(ssrOutPin, Output);
    }
    if (sicaklik >= 165 && sicaklik < 180) {
      Output = 4;
      analogWrite(ssrOutPin, Output);
    }

    if (sicaklik >= 181) {
      state = 3;
      timer1 = millis();
    }
  }

  /////STATE 3 180° PID kontrol REFLOW/Lehimleme
  if (state == 3) {

    readTemp();

    tft.setTextSize(2);
    tft.setTextColor(0xF800, 0x0000);
    tft.setCursor(0, 0);
    tft.setTextSize(2);
    tft.print("DURUM 4");
    tft.setCursor(90, 0);

    tft.print("REFLOW");

    tft.setCursor(0, 20);
    tft.setTextColor(0x07E0, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...
    tft.print("60sn.  180");

    tft.setCursor(0, 22);
    tft.drawCircle(126, 22, 3, ST77XX_GREEN);

    tft.setCursor(133, 20);
    tft.print("C");

    tft.setCursor(0, 73);
    tft.print("KONTROL:PID");

    tft.setTextColor(0xFFFF, 0x0000);  //BEYAZ
    tft.setCursor(0, 94);
    tft.print("OUT VAL:");

    tft.setCursor(100, 94);
    tft.print(Output_c);

    Output = constrain(Output, 0, 6);
    Input = sicaklik;
    Setpoint = 180;

    PID hotPlate_PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
    hotPlate_PID.SetMode(AUTOMATIC);
    hotPlate_PID.Compute();
    analogWrite(ssrOutPin, Output);


    if (millis() - timer1 > 60000) {
      tone(buzzer, 250, 100);
      delay(140);
      tone(buzzer, 250, 100);
      delay(140);
      tone(buzzer, 450, 100);
      delay(140);
      tone(buzzer, 450, 100);
      delay(140);
      tone(buzzer, 650, 100);
      delay(140);
      tone(buzzer, 650, 100);
      tft.fillScreen(ST77XX_BLACK);
      state = 4;
    }
  }

  /////STATE 4 SOĞUMA <40°
  if (state == 4) {
    readTemp();
    Output = 0;
    analogWrite(ssrOutPin, Output);

    tft.setTextSize(2);
    tft.setTextColor(0xFFFF, 0x0000);  //BEYAZ
    tft.setCursor(0, 0);
    tft.setTextSize(2);
    tft.print("DURUM 5");
    tft.setCursor(90, 0);
    tft.print("SOGUMA");

    tft.setCursor(0, 20);
    tft.setTextColor(0x07E0, 0x0000);
    tft.print("HEDEF: <40");

    tft.setCursor(0, 22);
    tft.drawCircle(126, 22, 3, ST77XX_GREEN);

    tft.setCursor(133, 20);
    tft.print("C");

    tft.setTextColor(0xF800, 0x0000);  //KIRMIZI
    tft.setCursor(0, 73);
    tft.print("!SICAK PLAKA!");

    tft.setTextColor(0xFFFF, 0x0000);
    tft.setCursor(0, 94);
    tft.print("OUT VAL:");

    tft.setCursor(100, 94);
    tft.print(Output_c);

    if (sicaklik <= 40) {

      state = 5;
    }
  }

  /////STATE 5 THE END!
  if (state == 5) {
    tft.fillScreen(ST77XX_BLACK);
    Output = 0;
    analogWrite(ssrOutPin, Output);

    tft.setTextSize(2);

    tft.setCursor(28, 46);
    tft.setTextSize(2);
    tft.print("LEHiMLEME");
    tft.setCursor(45, 73);
    tft.print("BiTTi!");

    tft.invertDisplay(true);
    tone(buzzer, 800, 100);
    delay(500);
    tft.invertDisplay(false);
    tone(buzzer, 600, 100);
    delay(500);
  }

  if (state == 6) {
    Output = 0;
    analogWrite(ssrOutPin, Output);

    //tone(buzzer, 900, 400);

    tft.fillCircle(135, 28, 14, ST77XX_BLUE);
    tft.fillRoundRect(130, 10, 10, 20, 8, ST77XX_RED);
    tft.setTextSize(1);
    tft.setCursor(10, 20);
    tft.setTextColor(0xFFE0, 0x0000);  //SARI
    tft.print("Baslamak Icin...");
    tft.drawPixel(24, 29, ST77XX_YELLOW);
    tft.drawPixel(72, 29, ST77XX_YELLOW);
    tft.drawPixel(66, 16, ST77XX_YELLOW);

    tft.setTextColor(0xFFFF, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...
    tft.setTextSize(2);
    tft.setCursor(28, 46);
    tft.setTextSize(2);
    tft.print("BUTONA");
    tft.setCursor(45, 73);
    tft.print("BASINIZ");

    tft.setCursor(59, 105);
    tft.setTextSize(1);
    tft.setTextColor(0xFFE0, 0x0000);  //SARI
    tft.print("Profil: SnBiAg...");
    tft.setTextSize(2);
    tft.setCursor(90, 114);
    tft.setTextSize(2);
    //tft.setTextColor(0xF800, 0x0000);  //KIRMIZI
    tft.print("   ");


    //Okuma: "Steinhart–Hart" eşitliği
    sicaklik = log(((10240000 / analogRead(ntcPin)) - 10000));
    sicaklik = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * sicaklik * sicaklik)) * sicaklik);
    sicaklik = sicaklik - 273.15;      //Kelvin → Celcius
    tft.setTextColor(0xFFFF, 0x0000);  //BEYAZ
    char si_c[4];
    sprintf(si_c, "%3d", int(sicaklik));

    tft.setCursor(0, 113);
    tft.print(si_c);
    tft.drawCircle(39, 114, 2, ST77XX_WHITE);
    tft.setTextSize(1);
    tft.setCursor(43, 114);
    tft.print("C");

    btnState = digitalRead(btnPin);
    if (btnState == LOW) {
      if (sicaklik < 40) {
        tft.fillScreen(0);
        tft.setTextColor(0xFFFF, 0x0000);  //BEYAZ
        tft.setCursor(28, 55);
        tft.setTextSize(2);
        tft.print("BASLIYOR!");
        tft.drawPixel(56, 72, ST77XX_WHITE);
        delay(2000);
        tft.fillScreen(0);
        state = 0;
        tone(buzzer, 900, 100);
      } else {
        tft.setCursor(90, 114);
        tft.setTextSize(2);
        tft.setTextColor(0xF800, 0x0000);  //KIRMIZI
        tft.print("!!!");
        tone(buzzer, 250, 100);
      }
    }
  }
}




/********************************************************************
  VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs
********************************************************************/
void readTemp() {
  //Okuma: "Steinhart–Hart" eşitliği
  sicaklik = log(((10240000 / analogRead(ntcPin)) - 10000));
  sicaklik = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * sicaklik * sicaklik)) * sicaklik);
  sicaklik = sicaklik - 273.15;  //Kelvin → Celcius

  tft.setCursor(0, 0);
  tft.setTextColor(0xFFFF, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...

  if (sicaklik < 40) {
    tft.drawRect(1, 42, 159, 24, ST77XX_WHITE);
  }
  if (sicaklik >= 40 && sicaklik < 101) {
    tft.drawRect(1, 42, 159, 24, ST77XX_YELLOW);
  }
  if (sicaklik > 101) {
    tft.drawRect(1, 42, 159, 24, ST77XX_RED);
  }
  char si_c[4];
  sprintf(si_c, "%3d", int(sicaklik));

  tft.setCursor(6, 46);
  tft.setTextColor(0xFFFF, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...
  tft.setTextSize(2);
  tft.print("SCKLIK:") + tft.print(si_c);
  tft.drawCircle(130, 48, 3, ST77XX_WHITE);
  tft.setCursor(137, 46);
  tft.print("C");
}

void out() {

  sprintf(Output_c, "%03d", int(Output));

  tft.setCursor(38, 120);
  tft.setTextColor(0xFFFF, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...
  tft.setTextSize(1);
  tft.print("Profil: SnBiAg");
}


/* ___İletişim:
e-posta: bilgi@ronaer.com
https://www.instagram.com/dr.tronik2023/   
YouTube: Dr.TRonik: https://www.youtube.com/@DrTRonik
PCBWay: https://www.pcbway.com/project/member/shareproject/?bmbno=A0E12018-0BBC-4C

Color definitions for tft:
 #define BLACK 0x0000
 #define BLUE 0x001F
 #define RED 0xF800
 #define GREEN 0x07E0
 #define CYAN 0x07FF
 #define MAGENTA 0xF81F
 #define YELLOW 0xFFE0
 #define WHITE 0xFFFF

 Alternatif PID kontrol:
int WindowSize = 1000;
unsigned long now = millis();
if (now - windowStartTime > WindowSize) {  //time to shift the Relay Window
  windowStartTime += WindowSize;
}
if (Output > now - windowStartTime) digitalWrite(ssrOutPin, HIGH);
else digitalWrite(ssrOutPin, LOW);
*/