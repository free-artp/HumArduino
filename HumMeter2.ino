
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

#include <MiniPlanner.h>

// pins definition
#define HEATER_PWM 3
#define ONE_WIRE_BUS 4
#define KEY1 6
#define KEY2 7
#define KEY3 8
#define KEY4 9
#define LCD_ON 10

//-------------------------- Global vars.
Planner Sch;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int mode_count = 0;
unsigned long Tw, Tpid;
int workPID;
double Setpoint, Input, Output;
//
// 2, 5, 1
//
// R1
// 4, 0.2, 1
// 1, 0.05, 0.025
//
// R2
// 0.2, 0.01 0.005
//
// R3
// 0.1, 0.01 0.005
//
// R4
// 1.0, 0.001 0.005
//
// R5
// 0.2, 0.01 0.01
//
// R6
// 0.5, 0.01 0.01
//
PID pidA(&Input, &Output, &Setpoint, 0.5, 0.01, 0.01, DIRECT);  

//-------------------------- Device

void ShowData() {
  unsigned long now = millis();
//  sensors.requestTemperatures();
//  float t = sensors.getTempCByIndex(0);
  Serial.print("Time: ");  Serial.print( int(now/1000) );
  Serial.print(" Temp: "); Serial.print(Input);
  Serial.print(" Set: ");  Serial.print(Setpoint);
  Serial.print(" Val: ");  Serial.print(Output);
  Serial.print('\n');
}
//------------------------------- Switch on LCD
void lcd0() {
  digitalWrite( KEY1, LOW );
  Sch.schedule(15000, 1, mode );
}
void lcd1() {
  if ( digitalRead(LCD_ON) == LOW )
    digitalWrite( KEY1, HIGH);
  Sch.schedule(100, 1, lcd0 );
}

//-------------------------------- "ping" the LCD to prevent sleeping

void mode() {
  if ( mode_count % 2) {
    digitalWrite(KEY3, LOW);
  } else {
    digitalWrite(KEY3, HIGH);
  }
  mode_count++;
  if (mode_count == 8) {
    mode_count =0;
    Sch.schedule( 15000, 1, mode );
  } else {
    Sch.schedule( 100, 1, mode );
  }
}

//--------------------------------- Regulation of temp.
void GetTemp() {
  sensors.requestTemperatures();
  Input = sensors.getTempCByIndex(0);
}  

void DoPid() {
  GetTemp();
  pidA.Compute();
  analogWrite(HEATER_PWM, Output);
}

//---------------------------

unsigned long get_param() {
  char c;
  int i, l;
  unsigned long ret;
  l = i = 0;
  ret =0;
  while ( i<6 && l<10000  ) {
    if ( Serial.available()>0 ) {
      c = Serial.read();
      if ( (int)c>=48 && (int)c<=57) {
        ret *= 10;
        ret = ret + (int)c - 48;
        i++;
      }
    }
    l++;
  }
  return ret;
}
/*
  Sxxxxxx - Setpoint = xxx.xxx
  Pxxxxxx - Kp = xxx.xxx
  Ixxxxxx - Ki = xxx.xxx
  Dxxxxxx - Kd = xxx.xxx
  Txxxxxx - set dT for pid (msec)
  Wxxxxxx - set Interval for printing (msec)
  B - begin
  E - end
  H - print parametrs with '#': # S P I D T W
*/
void DoCmd() {
  switch ( Serial.read() ) {
    case '\n': 
      break;
    case 'B':
      if (workPID == 0) {
        workPID = 1;
        Sch.unschedule( GetTemp );
        Sch.schedule( Tpid, 0, DoPid);
      }
      break;
    case 'E':
      if ( workPID) {
        Sch.unschedule( DoPid );
        Output = 0;
        analogWrite(HEATER_PWM, Output);
        workPID = 0;
        Sch.schedule(Tpid, 0, GetTemp);
      }
      break;
    case 'S':
      Setpoint = get_param() / 1000.0;
      break;
    case 'P':
      pidA.SetTunings( get_param()/1000.0, pidA.GetKi(), pidA.GetKd() );
      break;
    case 'I':
      pidA.SetTunings( pidA.GetKp(), get_param()/1000.0, pidA.GetKd() );
      break;
    case 'D':
      pidA.SetTunings( pidA.GetKp(), pidA.GetKi(), get_param()/1000.0 );
      break;
    case 'T':
      Tpid = get_param();
      pidA.SetSampleTime( Tpid );
      if (workPID) {
        Sch.unschedule( DoPid );
        Sch.schedule( Tpid, 0, DoPid);
      } else {
        Sch.unschedule( GetTemp );
        Sch.schedule( Tpid, 0, GetTemp);
      }        
      break;
    case 'W':
      Sch.unschedule( ShowData );
      Tw = get_param();
      if (Tw > 0) Sch.schedule( Tw, 0, ShowData );
      break;
    case 'H':
      Serial.print("# St: ");  Serial.print( Setpoint );
      Serial.print(" Kp: ");  Serial.print( pidA.GetKp() );
      Serial.print(" Ki: ");  Serial.print( pidA.GetKi() );
      Serial.print(" Kd: ");  Serial.print( pidA.GetKd() );
      Serial.print(" Tp: ");  Serial.print( Tpid );
      Serial.print(" Tw: ");  Serial.print( Tw );
      Serial.print("\n");
      break;
  }
}

//---------------------------
void setup() {

  pinMode(HEATER_PWM, OUTPUT);  digitalWrite(HEATER_PWM, LOW);
  pinMode(LCD_ON, INPUT);

  pinMode(KEY1, OUTPUT);  digitalWrite(KEY1, LOW);  // on/off
  pinMode(KEY2, OUTPUT);  digitalWrite(KEY2, LOW);  // tare
  pinMode(KEY3, OUTPUT);  digitalWrite(KEY3, LOW);  // mode
  pinMode(KEY4, OUTPUT);  digitalWrite(KEY4, LOW);  // pcs
  
  Serial.begin(9600);
  
  pidA.SetMode(AUTOMATIC);
  Setpoint = 55;

  Tw = 1000;
  Sch.schedule( Tw, 0, ShowData);
  
  Tpid  = 200;
  pidA.SetSampleTime( Tpid );
  workPID = 0;
  Sch.schedule( Tpid, 0, GetTemp );  // while not working PID ...
  
  lcd1();
}
void loop() {
  Sch.cycle();
  if (Serial.available() > 0) DoCmd();
}
