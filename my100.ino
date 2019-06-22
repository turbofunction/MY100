#include "PS2X_lib.h"
#include "ServoDriver.h"

// PS2 controller receiver (module) pins
#define PS2_DAT 13
#define PS2_CMD 11
#define PS2_SEL 10
#define PS2_CLK 12

// PS2 controller confs
#define pressures true
#define rumble true

// motor/servo shield motor pins
#define DIRA 8
#define DIRB 7
#define PWMA  9
#define PWMB  6

PS2X ps2x;

typedef struct {
  int error;
  byte type;
  byte vibrate;
} ps2_t;

ps2_t volatile ps2 = { 0, 0, 0 };

typedef struct {
  const byte i;
  const float pos;
} axis_rest_t;

const axis_rest_t axes_rest[] = {
  { 0, .05f }
};

const byte axes_size = 1;

typedef struct {
  float pos;
} axis_t;

axis_t axes[] = {
  { axes_rest[0].pos }
};

bool grabber_initialized = false;

ServoDriver servos = ServoDriver();

#define SERVOMIN 104
#define SERVOMAX 482
#define TRAVEL (SERVOMAX - SERVOMIN) / 2.f

void (* resetFunc) (void) = 0;

void rest_axes() {
  byte axis;
  for (byte i = 0; i < axes_size; ++i) {
    axis = axes_rest[i].i;
    axes[axis].pos = axes_rest[i].pos;
    drive_servos(axis);
    delay(500);
  }
}

void setup() {
  servos.begin();
  servos.setPWMFreq(50);  // servos run at 50 Hz

  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  Serial.begin(115200);

  // delay for PS2 module to startup
  delay(500);

  ps2.error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  if (ps2.error == 0)
    Serial.println("Found Controller, configured successfully.");
  else if (ps2.error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips.");
  else if (ps2.error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips.");
  else if (ps2.error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it.");
  else
    Serial.println("Unknown controller error.");

  ps2.type = ps2x.readType();
  switch (ps2.type) {
    case 0:
      Serial.println("Unknown Controller type found.");
      break;
    case 1:
      Serial.println("DualShock Controller found.");
      break;
    case 2:
      Serial.println("GuitarHero Controller found.");
      break;
    case 3:
      Serial.println("Wireless Sony DualShock Controller found.");
      break;
   }
}

void drive_tracks() {
  byte fwd = ps2x.Analog(PSS_RY);
  byte turn = ps2x.Analog(PSS_RX);
  int m1 = fwd, m2 = fwd;

  if (fwd > 140)
    turn = 255 - turn;

  if (turn >= 128) {
    turn -= 128;
    m1 -= turn;
    m2 += turn;
  } else {
    turn = 127 - turn;
    m1 += turn;
    m2 -= turn;
  }

  m1 = constrain(m1, 0, 0xff);
  m2 = constrain(m2, 0, 0xff);

  /*
  Serial.print("fwd,turn,t,m1,m2: ");
  Serial.print(fwd, DEC);
  Serial.print(",");
  Serial.print(turn, DEC);
  Serial.print(",");
  Serial.print(m1, DEC);
  Serial.print(",");
  Serial.println(m2, DEC);
  */

  if (m1 >= 128) {
    digitalWrite(DIRA, true);
    m1 -= 128;
  } else {
    digitalWrite(DIRA, false);
    m1 = 127 - m1;
  }

  if (m2 >= 128) {
    digitalWrite(DIRB, true);
    m2 -= 128;
  } else {
    digitalWrite(DIRB, false);
    m2 = 127 - m2;
  }

  analogWrite(PWMA, m1 * 2);
  analogWrite(PWMB, m2 * 2);
}

float read_axis(byte x) {
  if (x >= 128) {
    return (x - 128) / 127.f;
  } else {
    return -((127 - x) / 127.f);
  }
}

void drive_servos(byte axis) {
  word pwm = SERVOMIN + TRAVEL + axes[axis].pos * TRAVEL + .5f;
  // Serial.print(", ");
  // Serial.println(pwm, DEC);

  switch (axis) {
    case 0:
      servos.setPWM(0, 0, pwm);
      break;
    default:
      break;
  }
}

void drive_grabber() {
  float turn = -read_axis(ps2x.Analog(PSS_LX));

  // Serial.print("turn, pos, pwm: ");
  // Serial.print(turn, DEC);

  axes[0].pos = constrain(axes[0].pos + (turn * .05f), -1., 1.);
  // Serial.print(", ");
  // Serial.print(axes[0].pos, DEC);

  drive_servos(0);

  float lift = read_axis(ps2x.Analog(PSS_LY));

  /*
  for (byte servo = 0; servo <= 15; ++servo) {
    servos.setPWM(servo, 0, SERVOMIN + ((SERVOMAX - SERVOMIN) / 2));
  }
  */
}

void loop() {
  /* You must Read Gamepad to get new values and set vibration values
     ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
     if you don't enable the rumble, use ps2x.read_gamepad(); with no values
     You should call this at least once a second
   */

  // no controller found
  if (ps2.error == 1)
    resetFunc();

  // Guitar Hero controller
  if (ps2.type == 2)
    resetFunc();

  if (!grabber_initialized) {
    rest_axes();
    grabber_initialized = true;
  }

  ps2x.read_gamepad(false, ps2.vibrate); //read controller and set large motor to spin at 'vibrate' speed

  if (ps2x.ButtonPressed(PSB_START)) {
    rest_axes();
  } else {
    drive_tracks();
    drive_grabber();
  }

  /*
  if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
    Serial.println("Start is being held");
  if(ps2x.Button(PSB_SELECT))
    Serial.println("Select is being held");

  if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
    Serial.print("Up held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
  }
  if(ps2x.Button(PSB_PAD_RIGHT)){
    Serial.print("Right held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
  }
  if(ps2x.Button(PSB_PAD_LEFT)){
    Serial.print("LEFT held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
  }
  if(ps2x.Button(PSB_PAD_DOWN)){
    Serial.print("DOWN held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
  }

  ps2.vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button

  if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
    if(ps2x.Button(PSB_L3))
      Serial.println("L3 pressed");
    if(ps2x.Button(PSB_R3))
      Serial.println("R3 pressed");
    if(ps2x.Button(PSB_L2))
      Serial.println("L2 pressed");
    if(ps2x.Button(PSB_R2))
      Serial.println("R2 pressed");
    if(ps2x.Button(PSB_TRIANGLE))
      Serial.println("Triangle pressed");
  }

  if(ps2x.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
    Serial.println("Circle just pressed");
  if(ps2x.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
    Serial.println("X just changed");
  if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
    Serial.println("Square just released");

  if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
    Serial.print("Stick Values:");
    Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_LX), DEC);
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RY), DEC);
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_RX), DEC);
  }
  */

  delay(50);
}
