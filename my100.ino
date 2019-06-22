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
#define PWMA 9
#define PWMB 6

#define BIG_STEP .05f
#define SMALL_STEP .02f

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

const float axes_rest[] = {
  .07f, 1, 1, 1, 0, 1, 1
};

const byte rest_seq[] = {
  1, 0, 2, 3, 4, 5, 6
};

/*
typedef struct {
  float pos;
  const float min, max;
} axis_t;
*/

float axes[] = {
  axes_rest[0],
  axes_rest[1],
  axes_rest[2],
  axes_rest[3],
  axes_rest[4],
  axes_rest[5],
  axes_rest[6],
};

const byte axes_size = 7;

byte axis_init = 0;

ServoDriver servos = ServoDriver();

#define SERVOMIN 104
#define SERVOMAX 482
#define TRAVEL (SERVOMAX - SERVOMIN) / 2.f

void (* resetFunc) (void) = 0;

void setup() {
  servos.begin();
  servos.setPWMFreq(50);  // servos run at 50 Hz

  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // reset just in case, I'm not taking much chances with this arduino crap...
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

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
    turn = 0xff - turn;

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

/* Stick to Float */
float stf(byte x) {
  if (x >= 128) {
    return (x - 128) / 127.f;
  } else {
    return -((127 - x) / 127.f);
  }
}

/* Analog to Float */
float atf(byte x) {
  return x / 255.f;
}

void drive_axis(byte axis_i) {
  float axis = constrain(axes[axis_i], -1.f, 1.f);
  axes[axis_i] = axis;
  word pwm = SERVOMIN + TRAVEL + axis * TRAVEL + .5f;

  switch (axis_i) {
    case 0:
      servos.setPWM(0, 0, pwm);
      break;
    case 1:
      // special case for the dual-servo arm
      servos.setPWM(1, 0, pwm);
      // negation to the other servo
      servos.setPWM(2, 0, SERVOMIN + TRAVEL + -axis * TRAVEL + .5f);
      break;
    default:
      servos.setPWM(axis_i + 1, 0, pwm);
      break;
  }
}

void drive_grabber() {
  // rotate
  float f = stf(ps2x.Analog(PSS_LX));
  if (f) {
    axes[0] += -f * BIG_STEP;
    drive_axis(0);
  }

  // pitch first segment
  f = stf(ps2x.Analog(PSS_LY));
  if (f) {
    axes[1] += f * BIG_STEP;
    drive_axis(1);
  }

  // pitch second segment
  if (ps2x.Button(PSB_PAD_UP)) {
    axes[2] += SMALL_STEP;
    drive_axis(2);
  } else if (ps2x.Button(PSB_PAD_DOWN)) {
    axes[2] -= SMALL_STEP;
    drive_axis(2);
  }

  // roll second segment
  if (ps2x.Button(PSB_PAD_LEFT)) {
    axes[3] += SMALL_STEP;
    drive_axis(3);
  } else if (ps2x.Button(PSB_PAD_RIGHT)) {
    axes[3] -= SMALL_STEP;
    drive_axis(3);
  }

  // pitch grabber
  if (ps2x.Button(PSB_L2)) {
    axes[4] += SMALL_STEP;
    drive_axis(4);
  } else if (ps2x.Button(PSB_R2)) {
    axes[4] -= SMALL_STEP;
    drive_axis(4);
  }

  // roll grabber
  byte x = ps2x.Analog(PSAB_SQUARE);
  if (x) {
    axes[5] += atf(x) * BIG_STEP;
    drive_axis(5);
  } else {
    x = ps2x.Analog(PSAB_CIRCLE);
    if (x) {
      axes[5] += -atf(x) * BIG_STEP;
      drive_axis(5);
    }
  }

  // open/close grabber
  x = ps2x.Analog(PSAB_TRIANGLE);
  if (x) {
    axes[6] += atf(x) * BIG_STEP;
    drive_axis(6);
  } else {
    x = ps2x.Analog(PSAB_CROSS);
    if (x) {
      axes[6] += -atf(x) * BIG_STEP;
      drive_axis(6);
    }
  }
}

void loop() {
  /* You must Read Gamepad to get new values and set vibration values
     ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
     if you don't enable the rumble, use ps2x.read_gamepad(); with no values
     You should call this at least once a second
   */
  byte axis;

  // no controller found
  if (ps2.error == 1)
    resetFunc();

  // Guitar Hero controller
  if (ps2.type == 2)
    resetFunc();

  bool ps2status = ps2x.read_gamepad(false, ps2.vibrate);

  if (ps2status != 1) {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    Serial.print("read_gamepad error ");
    Serial.println(ps2status, DEC);

  } else if (axis_init < axes_size) {
    // stop motors
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);

    Serial.print("Initializing axis ");
    Serial.print(axis_init, DEC);
    Serial.print("...");

    axis = rest_seq[axis_init];
    axes[axis] = axes_rest[axis];
    drive_axis(axis);
    ++axis_init;
    Serial.println("ok");

  } else if (ps2x.ButtonPressed(PSB_START)) {
    axis_init = 0;

  } else {
    drive_tracks();
    drive_grabber();
  }

  delay(30);
}
