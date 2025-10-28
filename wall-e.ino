const uint8_t CMD_MAX = 32;
char cmdBuf[CMD_MAX];
uint8_t idx = 0;
int led = 0;

class DCMotor {
  int spd = 255, out1 = LOW, out2 = LOW, pin1, pin2, pin3;

public:
  void Pinout(int in1, int in2, int in3) {
    pin1 = in1;
    pin2 = in2;
    pin3 = in3;
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pin3, OUTPUT);
  }
  void Speed(int in1) {
    spd = 255;
    // spd = analogRead(in1) >> 2;
  }
  void Forward() {
    out1 = HIGH;
    out2 = LOW;
  }
  void Backward() {
    out1 = LOW;
    out2 = HIGH;
  }
  void Stop() {
    out1 = LOW;
    out2 = LOW;
  }
  void Read() {
    digitalWrite(pin1, out1);
    digitalWrite(pin2, out2);
    analogWrite(pin3, spd);
    // Serial.println(out1);
  }
};

class LedsControl {
  int led_stateR = 0, led_stateG = 0, led_stateB = 0, ledR, ledG, ledB;

public:
  void Pinout(int pin1, int pin2, int pin3) {
    ledR = pin1;
    ledG = pin2;
    ledB = pin3;
    pinMode(ledR, OUTPUT);
    pinMode(ledG, OUTPUT);
    pinMode(ledB, OUTPUT);
  }
  void Red(int led_state) {
    led_stateR = led_state;
  }
  void Green(int led_state) {
    led_stateG = led_state;
  }
  void Blue(int led_state) {
    led_stateB = led_state;
  }
  void Read() {
    digitalWrite(ledR, led_stateR);
    digitalWrite(ledG, led_stateG);
    digitalWrite(ledB, led_stateB);
  }
};

DCMotor Motor1, Motor2;
LedsControl Led;

void setup() {
  Motor1.Pinout(8, 7, 5);
  Motor2.Pinout(9, 4, 6);

  Led.Pinnout(15, 16, 17);

  Serial.begin(9600);
  Serial3.begin(9600);
  Serial.println(F("\nBluetooth module ready. Type in Serial Monitor to send to phone.\n"));

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
}

void loop() {
  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);

  // Motor1.Speed(A8);
  // Motor2.Speed(A8);

  while (Serial3.available()) {
    char c = Serial3.read();
    Serial.println(c);

    if (c == '\r' || c == '\n') continue;
    if (!isPrintable(c)) continue;

    if (c == '0') {
      cmdBuf[idx] = '\0';
      Motor1.Stop();
      Motor2.Stop();

      idx = 0;

    } else {
      if (idx < CMD_MAX - 1) {
        cmdBuf[idx++] = c;
        if (idx > 0) {
          switch (cmdBuf[0]) {
            case 'F':
              Serial.println(F("Frente pressionado"));
              Motor1.Forward();
              Motor2.Forward();
              break;
            case 'R':
              Serial.println(F("Direita pressionado"));
              Motor1.Forward();
              Motor2.Backward();
              break;
            case 'L':
              Serial.println(F("Esquerda pressionado"));
              Motor1.Backward();
              Motor2.Forward();
              break;
            case 'B':
              Serial.println(F("Tras pressionado"));
              Motor1.Backward();
              Motor2.Backward();
              break;
            case 'X':
              Serial.println(F("X pressionado"));
              led ++;
              if (led > 6) {
                led = 0;
              } 
              break;
            default:
              Serial.println(F("nada pressionado"));
              break;
          }
        }
      } else {
        idx = 0;
      }
    }
  }

  switch (led) {
    case 1:
      Led.Red(1);
      Led.Green(0);
      Led.Blue(0);
      break;
    case 2:
      Led.Red(0);
      Led.Green(1);
      Led.Blue(0);
      break;
    case 3:
      Led.Red(0);
      Led.Green(0);
      Led.Blue(1);
      break;
    case 4:
      Led.Red(0);
      Led.Green(0);
      Led.Blue(0);
      break;
    case 5:
      Led.Red(0);
      Led.Green(0);
      Led.Blue(0);
      break;
    case 6:
      Led.Red(0);
      Led.Green(0);
      Led.Blue(0);
      break;
    default:
      Led.Red(0);
      Led.Green(0);
      Led.Blue(0);
      break;
  }

  Motor1.Read();
  Motor2.Read();
}

// 1 red
// 2 green
// 3 blue
// 4 
// 5
// 6

// if (Serial3.available()) {
//   c = Serial3.read();
//   Serial.println(c);
// }
// switch (c) {
//   case FOWARD: Serial.println(F("Frente pressionado")); break;
//   case RIGHT: Serial.println(F("Direita pressionado")); break;
//   case LEFT: Serial.println(F("Esquerda pressionado")); break;
//   case BACKWARD: Serial.println(F("Tras pressionado")); break;
//   case TRIANGLE: Serial.println(F("Triangulo pressionado")); break;
//   case CIRCLE: Serial.println(F("Circulo pressionado")); break;
//   case SQUARE: Serial.println(F("Quadrado pressionado")); break;
//   case X: Serial.println(F("X pressionado")); break;
//   default: Serial.println(F("nada pressionado")); break;
// }
