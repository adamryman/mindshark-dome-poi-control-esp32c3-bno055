#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

const int button_1_pin = 3;
const int button_2_pin = 9;

const int DATAGRAM_REPEATS = 10;

int old_button_1_state = 0;
int old_button_2_state = 0;
int button_1_2_hold_counter = 0;

IPAddress server(192,168,12,230);
WiFiUDP conn;
uint8_t id;
int16_t old_w = 0;
int16_t old_x = 0;
int16_t old_y = 0;
int16_t old_z = 0;
uint8_t action_flag = 0;
int action_flag_repeats = 0;

void setup(void)
{
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
  Wire.setPins(1, 10);
  Serial.begin(115200);
  delay(10);
  Serial.println("Starting up...");

  Serial.println("Connecting to wifi");
  WiFi.begin("source 2.4", "practicalflightwithalmost");
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to wifi");
  Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
  conn.begin(5004);
  Serial.println("UDP client created");
  id = WiFi.localIP()[3];

  pinMode(button_1_pin, INPUT_PULLUP);
  pinMode(button_2_pin, INPUT_PULLUP);

  Serial.println("Configuring BNO055");
  bno.begin(OPERATION_MODE_NDOF);
  bno.setExtCrystalUse(true);
  Serial.println("Setup completed");
}

void loop(void)
{

  int button_1_state = digitalRead(button_1_pin);
  int button_2_state = digitalRead(button_2_pin);
  // Actions:
  // 1: Button 1 clicked down (i.e. hold and release is disregarded)
  // 2: Button 2 clicked down
  // 4: press and hold buttons 1 and 2 for a bit (triggers calibration)
  // these set the action flag
  // once the action flag is set, it can't have any other value for some time
  // this is because we can't guarantee any particular UDP datagram gets received
  // so we send the same action flag in a series of datagrams in the hope that it gets picked up
  // action flag unset -- we're free to try to assign to it
  if (action_flag == 0) {
    // first we just check if any button is being pressed; i.e. the current state is HIGH and the old state is LOW
    if (button_1_state != old_button_1_state && button_1_state == LOW) {
      action_flag = 1;
    }
    if (button_2_state != old_button_2_state && button_2_state == LOW) {
      action_flag = 2;
    }
    // if in fact button 1 and 2 are being held then we override the action flag
    if (button_1_state == old_button_1_state && button_2_state == old_button_2_state && button_1_state == LOW && button_2_state == LOW) {
      button_1_2_hold_counter++;
    }
    if (button_1_2_hold_counter > 500) {
      action_flag = 4;
      button_1_2_hold_counter = 0;
    }
  }

  imu::Quaternion quat = bno.getQuat();
  int16_t w = quat.w() * 16384;
  int16_t x = quat.x() * 16384;
  int16_t y = quat.y() * 16384;
  int16_t z = quat.z() * 16384;
  if(old_w != w || old_x != x || old_y != y || old_z != z) {
    conn.beginPacket(server, 5005);
    conn.write(reinterpret_cast<uint8_t*>(&id), sizeof(id));
    auto time = millis();
    conn.write(reinterpret_cast<const uint8_t*>(&time), sizeof(time));
    conn.write(reinterpret_cast<uint8_t*>(&w), sizeof(w));
    conn.write(reinterpret_cast<uint8_t*>(&x), sizeof(x));
    conn.write(reinterpret_cast<uint8_t*>(&y), sizeof(y));
    conn.write(reinterpret_cast<uint8_t*>(&z), sizeof(z));
    conn.write(reinterpret_cast<uint8_t*>(&action_flag), sizeof(action_flag));
    conn.endPacket();
    if (action_flag != 0 && action_flag_repeats > DATAGRAM_REPEATS) {
      action_flag = 0;
      action_flag_repeats = 0;
    } else if (action_flag != 0) {
      action_flag_repeats++;
    }
    old_w = w;
    old_x = x;
    old_y = y;
    old_z = z;

  }


  old_button_1_state = button_1_state;
  old_button_2_state = button_2_state;
}