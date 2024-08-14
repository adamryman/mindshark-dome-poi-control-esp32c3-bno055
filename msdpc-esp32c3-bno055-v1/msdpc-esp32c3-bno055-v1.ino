#include "config.h"
// serial and print macros
#include "debug.h"
// wifi password
#include "secrets.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_BNO055.h>
#include <deque>  // Include the deque library for buffer

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

const int button_1_pin = 3;
const int button_2_pin = 9;

const int DATAGRAM_REPEATS = 10;
const int BUFFER_SIZE = 100;  // Size of the buffer for angular velocities

int old_button_1_state = 0;
int old_button_2_state = 0;
int button_1_2_hold_counter = 0;

WiFiUDP conn;
uint8_t id;
int16_t old_w = 0;
int16_t old_x = 0;
int16_t old_y = 0;
int16_t old_z = 0;
uint8_t action_flag = 0;
int action_flag_repeats = 0;

std::deque<float> angular_vel_buffer; // Buffer for storing angular velocities

uint16_t avg_angular_vel_short = 0;  // Average over short period
uint16_t avg_angular_vel_mid = 0;    // Average over medium period
uint16_t avg_angular_vel_long = 0;   // Average over long period

unsigned long last_time = 0;  // To store the time of the last quaternion update

void setup(void)
{
  SETUP_SERIAL();
  delay(10);
  PRINTLN("Starting up...");
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
  Wire.setPins(1, 10);
  

  PRINTLN("Connecting to wifi");
  // Populated in secrets.h
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    PRINT(".");
  }
  PRINTLN();
  PRINTLN("Connected to wifi");
  PRINTF("IP address: %s\n", WiFi.localIP().toString().c_str());
  conn.begin(5004);
  PRINTLN("UDP client created");
  id = WiFi.localIP()[3];

  pinMode(button_1_pin, INPUT_PULLUP);
  pinMode(button_2_pin, INPUT_PULLUP);

  PRINTF("Configuring BNO055");
  bno.begin(OPERATION_MODE_NDOF);
  bno.setExtCrystalUse(true);
  PRINTLN("Setup completed");
}

void loop(void)
{
  int button_1_state = digitalRead(button_1_pin);
  int button_2_state = digitalRead(button_2_pin);
  if (action_flag == 0) {
    if (button_1_state != old_button_1_state && button_1_state == LOW) {
      action_flag = 1;
    }
    if (button_2_state != old_button_2_state && button_2_state == LOW) {
      action_flag = 2;
    }
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
    unsigned long current_time = millis();
    float time_diff = (current_time - last_time) / 1000.0;  // Time difference in seconds

    // Calculate the angular velocity using the quaternion difference
    float angle = 2 * acos(old_w * quat.w() + old_x * quat.x() + old_y * quat.y() + old_z * quat.z());
    float angular_velocity = angle / time_diff;

    // Store the angular velocity in the buffer
    if (angular_vel_buffer.size() >= BUFFER_SIZE) {
      angular_vel_buffer.pop_front();  // Remove oldest if buffer is full
    }
    angular_vel_buffer.push_back(angular_velocity);

    // Calculate averages over different periods
    avg_angular_vel_short = calculateAverage(angular_vel_buffer, 10);
    avg_angular_vel_mid = calculateAverage(angular_vel_buffer, 50);
    avg_angular_vel_long = calculateAverage(angular_vel_buffer, 100);

    conn.beginPacket(dome_ip, 5005);
    conn.write(reinterpret_cast<uint8_t*>(&id), sizeof(id));
    conn.write(reinterpret_cast<const uint8_t*>(&current_time), sizeof(current_time));
    conn.write(reinterpret_cast<uint8_t*>(&w), sizeof(w));
    conn.write(reinterpret_cast<uint8_t*>(&x), sizeof(x));
    conn.write(reinterpret_cast<uint8_t*>(&y), sizeof(y));
    conn.write(reinterpret_cast<uint8_t*>(&z), sizeof(z));
    conn.write(reinterpret_cast<uint8_t*>(&action_flag), sizeof(action_flag));

    // Send the averages
    conn.write(reinterpret_cast<uint8_t*>(&avg_angular_vel_short), sizeof(avg_angular_vel_short));
    conn.write(reinterpret_cast<uint8_t*>(&avg_angular_vel_mid), sizeof(avg_angular_vel_mid));
    conn.write(reinterpret_cast<uint8_t*>(&avg_angular_vel_long), sizeof(avg_angular_vel_long));
    
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

    last_time = current_time;  // Update the last time variable
  }

  old_button_1_state = button_1_state;
  old_button_2_state = button_2_state;
}

uint16_t calculateAverage(const std::deque<float>& buffer, int period) {
  if (buffer.size() < period) period = buffer.size();  // Adjust period if buffer is not full
  float sum = 0;
  for (int i = buffer.size() - period; i < buffer.size(); i++) {
    sum += buffer[i];
  }
  return static_cast<uint16_t>(sum / period);
}
