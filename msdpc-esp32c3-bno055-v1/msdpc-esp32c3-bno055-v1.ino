#include "config.h"
// serial and print macros
#include "debug.h"
// wifi password
#include "secrets.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_BNO055.h>
#include <stdint.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

const int DATAGRAM_REPEATS = 10;

// Window time limits in milliseconds
const unsigned long SHORT_WINDOW = 10000;  // 10 seconds
const unsigned long MID_WINDOW = 30000;    // 30 seconds
const unsigned long LONG_WINDOW = 60000;   // 60 seconds

// Variables for cumulative sums and time using uint32_t
uint32_t cumulative_distance_short = 0;
uint32_t cumulative_time_short = 0;

uint32_t cumulative_distance_mid = 0;
uint32_t cumulative_time_mid = 0;

uint32_t cumulative_distance_long = 0;
uint32_t cumulative_time_long = 0;

unsigned long last_time = 0;  // To track the last update time

WiFiUDP conn;
uint8_t id;
int16_t old_w = 0;
int16_t old_x = 0;
int16_t old_y = 0;
int16_t old_z = 0;
uint8_t action_flag = 0;

const uint8_t DEVICE_TYPE = 2;  // Define device type 2 as poi

const uint32_t MAX_ANGLE_INT = 36000;  // Scaling factor for angle (e.g., 1 degree = 100 units)

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

  PRINTLN("Configuring BNO055");
  bno.begin(OPERATION_MODE_NDOF);
  bno.setExtCrystalUse(true);
  PRINTLN("Setup completed");

  last_time = millis();  // Initialize the last time
}

void loop(void)
{
  imu::Quaternion quat = bno.getQuat();
  int16_t w = quat.w() * 16384;
  int16_t x = quat.x() * 16384;
  int16_t y = quat.y() * 16384;
  int16_t z = quat.z() * 16384;

  if(old_w != w || old_x != x || old_y != y || old_z != z) {
    unsigned long current_time = millis();
    unsigned long time_diff = current_time - last_time;

    // Calculate the angular distance using the quaternion difference
    int32_t dot_product = old_w * w + old_x * x + old_y * y + old_z * z;
    int32_t angle = calculateScaledAngle(dot_product);  // Compute the scaled angular distance

    // Update cumulative distances and times for each window
    updateWindow(cumulative_distance_short, cumulative_time_short, angle, time_diff, SHORT_WINDOW);
    updateWindow(cumulative_distance_mid, cumulative_time_mid, angle, time_diff, MID_WINDOW);
    updateWindow(cumulative_distance_long, cumulative_time_long, angle, time_diff, LONG_WINDOW);

    // Scale the accumulated distances to 16-bit
    uint16_t scaled_distance_short = scaleToUint16(cumulative_distance_short, cumulative_time_short, SHORT_WINDOW);
    uint16_t scaled_distance_mid = scaleToUint16(cumulative_distance_mid, cumulative_time_mid, MID_WINDOW);
    uint16_t scaled_distance_long = scaleToUint16(cumulative_distance_long, cumulative_time_long, LONG_WINDOW);

    conn.beginPacket(dome_ip, 5005);
    conn.write(reinterpret_cast<uint8_t*>(&id), sizeof(id));
    conn.write(reinterpret_cast<const uint8_t*>(&current_time), sizeof(current_time));
    conn.write(&DEVICE_TYPE, sizeof(DEVICE_TYPE));  // Send the device type
    conn.write(reinterpret_cast<uint8_t*>(&w), sizeof(w));
    conn.write(reinterpret_cast<uint8_t*>(&x), sizeof(x));
    conn.write(reinterpret_cast<uint8_t*>(&y), sizeof(y));
    conn.write(reinterpret_cast<uint8_t*>(&z), sizeof(z));
    conn.write(reinterpret_cast<uint8_t*>(&action_flag), sizeof(action_flag));

    // Send the scaled average angular distances for different windows
    conn.write(reinterpret_cast<uint8_t*>(&scaled_distance_short), sizeof(scaled_distance_short));
    conn.write(reinterpret_cast<uint8_t*>(&scaled_distance_mid), sizeof(scaled_distance_mid));
    conn.write(reinterpret_cast<uint8_t*>(&scaled_distance_long), sizeof(scaled_distance_long));
    
    conn.endPacket();

    old_w = w;
    old_x = x;
    old_y = y;
    old_z = z;

    last_time = current_time;  // Update the last time variable
  }
}

int32_t calculateScaledAngle(int32_t dot_product) {
  // Ensure dot_product is within valid range
  dot_product = fminf(fmaxf(dot_product, -16384 * 16384), 16384 * 16384);
  // Scale the angle as an integer (e.g., 1 degree = 100 units)
  // Here we assume dot_product is between -1 and 1 after scaling, we can calculate angle in "scaled units"
  return (int32_t)(acos((float)dot_product / (16384.0f * 16384.0f)) * (MAX_ANGLE_INT / 6.28319f));
}

void updateWindow(uint32_t &cumulative_distance, uint32_t &cumulative_time, uint32_t angle, unsigned long time_diff, unsigned long window_limit) {
  cumulative_distance += angle;
  cumulative_time += time_diff;

  // If cumulative time exceeds window limit, scale down
  if (cumulative_time > window_limit) {
    uint32_t scale_factor = (cumulative_time / window_limit);
    cumulative_distance /= scale_factor;
    cumulative_time = window_limit;
  }
}

uint16_t scaleToUint16(uint32_t cumulative_distance, uint32_t cumulative_time, unsigned long window_limit) {
  if (cumulative_time == 0) return 0;
  
  // Scale the cumulative distance to a 16-bit range
  uint32_t average_distance = cumulative_distance / (cumulative_time / 1000.0);  // Get average distance per second
  return (uint16_t)((average_distance * 65535) / MAX_ANGLE_INT);
}
