#include "config.h"
// serial and print macros
#include "debug.h"
// wifi password
#include "secrets.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_BNO055.h>
#include <stdint.h>
#include <deque>
#include <algorithm>
#include <cmath>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

unsigned long last_time = 0;
WiFiUDP conn;
uint8_t id;
float old_w = 0;
float old_x = 0;
float old_y = 0;
float old_z = 0;
uint8_t action_flag = 0;

const uint8_t DEVICE_TYPE = 2;
const int BUFFER_SIZE = 100;
std::deque<float> distanceBuffer;  // Using deque of floats to store distances
int lastTimestamp = 0;

void setup(void) {
  SETUP_SERIAL();
  delay(10);
  PRINTF("Starting up...\n");
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
  Wire.setPins(1, 10);
  
  PRINTF("Connecting to wifi\n");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    PRINTF(".");
  }
  PRINTF("\nConnected to wifi\n");
  PRINTF("IP address: %s\n", WiFi.localIP().toString().c_str());
  conn.begin(5004);
  PRINTF("UDP client created\n");
  id = WiFi.localIP()[3];

  PRINTF("Configuring BNO055\n");
  bno.begin(OPERATION_MODE_NDOF);
  bno.setExtCrystalUse(true);
  PRINTF("Setup completed\n");

  last_time = millis();
}

float CalculateWeightedDistance(float old_w, float old_x, float old_y, float old_z,
                                float w, float x, float y, float z, int timeDiff) {
  // Calculate dot product
  float dot_product = old_w * w + old_x * x + old_y * y + old_z * z;
  dot_product = fminf(fmaxf(dot_product, -1.0f), 1.0f); // Clamp to valid range
  float angle = 2 * acos(dot_product); // Calculate angular distance in radians

  // Multiply the angle by time difference
  float weighted_angle = angle * static_cast<float>(timeDiff);

  return weighted_angle; // Return the weighted angle as a float
}

void StoreOrientation(int timestamp, float w, float x, float y, float z) {
  if (distanceBuffer.size() == BUFFER_SIZE) {
    distanceBuffer.pop_front(); // Remove the oldest entry if buffer is full
  }
  
  int timeDiff = timestamp - lastTimestamp;
  float weightedDistance = CalculateWeightedDistance(old_w, old_x, old_y, old_z, w, x, y, z, timeDiff);

  // Store the weighted distance in the buffer
  distanceBuffer.push_back(weightedDistance);

  // Update last orientation and timestamp
  old_w = w;
  old_x = x;
  old_y = y;
  old_z = z;
  lastTimestamp = timestamp;
}

float CalculateAverageDistance() {
  if (distanceBuffer.empty()) {
    return 0;
  }

  float totalDistance = 0.0f;
  for (const auto &weightedDistance : distanceBuffer) {
    totalDistance += weightedDistance;
  }

  float averageDistance = totalDistance / distanceBuffer.size();
  PRINTF("Average Distance: %f\n", averageDistance);
  return averageDistance;
}

void loop(void) {
  imu::Quaternion quat = bno.getQuat();
  float w = quat.w();
  float x = quat.x();
  float y = quat.y();
  float z = quat.z();

  if (w == 0 && x == 0 && y == 0 && z == 0) {
    PRINTF("Invalid quaternion detected, skipping...\n");
    delay(100);
    return; // Skip this loop iteration if the quaternion is invalid
  }
  if (old_w == 0 && old_x == 0 && old_y == 0 && old_z == 0) {
    PRINTF("Storing first valid quaternion\n");
    old_w = w;
    old_x = x;
    old_y = y;
    old_z = z;
    return; // Skip this loop iteration if the quaternion is invalid
  }

  if (old_w == w && old_x == x && old_y == y && old_z == z) {
    delay(10);
    return;
  }

  PRINTF("Quaternion: w = %f, x = %f, y = %f, z = %f\n", w, x, y, z);

  if (old_w != w || old_x != x || old_y != y || old_z != z) {
    int current_time = millis();

    StoreOrientation(current_time, w, x, y, z);
    float average_distance = CalculateAverageDistance();

    // Scale quaternion components for network transmission
    int16_t scaled_w = static_cast<int16_t>(w * 16384);
    int16_t scaled_x = static_cast<int16_t>(x * 16384);
    int16_t scaled_y = static_cast<int16_t>(y * 16384);
    int16_t scaled_z = static_cast<int16_t>(z * 16384);

    // Scale the average distance before sending
    uint16_t scaled_average_distance = static_cast<uint16_t>((average_distance / M_PI) * 65535.0f);

    conn.beginPacket(dome_ip, 5005);
    conn.write(reinterpret_cast<uint8_t*>(&id), sizeof(id));
    conn.write(reinterpret_cast<const uint8_t*>(&current_time), sizeof(current_time));
    conn.write(&DEVICE_TYPE, sizeof(DEVICE_TYPE));
    conn.write(reinterpret_cast<uint8_t*>(&scaled_w), sizeof(scaled_w));
    conn.write(reinterpret_cast<uint8_t*>(&scaled_x), sizeof(scaled_x));
    conn.write(reinterpret_cast<uint8_t*>(&scaled_y), sizeof(scaled_y));
    conn.write(reinterpret_cast<uint8_t*>(&scaled_z), sizeof(scaled_z));
    conn.write(reinterpret_cast<uint8_t*>(&action_flag), sizeof(action_flag));

    conn.write(reinterpret_cast<uint8_t*>(&scaled_average_distance), sizeof(scaled_average_distance));
    
    conn.endPacket();

    last_time = current_time;
  }
}
