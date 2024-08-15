#include "config.h"
// serial and print macros
#include "debug.h"
// wifi password
#include "secrets.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

const int DATAGRAM_REPEATS = 10;

// Window time limits in milliseconds
const unsigned long SHORT_WINDOW = 10000;  // 10 seconds
const unsigned long MID_WINDOW = 30000;    // 30 seconds
const unsigned long LONG_WINDOW = 60000;   // 60 seconds

// Variables for cumulative sums and time
float cumulative_distance_short = 0.0f;
unsigned long cumulative_time_short = 0;

float cumulative_distance_mid = 0.0f;
unsigned long cumulative_time_mid = 0;

float cumulative_distance_long = 0.0f;
unsigned long cumulative_time_long = 0;

unsigned long last_time = 0;  // To track the last update time

WiFiUDP conn;
uint8_t id;
int16_t old_w = 0;
int16_t old_x = 0;
int16_t old_y = 0;
int16_t old_z = 0;
uint8_t action_flag = 0;

const uint8_t DEVICE_TYPE = 2;  // Define device type 2 as poi

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
    float dot_product = old_w * w + old_x * x + old_y * y + old_z * z;
    dot_product = fminf(fmaxf(dot_product, -1.0f), 1.0f);  // Ensure dot product is within valid range
    float angle = 2 * acos(dot_product);  // Compute the angular distance (radians)


    // Update cumulative distances and times for each window
    updateWindow(cumulative_distance_short, cumulative_time_short, angle, time_diff, SHORT_WINDOW);
    updateWindow(cumulative_distance_mid, cumulative_time_mid, angle, time_diff, MID_WINDOW);
    updateWindow(cumulative_distance_long, cumulative_time_long, angle, time_diff, LONG_WINDOW);

    // Calculate average angular distances for each window
    float average_distance_short = calculateAverageDistance(cumulative_distance_short, cumulative_time_short);
    float average_distance_mid = calculateAverageDistance(cumulative_distance_mid, cumulative_time_mid);
    float average_distance_long = calculateAverageDistance(cumulative_distance_long, cumulative_time_long);

    conn.beginPacket(dome_ip, 5005);
    conn.write(reinterpret_cast<uint8_t*>(&id), sizeof(id));
    conn.write(reinterpret_cast<const uint8_t*>(&current_time), sizeof(current_time));
    conn.write(&DEVICE_TYPE, sizeof(DEVICE_TYPE));  // Send the device type
    conn.write(reinterpret_cast<uint8_t*>(&w), sizeof(w));
    conn.write(reinterpret_cast<uint8_t*>(&x), sizeof(x));
    conn.write(reinterpret_cast<uint8_t*>(&y), sizeof(y));
    conn.write(reinterpret_cast<uint8_t*>(&z), sizeof(z));
    conn.write(reinterpret_cast<uint8_t*>(&action_flag), sizeof(action_flag));

   
    /*
    uint16_t val1 = 2;
    uint16_t val2 = 32767;
    uint16_t val3 = 52767;
    
    
    uint16_t val1 = htons(2);
    uint16_t val2 = htons(32767);
    uint16_t val3 = htons(52767);
    
    conn.write(reinterpret_cast<uint8_t*>(&val1), sizeof(val1));
    conn.write(reinterpret_cast<uint8_t*>(&val2), sizeof(val2));
    conn.write(reinterpret_cast<uint8_t*>(&val3), sizeof(val3));
    */
    

    // Send the average angular distances for different windows
    conn.write(reinterpret_cast<uint8_t*>(&average_distance_short), sizeof(average_distance_short));
    conn.write(reinterpret_cast<uint8_t*>(&average_distance_mid), sizeof(average_distance_mid));
    conn.write(reinterpret_cast<uint8_t*>(&average_distance_long), sizeof(average_distance_long));
    
    conn.endPacket();

    old_w = w;
    old_x = x;
    old_y = y;
    old_z = z;

    last_time = current_time;  // Update the last time variable
  }
}

void updateWindow(float &cumulative_distance, unsigned long &cumulative_time, float angle, unsigned long time_diff, unsigned long window_limit) {
  cumulative_distance += angle;
  cumulative_time += time_diff;

  // If cumulative time exceeds window limit, scale down
  if (cumulative_time > window_limit) {
    float scale_factor = (float)window_limit / cumulative_time;
    cumulative_distance *= scale_factor;
    cumulative_time = window_limit;
  }
}

float calculateAverageDistance(float cumulative_distance, unsigned long cumulative_time) {
  return cumulative_time > 0 ? cumulative_distance / (cumulative_time / 1000.0) : 0.0f;
}
