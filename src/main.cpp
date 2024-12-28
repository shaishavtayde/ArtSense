#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SparkFunLSM6DSO.h>    // Accelerometer library
#include <Adafruit_CAP1188.h>   // Touch sensor library
#include <WiFi.h>
#include <WebServer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Wi-Fi Credentials
const char* ssid = "";
const char* password = "";

// Web Server Setup
WebServer server(80);

// Function prototypes
void handleRoot();
void handleSetMode();

// LED Setup
#define LED_PIN 15
#define NUM_LEDS 60
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Accelerometer Setup
LSM6DSO myIMU;

// Touch Sensor Setup
#define CAP1188_RESET_PIN 23    // Adjust as necessary
Adafruit_CAP1188 cap = Adafruit_CAP1188(); // I2C address 0x29

// Speaker Setup
#define SPEAKER_PIN 25          // Adjust as necessary

// LED Segments
#define SEGMENT_1_START 0
#define SEGMENT_1_END 17
#define SEGMENT_2_START 18
#define SEGMENT_2_END 29
#define SEGMENT_3_START 30
#define SEGMENT_3_END 47
#define SEGMENT_4_START 48
#define SEGMENT_4_END 59

// Calibration Values
float accelOffsetX = 0;
float accelOffsetY = 0;
float accelOffsetZ = 0;

const float ACCEL_THRESHOLD = 0.5;  // Threshold to start lighting LEDs
const float ACCEL_MAX = 2.0;        // Maximum expected acceleration for full lighting


// Mode Configuration
int mode = 1;
unsigned long modeChangeTime = 0;
const unsigned long modeHoldTime = 2000; // 2 seconds hold

// Brightness Configuration
uint8_t brightness = 128; // Default brightness (0-255)

// Note Frequencies (in Hz)
const int notesMode1[8] = {262, 294, 330, 349, 392, 440, 494, 523};  // C4 to C5
const int notesMode2[8] = {131, 147, 165, 175, 196, 220, 247, 262};  // C3 to C4
const int notesMode3[8] = {100, 200, 300, 400, 500, 600, 700, 800};  // Percussive tones

// LED Colors for Modes
const uint32_t modeColors[3] = {
  strip.Color(0, 0, 255),   // Blue for Mode 1
  strip.Color(0, 255, 0),   // Green for Mode 2
  strip.Color(255, 0, 0)    // Red for Mode 3
};

// BLE Setup
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-5678-90ab-cdef-1234567890ab"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;


float mapAccelToSpeedFactor(float accel, float threshold, float maxAccel) {
    // Map acceleration to a speed factor between 0 and 1
    if (accel < threshold) {
        return 0; // Ignore small accelerations
    }
    return constrain((accel - threshold) / (maxAccel - threshold), 0, 1);
}


// Function to set mode indicator
void setModeIndicator() {
  // Set all LEDs to mode color with current brightness
  uint32_t color = strip.Color(
    (modeColors[mode - 1] >> 16) & 0xFF,
    (modeColors[mode - 1] >> 8) & 0xFF,
    modeColors[mode - 1] & 0xFF
  );

  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, (color >> 16 & 0xFF) * brightness / 255,
                           (color >> 8 & 0xFF) * brightness / 255,
                           (color & 0xFF) * brightness / 255);
  }
  strip.show();
  delay(500);
  // Turn off LEDs
  strip.clear();
  strip.show();
}

// Callback class for BLE
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

// Callback class for BLE Characteristic
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        int newMode = atoi(value.c_str());
        if (newMode >= 1 && newMode <= 6) {
          mode = newMode;
          Serial.print("Mode changed via BLE to ");
          Serial.println(mode);
          setModeIndicator();
        }
      }
    }
};

// Function to initialize BLE
void initBLE() {
  BLEDevice::init("ESP32_IoT_Device");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE server started, waiting for connections...");
}

// Function to calibrate the sensor
void calibrateSensor() {
  Serial.println("Calibrating sensor...");
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 100;

  for (int i = 0; i < samples; i++) {
    sumX += myIMU.readFloatAccelX();
    sumY += myIMU.readFloatAccelY();
    sumZ += myIMU.readFloatAccelZ();
    delay(10);
  }
  accelOffsetX = sumX / samples;
  accelOffsetY = sumY / samples;
  accelOffsetZ = sumZ / samples;
  Serial.println("Calibration completed.");
}

// Function to generate smooth rainbow colors using HSV
uint32_t hsvToRgb(int h, int s, int v) {
    float f, p, q, t;
    int i;
    float r, g, b;

    if (s == 0) {
        r = g = b = v; // Achromatic (gray)
    } else {
        h /= 60;            // Sector 0 to 5
        i = floor(h);
        f = h - i;          // Fractional part of h
        p = v * (1 - s);
        q = v * (1 - f * s);
        t = v * (1 - (1 - f) * s);

        switch (i) {
            case 0: r = v; g = t; b = p; break;
            case 1: r = q; g = v; b = p; break;
            case 2: r = p; g = v; b = t; break;
            case 3: r = p; g = q; b = v; break;
            case 4: r = t; g = p; b = v; break;
            case 5: r = v; g = p; b = q; break;
        }
    }

    // Convert RGB values to a 32-bit color value
    return strip.Color(r * 255, g * 255, b * 255);
}





// Function to rotate mode
void rotateMode() {
  mode++;
  if (mode > 6) mode = 1;
  Serial.print("Switched to Mode ");
  Serial.println(mode);
  setModeIndicator();
}

// Function to handle touch input
void handleTouch(uint8_t touchNum) {
  Serial.print("Touch ");
  Serial.print(touchNum + 1);
  Serial.print(" in Mode ");
  Serial.println(mode);

  int frequency = 0;

  switch (mode) {
    case 1:
      frequency = notesMode1[touchNum];
      strip.setPixelColor(touchNum * (NUM_LEDS / 8), modeColors[0]); // Map touchNum to LED position
      break;
    case 2:
      frequency = notesMode2[touchNum];
      strip.setPixelColor(touchNum * (NUM_LEDS / 8), modeColors[1]); // Map touchNum to LED position
      break;
    case 3:
      frequency = notesMode3[touchNum];
      strip.setPixelColor(touchNum * (NUM_LEDS / 8), modeColors[2]); // Map touchNum to LED position
      break;
    case 4:
    case 5:
    case 6:
      // In modes 4-6, touch is used for brightness control
      if (touchNum < 4) {
        // Increase brightness
        brightness += 32;
        if (brightness > 255) brightness = 255;
        Serial.print("Brightness increased to ");
        Serial.println(brightness);
      } else {
        // Decrease brightness
        brightness -= 32;
        if (brightness < 32) brightness = 32;
        Serial.print("Brightness decreased to ");
        Serial.println(brightness);
      }
      strip.setBrightness(brightness);
      strip.show();
      return; // No tone to play in brightness modes
  }

  strip.show();

  // Play tone using tone32
  if (frequency > 0) {
    ledcWriteTone(0, frequency); // Play the tone
    ledcWrite(0, 255);
    delay(200);                  // Tone duration
    ledcWriteTone(0, 0);         // Stop the tone
  }

  // Clear LED after touch
  strip.setPixelColor(touchNum * (NUM_LEDS / 8), 0);
  strip.show();
}

// Function to handle touch inputs
void handleTouchInputs() {
  // Check for mode change via Touch Pad 8 (Zero-indexed)
  if (cap.touched() & (1 << 7)) { // Touch Pad 8
    if (modeChangeTime == 0) {
      modeChangeTime = millis();
    } else if (millis() - modeChangeTime >= modeHoldTime) {
      rotateMode();
      modeChangeTime = 0;
    }
  } else {
    modeChangeTime = 0;
  }

  // Read touch inputs
  uint8_t touched = cap.touched();
  for (uint8_t i = 0; i < 8; i++) {
    if (touched & (1 << i)) {
      handleTouch(i);
      delay(200); // Debounce delay
    }
  }
}

void handleRoot() {
  server.send(200, "text/plain", "ESP32 is running.");
}

void handleSetMode() {
  if (server.hasArg("mode")) {
    String modeStr = server.arg("mode");
    int newMode = modeStr.toInt();
    if (newMode >= 1 && newMode <= 6) {
      mode = newMode;
      Serial.print("Mode changed to ");
      Serial.println(mode);
      setModeIndicator();
      server.send(200, "text/plain", "Mode changed to " + String(mode));
    } else {
      server.send(400, "text/plain", "Invalid mode value.");
    }
  } else {
    server.send(400, "text/plain", "Mode parameter missing.");
  }
}

// Rainbow Effect
void rainbowEffect() {
    static uint16_t startHue = 0;  // Starting hue value
    for (int i = 0; i < strip.numPixels(); i++) {
        // Calculate hue for each pixel
        uint16_t hue = startHue + (i * 65536L / strip.numPixels());
        // Set pixel color using HSV to RGB conversion for smooth gradient
        strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(hue)));
    }
    strip.show();
    startHue += 256;  // Adjust for the speed of the rainbow flow
}

// Wave Effect
void waveEffect() {
    static float position = 0.0;
    position += 0.1;  // Adjust this value to change the wave speed

    for (int i = 0; i < strip.numPixels(); i++) {
        // Calculate the wave value using a sine function for smooth transitions
        float wave = (sin((i / 5.0) + position) + 1.0) * 127.5;
        uint8_t red = (uint8_t)wave;
        uint8_t blue = 255 - red;
        // Set pixel color with smooth transitions between red and blue
        strip.setPixelColor(i, strip.Color(red, 0, blue));
    }
    strip.show();
}

// Ripple Effect
void rippleEffect() {
    static int center = -1;
    static int step = -1;
    const int maxSteps = 20;  // Increase for a smoother and longer ripple

    if (step == -1) {  // Reset ripple
        center = random(0, strip.numPixels());  // Choose a random LED as the center
        step = 0;
    }

    for (int i = 0; i < strip.numPixels(); i++) {
        float distance = abs(i - center);
        // Use a Gaussian function for smooth intensity transition
        float intensity = exp(-pow(distance - step, 2) / 20.0) * 255;
        if (intensity > 0) {
            strip.setPixelColor(i, strip.Color(intensity * brightness / 255, intensity * brightness / 255, 255 * brightness / 255));
        } else {
            strip.setPixelColor(i, 0);  // Turn off the LED if intensity is zero
        }
    }
    strip.show();
    step++;

    // Reset the ripple effect after it has expanded sufficiently
    if (step > (strip.numPixels() / 2)) {
        step = -1;
    }
}


void setup() {
  Serial.begin(9600);

  // Initialize LED strip
  strip.begin();
  strip.setBrightness(brightness);
  strip.show(); // Turn off all LEDs initially

  // Initialize I2C for LSM6DSO and CAP1188
  Wire.begin(21,22);

  if (!myIMU.begin()) {
      Serial.println("Failed to initialize LSM6DSO!");
      while (1); // Loop forever if initialization fails
  }

  // Set accelerometer range to 2g
  if (!myIMU.setAccelRange(2)) {
      Serial.println("Failed to set accelerometer range!");
  } else {
      Serial.println("Accelerometer range set to 2g");
  }

  // Set accelerometer data rate to 104 Hz
  if (!myIMU.setAccelDataRate(104)) {
      Serial.println("Failed to set accelerometer data rate!");
  } else {
      Serial.println("Accelerometer data rate set to 104 Hz");
  }

  // Confirm settings
  float currentRate = myIMU.getAccelDataRate();
  Serial.print("Current accelerometer data rate: ");
  Serial.println(currentRate);

  uint8_t range = myIMU.getAccelRange();
  Serial.print("Current accelerometer range: ");
  Serial.println(range);

  // Initialize touch sensor
  pinMode(CAP1188_RESET_PIN, OUTPUT);
  digitalWrite(CAP1188_RESET_PIN, HIGH); // Ensure the reset pin is high
  if (!cap.begin(0x29)) {   // Pass the reset pin to begin()
    Serial.println("CAP1188 not found");
    while (1);
  }
  Serial.println("CAP1188 found!");

  // Initialize tone channel
  ledcSetup(0, 2000, 8); // Channel 0, 2kHz, 8-bit resolution
  ledcAttachPin(SPEAKER_PIN, 0);

  // Calibrate accelerometer
  calibrateSensor();

  // Set initial mode indicator
  setModeIndicator();

  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);

  // Wait until connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to Wi-Fi.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Set up server routes
  server.on("/", handleRoot);
  server.on("/setMode", handleSetMode);

  // Start the server
  server.begin();
  Serial.println("HTTP server started.");

  // Initialize BLE
  initBLE();
}

void loop() {
    // Handle touch inputs
    handleTouchInputs();

    // Handle web server
    server.handleClient();

    // Handle BLE
    // BLE tasks are handled by the BLE library in the background

    // Read accelerometer values
    float accelX = myIMU.readFloatAccelX() - accelOffsetX;
    float accelY = myIMU.readFloatAccelY() - accelOffsetY;

    // Calculate speed factors for each direction
    float speedFactorPosX = (accelX > 0) ? mapAccelToSpeedFactor(accelX, ACCEL_THRESHOLD, ACCEL_MAX) : 0;
    float speedFactorNegX = (accelX < 0) ? mapAccelToSpeedFactor(-accelX, ACCEL_THRESHOLD, ACCEL_MAX) : 0;
    float speedFactorPosY = (accelY > 0) ? mapAccelToSpeedFactor(accelY, ACCEL_THRESHOLD, ACCEL_MAX) : 0;
    float speedFactorNegY = (accelY < 0) ? mapAccelToSpeedFactor(-accelY, ACCEL_THRESHOLD, ACCEL_MAX) : 0;

    // Identify the dominant direction
    float maxFactor = speedFactorPosX;
    int dominantDirection = 1; // 1: +X, 2: -X, 3: +Y, 4: -Y

    if (speedFactorNegX > maxFactor) {
        maxFactor = speedFactorNegX;
        dominantDirection = 2;
    }
    if (speedFactorPosY > maxFactor) {
        maxFactor = speedFactorPosY;
        dominantDirection = 3;
    }
    if (speedFactorNegY > maxFactor) {
        maxFactor = speedFactorNegY;
        dominantDirection = 4;
    }

    // Reset LEDs before updating
    strip.clear();

    // Function to light up a single segment based on speedFactor
    auto lightUpSegment = [&](int start, int end, float factor) {
        if (factor <= 0.0) return;
        int segmentLength = end - start + 1;
        int ledsToLight = factor * segmentLength;
        if (ledsToLight < 1) ledsToLight = 1; // Ensure at least one LED is lit

        for (int i = 0; i < ledsToLight && (start + i) <= end; i++) {
            // Use a solid color based on the current mode
            strip.setPixelColor(start + i, modeColors[mode - 1]);
        }
    };

    // Light up only the dominant segment
    switch (dominantDirection) {
        case 1: // Positive X
            lightUpSegment(SEGMENT_1_START, SEGMENT_1_END, speedFactorPosX);
            break;
        case 2: // Negative X
            lightUpSegment(SEGMENT_3_START, SEGMENT_3_END, speedFactorNegX);
            break;
        case 3: // Positive Y
            lightUpSegment(SEGMENT_2_START, SEGMENT_2_END, speedFactorPosY);
            break;
        case 4: // Negative Y
            lightUpSegment(SEGMENT_4_START, SEGMENT_4_END, speedFactorNegY);
            break;
        default:
            // No dominant direction; all LEDs remain off
            break;
    }

    // Update the LED strip to show the changes
    strip.show();

    // Execute mode-specific effects
    switch (mode) {
        case 4:
            rainbowEffect();
            break;
        case 5:
            waveEffect();
            break;
        case 6:
            rippleEffect();
            break;
        default:
            // Handle modes 1-3 if necessary
            break;
    }

    delay(200); // Reduced delay for smoother animation
}


