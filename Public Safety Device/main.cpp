#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// WiFi Credentials
const char* ssid = "Alienware X18";
const char* password = "Password";

unsigned long lastWebRequest = 0;
bool webClientActive = false;

// Create server on port 80
WebServer server(80);

// Serial instances
HardwareSerial gpsSerial(1);    // GPS Module (UART1)
HardwareSerial sim800l(2);      // SIM800L Module (UART2)
TinyGPSPlus gps;

// GPS Pins
#define GPS_RX 16  // GPS TX -> ESP32 RX
#define GPS_TX 17  // GPS RX -> ESP32 TX

// SIM800L Pins
#define SIM800_TX 26  // ESP32 TX -> SIM800L RX
#define SIM800_RX 27  // ESP32 RX -> SIM800L TX

// Push Button Pin
#define BUTTON_PIN 12

// GPS Data
float latitude = 0.0, longitude = 0.0;
bool alert = false;

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
    sim800l.begin(9600, SERIAL_8N1, SIM800_TX, SIM800_RX);

    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Setup button pin

    // WiFi Setup
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ WiFi Connected. IP: " + WiFi.localIP().toString());

    // Web server endpoint
    server.on("/", HTTP_GET, handleGPSRequest);
    server.begin();

    Serial.println("ESP32 GPS + SIM800L Initialized.");
}

void loop() {
    server.handleClient();

    // Inactivity timeout
    if (webClientActive && (millis() - lastWebRequest > 10000)) {
        webClientActive = false;
        Serial.println("⏱ Web client inactive for 10 seconds.");
    }

    // If button is pressed (active LOW)
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("🔘 Button Pressed: Fetching GPS...");
        fetchGPSData();
        delay(1000);  // Debounce
    }
}

// Only read GPS when button is pressed
void fetchGPSData() {
    unsigned long timeout = millis();
    while (millis() - timeout < 3000) {  // Try for up to 3 seconds
        while (gpsSerial.available()) {
            char c = gpsSerial.read();
            gps.encode(c);

            if (gps.location.isUpdated()) {
                latitude = gps.location.lat();
                longitude = gps.location.lng();
                alert = true;

                Serial.println("\n====== GPS Data (Button Triggered) ======");
                Serial.print("Latitude: ");
                Serial.println(latitude, 6);
                Serial.print("Longitude: ");
                Serial.println(longitude, 6);
                Serial.println("=========================================");

                sendGPSDataToGSM(latitude, longitude);  // Optional SMS
                return;
            }
        }
    }

    Serial.println("⚠ No valid GPS fix after button press.");
}

// Web response in JSON
void handleGPSRequest() {
    Serial.println("📡 Web client requested GPS data...");
    Serial.print("🌐 Request received from IP: ");
    Serial.println(server.client().remoteIP());

    lastWebRequest = millis();  // Update the timestamp
    webClientActive = true;

    String json = "{";
    json += "\"lat\": \"" + String(latitude, 6) + "\",";
    json += "\"lon\": \"" + String(longitude, 6) + "\",";
    json += "\"alert\": \"" + String(alert ? "true" : "false") + "\"";
    json += "}";

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", json);
}

// Optional: Send GPS data via SIM800L SMS
void sendGPSDataToGSM(float lat, float lon) {
    String message = "Location: https://maps.google.com/?q=" + String(lat, 6) + "," + String(lon, 6);
    
    Serial.println("📡 Sending SMS (if SIM present)...");
    Serial.println("SMS Content: " + message);
    
    // Uncomment to send SMS
    // sim800l.println("AT+CMGF=1");
    // delay(200);
    // sim800l.println("AT+CMGS=\"+1234567890\"");  // Replace with recipient
    // delay(200);
    // sim800l.println(message);
    // delay(200);
    // sim800l.write(26);  // Ctrl+Z
    // Serial.println("✅ SMS Sent!");
}
