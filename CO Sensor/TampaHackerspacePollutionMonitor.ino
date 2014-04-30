#include <DigiFi.h>

////////////////////////////////////////////////////////////////////////////
// Network and location information
////////////////////////////////////////////////////////////////////////////
#define USER_ID "3"
#define SENSOR_ID "1"
#define API_KEY "DEMOKEY"
#define LATITUDE "27.967946"
#define LONGITUDE "-82.485491"

////////////////////////////////////////////////////////////////////////////
// Debugging
////////////////////////////////////////////////////////////////////////////
#define DEBUG

////////////////////////////////////////////////////////////////////////////
// Hardware connections
////////////////////////////////////////////////////////////////////////////
#define MQ7_SENSOR_PIN (A2)
#define MQ7_HEATER_PIN (A0)

////////////////////////////////////////////////////////////////////////////
// Timing and data sizes
////////////////////////////////////////////////////////////////////////////
// Sensors
// Number of sensor readings to keep in memory
#define SENSOR_READING_QUEUE_COUNT (250)
// How ofter to look for sensor readings
#define SENSOR_READING_INTERVAL_MS (15L*60L*1000L)
// How often to collect detailed readings. These are averaged and reported as a single reading.
#define SENSOR_READING_DETAIL_INTERVAL_MS (10L*1000L)
// Maximum number of detailed readings to average into a single reading event
#define SENSOR_READING_DETAIL_MAX_COUNT (10)

// Wifi
#define TIME_BETWEEN_WIFI_SENDS_MS (8L*60L*1000L)

// MQ-7
#define MQ7_HEATER_HIGH_MS (60L*1000L)
#define MQ7_HEATER_LOW_MS (90L*1000L)

////////////////////////////////////////////////////////////////////////////
// Circular queue of sensor readings
////////////////////////////////////////////////////////////////////////////
#define SENSOR_FLAG_HAS_DATA (0x01)
#define SENSOR_FLAG_HAS_BEEN_SENT (0x02)

class SensorReading {
public:
  int mq7; // CO sensor
  int flags;
};

class CircularQueue {
public:
  int head, tail, maxCount, count;
  
  CircularQueue(int maxCountToUse)
      : head(0), tail(0), maxCount(maxCountToUse), count(0) {
  }

  void removeAll() {
    head = tail = count = 0;
  }
  
  int isEmpty() {
    return count <= 0;
  }

  int getCount() const {
    return count;
  }
  
  int getOldest() const {
    return tail;
  }

  int add() { // Returns the index to use for the added item
    int oldHead = head;
    if(count < maxCount) {
      count++;
      head = (head + 1) % maxCount;
    } else {
      // Remove an item by advancing the tail index
      tail = (tail + 1) % maxCount;
      // Add an item by advancing the head index
      head = (head + 1) % maxCount;
    }
    return oldHead;
  }
  
  void removeOldest() {
    if(count > 0) {
      count--;
      tail = (tail + 1) % maxCount;
    }
  }
};

////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////
DigiFi wifi;
int hasWifi = 0;

CircularQueue sensorQueueManager(SENSOR_READING_QUEUE_COUNT);
SensorReading sensorValues[SENSOR_READING_QUEUE_COUNT]; // buffer for circular data

unsigned long lastMillis = 0;
unsigned long deltaMillis = 0;
unsigned long msToNextWifiSend = TIME_BETWEEN_WIFI_SENDS_MS;
unsigned long msToNextMQ7Event = 0;

int mq7State = 2; // 0: Off, 1: Ready to collect data, 2: Heating

////////////////////////////////////////////////////////////////////////////
// setup(), one time
////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600); 

  // Clear the queue
  for(int i=0;i<SENSOR_READING_QUEUE_COUNT;i++) {
    // Clear all flags in the buffer. This will prevent daily summaries from
    // misinterpreting unfilled values.
    sensorValues[i].flags = 0;
  }
  sensorQueueManager.removeAll();

  // Note the time for state changes
  lastMillis = millis();

  // Setup the machinery
  mq7State = 2; // start with heating because it must be done before data can be collected
  operateMachinery();
}

////////////////////////////////////////////////////////////////////////////
// loop(), always
////////////////////////////////////////////////////////////////////////////
void loop() {
  // Determine how long as elapsed since the last loop,
  // taking into account integer overflow.
  unsigned long nowMillis = millis();
  if(nowMillis >= lastMillis) {
    deltaMillis = nowMillis - lastMillis;
  } else {
    deltaMillis = nowMillis + 1;
    deltaMillis += ((unsigned long)0xFFFFFFFF) - lastMillis;
  }
  lastMillis = nowMillis;

  // Gather and send data
  operateMachinery();
  gatherData();
  sendData();
  showStatus();
  
  delay(250);
}

////////////////////////////////////////////////////////////////////////////
// Machinery operation
////////////////////////////////////////////////////////////////////////////
int priorMQ7State = -1;

// Send control signals to all machines.
// The MQ-7 CO sensor is a machine in that it's heater element must be controlled.
void operateMachinery() {
  // Countdown to next MQ-7 event
  if(msToNextMQ7Event > 0) {
    if(msToNextMQ7Event >= deltaMillis) {
      msToNextMQ7Event -= deltaMillis;
    } else {
      msToNextMQ7Event = 0;
    }
  }
  // Determine what to do next to the MQ-7
  int done = 0;
  while(!done) {
    done = 1;
    switch(mq7State) {
      default:
        mq7State = 0;
        break;
      case 0: // Off
        if(mq7State != priorMQ7State) {
          // Turn off the heater
          analogWrite(MQ7_HEATER_PIN,0);
          msToNextMQ7Event = 0;
        }
        break;
      case 1: // Low voltage, ok to collect data.
        if(mq7State != priorMQ7State) {
          // Turn on PWM to get 1.4V average.
          // 1.4V   PWM Value
          // ---- = ---------, PWM Value = 71.4 -> 72
          // 5.0V     255
          analogWrite(MQ7_HEATER_PIN,72);
          msToNextMQ7Event = MQ7_HEATER_LOW_MS;
        } else if(msToNextMQ7Event == 0) {
          mq7State = 2;
          priorMQ7State = -1;
          done = 0; // loop around to change state right now.
        }
        break;
      case 2: // High voltage, heating the element. Do not collect data here.
        if(mq7State != priorMQ7State) {
          // Turn on high voltage.
          analogWrite(MQ7_HEATER_PIN,255);
          msToNextMQ7Event = MQ7_HEATER_HIGH_MS;
        } else if(msToNextMQ7Event == 0) {
          mq7State = 1;
          priorMQ7State = -1;
          done = 0; // loop around to change state right now.
        }
        break;
    }
  }
  priorMQ7State = mq7State;
}

////////////////////////////////////////////////////////////////////////////
// Data collection
////////////////////////////////////////////////////////////////////////////
unsigned long msToNextMQ7Reading = SENSOR_READING_DETAIL_INTERVAL_MS;
  // SENSOR_READING_DETAIL_INTERVAL_MS between groups of readings.
  // SENSOR_READING_DETAIL_INTERVAL_MS between readings within a group.
int mq7DetailedReadings[SENSOR_READING_DETAIL_MAX_COUNT];
int countOfMQ7DetailedReadings = 0;

// Peridoically collect sensor data.
// Data is gathered while a machine is in an acceptable state, summarized and queued for transmission.
void gatherData() {
  // Countdown to when data should be collected
  if(msToNextMQ7Reading > 0) {
    if(msToNextMQ7Reading >= deltaMillis) {
      msToNextMQ7Reading -= deltaMillis;
    } else {
      msToNextMQ7Reading = 0;
    }
  }
  if(msToNextMQ7Reading == 0) { // If it is time to read data
    // mq7State: 0: Off, 1: Ready to collect data, 2: Heating
    if(countOfMQ7DetailedReadings >= SENSOR_READING_DETAIL_MAX_COUNT
        || (mq7State != 1 && countOfMQ7DetailedReadings > 0)) {
      processMQ7DetailedReadings();
      countOfMQ7DetailedReadings = 0;
      msToNextMQ7Reading = SENSOR_READING_DETAIL_INTERVAL_MS; // restart the countdown
    }
    if(mq7State == 1) { // If Ready to collect data
      mq7DetailedReadings[countOfMQ7DetailedReadings] = analogRead(MQ7_SENSOR_PIN);
      countOfMQ7DetailedReadings++;
      msToNextMQ7Reading = SENSOR_READING_DETAIL_INTERVAL_MS; // restart the countdown
    }
  }
}

// Average the detailed readings, discarding the highest and lowest values as noise.
void processMQ7DetailedReadings() {
  if(countOfMQ7DetailedReadings > 1) {
    int highIndex = 0, lowIndex = 0;
    for(int i=1;i<countOfMQ7DetailedReadings;i++) {
      if(mq7DetailedReadings[i] > mq7DetailedReadings[highIndex]) {
        highIndex = i;
      }
      if(mq7DetailedReadings[i] < mq7DetailedReadings[lowIndex]) {
        lowIndex = i;
      }
    }
    if(countOfMQ7DetailedReadings == 2 && highIndex != lowIndex) {
      lowIndex = highIndex;
    }
    long total = 0;
    int n = 0;
    for(int i=0;i<countOfMQ7DetailedReadings;i++) {
      if(i != highIndex && i != lowIndex) {
        total += mq7DetailedReadings[i];
        n++;
      }
    }
    if(n > 0) {
      mq7DetailedReadings[0] = total/n;
    }
  }
  // Queue the reading, which is now stored in mq7DetailedReadings[0]
  int readingIndex = sensorQueueManager.add();
  sensorValues[readingIndex].mq7 = mq7DetailedReadings[0];
  sensorValues[readingIndex].flags = SENSOR_FLAG_HAS_DATA;
  if(msToNextWifiSend > 10000L) {
    msToNextWifiSend = 10000L;
  }
}

// Periodically send queued data.
void sendData() {
  // Countdown to when Wifi should be accessed
  if(msToNextWifiSend > 0) {
    if(msToNextWifiSend >= deltaMillis) {
      msToNextWifiSend -= deltaMillis;
    } else {
      msToNextWifiSend = 0;
    }
  }
  if(msToNextWifiSend == 0) { // If it is time to send data
    if(!sensorQueueManager.isEmpty()) { // If there is data to send
      sendOutstandingQueuedPackets(); // send oldest to newest packets
    }
    msToNextWifiSend = TIME_BETWEEN_WIFI_SENDS_MS; // restart the countdown
  }
}

////////////////////////////////////////////////////////////////////////////
// Wifi and data transmission
////////////////////////////////////////////////////////////////////////////
void setupWifi()
{
  if(hasWifi) {
    stopWifi();
  }

  Serial.println("Connecting Arduino to network...");
  Serial.println();  

  delay(1000);
  
  // Connect to network amd obtain an IP address using DHCP
  wifi.begin(9600);
  while(wifi.ready() != 1) {
    Serial.println("Connecting to network...");
    delay(1000);
  }  
  
  delay(1000);
  hasWifi = 1;
}

void stopWifi() {
  wifi.close();
  hasWifi = 0;
}

void sendOutstandingQueuedPackets() {
  setupWifi();
  // Send packets, dequeuing each upon success, until done with packets or until an error occurs.
  // Send a relative minutes ago time for each reading sent. "minutesDelay"
  /*
  user
  sensor id
  api key
  version number
  latitude
  longitude
  raw data
  minute delay - how many minutes before the posting time was the reading taken
  */
  while(!sensorQueueManager.isEmpty()) { // While there is data to send
    int index = sensorQueueManager.getOldest();
    int minutesAgo = (long)sensorQueueManager.getCount() * SENSOR_READING_INTERVAL_MS / 1000L / 60L;
    String postContent = 
        "user="+wifi.URLEncode(USER_ID)
        +"&sensorid="+wifi.URLEncode(SENSOR_ID)
        +"&apikey="+wifi.URLEncode(API_KEY)
        +"&versionnumber=1"
        +"&latitude="+wifi.URLEncode(LATITUDE)
        +"&longitude="+wifi.URLEncode(LONGITUDE)
        +"&minutedelay="+minutesAgo
        +"&rawdata="+sensorValues[index].mq7;
#ifdef DEBUG
  String debugText("postContent=");
  debugText += postContent;
  Serial.println(debugText);
#endif

    if(wifi.post("tampahackerspace.com",
        "/dontpollute/feed.php",
        postContent
        )){
      String body = wifi.body();
#ifdef DEBUG
      Serial.println(body);
#endif
      if(body.indexOf("thsafe") < 0) {
        // No acceptance flag was found
#ifdef DEBUG
        Serial.println("ERROR: No thsafe acknowledgement found in HTML response!");
#endif
        break;
      }
      sensorValues[index].flags |= SENSOR_FLAG_HAS_BEEN_SENT;
#ifdef DEBUG
      Serial.println("OK: Successfully sent a reading.");
#endif
    } else{
#ifdef DEBUG
      Serial.println("ERROR: Unable to POST");
#endif
      break;
    }
#ifdef DEBUG
    sensorQueueManager.removeOldest();
#endif
  }

  stopWifi();
}

////////////////////////////////////////////////////////////////////////////
// Status display
////////////////////////////////////////////////////////////////////////////
void showStatus() {
#ifdef DEBUG
  String debugText("deltaMillis=");
  debugText += deltaMillis;
  debugText += ", msToNextMQ7Event=";
  debugText += msToNextMQ7Event;
  debugText += ", msToNextMQ7Reading=";
  debugText += msToNextMQ7Reading;
  debugText += ", msToNextWifiSend=";
  debugText += msToNextWifiSend;
  debugText += ", mq7State=";
  debugText += mq7State;
  if(mq7State == 1) { // If Ready to collect data
    debugText += ", MQ7=";
    debugText += analogRead(MQ7_SENSOR_PIN);
  }
  Serial.println(debugText);
#endif

  // Toggle the board LED to indicate internal state
  // TODO
}

