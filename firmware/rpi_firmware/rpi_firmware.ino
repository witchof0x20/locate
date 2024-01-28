#include <SerialBT.h>
#include <assert.h>

//#define DEBUG
//#define DEBUG_LIDAR_SERIAL
#define DEBUG_PACKETS
//#define VERBOSE
//#define DEBUG_CHECKSUM
//#define DEBUG_BLUETOOTH

// ----- START LIDAR RELATED STUFF -----
#define SerialLidar Serial1
const size_t LIDAR_COMMAND_LEN = 1;
uint8_t LIDAR_COMMAND_START[LIDAR_COMMAND_LEN] = {'b'};
uint8_t LIDAR_COMMAND_STOP[LIDAR_COMMAND_LEN] = {'e'};

uint8_t LIDAR_SYNC_BYTE = 0xFA;
uint8_t LIDAR_START_ANGLE = 0xA0;
uint8_t LIDAR_END_ANGLE = 0xDB;


// Info for a single LIDAR degree
struct LidarAngle {
  // ??
  uint16_t intensity;
  // Distance
  uint16_t dist_m;
  // ??
  uint16_t reserved;
};

// Frames contain 6 degrees
// Declared as 2 different types because why not
const uint16_t DEGREES_PER_FRAME_u16 = 6;
const size_t DEGREES_PER_FRAME = (size_t)DEGREES_PER_FRAME_u16;
static_assert(sizeof(size_t) >= sizeof(uint16_t));
// Frame, as specified in the docs
struct LidarFrame {
  // sync byte, always 0xFA
  byte sync;
  // Degree base, from 0xA0 (0) to 0xDB (59)
  // Multiply by 6 to get actual degree base
  byte degree;
  // RPM of the motor
  uint16_t rpm;
  // Info for each individual angle
  LidarAngle angles[DEGREES_PER_FRAME];
  // Checksum for the packet
  uint8_t checksum[2];
};
static_assert(sizeof(LidarFrame) == 42);



// Union allowing safer-ish writing to the frame
union LidarFrameData {
  // Frame data
  LidarFrame frame;
  // Byte buffer
  byte buf[sizeof(LidarFrame)];
};
// ----- END LIDAR RELATED STUFF -----

// ----- START BT RELATED STUFF -----
#ifndef DEBUG_BLUETOOTH
#define SerialOut SerialBT
#else
#define SerialOut Serial
#endif

// The info we care about for a single degree
struct AngleEntry {
  uint16_t dist;
};

// The info we care about for a frame
struct FrameEntry {
  // RPM of rotation
  //uint16_t rpm;
  // Base angle (all angles within start here)
  uint16_t angle_base;
  AngleEntry angles[DEGREES_PER_FRAME];
};
static_assert(sizeof(FrameEntry) == 14);
// Packet that will be sent over bluetooth
// Max number of frames to send at once
const size_t MAX_FRAMES_PER_MESSAGE = 60; // 360/6
// value we will use for magic
const size_t MAGIC_LEN = 4;
const uint8_t MAGIC[MAGIC_LEN] = {0xAA, 0xBB, 0xCC, 0xDD};;
// Data contained in the packet
struct BluetoothPacketFields {
  // Header indicating the start of a packet
  uint8_t magic[MAGIC_LEN];
  // Number of frames we have read
  uint16_t num_frames;
  // Array of frames
  FrameEntry frames[MAX_FRAMES_PER_MESSAGE];
};

// Size of a max-sized bluetooth packet
const size_t MAX_BLUETOOTH_PACKET_SIZE = sizeof(BluetoothPacketFields);
// Union for convenient access to the underlying bytes
union BluetoothPacket {
  // The packet itself
  BluetoothPacketFields fields;
  // The packet as bytes
  byte buf[MAX_BLUETOOTH_PACKET_SIZE];
};

// Write the packet over bluetooth, ensuring everything gets written properly
void bluetooth_write(BluetoothPacket& packet) {
  // Set the magic
  memcpy((void*)(uint8_t*)(&(packet.fields.magic)), (void*)(uint8_t*)(&MAGIC), MAGIC_LEN);
  // Calculate the length of the packet
  size_t len = MAGIC_LEN + sizeof(uint16_t) + ((size_t)packet.fields.num_frames) * sizeof(FrameEntry);
#ifdef DEBUG_PACKETS
  Serial.print("Sending bluetooth packet with ");
  Serial.print(packet.fields.num_frames);
  Serial.print(" frames and ");
  Serial.print(len);
  Serial.println(" bytes. ");
  for (size_t i = 0; i < len; ++i) {
    Serial.print(packet.buf[i], HEX);
  }
  Serial.println();
#endif
  // Ensure we actually can send the packet (which could avoid issues)
  while (!SerialOut.availableForWrite()) { }
  // Start with 0 bytes written
  size_t bytes_written = 0;
  // Write until everything is written
  while (bytes_written < len) {
    bytes_written += SerialOut.write((byte*)(&packet.buf) + bytes_written, len - bytes_written);
  }
  SerialOut.flush();
  // If our underlying library wrote more bytes than we asked for,,
  // then a major violation has occurred
  if (bytes_written != len) [[unlikely]] {
    panic();
  }
}
// ----- END BT RELATED STUFF -----

void panic() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}
bool angle_is_valid(uint8_t angle_byte)  {
  return angle_byte >= LIDAR_START_ANGLE && angle_byte <= LIDAR_END_ANGLE;
};
bool checksum_is_valid(LidarFrameData& frame_data) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < 40; ++i) {
    checksum += frame_data.buf[i];
  }
  checksum = 0xFF - checksum;
  bool is_valid = checksum == frame_data.frame.checksum[0] || checksum == frame_data.frame.checksum[0];
#ifdef DEBUG_CHECKSUM
  if (!is_valid) {
    Serial.print("Expected checksum ");
    Serial.print(frame_data.frame.checksum[0], HEX);
    Serial.print(frame_data.frame.checksum[1], HEX);
    Serial.print(" calculated ");
    Serial.println(checksum, HEX);
  }
#endif
  return is_valid;
}

// Prototype to add default arguments
// offset: what offset to start reading into
// len: How many bytes to read. If zero, defaults to sizeof(LidarFrame) - offset (e.g. the remainder)
byte* lidar_read_frame(LidarFrameData& frame_data, size_t offset = 0, size_t len = 0);
byte* lidar_read_frame(LidarFrameData& frame_data, size_t offset, size_t len) {
  // If no length is provided, automatically generate it
  // assuming we are reading the remainder of a frame
  if (len == 0) [[likely]] {
    len = sizeof(LidarFrame) - offset;
  }
#ifdef VERBOSE
  Serial.print("Reading a frame from offset ");
  Serial.print(offset);
  Serial.print(" and with length ");
  Serial.print(len);
  Serial.println();
#endif
  // Get the buffer pointer and length
  byte* buf = (byte*)(&(frame_data.buf)) + offset;
  // Read bytes
  size_t bytes_read = 0;
  // Read until entirely read
  while (bytes_read < len) {
    // Request some bytes
    bytes_read += SerialLidar.readBytes(buf + bytes_read, len - bytes_read);
  }
  // If this happens, something has gone wrong
  if (bytes_read != len) [[unlikely]] {
    panic();
    return nullptr;
  }
#ifdef VERBOSE
  Serial.println("Read succeeded");
#endif
  return buf;
}

void restart_lidar() {
  SerialLidar.write((byte*)LIDAR_COMMAND_STOP, LIDAR_COMMAND_LEN);
  delay(500);
  SerialLidar.write((byte*)LIDAR_COMMAND_START, LIDAR_COMMAND_LEN);
  delay(1);
}

// Read frame from the LIDAR
bool read_frame(bool& sync, LidarFrameData& frame_data, FrameEntry& frame_out, uint8_t expected_angle) {
  // If already synchronized, just read the frame
  if (sync) [[likely]] {
    // Read the frame
    lidar_read_frame(frame_data);
    // Ensure the initial byte is correct
    if (frame_data.frame.sync == LIDAR_SYNC_BYTE) [[likely]] {
      // Check the 2nd byte
      if (angle_is_valid(frame_data.frame.degree)) [[likely]] {
        // Check whether the value is within expected
        if (frame_data.frame.degree != expected_angle) {
#ifdef DEBUG_LIDAR_SERIAL
          Serial.print("Found degree ");
          Serial.print((uint32_t)frame_data.frame.degree, HEX);
          Serial.print(" instead of the expected ");
          Serial.print(expected_angle, HEX);
          Serial.print("");
          Serial.println();
#endif
        }
      }
      else [[unlikely]] {
#ifdef DEBUG_LIDAR_SERIAL
        Serial.print("Found degree ");
        Serial.print((uint32_t)frame_data.frame.degree);
        Serial.print(" instead of a value 0xA0-0xDB. this results in a recalibration");
        Serial.println();
#endif
        // Bail out because this value is very invalid
        sync = false;
        return false;
      }
    } else [[unlikely]] {
#ifdef DEBUG_LIDAR_SERIAL
      Serial.print("Found ");
      Serial.print((uint32_t)frame_data.frame.sync);
      Serial.print(" instead of 0xFA. this results in a recalibration");
      Serial.println();
#endif
      sync = false;
      return false;
    }
  }
  // We are unsynced
  else [[unlikely]] {
      // Track how many bytes we ate
      size_t discarded_bytes = 0;
      // Read until we get 0xFA 0xA0
    while (!sync) {
    // Read the first byte
    byte* potential_sync_byte = lidar_read_frame(frame_data, 0, 1);
      // Check if it's the check byte
      if (*potential_sync_byte == LIDAR_SYNC_BYTE) {
        // Read the 2nd byte
        byte* potential_start_angle_byte = lidar_read_frame(frame_data, 1, 1);
        // Check if it's the start angle
        if (angle_is_valid(*potential_start_angle_byte)) {
          // We have our first frame
          // Read the rest of the frame, with an offset
          lidar_read_frame(frame_data, 2);
          // We are now synchronized
          sync = true;
        }
        else {
          // Track the discarded byte
          discarded_bytes += 1;
          // TODO: even if we don't find our start byte here, theres a high chance we're in a frame
          // Ideally we would read the rest of it and try for the next one, speeding this loop up a bit
#ifdef VERBOSE
          Serial.print("Found ");
          Serial.print((uint32_t)*potential_start_angle_byte);
          Serial.print(" instead of 0xA0 ");
          Serial.println();
#endif
        }
      }
      // If we don't see the sync byte, iterate to the next loop
      else {
        // Track the discarded byte
        discarded_bytes += 1;
#ifdef VERBOSE
        Serial.print("Found ");
        Serial.print((uint32_t)*potential_sync_byte);
        Serial.print(" instead of 0xFA ");
        Serial.println();
#endif
      }
    }
#ifdef DEBUG_LIDAR_SERIAL
  Serial.print("Discarded  ");
  Serial.print(discarded_bytes);
  Serial.println(" bytes in calibration");
#endif

  }
  // Store info from the frame
  //frame_out.rpm = frame_data.frame.rpm;
  frame_out.angle_base =  DEGREES_PER_FRAME_u16 * (uint16_t)(frame_data.frame.degree - LIDAR_START_ANGLE);
  // Do checksum checking
  if (!checksum_is_valid(frame_data)) {
#ifdef DEBUG_LIDAR_SERIAL
        Serial.println("Wrong checksum, recalibrating");
#endif
    sync = false;
    return false;
  }
  // Store info for each angle
  for (size_t i = 0; i < DEGREES_PER_FRAME; ++i) {
    frame_out.angles[i].dist = frame_data.frame.angles[i].dist_m;
  }
  // We successfully read the frame
  return true;
}

void update_expected(uint8_t& expected, uint8_t last_angle) {
  expected = last_angle + 1;
  if (expected > LIDAR_END_ANGLE) {
    expected = LIDAR_START_ANGLE;
  }
}

// ----------------- START GLOBAL STATE -----------------
// Whether the serial from the LIDAR is synchronized
// (knows where it is and what to expect)
bool SYNCHRONIZED = false;
// The id of the last angle
uint8_t EXPECTED = 0;
// Allocate space for a bluetooth message
BluetoothPacket  BT_PACKET;
// ----------------- END GLOBAL STATE -----------------

void setup() {
  // Debug serial
#ifdef DEBUG
  Serial.begin(115200);
#endif
  // Start a bluetooth serial connection
  SerialOut.begin(115200);
  //SerialBT.setFIFOSize(BLUETOOTH_PACKET_SIZE);
  // Connect to the LIDAR
  SerialLidar.begin(230400);
  // Start the LIDAR
  SerialLidar.write((byte*)LIDAR_COMMAND_START, LIDAR_COMMAND_LEN);
}


void loop() {
  // Allocate space to read a frame
  LidarFrameData frame;
  // Initialize the packet
  BT_PACKET.fields.num_frames = 0;
  // Read the first frame
  size_t i = 0;
#ifdef VERBOSE
  Serial.print("Reading frame ");
  Serial.print(i);
  Serial.println();
  Serial.flush();
#endif
  // Check validity of the first frame
  if (read_frame(SYNCHRONIZED, frame, BT_PACKET.fields.frames[i], EXPECTED)) [[likely]] {
    // Increment the number of frames
    ++BT_PACKET.fields.num_frames;
  }
  else [[unlikely]] {
      // If frame is invalid, exit and retry the process
      return;
    }
    // The first frame sets the standard for subsequent frames
    update_expected(EXPECTED, frame.frame.degree);
  // Read the remaining frames
  for (i = 1; i < MAX_FRAMES_PER_MESSAGE; ++i) {
#ifdef VERBOSE
    Serial.print("Reading frame ");
    Serial.print(i);
    Serial.println();
#endif
    // Read the frame, storing data into the buffer
    // Check validity
    if (read_frame(SYNCHRONIZED, frame, BT_PACKET.fields.frames[i], EXPECTED)) {
      // Increment the number of frames
      ++BT_PACKET.fields.num_frames;
      // Adjust expected
      update_expected(EXPECTED, frame.frame.degree);
    }
    // If frame is invalid, we stop here
    else [[unlikely]] {
        break;
      }
    }
  // Now that we have enough for the message, write it over serial
  bluetooth_write(BT_PACKET);
}
