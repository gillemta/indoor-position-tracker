#include "dw3000.h"
#include <WiFi.h>
#include <chrono>
#include <map>

// Define the pins used for the UWB module
#define PIN_RST 27  // Reset Pin
#define PIN_IRQ 34  // Interrupt Request pin
#define PIN_SS 4    // Chip Select pin for SPI

// Delays
#define RNG_DELAY_MS 1000   // Range Delay
#define TX_ANT_DLY 16385  // Transmit antenna delay
#define RX_ANT_DLY 16385  // Receive antenna delay
#define POLL_TX_TO_RESP_RX_DLY_UUS 240 // Delay between poll transmission and listening for response
#define RESP_RX_TIMEOUT_UUS 400 // The response reception timeout
#define TIME_SLOT_DLY 5000

// Indexes
#define ALL_MSG_SN_IDX 2    // Sequence number index
#define RESP_MSG_POLL_RX_TS_IDX 10  // Timestamp of message reception by anchor index
#define RESP_MSG_RESP_TX_TS_IDX 14  // Timestamp of message transmission by anchor index
#define ANCHOR_ID_IDX 18    // Anchor Id index
#define POS_START_IDX 19    // Start of Position Index

// Other
#define MAX_ANCHORS 3
#define MAX_TAGS 2
#define ALL_MSG_COMMON_LEN 10

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

// Message to be sent by the tag when initiating a ranging process ("poll" message)
static uint8_t tx_poll_msg[] = {
    0x41, 0x88, // Frame header
    0,          // Sequence number placeholder
    0xCA, 0xDE, // Example addressing information
    'W', 'A', 'V', 'E', // Custom payload
    0xE0,       // Function code for poll message
    0, 0        // Additional payload or placeholder
};

// Template for the expected response message after sending a poll
static uint8_t rx_resp_msg[] = {
    0x41, 0x88, // Frame header
    0,          // Sequence number placeholder
    0xCA, 0xDE, // Example addressing information
    'V', 'E', 'W', 'A', // Custom payload
    0xE1,       // Function code for response message
    // Timestamps and other information will be filled here when received
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    // Insert Position Data Here (12 bytes required)
    0, 0, 0, 0, // Position.x
    0, 0, 0, 0, // Position.y
    0, 0, 0, 0 // Position.z
};

static uint8_t tx_resp_msg[] = {
  0x41, 0x88, // Frame header
  0,          // Sequence number placeholder
  0xCA, 0xDE, // Example addressing information
  'V', 'E', 'W', 'A', // Custom payload
  0xE2,       // Function code for response message
  // Remaining bytes
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  // Insert Position Data Here (12 bytes required)
  0, 0, 0, 0, // Position.x
  0, 0, 0, 0, // Position.y
  0, 0, 0, 0 // Position.z
};

// Define a strucute for Position, assuming 3D coordinates (x, y, z)
struct Position {
  float x, y, z;
};

byte tagID = 0;
int anchorIds[MAX_ANCHORS] = {132, 28, 24};
// int anchorIds[MAX_ANCHORS] = {28};

// Stores distance from each anchor to tag
double tagPosition[3];

// Variable to keep track of the frame sequence number
static uint8_t frame_seq_nb = 0;

// Buffer to store incoming messages
static uint8_t rx_buffer[33];

// Variable to store the status register value of the UWB module
static uint32_t status_reg = 0;

// // Variables for Time of Flight and distance calculation
static double tof;      // Time of Flight
static double distance; // Calculated distance based on the Time of Flight

// Variable used for saftey bubble distance
static double saftey_bubble = 1.75; // TODO: Update this value after distance has been calculated to better match the output from that

// External configuration for UWB transmission parameters
extern dwt_txconfig_t txconfig_options;

std::map<int, int> tagMappings = {
    {184, 0},
    {196, 1}
};

void getID() {
  byte mac[6];
  WiFi.macAddress(mac);
  int lastByteOfMac = mac[5];

  if (tagMappings.find(lastByteOfMac) != tagMappings.end()) {
    tagID = tagMappings[lastByteOfMac];
  } else {
    Serial.printf("Device %d hasn't been configured\n", lastByteOfMac);
  }
}

void setup() {
  // Initialize UART for debugging
  UART_init();

  // Start serial communication for debugging or data output
  Serial.begin(115200);

  // Initialize SPI communication with UWB module
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  // Short delay to allow UWB module to startup
  delay(2);

  // Check if UWB module is in idle state, halt if not
  while (!dwt_checkidlerc())
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ; // Infinite loop on failure
  }

  // Initialize the UWB module, halt on faillure
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ; // Infinite loop on failure
  }

  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }
  
  // Configure transmission spectrum parameters
  dwt_configuretxrf(&txconfig_options);

  // Set antenna delays for accurate time-of-flight calculation
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  // Set delay and timeout for response after transmission
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  delay(1000);
  getID();
}

uint64_t timeSinceEpochMillisec() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void loop() {
  uint64_t currentTime = timeSinceEpochMillisec();

  unsigned long activeTimeSlotLength = 4500; // 4 seconds active
  unsigned long overlapDuration = 1500; // 1 second overlap
  unsigned long totalCycleDuration = MAX_TAGS * (activeTimeSlotLength + overlapDuration);

  unsigned long timeSlotStart = tagID * (activeTimeSlotLength + overlapDuration);
  unsigned long overlapStart = timeSlotStart + activeTimeSlotLength;
  unsigned long nextTimeSlotStart = timeSlotStart + activeTimeSlotLength + overlapDuration;

  // Check if we are in the current tag's time slot
  if (currentTime % totalCycleDuration >= timeSlotStart && currentTime % totalCycleDuration < overlapStart) {
    for (int i = 0; i < MAX_ANCHORS; i++) {
      sendPollMessage();

      delay(200);

      if (waitForResponse()) {
        processResponse();
      }
      delay(300);
    }
  } else {
      // Calculate the current position of the tag
      Position tagPosition = calculatePosition();
      // Broadcast the current position to other tags
      broadcastPosition(tagPosition);

      // Receive positions from other tags and store them
      Position otherTagsPositions[MAX_TAGS]; // MAX_TAGS should be defined based on the systems capacity
      receivePositions(tagPosition);
  }
}

void sendPollMessage() {
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}

bool waitForResponse() {
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {
  };

  frame_seq_nb++;

  return (status_reg & SYS_STATUS_RXFCG_BIT_MASK) != 0;
}

void processResponse() {
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

  if (frame_len <= sizeof(rx_buffer)) {
    dwt_readrxdata(rx_buffer, frame_len, 0);
  } else {
    return;
  }

  rx_buffer[ALL_MSG_SN_IDX] = 0;
  
  if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
    byte receivedAnchorId = rx_buffer[ANCHOR_ID_IDX];
    
    for (int i = 0; i < MAX_ANCHORS; i++) {
      if (anchorIds[i] == receivedAnchorId) {
        tagPosition[i] = getAnchorDistance(receivedAnchorId);
        break;
      }
    }
  } else {
    return;
  }
}

// Get Distance from tag to anchor
double getAnchorDistance(int anchorId) {
  uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
  int32_t rtd_init, rtd_resp;
  float clockOffsetRatio;

  /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
  poll_tx_ts = dwt_readtxtimestamplo32();
  resp_rx_ts = dwt_readrxtimestamplo32();

  /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
  clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

  /* Get timestamps embedded in response message. */
  resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
  resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

  /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
  rtd_init = resp_rx_ts - poll_tx_ts;
  rtd_resp = resp_tx_ts - poll_rx_ts;

  tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
  distance = tof * SPEED_OF_LIGHT;

  if (distance < 0) {
    distance = 0.00;
  }

  return distance;
}

// Calculate the position of the tag using UWB signals
Position calculatePosition() {
  float x, y, z;
  
  x = tagPosition[0];
  y = tagPosition[1];
  z = tagPosition[2];
  
  return {x, y, z};
}

// Broadcast the tag's position to other tags using UWB
void broadcastPosition(Position position) {
  memcpy(tx_resp_msg + POS_START_IDX, &position.x, sizeof(float));
  memcpy(tx_resp_msg + POS_START_IDX + sizeof(float), &position.y, sizeof(float));
  memcpy(tx_resp_msg + POS_START_IDX + 2 * sizeof(float), &position.z, sizeof(float));

  // Set the sequence number and send the message
  tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb++;
  dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
  dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}

// Receive position from other tags
void receivePositions(Position myPosition) {
  bool messageReceived = false;

  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
    // Loop until frame is received, a timeout occurs, or an error is detected
  };

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    int frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;

    if (frame_len == sizeof(rx_resp_msg)) {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
      messageReceived = true;
    }
  } else {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  }

  if (messageReceived) {
    Position receivedPosition;
    double tagDistance;

    memcpy(&receivedPosition.x, rx_buffer + POS_START_IDX, sizeof(float));
    memcpy(&receivedPosition.y, rx_buffer + POS_START_IDX + sizeof(float), sizeof(float));
    memcpy(&receivedPosition.z, rx_buffer + POS_START_IDX + 2 * sizeof(float), sizeof(float));

    calculateDistance(myPosition, receivedPosition);
  }
}

// Calculate the distance to another tag based on positions
void calculateDistance(Position myPosition, Position otherPosition) {

  // This returns the distance formula for 3 points. It first subtracts each value (x,y,z) from the other, and squares those. Then adds all the squares together, before taking the
  // absolute value and getting the square root. It returns a distance to the hundreths position.
  double distance =  sqrt( abs( pow( (otherPosition.x - myPosition.x), 2 ) + pow( (otherPosition.y - myPosition.y), 2 ) + pow( (otherPosition.z - myPosition.z), 2 ) ));
  Serial.print("The distance is: ");
  Serial.println(distance);

  if (isWithinSafetyBubble(distance)) {
    triggerAlert();
  }
  else{
    stopAlert();
  }
}

// Check if another tag is within the predefined safety bubble
bool isWithinSafetyBubble(double distance) {
  if(distance < saftey_bubble){
    return true;
  }
  return false;
  
}

// Trigger an alert mechanism
void triggerAlert() {

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK); 

}

// Stop the alert mechanism
void stopAlert(){

  dwt_setleds(DWT_LEDS_DISABLE);

}