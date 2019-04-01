// send fake MAG data
// begin parameters

#define SERIAL_PORT_BAUD_RATE                        115200
#define MAG_SAMPLE_RATE                                  50
//#define MAG_SAMPLE_RATE                                   5     // for SLOMO factor 10
//#define INDUCTION_VALUE_INTERVAL_DELAY_MICROS           120
#define INDUCTION_VALUE_INTERVAL_DELAY_MICROS             0

#define INDUCTION_VALUE_INTERVAL_CORRECTION_MILLIS  1
#define INDUCTION_VALUE_INTERVAL_CORRECTION_MICROS  200

#define IS_GENERATE_FAKE_INDUCTION_VALUES_CONST 0
#define IS_GENERATE_FAKE_INDUCTION_VALUES_SIN   1
#define IS_USE_BIG_ENDIAN                       1
#define FAKE_INDUCTION_VALUE_AMPLITUDE          (0x7fffff)    // MAX_INT24_T
//#define FAKE_INDUCTION_VALUE_AMPLITUDE          100
#define FAKE_INDUCTION_VALUE_PERIODE            10            // *2*pi*seconds

// end parameters, no need to change defines from here for configuration!

static_assert(IS_GENERATE_FAKE_INDUCTION_VALUES_CONST +
              IS_GENERATE_FAKE_INDUCTION_VALUES_SIN == 1,
              "Must select exactly 1 fake induction generation method!");

#define GET_BYTE_FROM_NUMBER(N, B) ( ( (N) >> (8 * (B)) ) & 0xFF  )
#define GET_MIN(A, B)               ( ( (A) < (B) ) ? (A) : (B) )
#define GET_MAX(A, B)               ( ( (A) > (B) ) ? (A) : (B) )

//#define INDUCTION_VALUE_INTERVAL_MICROS (1E6/MAG_SAMPLE_RATE-INDUCTION_VALUE_INTERVAL_DELAY_MICROS)
#define INDUCTION_VALUE_INTERVAL_MICROS (1000000/MAG_SAMPLE_RATE-INDUCTION_VALUE_INTERVAL_DELAY_MICROS)
#define INDUCTION_VALUE_INTERVAL_MILLI_PART (INDUCTION_VALUE_INTERVAL_MICROS / 1000)
#define INDUCTION_VALUE_INTERVAL_MICRO_PART (INDUCTION_VALUE_INTERVAL_MICROS % 1000)

#define INDUCTION_VALUE_INTERVAL_TOTAL_MICROS \
  (INDUCTION_VALUE_INTERVAL_MILLI_PART*1000 + INDUCTION_VALUE_INTERVAL_MICRO_PART \
  - INDUCTION_VALUE_INTERVAL_CORRECTION_MICROS)

//
#define INDUCTION_VALUE_INTERVAL_STEP_MILLIS \
  GET_MIN( (INDUCTION_VALUE_INTERVAL_MILLI_PART / 10), INDUCTION_VALUE_INTERVAL_CORRECTION_MILLIS)

//#define INDUCTION_VALUE_INTERVAL_STEP_MICROS  (INDUCTION_VALUE_INTERVAL_MICRO_PART / 100)
//#define INDUCTION_VALUE_INTERVAL_STEP_MICROS \
  GET_MAX( (INDUCTION_VALUE_INTERVAL_MICRO_PART / 100) , 10)
#define INDUCTION_VALUE_INTERVAL_STEP_MICROS 10  

#define INVALID_MAG_INDUCTION_VALUE           (0x7fffffff)  // MAX_INT32_T




int counter;          // counting sent values
uint32_t timestamp_micros;
// for scheduler
//uint32_t timestamp_sched; 
uint32_t timestamp_sched0; // timestamp 2 tacts before
uint32_t timestamp_sched1; // timestamp of last tact
//uint32_t timestamp_millis;

bool init_sending_mag_values() {
  counter = 0;
  Serial.begin(SERIAL_PORT_BAUD_RATE);
  // send transmission init sequence
  Serial.print("CSMAG");
  // fill init sequence to 8 Bytes for better readability in hex dump
  Serial.print("210");    
}

void alter_led_time(int time_ms, bool led_status) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, led_status ? HIGH : LOW);
  delay(time_ms);
  digitalWrite(LED_BUILTIN, !led_status ? HIGH : LOW);
}

void setup() {
  // put your setup code here, to run once:
  //delay(2000);        // give developer chance to turn on serial port sniffer
  alter_led_time(2000, true);
  init_sending_mag_values(); 
  //
  timestamp_sched0 = 0;
  timestamp_sched1 = 0;
}

int32_t generate_fake_induction_value() {
  int32_t induction_i = 0;
#if IS_GENERATE_FAKE_INDUCTION_VALUES_CONST
  induction_i = 42;
  //induction_i = 0xFEDCBA98;
#elif IS_GENERATE_FAKE_INDUCTION_VALUES_SIN
  induction_i = FAKE_INDUCTION_VALUE_AMPLITUDE * 
                  sin(timestamp_micros / 1e6 / FAKE_INDUCTION_VALUE_PERIODE);
#endif 
  return induction_i;
}

int serial_write_int32(int32_t n, bool is_big_endian) {
  //Serial.write(n & 0xff000000)
  int i;
  if (is_big_endian) {
    for (i = 3; i >= 0; i--) {
      Serial.write(GET_BYTE_FROM_NUMBER(n, i));
    }
  } else {
    // little endian: send LSB first
    for (i = 0; i < 4; i++) {
      Serial.write(GET_BYTE_FROM_NUMBER(n, i));
    }
  }
  return i;
}

bool send_induction_value(int32_t induction_i) {
  Serial.write('T');          // mark sending timestamp
  timestamp_micros = micros();       // get us timestamp
  //Serial.print("%ud", timestamp);
  serial_write_int32(timestamp_micros, IS_USE_BIG_ENDIAN);
  Serial.write('I');          // mark sending induction value
  //Serial.print("%d", induction_i);
  serial_write_int32(induction_i, IS_USE_BIG_ENDIAN);
  counter++;
  return true;
}

// delay millim ms AND micro us (to allow longer delays with 16 bit ints in us precision)
void delayMilliMicro(int milli, int micro) {
  delay(milli);
  delayMicroseconds(micro);
}

//void loop() {
//  int32_t induction_i;
//  // TODO: read real induction value here
//  induction_i = generate_fake_induction_value();
//  send_induction_value(induction_i);
//  //delayMicroseconds(INDUCTION_VALUE_INTERVAL_MICROS);
//  delayMilliMicro(INDUCTION_VALUE_INTERVAL_MILLI_PART, 
//                  INDUCTION_VALUE_INTERVAL_MICRO_PART);
//  //delay(1000);
//}

void loop() {
  timestamp_sched0 = timestamp_sched1;
  timestamp_sched1 = micros();
  int32_t induction_i;
  // TODO: read real induction value here
  induction_i = generate_fake_induction_value();
  send_induction_value(induction_i);  // also updates global timestamp_micros
  //delayMicroseconds(INDUCTION_VALUE_INTERVAL_MICROS);
  //delayMilliMicro(INDUCTION_VALUE_INTERVAL_MILLI_PART, INDUCTION_VALUE_INTERVAL_MICRO_PART);
  //delay(1000);

  
  // delay millisecond chunk
  while ( (millis() - timestamp_sched1/1000) < 
    (INDUCTION_VALUE_INTERVAL_MILLI_PART - INDUCTION_VALUE_INTERVAL_CORRECTION_MILLIS) ) {
      
    delay(INDUCTION_VALUE_INTERVAL_STEP_MILLIS);
  }
  

  // delay microsecond chunk
  while ( (micros() - timestamp_sched1) < INDUCTION_VALUE_INTERVAL_TOTAL_MICROS ) {
      
    delayMicroseconds(INDUCTION_VALUE_INTERVAL_STEP_MICROS);
  }

  // delay to synchronize with 2 tacts ago (to prevent from adding up delays)
  if (timestamp_sched0) {
    // if not in first cycle (timestamp_sched0 > 0)

    while ( (((uint32_t) micros()) - timestamp_sched0) < 
      (2L * INDUCTION_VALUE_INTERVAL_TOTAL_MICROS + INDUCTION_VALUE_INTERVAL_CORRECTION_MICROS) ) {
      delayMicroseconds(INDUCTION_VALUE_INTERVAL_STEP_MICROS);
    }
  }

  // works for 50 Hz
  /*
  while ( (millis() - timestamp_micros/1000) < (20 - 1) ) {
    delay(1);
  }
  while ( (micros() - timestamp_micros) < (20 * 1000 - 100) ) {
    delayMicroseconds(10);
  }
  */
}
