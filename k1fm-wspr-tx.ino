#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <si5351.h>
#include <JTEncode.h>

#define WSPR_TONE_SPACING       146           // ~1.46 Hz
#define WSPR_DELAY              683          // Delay value for WSPR
#define WSPR_DEFAULT_FREQ       14095600UL
#define FREQ_CALIBRATION        1780UL

#define BUTTON                  7
#define LED_PIN                 10

Si5351 si5351;
JTEncode jtencode;

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

unsigned long freq;
char call[] = "K1FM";
char loc[] = "FN30";
uint8_t dbm = 5;
uint8_t tx_buffer[255];
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;

void setup()
{
  Serial.println(F("Starting"));
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON, INPUT_PULLUP);

  freq = WSPR_DEFAULT_FREQ + FREQ_CALIBRATION;
  symbol_count = WSPR_SYMBOL_COUNT;
  tone_spacing = WSPR_TONE_SPACING;
  tone_delay = WSPR_DELAY;

  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially
  
  set_tx_buffer();
  
  Serial.begin(115200);
  ss.begin(GPSBaud);
}

void loop()
{ 
  if (1) {  // Tuning
  //if (gps.location.isValid()) {
    TinyGPSTime t = gps.time;
    uint8_t seconds = t.second();
    uint8_t minutes = t.minute();
    if ( (minutes %2 == 0) and seconds <= 1) {
      Serial.println(F("Transmit"));
      ss.end();
      si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
      si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially
      set_tx_buffer();
      encode();
      ss.begin(GPSBaud);
    } else {
      calcLocator(loc, gps.location.lat(), gps.location.lng());
      Serial.print(F("calculated locator: "));
      Serial.print(loc);
      Serial.print(F(" "));
      Serial.print(minutes);
      Serial.print(F(" minutes "));
      Serial.print(seconds);
      Serial.println(F(" seconds"));
    }
  } else {
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    printDateTime(gps.date, gps.time);
    Serial.println(F("Invalid fix"));
  }
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println("No GPS data received: check wiring");
}


void encode()
{
  uint8_t i;

  // Reset the tone to the base frequency and turn on the output
  si5351.output_enable(SI5351_CLK0, 0);
  // si5351.output_enable(SI5351_CLK0, 1);
  digitalWrite(LED_PIN, HIGH);

  for(i = 0; i < symbol_count; i++)
  {
      si5351.set_freq((freq * 100) + (tx_buffer[i] * tone_spacing), SI5351_CLK0);
      delay(tone_delay);
  }
  
  // Turn off the output
  si5351.output_enable(SI5351_CLK0, 0); 
  digitalWrite(LED_PIN, LOW);
}

void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);
  jtencode.wspr_encode(call, loc, dbm, tx_buffer);
}

void calcLocator(char *dst, double lat, double lon) {
  int o1, o2, o3;
  int a1, a2, a3;
  double remainder;
  // longitude
  remainder = lon + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (double)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  remainder = remainder - 2.0 * (double)o2;
  o3 = (int)(12.0 * remainder);

  // latitude
  remainder = lat + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (double)a1 * 10.0;
  a2 = (int)(remainder);
  remainder = remainder - (double)a2;
  a3 = (int)(24.0 * remainder);
  dst[0] = (char)o1 + 'A';
  dst[1] = (char)a1 + 'A';
  dst[2] = (char)o2 + '0';
  dst[3] = (char)a2 + '0';
  dst[4] = (char)0;
  //dst[4] = (char)o3 + 'A';
  //dst[5] = (char)a3 + 'A';
  //dst[6] = (char)0;
}


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

