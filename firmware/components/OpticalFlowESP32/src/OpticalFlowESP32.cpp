#include "OpticalFlowESP32.h"

// ====== Public ======

/* (A) Library-owned SPI */
OpticalFlowESP32::OpticalFlowESP32(uint8_t bus)
: _ownsSpi(true),
  _bus(bus),
  _cfgId(250000, MSBFIRST, SPI_MODE3),     // matches your sketch
  _cfgRun(1000000, MSBFIRST, SPI_MODE3)    // matches your sketch
{
  _spi = new SPIClass(_bus);
}

/* (B) Externally-owned SPI */
OpticalFlowESP32::OpticalFlowESP32(SPIClass& extSpi)
: _ownsSpi(false),
  _bus(1),                                  // not used when we borrow SPI
  _spi(&extSpi),
  _cfgId(250000, MSBFIRST, SPI_MODE3),
  _cfgRun(1000000, MSBFIRST, SPI_MODE3)
{}

/* Begin: library owns SPI, given pinout */
bool OpticalFlowESP32::begin(int sck, int miso, int mosi, int cs,
                             Model model, char orientation)
{
  _sck = sck; _miso = miso; _mosi = mosi; _cs = cs;
  _model = model; _orientation = orientation;

  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  // IMPORTANT: do NOT pass CS to begin(); we toggle CS in software.
  if (_ownsSpi && _spi) {
    _spi->begin(_sck, _miso, _mosi);
    _spi->setHwCs(false);  // belt & braces: disable hardware CS
  }

  return _initDevice(_model, _orientation);
}

/* Begin: external SPI already begun in the sketch */
bool OpticalFlowESP32::begin(SPIClass& spi, int cs,
                             Model model, char orientation)
{
  _spi = &spi; _ownsSpi = false;
  _cs = cs; _model = model; _orientation = orientation;

  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  if (_spi) _spi->setHwCs(false); // ensure hardware CS won't toggle

  return _initDevice(_model, _orientation);
}

// ====== Public: motion APIs ======

void OpticalFlowESP32::readMotionSimple(int16_t& dx, int16_t& dy) {
  (void)r(0x02); // latch
  uint8_t xl = r(0x03);
  uint8_t xh = r(0x04);
  uint8_t yl = r(0x05);
  uint8_t yh = r(0x06);
  dx = (int16_t)((int16_t)xh << 8 | xl);
  dy = (int16_t)((int16_t)yh << 8 | yl);
}

bool OpticalFlowESP32::readMotionBurst(int16_t& dx, int16_t& dy, uint8_t* quality) {
  const uint8_t REG_MOTION_BURST = 0x16;

  _spi->beginTransaction(_cfgRun);
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);
  _spi->transfer(REG_MOTION_BURST);

  // 12 bytes: DR, OBS, DX_L, DX_H, DY_L, DY_H, SQUAL, RSUM, RMAX, RMIN, SHUT_H, SHUT_L
  uint8_t b[12];
  for (int i = 0; i < 12; ++i) b[i] = _spi->transfer(0x00);

  delayMicroseconds(50);
  digitalWrite(_cs, HIGH);
  _spi->endTransaction();

  uint8_t dr       = b[0];
  int16_t x        = (int16_t)((int16_t)b[3] << 8 | b[2]);
  int16_t y        = (int16_t)((int16_t)b[5] << 8 | b[4]);
  uint8_t squal    = b[6];
  uint8_t shutterH = b[10];

  if (quality) *quality = squal;

  // Same gating as your sketch: require DataReady and reject poor frames
  if ((dr & 0x80) && !(squal < 0x19 && shutterH == 0x1F)) {
    dx = x; dy = y;
    return true;
  }
  return false;
}

void OpticalFlowESP32::setOrientation(char o) {
  // From your original mapping
  bool invert_x = true, invert_y = true, swap_xy = true;
  switch (o) {
    case 'S': invert_x = true;  invert_y = true;  swap_xy = true;  break;
    case 'E': invert_x = false; invert_y = true;  swap_xy = false; break;
    case 'N': invert_x = false; invert_y = false; swap_xy = true;  break;
    case 'W': invert_x = true;  invert_y = false; swap_xy = false; break;
  }
  uint8_t v = 0;
  if (swap_xy) v |= 0b10000000;
  if (invert_y) v |= 0b01000000;
  if (invert_x) v |= 0b00100000;
  w(0x5B, v);
}

void OpticalFlowESP32::setLed(bool on) {
  // From your original; bank select then drive LED reg
  w(0x7F, 0x14);
  w(0x6F, on ? 0x1C : 0x00);
  w(0x7F, 0x00);
  delay(2);
}

// ====== Private: device init (moved from old begin) ======

bool OpticalFlowESP32::_initDevice(Model model, char orientation)
{
  // Reset + slow ID probe (identical timing to your sketch)
  wAt(_cfgId, 0x3A, 0x5A);   // POWER_UP_RESET
  delay(5);
  _chipId    = rAt(_cfgId, 0x00);
  _revision  = rAt(_cfgId, 0x01);
  _inverseId = rAt(_cfgId, 0x5F);

  // Same acceptance logic: product 0x49; inverse 0xB8/0xB6; rev 0x00/0x01
  const bool inv_ok = (_inverseId == 0xB8) || (_inverseId == 0xB6);
  const bool rev_ok = (_revision == 0x00) || (_revision == 0x01);
  if (_chipId != 0x49 || !inv_ok || !rev_ok) {
    if (_chipId != 0x49) return false;
    // (Keep going if inverse/rev don't match; mirrors your original behavior.)
  }

  // Clear motion registers once
  (void)r(0x02); (void)r(0x03); (void)r(0x04);
  (void)r(0x05); (void)r(0x06);
  delay(1);

  // Prelude + init sequences (unchanged)
  secretSaucePrelude(); // tables below are all from your file. :contentReference[oaicite:4]{index=4}

  if (model == PAA5100) {
    initRegistersPAA5100();  // :contentReference[oaicite:5]{index=5}
  } else {
    initRegistersPMW3901();  // default first, as in your code. :contentReference[oaicite:6]{index=6}
  }

  // Post-init sanity (non-fatal reads)
  (void)r(0x00);
  (void)r(0x01);

  setOrientation(orientation);
  return true;
}

// ====== Private: low-level ======

inline uint8_t OpticalFlowESP32::r(uint8_t reg) {
  reg &= ~0x80u;  // read
  _spi->beginTransaction(_cfgRun);
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);
  _spi->transfer(reg);
  delayMicroseconds(50);
  uint8_t v = _spi->transfer(0x00);
  delayMicroseconds(100);
  digitalWrite(_cs, HIGH);
  _spi->endTransaction();
  return v;
}

inline void OpticalFlowESP32::w(uint8_t reg, uint8_t val) {
  reg |= 0x80u;   // write
  _spi->beginTransaction(_cfgRun);
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);
  _spi->transfer(reg);
  _spi->transfer(val);
  delayMicroseconds(50);
  digitalWrite(_cs, HIGH);
  _spi->endTransaction();
  delayMicroseconds(200);
}

uint8_t OpticalFlowESP32::rAt(const SPISettings& cfg, uint8_t reg) {
  reg &= ~0x80u;
  _spi->beginTransaction(cfg);
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);
  _spi->transfer(reg);
  delayMicroseconds(50);
  uint8_t v = _spi->transfer(0x00);
  delayMicroseconds(100);
  digitalWrite(_cs, HIGH);
  _spi->endTransaction();
  return v;
}

void OpticalFlowESP32::wAt(const SPISettings& cfg, uint8_t reg, uint8_t val) {
  reg |= 0x80u;
  _spi->beginTransaction(cfg);
  digitalWrite(_cs, LOW);
  delayMicroseconds(50);
  _spi->transfer(reg);
  _spi->transfer(val);
  delayMicroseconds(50);
  digitalWrite(_cs, HIGH);
  _spi->endTransaction();
  delayMicroseconds(200);
}

// ====== Private: setup sequences (copied from your working sketch) ======

void OpticalFlowESP32::secretSaucePrelude() {
  // This whole block matches your file. :contentReference[oaicite:7]{index=7}
  w(0x7F, 0x00);
  w(0x55, 0x01);
  w(0x50, 0x07);
  w(0x7F, 0x0E);
  w(0x43, 0x10);

  int temp = r(0x67);
  if (temp & 0b10000000) w(0x48, 0x04);
  else                   w(0x48, 0x02);

  w(0x7F, 0x00);
  w(0x51, 0x7B);
  w(0x50, 0x00);
  w(0x55, 0x00);
  w(0x7F, 0x0E);

  temp = r(0x73);
  if (temp == 0x00) {
    int c1 = r(0x70);
    int c2 = r(0x71);
    if (c1 <= 28) c1 += 14;
    if (c1 >  28) c1 += 11;
    c1 = max(0, min(0x3F, c1));
    c2 = (c2 * 45) / 100;     // corrected division per your note
    w(0x7F, 0x00);
    w(0x61, 0xAD);
    w(0x51, 0x70);
    w(0x7F, 0x0E);
    w(0x70, (uint8_t)c1);
    w(0x71, (uint8_t)c2);
  }
}

void OpticalFlowESP32::initRegistersPMW3901() {
  // Exact sequence from your file. :contentReference[oaicite:8]{index=8}
  w(0x7F, 0x00);
  w(0x61, 0xAD);
  w(0x7F, 0x03);
  w(0x40, 0x00);
  w(0x7F, 0x05);

  w(0x41, 0xB3);
  w(0x43, 0xF1);
  w(0x45, 0x14);
  w(0x5B, 0x32);
  w(0x5F, 0x34);
  w(0x7B, 0x08);
  w(0x7F, 0x06);
  w(0x44, 0x1B);
  w(0x40, 0xBF);
  w(0x4E, 0x3F);
  w(0x7F, 0x08);
  w(0x65, 0x20);
  w(0x6A, 0x18);

  w(0x7F, 0x09);
  w(0x4F, 0xAF);
  w(0x5F, 0x40);
  w(0x48, 0x80);
  w(0x49, 0x80);

  w(0x57, 0x77);
  w(0x60, 0x78);
  w(0x61, 0x78);
  w(0x62, 0x08);
  w(0x63, 0x50);
  w(0x7F, 0x0A);
  w(0x45, 0x60);
  w(0x7F, 0x00);
  w(0x4D, 0x11);

  w(0x55, 0x80);
  w(0x74, 0x1F);
  w(0x75, 0x1F);
  w(0x4A, 0x78);
  w(0x4B, 0x78);

  w(0x44, 0x08);
  w(0x45, 0x50);
  w(0x64, 0xFF);
  w(0x65, 0x1F);
  w(0x7F, 0x14);
  w(0x65, 0x60);
  w(0x66, 0x08);
  w(0x63, 0x78);
  w(0x7F, 0x15);
  w(0x48, 0x58);
  w(0x7F, 0x07);
  w(0x41, 0x0D);
  w(0x43, 0x14);

  w(0x4B, 0x0E);
  w(0x45, 0x0F);
  w(0x44, 0x42);
  w(0x4C, 0x80);
  w(0x7F, 0x10);
  w(0x5B, 0x02);
  w(0x7F, 0x07);
  w(0x40, 0x41);
  w(0x70, 0x00);

  delay(100);
  w(0x32, 0x44);
  w(0x7F, 0x07);
  w(0x40, 0x40);
  w(0x7F, 0x06);
  w(0x62, 0xF0);
  w(0x63, 0x00);
  w(0x7F, 0x0D);
  w(0x48, 0xC0);
  w(0x6F, 0xD5);
  w(0x7F, 0x00);

  w(0x5B, 0xA0);
  w(0x4E, 0xA8);
  w(0x5A, 0x50);
  w(0x40, 0x80);
}

void OpticalFlowESP32::initRegistersPAA5100() {
  // Exact sequence from your file. :contentReference[oaicite:9]{index=9}
  w(0x7F, 0x00);
  w(0x61, 0xAD);

  w(0x7F, 0x03);
  w(0x40, 0x00);

  w(0x7F, 0x05);
  w(0x41, 0xB3);
  w(0x43, 0xF1);
  w(0x45, 0x14);

  w(0x5F, 0x34);
  w(0x7B, 0x08);
  w(0x5E, 0x34);
  w(0x5B, 0x11);
  w(0x6D, 0x11);
  w(0x45, 0x17);
  w(0x70, 0xE5);
  w(0x71, 0xE5);

  w(0x7F, 0x06);
  w(0x44, 0x1B);
  w(0x40, 0xBF);
  w(0x4E, 0x3F);

  w(0x7F, 0x08);
  w(0x66, 0x44);
  w(0x65, 0x20);
  w(0x6A, 0x3A);
  w(0x61, 0x05);
  w(0x62, 0x05);

  w(0x7F, 0x09);
  w(0x4F, 0xAF);
  w(0x5F, 0x40);
  w(0x48, 0x80);
  w(0x49, 0x80);
  w(0x57, 0x77);
  w(0x60, 0x78);
  w(0x61, 0x78);
  w(0x62, 0x08);
  w(0x63, 0x50);

  w(0x7F, 0x0A);
  w(0x45, 0x60);

  w(0x7F, 0x00);
  w(0x4D, 0x11);
  w(0x55, 0x80);
  w(0x74, 0x21);
  w(0x75, 0x1F);
  w(0x4A, 0x78);
  w(0x4B, 0x78);
  w(0x44, 0x08);

  w(0x45, 0x50);
  w(0x64, 0xFF);
  w(0x65, 0x1F);

  w(0x7F, 0x14);
  w(0x65, 0x67);
  w(0x66, 0x08);
  w(0x63, 0x70);
  w(0x6F, 0x1C);

  w(0x7F, 0x15);
  w(0x48, 0x48);

  w(0x7F, 0x07);
  w(0x41, 0x0D);
  w(0x43, 0x14);
  w(0x4B, 0x0E);
  w(0x45, 0x0F);
  w(0x44, 0x42);
  w(0x4C, 0x80);

  w(0x7F, 0x10);
  w(0x5B, 0x02);

  w(0x7F, 0x07);
  w(0x40, 0x41);

  delay(100);

  w(0x7F, 0x00);
  w(0x32, 0x00);

  w(0x7F, 0x07);
  w(0x40, 0x40);

  w(0x7F, 0x06);
  w(0x68, 0xF0);
  w(0x69, 0x00);

  w(0x7F, 0x0D);
  w(0x48, 0xC0);
  w(0x6F, 0xD5);

  w(0x7F, 0x00);
  w(0x5B, 0xA0);
  w(0x4E, 0xA8);
  w(0x5A, 0x90);
  w(0x40, 0x80);
  w(0x73, 0x1F);

  delay(100);
  w(0x73, 0x00);
}
