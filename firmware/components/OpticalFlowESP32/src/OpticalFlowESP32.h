#pragma once
#include <Arduino.h>
#include <SPI.h>

class OpticalFlowESP32 {
public:
  enum Model : uint8_t { AUTO_DETECT = 0, PMW3901 = 1, PAA5100 = 2 };

  // (A) Library-owned SPI (same behavior as before)
  explicit OpticalFlowESP32(uint8_t bus = 1);

  // Initialize by giving pinout (library will call spi.begin on its own bus)
  bool begin(int sck, int miso, int mosi, int cs,
             Model model = AUTO_DETECT, char orientation = 'N');

  // (B) NEW: Externally-owned SPI
  explicit OpticalFlowESP32(SPIClass& extSpi);

  // Initialize by passing an already-begun SPI + the CS/model/orientation
  bool begin(SPIClass& spi, int cs,
             Model model = AUTO_DETECT, char orientation = 'N');

  // Read motion using the simple register method (always returns a reading)
  void readMotionSimple(int16_t& dx, int16_t& dy);

  // Read motion using burst (returns true when a "good" frame is available)
  bool readMotionBurst(int16_t& dx, int16_t& dy, uint8_t* quality = nullptr);

  // Optionally change orientation at runtime
  void setOrientation(char o);

  // Optionally control LED (supported on PAA5100 sequences in your code)
  void setLed(bool on);

  // Access the probed IDs (set during begin)
  uint8_t chipId() const     { return _chipId; }
  uint8_t revision() const   { return _revision; }
  uint8_t inverseId() const  { return _inverseId; }

private:
  // Common device initialization (moved from old begin)
  bool _initDevice(Model model, char orientation);

  // Low-level helpers (SPI transactions + CS)
  inline uint8_t r(uint8_t reg);
  inline void    w(uint8_t reg, uint8_t val);
  uint8_t rAt(const SPISettings& cfg, uint8_t reg);
  void    wAt(const SPISettings& cfg, uint8_t reg, uint8_t val);

  // Setup sequences (from your originals)
  void secretSaucePrelude();
  void initRegistersPMW3901();
  void initRegistersPAA5100();

  // Internal state
  SPIClass*   _spi      = nullptr; // now a pointer so we can own or borrow
  bool        _ownsSpi  = false;   // true when we allocated the SPIClass
  uint8_t     _bus      = 1;       // only meaningful when we own SPI

  // SPI settings & pins
  SPISettings _cfgId;              // slow probe
  SPISettings _cfgRun;             // run speed
  int         _sck = -1, _miso = -1, _mosi = -1, _cs = -1;

  // Model/orientation
  Model       _model = AUTO_DETECT;
  char        _orientation = 'S';

  // IDs captured at begin()
  uint8_t     _chipId = 0, _revision = 0, _inverseId = 0;
};
