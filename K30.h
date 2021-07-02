// Senseair K30 library using serial (modbus)

template <typename ISerial> class K30 {
public:
  K30(){};
  k30_err_t begin(ISerial& serial, uint8_t tx_pin, uint8_t rx_pin);
};