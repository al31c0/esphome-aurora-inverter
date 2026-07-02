// aurora_monitor.cpp
#include "aurora_monitor.h"
#include "esphome/core/log.h"

#if defined(USE_ESP32)
#include "soc/soc_caps.h"
// ESP32-S2 and C3 only have 2 UARTs, so Serial2 does not exist there
#if SOC_UART_NUM > 2
#define AURORA_SERIAL Serial2
#else
#define AURORA_SERIAL Serial1
#endif
#elif defined(USE_ESP8266)
#define AURORA_SERIAL Serial
#else
#error Unsupported device
#endif


namespace esphome {
namespace aurora_monitor {

static const char *TAG = "aurora_monitor";

void AuroraMonitor::register_sensor(sensor::Sensor *sensor, DSP_VALUE_TYPE type) {
  auto entry = new sensor_dsp{sensor, type};
  this->sensors_.push_back(entry);
}

void AuroraMonitor::register_cumulative_sensor(sensor::Sensor *sensor, CUMULATED_ENERGY_TYPE type) {
  auto entry = new sensor_cumulative{sensor, type};
  this->cumulative_sensors_.push_back(entry);
}

void AuroraMonitor::register_temperature_sensor(sensor::Sensor *sensor, DSP_VALUE_TYPE type) {
  auto entry = new sensor_dsp{sensor, type};
  this->temperature_sensors_.push_back(entry);
}

void AuroraMonitor::register_text_sensor(text_sensor::TextSensor *ts, INFO_TYPE type) {
  auto entry = new sensor_text{ts, type};
  this->text_sensors_.push_back(entry);
}

void AuroraMonitor::setup() {
  ESP_LOGD(TAG, "Setting up Aurora Monitor");
  ABBAurora::setup(AURORA_SERIAL, rx_pin_, tx_pin_, tx_control_pin_);
  inverter_ = new ABBAurora(address_);
}

void AuroraMonitor::update() {
  for (auto *s : this->text_sensors_) {
    enum INFO_TYPE type = s->type;
    text_sensor::TextSensor *sensor = s->sensor;    
    switch (type)
    {
    case CONNECTION_STATUS:
      sensor->publish_state(this->inverter_->SendStatus ? "CONNECTED" : "DISCONNECTED");
      break;
    case SYSTEM_PN:
      if (sensor->get_state().empty() && this->inverter_->ReadSystemPN())
        sensor->publish_state(this->inverter_->SystemPN.PN.c_str());
      break;
    case SYSTEM_SERIAL_NUMBER:
      if (sensor->get_state().empty() && this->inverter_->ReadSystemSerialNumber())
        sensor->publish_state(this->inverter_->SystemSerialNumber.SerialNumber.c_str());
      break;
    case FIRMWARE_RELEASE:
      if (sensor->get_state().empty() && this->inverter_->ReadFirmwareRelease())
        sensor->publish_state(this->inverter_->FirmwareRelease.Release.c_str());
      break;
    default:
      break;
    }
  }

  // regular DSP sensors
  for (auto *s : this->sensors_) {
    DSP_VALUE_TYPE type = s->type;
    sensor::Sensor *sensor = s->sensor;    

    if (this->inverter_->ReadDSPValue(type, MODULE_MEASUREMENT))
      sensor->publish_state(this->inverter_->DSP.Value);
  }

  // temperature sensors (scale by /10)
  for (auto *s : this->temperature_sensors_) {
    DSP_VALUE_TYPE type = s->type;
    sensor::Sensor *sensor = s->sensor;    

    if (this->inverter_->ReadDSPValue(type, MODULE_MEASUREMENT))
      sensor->publish_state(this->inverter_->DSP.Value / 10.0f);
  }

  // Cumulative sensors
  for (auto *s : this->cumulative_sensors_) {
    CUMULATED_ENERGY_TYPE type = s->type;
    sensor::Sensor *sensor = s->sensor;    

    if (this->inverter_->ReadCumulatedEnergy(type))
      sensor->publish_state(this->inverter_->CumulatedEnergy.Energy);
  }
}

void AuroraMonitor::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora Inverter Monitor:");
  ESP_LOGCONFIG(TAG, "  RX Pin: %u", this->rx_pin_);
  ESP_LOGCONFIG(TAG, "  TX Pin: %u", this->tx_pin_);
  ESP_LOGCONFIG(TAG, "  TX Control Pin: %u", this->tx_control_pin_);
  ESP_LOGCONFIG(TAG, "  Address: %u", this->address_);
}

}  // namespace aurora_monitor
}  // namespace esphome

