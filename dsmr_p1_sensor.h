#include "esphome.h"
#include "dsmr.h"

using namespace esphome;

#define P1_MAXTELEGRAMLENGTH 1500
#define DELAY_MS 5000 // Delay in miliseconds before reading another telegram
#define WAIT_FOR_DATA_MS 2000
#define OUTPUT_PIN 4 // GPIO4

// Use data structure according to: https://github.com/matthijskooijman/arduino-dsmr

using MyData = ParsedData <
  /* FixedValue */ energy_delivered_tariff1,
  /* FixedValue */ energy_delivered_tariff2,
  /* FixedValue */ energy_returned_tariff1,
  /* FixedValue */ energy_returned_tariff2,
  /* FixedValue */ power_delivered,
  /* FixedValue */ power_returned,
  /* FixedValue */ voltage_l1,
  /* FixedValue */ current_l1,
  /* FixedValue */ power_delivered_l1,
  /* FixedValue */ power_returned_l1,

	/* uint16_t */ 							gas_device_type,
  /* uint8_t */ 							gas_valve_position,
  /* TimestampedFixedValue */ gas_delivered,
  /* uint16_t */ 							thermal_device_type,
  /* uint8_t */ 							thermal_valve_position,
  /* TimestampedFixedValue */ thermal_delivered,
  /* uint16_t */ 							water_device_type,
  /* uint8_t */ 							water_valve_position,
  /* TimestampedFixedValue */ water_delivered,
  /* uint16_t */ 							slave_device_type,
  /* uint8_t */ 							slave_valve_position,
  /* TimestampedFixedValue */ slave_delivered,

  /* uint32_t */ 	electricity_failures,
  /* uint32_t */ 	electricity_long_failures,
  /* String */ 		electricity_failure_log,
  /* uint32_t */ 	electricity_sags_l1,
  /* uint32_t */ 	electricity_swells_l1
>;

class CustomP1UartComponent : public Component, public uart::UARTDevice {
	protected:
		char telegram[P1_MAXTELEGRAMLENGTH];
		char c;
		int telegramlen;
		bool headerfound;
		bool footerfound;
		int message_count = 0;
		unsigned long lastread;
		int bytes_read;

  bool data_available() {
		// See if there's data available.
		unsigned long currentMillis = millis();
		unsigned long previousMillis = currentMillis;

		while (currentMillis - previousMillis < WAIT_FOR_DATA_MS) { // wait in miliseconds
			currentMillis = millis();
			if (available()) {
				return true;
			}
		}
		return false;
  }

  bool read_message() {
		//ESP_LOGD("DmsrCustom","Read message");
		headerfound = false;
		footerfound = false;
		telegramlen = 0;
		bytes_read = 0;
		unsigned long currentMillis = millis();
		unsigned long previousMillis = currentMillis;

		if (available()) { // Check to be sure
			// Messages come in batches. Read until footer.
			while (!footerfound && currentMillis - previousMillis < 5000) { // Loop while there's no footer found with a maximum of 5 seconds
				currentMillis = millis();
				// Loop while there's data to read
				while (available()) { // Loop while there's data
					if (telegramlen >= P1_MAXTELEGRAMLENGTH) {  // Buffer overflow
						headerfound = false;
						footerfound = false;
						ESP_LOGD("DmsrCustom","Error: Message larger than buffer");
					}
					bytes_read++;
					c = read();
					if (c == 47) { // header: forward slash
						// ESP_LOGD("DmsrCustom","Header found");
						// ESP_LOGD("DmsrCustom",to_string(message_count).c_str());
						headerfound = true;
						telegramlen = 0;
					}
					if (headerfound) {
						telegram[telegramlen] = c;
						telegramlen++;
						if (c == 33) { // footer: exclamation mark
							// ESP_LOGD("DmsrCustom","Footer found");
							footerfound = true;
						} else {
							if (footerfound && c == 10) { // last \n after footer
								// Parse message
								MyData data;
								// ESP_LOGD("DmsrCustom","Parsing");
								// ESP_LOGD("DmsrCustom",telegram);
								ParseResult<void> res = P1Parser::parse(&data, telegram, telegramlen, false); // Parse telegram accoring to data definition. Ignore unknown values.
								message_count++;
								if (res.err) {
									// Parsing error, show it
									Serial.println(res.fullError(telegram, telegram + telegramlen));
								} else {
									publish_sensors(data);
									return true; // break out function
								}
							}
						}
					}
				} // While data available
			} // !footerfound
		}
		return false;
  }

  void publish_sensors(MyData data){
		if(data.energy_delivered_tariff1_present)s_energy_delivered_tariff1->publish_state(data.energy_delivered_tariff1);
		if(data.energy_delivered_tariff2_present)s_energy_delivered_tariff2->publish_state(data.energy_delivered_tariff2);
		if(data.energy_returned_tariff1_present)s_energy_returned_tariff1->publish_state(data.energy_returned_tariff1);
		if(data.energy_returned_tariff2_present)s_energy_returned_tariff2->publish_state(data.energy_returned_tariff2);
		if(data.power_delivered_present)s_power_delivered->publish_state(data.power_delivered);
		if(data.power_returned_present)s_power_returned->publish_state(data.power_returned);
		if(data.voltage_l1_present)s_voltage_l1->publish_state(data.voltage_l1);
		if(data.current_l1_present)s_current_l1->publish_state(data.current_l1);
		if(data.power_delivered_l1_present)s_power_delivered_l1->publish_state(data.power_delivered_l1);
		if(data.power_returned_l1_present)s_power_returned_l1->publish_state(data.power_returned_l1);

		if(data.gas_device_type_present)s_gas_device_type->publish_state(data.gas_device_type);
		if(data.gas_valve_position_present)s_gas_valve_position->publish_state(data.gas_valve_position);
		if(data.gas_delivered_present)s_gas_delivered->publish_state(data.gas_delivered);

		if(data.thermal_device_type_present)s_thermal_device_type->publish_state(data.thermal_device_type);
		if(data.thermal_valve_position_present)s_thermal_valve_position->publish_state(data.thermal_valve_position);
		if(data.thermal_delivered_present)s_thermal_delivered->publish_state(data.thermal_delivered);

		if(data.water_device_type_present)s_water_device_type->publish_state(data.water_device_type);
		if(data.water_valve_position_present)s_water_valve_position->publish_state(data.water_valve_position);
		if(data.water_delivered_present)s_water_delivered->publish_state(data.water_delivered);

		if(data.slave_device_type_present)s_slave_device_type->publish_state(data.slave_device_type);
		if(data.slave_valve_position_present)s_slave_valve_position->publish_state(data.slave_valve_position);
		if(data.slave_delivered_present)s_slave_delivered->publish_state(data.slave_delivered);

		if(data.electricity_failures_present)s_electricity_failures->publish_state(data.electricity_failures);
    if(data.electricity_long_failures_present)s_electricity_long_failures->publish_state(data.electricity_long_failures);
    if(data.electricity_failure_log_present)s_electricity_failure_log->publish_state(data.electricity_failure_log.c_str());
    if(data.electricity_sags_l1_present)s_electricity_sags_l1->publish_state(data.electricity_sags_l1);
    if(data.electricity_swells_l1_present)s_electricity_swells_l1->publish_state(data.electricity_swells_l1);
  };

  public:
		CustomP1UartComponent(UARTComponent *parent) : UARTDevice(parent) {}
			Sensor *s_energy_delivered_tariff1 = new Sensor();
			Sensor *s_energy_delivered_tariff2 = new Sensor();
			Sensor *s_energy_returned_tariff1 = new Sensor();
			Sensor *s_energy_returned_tariff2 = new Sensor();
			Sensor *s_electricity_tariff = new Sensor();
			Sensor *s_power_delivered = new Sensor();
			Sensor *s_power_returned = new Sensor();
			Sensor *s_electricity_threshold = new Sensor();
			Sensor *s_voltage_l1 = new Sensor();
			Sensor *s_current_l1 = new Sensor();
			Sensor *s_power_delivered_l1 = new Sensor();
			Sensor *s_power_returned_l1 = new Sensor();

			Sensor *s_gas_device_type = new Sensor();
			Sensor *s_gas_valve_position = new Sensor();
			Sensor *s_gas_delivered = new Sensor();

			Sensor *s_thermal_device_type = new Sensor();
			Sensor *s_thermal_valve_position = new Sensor();
			Sensor *s_thermal_delivered = new Sensor();

			Sensor *s_water_device_type = new Sensor();
			Sensor *s_water_valve_position = new Sensor();
			Sensor *s_water_delivered = new Sensor();

			Sensor *s_slave_device_type = new Sensor();
			Sensor *s_slave_valve_position = new Sensor();
			Sensor *s_slave_delivered = new Sensor();

			Sensor *s_electricity_failures = new Sensor();
			Sensor *s_electricity_long_failures = new Sensor();
			TextSensor *s_electricity_failure_log = new TextSensor();
			Sensor *s_electricity_sags_l1 = new Sensor();
			Sensor *s_electricity_swells_l1 = new Sensor();

			void setup() override {
				lastread = 0;
				pinMode(OUTPUT_PIN, OUTPUT); // Set OUTPUT_PIN as output pin
				digitalWrite(OUTPUT_PIN,LOW); // Set low, don't request message from P1 port
			}

			void loop() override {
			unsigned long now = millis();

			if (now - lastread > DELAY_MS || lastread == 0) {
				lastread = now;
				digitalWrite(OUTPUT_PIN,HIGH); // Set high, request new message from P1 port
				if (data_available()) { // Check for x seconds if there's data available
					bool have_message = read_message();
					if (have_message) {
						digitalWrite(OUTPUT_PIN,LOW); // Set low, stop requesting messages from P1 port
					} // If No message was read, keep output port high and retry later
				} else {
						ESP_LOGD("DmsrCustom","No data available. Is P1 port connected?");
				}
			}
		}
};
