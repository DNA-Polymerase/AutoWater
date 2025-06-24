## Pin Diagram (Arduino Uno)

| Arduino Pin | Function                      | Connected Component       |
|-------------|-------------------------------|--------------------------|
| A0          | Soil Moisture Sensor (read)   | Soil Sensor OUT          |
| 7           | Soil Sensor VCC (power)       | Soil Sensor VCC          |
| A1          | Tank Level Sensor (read)      | Tank Sensor OUT          |
| 6           | Tank Sensor VCC (power)       | Tank Sensor VCC          |
| 8           | Pump Control                  | Relay/Transistor IN      |
| 2           | Manual Button (INPUT_PULLUP)  | Button (one side to GND) |
| A4 (SDA)    | I2C Data                      | LCD, RTC                 |
| A5 (SCL)    | I2C Clock                     | LCD, RTC                 |

**Note:**
- For the sensors, VCC is switched via digital pins 6 (tank) and 7 (soil) to increase sensor lifespan.
- I2C LCD and RTC modules share the SDA/SCL pins (A4/A5).
- Manual Button should be wired between pin 2 and GND.

