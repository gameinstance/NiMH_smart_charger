/*
 * NiMH dual cell smart charger
 * using the Arduino328p
 * 
 * GameInstance.com
 * 2017
 */
#include "NTCThermistor.h"

static const byte V_BAT0 = A1;
static const byte V_BAT1 = A0;
static const byte I_BAT0 = A6;
static const byte I_BAT1 = A7;
static const byte NTC0 = A3;
static const byte NTC1 = A2;

static const byte CH_PWM0 = 9;
static const byte CH_PWM1 = 10;
static const byte DSCH_0 = 12;
static const byte DSCH_1 = 11;
static const byte LED_0 = 13;
static const byte LED_1 = 100; // NOT USED

static const float I_HIGH_CURRENT_CHARGING = 0.8; // amps
static const float I_LOW_CURRENT_CHARGING = 0.24; // amps
static const float V_BAT_MIN = 0.5; // volts
static const float V_BAT_CHARGE = 1.0; // volts
static const float V_BAT_MAX_NIMH = 1.475; // volts
static const float V_BAT_MAX = 1.8; // volts
static const float VOUT_MAX = 4.0; // volts
static const float VCC = 5.04; //4.72; //5.00; // volts
static const float SHUNT_RESISTOR = 1.8; // ohms

static const byte LED_NONE = 0;
static const byte LED_WAITING = 1;
static const byte LED_DISCHARGING = 2;
static const byte LED_CHARGING = 3;
static const byte LED_READY = 4;
static const byte LED_ERROR = 5;


void ConfigPWM() {
  // fast PWM non-inverting and no prescaling
  // using 2047 as the max counter value
  // on pins 9 and 10
  DDRB |= _BV(PB1) | _BV(PB2);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = 2047;
}

void SetPWM(unsigned char pin, unsigned int value) {
  // sets the PWM level
  switch (pin) {
    //
    case 9: 
      // 
      OCR1A = value;
      break;
      
    case 10:
      // 
      OCR1B = value;
      break;
  }
}


class BatteryCharger {

  public:
  
    /// default constructor
    BatteryCharger() : 
      state(0) {
      // 
      ntc.Config(100000, 25, 3950, 100250, 65536);
    }
    /// destructor
    virtual ~BatteryCharger() {
      // 
    }

    /// configures the object
    void Config(
      unsigned char vbat_pin, 
      unsigned char ibat_pin, 
      unsigned char tbat_pin, 
      unsigned char pwm_pin, 
      unsigned char dschrg_pin, 
      unsigned char led_pin, 
      unsigned char id) {
      // 
      pin_vbat = vbat_pin;
      pin_ibat = ibat_pin;
      pin_tbat = tbat_pin;
      pin_pwm = pwm_pin;
      pin_dschrg = dschrg_pin;
      pin_led = led_pin;
      iid = id;
    }
    /// executes the automate
    void Execute() {
      // 
      ExecuteLED();
      switch (state) {
        // 
        case 0: {
          // initial state
          value_vbat = 0;
          SetLED(LED_NONE);
          SetCharger(0);
          capacity_in = 0.0f;
          capacity_out = 0.0f;
          
          Serial.println("INIT " + (String)iid);
          delay(2000);
          
          state = 1;
          break;
        }

        // BATTERY DETECTION AND EVALUATION
        {
          
          case 1: {
            // battery presence test
            value_vbat = ReadMultiDecimated(pin_vbat);
            voltage_vbat = GetVoltage(value_vbat, 65536);

            if (voltage_vbat > VOUT_MAX) {
              // device is powered and the ouput voltage got HIGH 
              // slowly through the HIGH yet finite Rds of closed MOSFET
              break;
            }
            // some battery was connected
            if (voltage_vbat >= V_BAT_MAX_NIMH) {
              // alkaline battery detected or a fresh-from-the-charger NiMH
              // anyway, not needing discharging or recharging

              Serial.println("DEVICE " + (String)iid + ": INVALID BATTERY DETECTED ");

              state = 100;
              break;
            }
            // 
            if (voltage_vbat > V_BAT_MIN) {
              // some decent voltage available at the battery slot pins
  
              Serial.println("DEVICE " + (String)iid + ": NiMH LIKE VOLTAGE DETECTED ");
            
              state = 2;
            }
            break;
          }
  
          case 2: {
            // waiting for temperature and voltage to stabilize
            SetLED(LED_WAITING);
            end_ts = millis() + 10000;
            state = 3;
            break;
          }
  
          case 3: {
            // waiting for temperature and voltage to stabilize
            if (millis() > end_ts) {
              // time to end the test
              SetLED(LED_NONE);
              state = 4;
            }
            break;
          }
  
          case 4: {
            // probing for open-circuit voltage and temperature
            value_vbat = ReadMultiDecimated(pin_vbat);
            voltage_vbat = GetVoltage(value_vbat, 65536);
            voltage_vbat_noload = voltage_vbat;
            temperature = ntc.TemperatureC(ReadMultiDecimated(pin_tbat));

            Serial.print("DEVICE " + (String)iid + ": ");
            Serial.print("Battery Voltage: ");
            Serial.print(voltage_vbat, 4);
            Serial.print(" V; Battery Temperature: ");
            Serial.print(ntc.TemperatureC(ReadMultiDecimated(pin_tbat)), 4);
            Serial.println(" deg C ");
            
            state = 5;
            break;
          }
  
          case 5: {
            // probing for closed-circuit voltage
            SetDischarger(true);
            SetLED(LED_WAITING);
            end_ts = millis() + 2000;
            state = 6;
            break;
          }
  
          case 6: {
            // waiting for discharge voltage and current to stabilize
            if (millis() > end_ts) {
              // time to end the test
              value_ibat = ReadMultiDecimated(pin_ibat);
              voltage_ibat = GetVoltage(value_ibat, 65536);
              SetDischarger(false);
              SetLED(LED_NONE);
              state = 7;
            }
            break;
          }
  
          case 7: {
            // DEBUG
            // state = 8; // DEBUG
            if (voltage_vbat > V_BAT_CHARGE + 0.01) {
              // discharging
              state = 20;
            } else {
              // charging
              state = 8;
            }
            break;
          }

          case 8: {
            // 
            Serial.println("DEVICE " + (String)iid + ": " + (String)voltage_vbat + " V; " + (String)temperature + " degC");
            if ((voltage_vbat < V_BAT_CHARGE - 0.01) 
              || (temperature < 10) 
              || (temperature > 40)) {
              // low current charging
              state = 30;
            } else {
              // high current charging
              state = 40;
            }
            break;
          }
        }

        // DISCHARGING
        {
          
          case 20: {
            // discharging the battery
            SetDischarger(true);
            SetLED(LED_DISCHARGING);
            
            start_ts = millis();
            end_ts = start_ts + 18000000; // 5 hours
            ts = start_ts + 5000; // 5 seconds
            
            delay(500);
  
            Serial.println("DEVICE " + (String)iid + ": DISCHARGER ON ");
            
            state = 21;
            break;
          }
  
          case 21: {
            // battery discharge current test: PROBING
            value_vbat = ReadMultiDecimated(pin_vbat);
            voltage_vbat = GetVoltage(value_vbat, 65536);
            
            if (voltage_vbat <= V_BAT_CHARGE) {
              // steep voltage drop
              
              Serial.println("DEVICE " + (String)iid + ": VOLTAGE " + (String)voltage_vbat + " BELOW THRESHOLD");
              
              state = 22;
              break;
            }

            now_ts = millis();
            if (now_ts > end_ts) {
              // time to end the discharging
              
              Serial.println("DEVICE " + (String)iid + ": DISCHARGE TIMEOUT");
              
              state = 22;
              break;
            }

            if (now_ts > ts) {
              // 
              //ts = now_ts + 5000;// - (now_ts - ts);
              value_ibat = ReadMultiDecimated(pin_ibat);
              voltage_ibat = GetVoltage(value_ibat, 65536);
              current = (voltage_vbat - voltage_ibat) / SHUNT_RESISTOR;
              capacity_out += current * 5.0 / 3.6;
              ts += 5000;
  
              temperature = ntc.TemperatureC(ReadMultiDecimated(pin_tbat));
              if ((temperature < 10) || (temperature > 40)) {
                // temperature error

                Serial.println("DEVICE " + (String)iid + ": ABNORMAL DISCHARGE TEMPERATURE");
                
                state = 23;
                break;
              }

              if (0 && (current < 0.01)) {
                // abnormally small discharge current

                Serial.println("DEVICE " + (String)iid + ": ABNORMAL DISCHARGE CURRENT: " + (String)current);
                
                state = 22;
                break;
              }
              
              double Rmosfet = voltage_ibat / current;
              double Rint = voltage_vbat_noload * (SHUNT_RESISTOR + Rmosfet) / voltage_vbat - SHUNT_RESISTOR - Rmosfet;

              Serial.print("DEVICE " + (String)iid + ": ");
              Serial.print(now_ts);
              Serial.print(" Vbat = ");
              Serial.print(voltage_vbat, 6);
              Serial.print(" V; Vshunt = ");
              Serial.print(voltage_vbat - voltage_ibat, 6);
              Serial.print(" V; Ishunt = ");
              Serial.print(current, 6);
              Serial.print(" A; Ploss = ");
              Serial.print(voltage_vbat * current, 6);
              Serial.print(" W; Rmosfet = ");
              Serial.print(Rmosfet, 6);
              Serial.print(" Ohm; Rbat = ");
              Serial.print(Rint, 6);
              Serial.print(" Ohm; Tbat = ");
              Serial.print(temperature, 6);
              Serial.print(" degC; Cout = ");
              Serial.print(capacity_out, 6);
              Serial.println(" mAh");
              
            }
            break;
          }
  
          case 22: {
            // battery discharge current test: END
            SetDischarger(false);
            SetLED(LED_DISCHARGING);
            
            Serial.println("DEVICE " + (String)iid + ": DISCHARGER OFF ");
  
            delay(1000);
            state = 5;
            break;
          }
  
          case 23: {
            // battery discharge current test: END
            SetDischarger(false);
            SetLED(LED_ERROR);
            
            Serial.println("DEVICE " + (String)iid + ": DISCHARGER OFF - ERROR");
  
            state = 24;
            break;
          }
  
          case 24: {
            // battery discharge error
            
            break;
          }
        }

        // CHARGING - LOW CURRENT
        {

          case 30: {
            // turning on the charger at minimum PWM level
            SetDischarger(false);
            delay(1000);
            SetLED(LED_CHARGING);
            SetCharger(100);
            level_pwm_increment = 32;
            current_avg = 0.0f;
            temperature_slope = 0.0f;
            temperature_avg = 0.0f;
            temperature_last = ntc.TemperatureC(ReadMultiDecimated(pin_tbat));

            start_ts = millis();
            end_ts = start_ts + 54000000; // 15 hours
            ts = start_ts + 5000;
            minute_ts = start_ts + 60000;

            Serial.println("DEVICE " + (String)iid + ": LOW CURRENT CHARGER ON ");
            
            state = 31;
            break;
          }

          case 31: {
            // 
            value_vbat = ReadMultiDecimated(pin_vbat);
            voltage_vbat = GetVoltage(value_vbat, 65536);
            if (voltage_vbat > VOUT_MAX) {
              // HARDWARE PROTECTION
              SetDischarger(false);
              SetCharger(0);

              Serial.println("DEVICE " + (String)iid + ": ABNORMALY HIGH OUTPUT VOLTAGE");
              
              state = 100;
              break;
            }
            
            temperature = ntc.TemperatureC(ReadMultiDecimated(pin_tbat));
            
            if (0 
              && (voltage_vbat > V_BAT_CHARGE + 0.1) 
              && (temperature >= 10) && (temperature <= 30)) {
              // voltage above critical limit
              
              Serial.println("DEVICE " + (String)iid + ": FITS HIGH CURRENT CHARGING");
              
              state = 32;
              break;
            }

            now_ts = millis();
            if (now_ts > end_ts) {
              // time to end the charging
              
              Serial.println("DEVICE " + (String)iid + ": LOW CURRENT CHARGING: TIMEOUT");
              
              state = 33;
              break;
            }

            value_ibat = ReadMultiDecimated(pin_ibat);
            voltage_ibat = GetVoltage(value_ibat, 65536);
            current = (voltage_ibat - voltage_vbat) / SHUNT_RESISTOR;
            
            if (current < I_LOW_CURRENT_CHARGING) {
              // 
              if (level_pwm_increment < 0) {
                // was falling
                level_pwm_increment = (abs(level_pwm_increment) < 2) ? 
                  -level_pwm_increment : -level_pwm_increment / 2;
              }
              SetCharger(level_pwm + level_pwm_increment);
            } else {
              // was rising
              if (level_pwm_increment > 0) {
                // 
                level_pwm_increment = (abs(level_pwm_increment) < 2) ? 
                  -level_pwm_increment : -level_pwm_increment / 2;
              }
              SetCharger(level_pwm + level_pwm_increment);
            }
            delay(100);

            if (level_pwm >= 2047) {
              // abnormal PWM level

              Serial.println("DEVICE " + (String)iid + ": ABNORMAL PWM LEVEL");
              
              state = 33;
              break;
            }
            
            if ((level_pwm > 400) && (current < 0.1)) {
              // voltage drop on shunt does not keep-up with the PWM level

              Serial.println("DEVICE " + (String)iid + ": CHARGING CURRENT NOT KEEPING UP WITH PWM LEVEL");
              
              state = 33;
              break;
            }

            if (now_ts > minute_ts) {
              // time for stats gathering
              //minute_ts = now_ts + 60000; // - (now_ts - minute_ts);
              minute_ts += 60000;
              temperature_slope = temperature_avg - temperature_last;
              temperature_last = temperature_avg;
            }
            
            if (now_ts > ts) {
              // time to test current
              //ts = now_ts + 5000; // - (now_ts - ts);
              ts += 5000;
              if (current_avg == 0) {
                // 
                current_avg = current;
              } else {
                // 
                current_avg = (11 * current_avg + current) / 12;
              }

              capacity_in = (double)(now_ts - start_ts) * current_avg / 3600.0;
              
              if ((temperature < 5) || (temperature > 35)) {
                // abnormal temperature

                Serial.println("DEVICE " + (String)iid + ": LOW CURRENT CHARGING ABNORMAL TEMPERATURE");
                
                state = 33;
                break;
              }

              if (temperature_avg == 0) {
                // 
                temperature_avg = temperature;
              } else {
                // 
                temperature_avg = (temperature_avg * 11.0 + temperature) / 12.0;
              }
              
              Serial.print("DEVICE " + (String)iid + ": ");
              Serial.print(now_ts);
              Serial.print(" Vbat = ");
              Serial.print(voltage_vbat, 6);
              Serial.print(" V; Vshunt = ");
              Serial.print(voltage_ibat - voltage_vbat, 6);
              Serial.print(" V; Ishunt = ");
              Serial.print(current, 6);
              Serial.print(" A; PWM = ");
              Serial.print(level_pwm);
              Serial.print(" Ploss = ");
              Serial.print(voltage_vbat * current, 6);
              Serial.print(" W; Tbat = ");
              Serial.print(temperature, 6);
              Serial.print(" degC; Tslope = ");
              Serial.print(temperature_slope, 6);
              Serial.print(" degC/min; Cout = ");
              Serial.print(capacity_in, 6);
              Serial.println(" mAh");

            }
            break;
          }

          case 32: {
            // 
            state = 8;
            break;
          }

          case 33: {
            // low current charge timed out
            SetDischarger(false);
            SetCharger(0);
            
            state = 100;
            break;
          }
          
        }

        // CHARGING - HIGH CURRENT
        {
          
          case 40: {
            // turning on the charger at minimum PWM level
            SetDischarger(false);
            delay(1000);
            SetLED(LED_CHARGING);
            temperature_slope = 0.0f;
            temperature_avg = 0.0f;
            temperature_last = ntc.TemperatureC(ReadMultiDecimated(pin_tbat));
            for (byte i = 0; i < TEMPERATURE_SLOPE_SEQUENCE_LENGTH; i ++) {
              // 
              temperature_slope_sequence[i] = 0.0f;
            }
            voltage_max = 0.0f;
            voltage_drop = 0.0f;
            SetCharger(level_pwm == 0 ? 200 : level_pwm);
            level_pwm_increment = 32;
            current_avg = 0.0f;

            start_ts = millis();
            end_ts = start_ts + 18000000; // 5 hours
            ts = start_ts + 5000; // 5 seconds
            minute_ts = start_ts + 60000; // 1 minute

            Serial.println("DEVICE " + (String)iid + ": HIGH CURRENT CHARGER ON ");
            
            state = 41;
            break;
          }

          case 41: {
            // 
            value_vbat = ReadMultiDecimated(pin_vbat);
            voltage_vbat = GetVoltage(value_vbat, 65536);
            temperature = ntc.TemperatureC(ReadMultiDecimated(pin_tbat));

            if (voltage_vbat > VOUT_MAX) {
              // HARDWARE PROTECTION
              SetDischarger(false);
              SetCharger(0);

              Serial.println("DEVICE " + (String)iid + ": ABNORMALY HIGH OUTPUT VOLTAGE");
              
              state = 100;
              break;
            }
            
            if (voltage_vbat > V_BAT_MAX) {
              // voltage above max limit
              
              Serial.println("DEVICE " + (String)iid + ": VOLTAGE ABOVE MAX LIMIT");
              
              state = 42;
              break;
            }

            now_ts = millis();
            if (now_ts > end_ts) {
              // time to end the charging
              
              Serial.println("DEVICE " + (String)iid + ": HIGH CURRENT CHARGING TIMEOUT");
              
              state = 42;
              break;
            }
            
            value_ibat = ReadMultiDecimated(pin_ibat);
            voltage_ibat = GetVoltage(value_ibat, 65536);
            current = (voltage_ibat - voltage_vbat) / SHUNT_RESISTOR;

            if (current < I_HIGH_CURRENT_CHARGING) {
              // 
              if (level_pwm_increment < 0) {
                // was falling
                level_pwm_increment = (abs(level_pwm_increment) < 2) ? 
                  -level_pwm_increment : -level_pwm_increment / 2;
              }
              SetCharger(level_pwm + level_pwm_increment);
            } else {
              // was rising
              if (level_pwm_increment > 0) {
                // 
                level_pwm_increment = (abs(level_pwm_increment) < 2) ? 
                  -level_pwm_increment : -level_pwm_increment / 2;
              }
              SetCharger(level_pwm + level_pwm_increment);
            }
            delay(200);
            
            if (level_pwm >= 2047) {
              // abnormal PWM level

              Serial.println("DEVICE " + (String)iid + ": ABNORMAL PWM LEVEL");
              
              state = 42;
              break;
            }
            
            if ((level_pwm > 400) && (current < 0.1)) {
              // voltage drop on shunt does not keep-up with the PWM level

              Serial.println("DEVICE " + (String)iid + ": CHARGING CURRENT NOT KEEPING UP WITH PWM LEVEL");
              
              state = 42;
              break;
            }

            if (now_ts > minute_ts) {
              // time for stats gathering
              //minute_ts = now_ts + 60000;// - (now_ts - minute_ts);
              minute_ts += 60000;
              temperature_slope = temperature_avg - temperature_last;
              temperature_last = temperature_avg;
              
              bool bCorrectSequence = true;
              for (byte i = 0; i < TEMPERATURE_SLOPE_SEQUENCE_LENGTH - 1; i ++) {
                // 
                if (temperature_slope_sequence[i] > temperature_slope_sequence[i + 1]) {
                  // 
                  bCorrectSequence  = false;
                }
                if (temperature_slope_sequence[i] <= 0) {
                  // 
                  bCorrectSequence  = false;
                }
                temperature_slope_sequence[i] = temperature_slope_sequence[i + 1];
              }
              temperature_slope_sequence[9] = temperature_slope;
              
              if (bCorrectSequence) {
                // a steady growing temperature slope
                // consistent with dT end condition

                Serial.println("DEVICE " + (String)iid + ": HIGH CURRENT CHARGING COMPLETE: dT event, Tslope sequence:");
                for (byte i = 0; i < TEMPERATURE_SLOPE_SEQUENCE_LENGTH - 1; i ++) {
                  // 
                  Serial.print(temperature_slope_sequence[i], 6);
                  Serial.print(", ");
                }
                Serial.println(" ");
                state = 42;
                break;
              }
            }
            
            if (now_ts > ts) {
              // time for more stuff
              //ts = now_ts + 5000;// - (now_ts - ts);
              ts += 5000;
              if (current_avg == 0) {
                // 
                current_avg = current;
              } else {
                // 
                current_avg = (11 * current_avg + current) / 12;
              }

              capacity_in = (double)(now_ts - start_ts) * current_avg / 3600;
              
              if (temperature > 40) {
                // temperature too high for high current charging

                Serial.println("DEVICE " + (String)iid + ": HIGH CURRENT CHARGING : OVER TEMPERATURE");
                
                state = 42;
                break;
              }

              if (temperature < 10) {
                // temperature too low for high current charging

                Serial.println("DEVICE " + (String)iid + ": HIGH CURRENT CHARGING : UNDER TEMPERATURE");
                
                state = 8;
                break;
              }

              if (voltage_vbat > voltage_max) {
                // 
                voltage_max = voltage_vbat;
              }
              // 
              voltage_drop = (voltage_drop * 11 + (voltage_max - voltage_vbat)) / 12;

              if (temperature_avg == 0) {
                // 
                temperature_avg = temperature;
              } else {
                // 
                temperature_avg = (temperature_avg * 11.0 + temperature) / 12.0;
              }

              if (voltage_drop >= 0.01) {
                // -dV end condition

                Serial.println("DEVICE " + (String)iid + ": HIGH CURRENT CHARGING COMPLETE: -dV");
                
                state = 42;
                break;
              }

              if (temperature_slope >= 1.0) {
                // dT end condition

                Serial.println("DEVICE " + (String)iid + ": HIGH CURRENT CHARGING COMPLETE: dT=" + (String)temperature_slope);
                
                state = 42;
                break;
              }
              
              Serial.print("DEVICE " + (String)iid + ": ");
              Serial.print(now_ts);
              Serial.print(" Vbat = ");
              Serial.print(voltage_vbat, 6);
              Serial.print(" V; Vdrop = ");
              Serial.print(voltage_drop, 6);
              Serial.print(" V; Vshunt = ");
              Serial.print(voltage_ibat - voltage_vbat, 6);
              Serial.print(" V; Ishunt = ");
              Serial.print(current, 6);
              Serial.print(" A; PWM = ");
              Serial.print(level_pwm);
              Serial.print(" Ploss = ");
              Serial.print(voltage_vbat * current, 6);
              Serial.print(" W; Tbat = ");
              Serial.print(temperature, 6);
              Serial.print(" degC; Tslope = ");
              Serial.print(temperature_slope, 6);
              Serial.print(" degC/min; Cout = ");
              Serial.print(capacity_in, 6);
              Serial.println(" mAh");

            }
            break;
          }

          case 42: {
            // stopping charger
            SetDischarger(false);
            SetCharger(0);
            
            state = 100;
            break;
          }
          
        }
        
        case 100: {
          // battery removal notice
          SetCharger(0);
          SetDischarger(false);
          
          Serial.println("DEVICE " + (String)iid + ": PLEASE REMOVE BATTERY");
          
          state = 101;
          break;
        }

        case 101: {
          // waiting for battery removal
          value_vbat = ReadMultiDecimated(pin_vbat);
          voltage_vbat = GetVoltage(value_vbat, 65536);
          value_ibat = ReadMultiDecimated(pin_ibat);
          voltage_ibat = GetVoltage(value_vbat, 65536);
          current = (voltage_vbat - voltage_ibat) / SHUNT_RESISTOR;
          if (voltage_vbat < V_BAT_MIN - 0.1) {
            // device not powered
            // battery removed
            state = 0;
          } else if (voltage_vbat > VOUT_MAX) {
            // device powered 
            // battery removed
            state = 0;
          } else {
            // 
            Serial.print("DEVICE " + (String)iid + ": Vbat = ");
            Serial.print(voltage_vbat, 6);
            Serial.print(" V, Ibat = ");
            Serial.print(current, 6);
            Serial.println(" A");
            
            delay(5000);
          }
          break;
        }
        
      }
    }

  private:

    /// reads and accumulates multiple samples and decimates the result
    static unsigned long ReadMultiDecimated(byte pin, byte bits = 16) {
      // 
      unsigned long total = 0;
      bits -= 10;
      int N = B00000001 << (2 * bits);
      for (int i = 0; i < N; i++) {
        // 
        total += analogRead(pin);
      }
      return total >> bits;
    }
    /// gets the voltage from the given ADC value
    static double GetVoltage(unsigned long value, unsigned long resolution = 1024, float vcc = VCC) {
      // 
      return (double) value / (resolution - 1) * vcc;
    }
    /// sets the PWM level for the charger's buck converter
    void SetCharger(unsigned int level) {
      //
      level_pwm = level;
      SetPWM(pin_pwm, level_pwm);
    }
    /// turns on and off the discharger
    void SetDischarger(bool on) {
      // 
      digitalWrite(pin_dschrg, on ? HIGH : LOW);
    }
    /// sets the LED state
    void SetLED(unsigned char state) {
      // 
      stateLED = state;
      LEDon = false;
      digitalWrite(13, LOW);
    }
    /// executes the LED sequence
    void ExecuteLED() {
      // 
      if (stateLED == LED_NONE) {
        // do nothing
        return;
      }
      // do something
      if (stateLED == LED_READY) {
        //
        if (!LEDon) {
          // 
          LEDon = true;  
          digitalWrite(pin_led, HIGH);
        }
        return;
      }
      if (millis() > LED_ts) {
        // 
        if (LEDon) {
          // 
          LEDon = false;
          digitalWrite(pin_led, LOW);
        } else {
          // 
          LEDon = true;  
          digitalWrite(pin_led, HIGH);
        }
        if (stateLED == LED_WAITING) {
          // 
          LED_ts = millis() + 500;
        } else if (stateLED == LED_DISCHARGING) {
          // 
          LED_ts = millis() + LEDon ? 250 : 750;
        } else if (stateLED == LED_CHARGING) {
          // 
          LED_ts = millis() + LEDon ? 750 : 250;
        } else if (stateLED == LED_ERROR) {
          // 
          LED_ts = millis() + LEDon ? 250 : 250;
        }
      }
    }

    /// temperature slope sequence length
    static const byte TEMPERATURE_SLOPE_SEQUENCE_LENGTH = 10;
    
    /// state
    unsigned char state = 0;
    /// instance id
    unsigned char iid = 0;
    /// the ADC read value for Vbat
    unsigned int value_vbat = 0, value_ibat = 0;
    /// measured voltages 
    double voltage_vbat = 0.0f, voltage_ibat = 0.0f, voltage_vbat_noload = 0.0f;
    /// end-charging primary variables
    double voltage_max = 0.0f, temperature_last = 0.0f;
    /// end-charging secondary variables
    double temperature_slope = 0.0f, temperature_avg = 0.0f, voltage_drop = 0.0f;
    /// temperature slope sequence
    double temperature_slope_sequence[TEMPERATURE_SLOPE_SEQUENCE_LENGTH];
    /// measured temperature
    double temperature = 0.0f;
    /// determined shunt current
    double current = 0.0f, current_avg = 0.0f;
    /// determined powers
    double capacity_in = 0.0f, capacity_out = 0.0f;
    /// input Vbat, Ibat, Tbat pins
    unsigned char pin_vbat, pin_ibat, pin_tbat, pin_led;
    /// control charge_pwm and discharge pins
    unsigned char pin_pwm, pin_dschrg;
    /// LED sequence state
    unsigned char stateLED = 0;
    /// LED state
    bool LEDon = false;
    /// timestamps
    unsigned long now_ts = 0, start_ts = 0, end_ts = 0, ts = 0, minute_ts = 0, LED_ts = 0;
    /// pwm level
    unsigned int level_pwm = 0;
    /// pwm level increment
    signed char level_pwm_increment = 0;
    /// thermistor probe
    NTCThermistor ntc;
};


BatteryCharger charger_0, charger_1;

void setup() {
  // 

  pinMode(V_BAT0, INPUT);
  pinMode(V_BAT1, INPUT);
  pinMode(NTC0, INPUT);
  pinMode(NTC1, INPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(I_BAT0, INPUT);
  pinMode(I_BAT1, INPUT);
  for (int i = 2; i <= 13; i ++) {
    // 
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
  }
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);

  charger_0.Config(V_BAT0, I_BAT0, NTC0, CH_PWM0, DSCH_0, LED_0, 1);
  charger_1.Config(V_BAT1, I_BAT1, NTC1, CH_PWM1, DSCH_1, LED_1, 2);

  // sets the PWM resolution to 11 bits at 8kHz
  ConfigPWM();
  SetPWM(CH_PWM0, 0);
  SetPWM(CH_PWM1, 0);
  
  Serial.begin(9600);
}

void loop() {
  // 

  charger_0.Execute();
  charger_1.Execute();
}
