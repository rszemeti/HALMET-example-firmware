#include "halmet_analog.h"

#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/transforms/moving_average.h"


namespace halmet {

// HALMET constant measurement current (A)
const float kMeasurementCurrent = 1.0;

// Default fuel tank size, in m3
const float kTankDefaultSize = 120. / 1000;

sensesp::FloatProducer* ConnectTankResistanceSender(Adafruit_ADS1115* ads1115,
                                          int channel, const String& name,
                                          const String& sk_id, int sort_order,
                                          bool enable_signalk_output) {
  const uint ads_read_delay = 500;  // ms

  // Configure the sender resistance sensor

  auto sender_resistance_raw =
      new sensesp::RepeatSensor<float>(ads_read_delay, [ads1115, channel]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(channel);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        return kVoltageDividerScale * adc_output_volts / kMeasurementCurrent;
      });

  auto sender_resistance = new sensesp::MovingAverage(20, 1.0, "/Resistance A1");
  sender_resistance_raw->connect_to(sender_resistance);



  // Configure the piecewise linear interpolator for the tank level (ratio)

  char curve_config_path[80];
  snprintf(curve_config_path, sizeof(curve_config_path),
           "/Tanks/%s/Level Curve", name.c_str());
  char curve_title[80];
  snprintf(curve_title, sizeof(curve_title), "%s Tank Level Curve",
           name.c_str());
  char curve_description[80];
  snprintf(curve_description, sizeof(curve_description),
           "Piecewise linear curve for the %s tank level", name.c_str());

  auto tank_level = (new sensesp::CurveInterpolator(nullptr, curve_config_path))
                        ->set_input_title("Sender Resistance (ohms)")
                        ->set_output_title("Fuel Level (ratio)");

  ConfigItem(tank_level)
      ->set_title(curve_title)
      ->set_description(curve_description)
      ->set_sort_order(sort_order + 1);

  if (tank_level->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    tank_level->clear_samples();
    tank_level->add_sample(sensesp::CurveInterpolator::Sample(0, 0));
    tank_level->add_sample(sensesp::CurveInterpolator::Sample(180., 1));
    tank_level->add_sample(sensesp::CurveInterpolator::Sample(1000., 1));
  }

  sender_resistance->connect_to(tank_level);

  if (enable_signalk_output) {
    char level_config_path[80];
    snprintf(level_config_path, sizeof(level_config_path),
             "/Tanks/%s/Current Level SK Path", name.c_str());
    char level_title[80];
    snprintf(level_title, sizeof(level_title), "%s Tank Level SK Path",
             name.c_str());
    char level_description[80];
    snprintf(level_description, sizeof(level_description),
             "Signal K path for the %s tank level", name.c_str());
    char level_sk_path[80];
    snprintf(level_sk_path, sizeof(level_sk_path), "tanks.%s.currentLevel",
             sk_id.c_str());
    char level_meta_display_name[80];
    snprintf(level_meta_display_name, sizeof(level_meta_display_name),
             "Tank %s level", name.c_str());
    char level_meta_description[80];
    snprintf(level_meta_description, sizeof(level_meta_description),
             "Tank %s level", name.c_str());

    auto tank_level_sk_output = new sensesp::SKOutputFloat(
        level_sk_path, level_config_path,
        new sensesp::SKMetadata("ratio", level_meta_display_name,
                                level_meta_description));

    ConfigItem(tank_level_sk_output)
        ->set_title(level_title)
        ->set_description(level_description)
        ->set_sort_order(sort_order + 2);

    tank_level->connect_to(tank_level_sk_output);
  }

  // Configure the linear transform for the tank volume

  char volume_config_path[80];
  snprintf(volume_config_path, sizeof(volume_config_path),
           "/Tanks/%s/Total Volume", name.c_str());
  char volume_title[80];
  snprintf(volume_title, sizeof(volume_title), "%s Tank Total Volume",
           name.c_str());
  char volume_description[80];
  snprintf(volume_description, sizeof(volume_description),
           "Calculated total volume of the %s tank", name.c_str());
  auto tank_volume =
      new sensesp::Linear(kTankDefaultSize, 0, volume_config_path);

  ConfigItem(tank_volume)
      ->set_title(volume_title)
      ->set_description(volume_description)
      ->set_sort_order(sort_order + 3);

  tank_level->connect_to(tank_volume);

  if (enable_signalk_output) {
    char volume_sk_config_path[80];
    snprintf(volume_sk_config_path, sizeof(volume_sk_config_path),
             "/Tanks/%s/Current Volume SK Path", name.c_str());
    char volume_title[80];
    snprintf(volume_title, sizeof(volume_title), "%s Tank Volume SK Path",
             name.c_str());
    char volume_description[80];
    snprintf(volume_description, sizeof(volume_description),
             "Signal K path for the %s tank volume", name.c_str());
    char volume_sk_path[80];
    snprintf(volume_sk_path, sizeof(volume_sk_path), "tanks.%s.currentVolume",
             sk_id.c_str());
    char volume_meta_display_name[80];
    snprintf(volume_meta_display_name, sizeof(volume_meta_display_name),
             "Tank %s volume", name.c_str());
    char volume_meta_description[80];
    snprintf(volume_meta_description, sizeof(volume_meta_description),
             "Calculated tank %s remaining volume", name.c_str());

    auto tank_volume_sk_output = new sensesp::SKOutputFloat(
        volume_sk_path, volume_sk_config_path,
        new sensesp::SKMetadata("m3", volume_meta_display_name,
                                volume_meta_description));

    ConfigItem(tank_volume_sk_output)
        ->set_title(volume_title)
        ->set_description(volume_description)
        ->set_sort_order(sort_order + 4);

    tank_volume->connect_to(tank_volume_sk_output);
  }

  return tank_level;
}

sensesp::FloatProducer* ConnectTankVoltageSender(Adafruit_ADS1115* ads1115,
  int channel, const String& name,
  const String& sk_id, int sort_order,
  bool enable_signalk_output) {

  const uint ads_read_delay = 500;  // ms

// Configure the sender voltage

  auto sender_voltage =
  new sensesp::RepeatSensor<float>(ads_read_delay, [ads1115, channel]() {
    int16_t adc_output = ads1115->readADC_SingleEnded(channel);
    float adc_output_volts = ads1115->computeVolts(adc_output);
    return adc_output_volts;
  });

// Configure the piecewise linear interpolator for the tank level (ratio)

char curve_config_path[80];
snprintf(curve_config_path, sizeof(curve_config_path),
"/Tanks/%s/Voltage Level Curve", name.c_str());
char curve_title[80];
snprintf(curve_title, sizeof(curve_title), "%s Tank Level Curve",
name.c_str());
char curve_description[80];
snprintf(curve_description, sizeof(curve_description),
"Piecewise linear curve for the %s tank level", name.c_str());

  auto tank_level_curve = (new sensesp::CurveInterpolator(nullptr, curve_config_path))
    ->set_input_title("Sender Voltage (V)")
    ->set_output_title("Fuel Level (ratio)");

  ConfigItem(tank_level_curve)
    ->set_title(curve_title)
    ->set_description(curve_description)
    ->set_sort_order(sort_order + 1);

  if (tank_level_curve->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    tank_level_curve->clear_samples();
    tank_level_curve->add_sample(sensesp::CurveInterpolator::Sample(0.0, 0.));
    tank_level_curve->add_sample(sensesp::CurveInterpolator::Sample(10.0, 1.0));
  }

  sender_voltage->connect_to(tank_level_curve);

  

  return tank_level_curve;
}

}  // namespace halmet
