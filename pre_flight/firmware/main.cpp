/**
 * SAMWISE ADCS pre-flight USB test firmware.
 *
 * This image is never intended for flight. It exposes sensor samples and
 * short, bounded magnetorquer pulses over USB CDC for the host-side tools in
 * pre_flight/run.py.
 */

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "apps/adcs_app/params.h"
#include "apps/adcs_app/pins.h"
#include "drivers/imu/imu.h"
#include "drivers/magnetometer/magnetometer.h"
#include "drivers/magnetorquers/magnetorquers.h"
#include "drivers/sun_sensors/ads7830.h"
#include "drivers/sun_sensors/rp2350b_adc.h"
#include "drivers/sun_sensors/sun_sensors.h"
#include "linalg.h"

using namespace linalg::aliases;

namespace {

enum class StreamMode { off, magnetometer, imu, sun, all };

constexpr uint32_t kProtocolVersion = 1;
constexpr uint32_t kMagnetorquerArmWindowMs = 30000;
constexpr float kMagnetorquerMaxDuty = 0.25f;
constexpr uint32_t kMagnetorquerMinPulseMs = 150;
constexpr uint32_t kMagnetorquerMaxPulseMs = 500;
constexpr uint32_t kWatchdogLowUs = 1000000;
constexpr uint32_t kWatchdogHighUs = 200000;

StreamMode stream_mode = StreamMode::off;
uint32_t stream_period_ms = 100;
uint64_t next_stream_us = 0;

bool imu_alive = false;
bool magnetometer_alive = false;
bool sun_pyramid_alive = false;
bool sun_ads_alive = false;

bool magnetorquer_initialized = false;
bool magnetorquer_armed = false;
uint64_t magnetorquer_arm_deadline_us = 0;

bool watchdog_high = false;
uint64_t watchdog_last_transition_us = 0;

void service_watchdog()
{
    const uint64_t now = time_us_64();
    const uint64_t interval = watchdog_high ? kWatchdogHighUs : kWatchdogLowUs;
    if (now - watchdog_last_transition_us >= interval)
    {
        watchdog_high = !watchdog_high;
        gpio_put(SAMWISE_ADCS_WATCHDOG_FEED, watchdog_high ? 1 : 0);
        watchdog_last_transition_us = now;
    }
}

void safe_sleep_ms(uint32_t duration_ms)
{
    const uint64_t deadline = time_us_64() + duration_ms * 1000ULL;
    while (time_us_64() < deadline)
    {
        service_watchdog();
        sleep_ms(5);
    }
}

void init_watchdog_output()
{
    gpio_init(SAMWISE_ADCS_WATCHDOG_FEED);
    gpio_set_dir(SAMWISE_ADCS_WATCHDOG_FEED, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_WATCHDOG_FEED, 0);
    watchdog_high = false;
    watchdog_last_transition_us = time_us_64();
}

void init_i2c_buses()
{
    gpio_set_function(SAMWISE_ADCS_I2C0_SCL, GPIO_FUNC_I2C);
    gpio_set_function(SAMWISE_ADCS_I2C0_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SAMWISE_ADCS_I2C0_SCL);
    gpio_pull_up(SAMWISE_ADCS_I2C0_SDA);
    i2c_init(i2c0, SAMWISE_ADCS_I2C0_BAUD);

    gpio_set_function(SAMWISE_ADCS_I2C1_SCL, GPIO_FUNC_I2C);
    gpio_set_function(SAMWISE_ADCS_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SAMWISE_ADCS_I2C1_SCL);
    gpio_pull_up(SAMWISE_ADCS_I2C1_SDA);
    i2c_init(i2c1, SAMWISE_ADCS_I2C1_BAUD);
}

void stop_and_disarm_magnetorquers()
{
    if (magnetorquer_initialized)
    {
        stop_magnetorquer_pwm();
    }
    gpio_init(SAMWISE_ADCS_EN_MAGDRV);
    gpio_set_dir(SAMWISE_ADCS_EN_MAGDRV, GPIO_OUT);
    gpio_put(SAMWISE_ADCS_EN_MAGDRV, 0);
    magnetorquer_armed = false;
    magnetorquer_arm_deadline_us = 0;
}

void print_status()
{
    std::printf("PF,STATUS,imu=%d,mag=%d,sun_pyramid=%d,sun_ads=%d,mt_armed=%d\n",
                imu_alive, magnetometer_alive, sun_pyramid_alive,
                sun_ads_alive, magnetorquer_armed);
}

bool emit_magnetometer()
{
    if (!magnetometer_alive)
    {
        return false;
    }

    float3 body{};
    float3 raw{};
    if (rm3100_get_reading(&body, &raw) != RM3100_OK)
    {
        return false;
    }

    std::printf("PF,MAG,%llu,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n",
                static_cast<unsigned long long>(time_us_64() / 1000ULL), raw.x,
                raw.y, raw.z, body.x, body.y, body.z);
    return true;
}

bool emit_imu()
{
    if (!imu_alive)
    {
        return false;
    }

    float3 gyro{NAN, NAN, NAN};
    float3 accel{NAN, NAN, NAN};
    const bool gyro_ok = imu_get_rotation(&gyro);
    const bool accel_ok = imu_get_accel(&accel);
    if (!gyro_ok && !accel_ok)
    {
        return false;
    }

    std::printf("PF,IMU,%llu,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
                static_cast<unsigned long long>(time_us_64() / 1000ULL), gyro.x,
                gyro.y, gyro.z, accel.x, accel.y, accel.z);
    return true;
}

bool read_sun_sensors(uint16_t intensities[NUM_SUN_SENSORS])
{
    bool any_ok = false;
    std::fill(intensities, intensities + NUM_SUN_SENSORS, 0);

    if (sun_pyramid_alive)
    {
        uint16_t values[8]{};
        if (rp2350b_adc_read_all_channels(values))
        {
            for (size_t i = 0; i < 8; ++i)
            {
                intensities[i] = std::min(values[i], SUN_SENSOR_SATURATION_VALUE);
            }
            any_ok = true;
        }
    }

    if (sun_ads_alive)
    {
        uint8_t values[8]{};
        if (ads7830_read_all_channels(values))
        {
            for (size_t i = 0; i < 8; ++i)
            {
                intensities[i + 8] = static_cast<uint16_t>(values[i]) *
                                     SUN_SENSOR_SATURATION_VALUE /
                                     MAX_VALUE_ADS7830;
            }
            any_ok = true;
        }
    }
    return any_ok;
}

bool emit_sun()
{
    uint16_t values[NUM_SUN_SENSORS]{};
    if (!read_sun_sensors(values))
    {
        return false;
    }

    std::printf("PF,SUN,%llu", static_cast<unsigned long long>(time_us_64() / 1000ULL));
    for (const uint16_t value : values)
    {
        std::printf(",%u", value);
    }
    std::printf("\n");
    return true;
}

bool average_raw_magnetometer(float3 *average, uint32_t wanted_samples)
{
    if (!magnetometer_alive || average == nullptr)
    {
        return false;
    }

    float3 sum{};
    uint32_t count = 0;
    const uint64_t deadline = time_us_64() + 500000ULL;
    while (count < wanted_samples && time_us_64() < deadline)
    {
        float3 body{};
        float3 raw{};
        if (rm3100_get_reading(&body, &raw) == RM3100_OK)
        {
            sum += raw;
            ++count;
        }
        safe_sleep_ms(15);
    }

    if (count == 0)
    {
        return false;
    }
    *average = sum / static_cast<float>(count);
    return true;
}

void run_magnetorquer_pulse(char axis, int sign, float duty,
                            uint32_t duration_ms)
{
    if (!magnetorquer_armed || time_us_64() >= magnetorquer_arm_deadline_us)
    {
        stop_and_disarm_magnetorquers();
        std::printf("PF,ERROR,MT_NOT_ARMED\n");
        return;
    }

    duty = std::clamp(std::fabs(duty), 0.01f, kMagnetorquerMaxDuty);
    duration_ms = std::clamp(duration_ms, kMagnetorquerMinPulseMs,
                             kMagnetorquerMaxPulseMs);
    sign = sign >= 0 ? 1 : -1;

    float3 command{};
    const float signed_duty = duty * static_cast<float>(sign);
    switch (axis)
    {
        case 'X': command.x = signed_duty; break;
        case 'Y': command.y = signed_duty; break;
        case 'Z': command.z = signed_duty; break;
        default:
            std::printf("PF,ERROR,BAD_MT_AXIS\n");
            return;
    }

    float3 baseline{NAN, NAN, NAN};
    float3 energized{NAN, NAN, NAN};
    const bool baseline_ok = average_raw_magnetometer(&baseline, 5);

    std::printf("PF,MT,START,%c,%d,%.3f,%u\n", axis, sign, duty, duration_ms);
    if (!do_magnetorquer_pwm(command))
    {
        stop_and_disarm_magnetorquers();
        std::printf("PF,ERROR,MT_DRIVER_REJECTED\n");
        return;
    }

    const uint64_t pulse_start = time_us_64();
    safe_sleep_ms(50);
    const bool energized_ok = average_raw_magnetometer(&energized, 5);
    while ((time_us_64() - pulse_start) / 1000ULL < duration_ms)
    {
        safe_sleep_ms(5);
    }
    stop_magnetorquer_pwm();

    if (baseline_ok && energized_ok)
    {
        const float3 delta = energized - baseline;
        std::printf("PF,MT,RESULT,%c,%d,%.3f,%u,%.7f,%.7f,%.7f\n", axis,
                    sign, duty, duration_ms, delta.x, delta.y, delta.z);
    }
    else
    {
        std::printf("PF,MT,RESULT,%c,%d,%.3f,%u,nan,nan,nan\n", axis, sign,
                    duty, duration_ms);
    }
}

StreamMode parse_stream_mode(const char *name)
{
    if (std::strcmp(name, "MAG") == 0) return StreamMode::magnetometer;
    if (std::strcmp(name, "IMU") == 0) return StreamMode::imu;
    if (std::strcmp(name, "SUN") == 0) return StreamMode::sun;
    if (std::strcmp(name, "ALL") == 0) return StreamMode::all;
    return StreamMode::off;
}

void handle_command(const char *command)
{
    if (std::strcmp(command, "PING") == 0)
    {
        std::printf("PF,PONG,%u\n", kProtocolVersion);
        return;
    }
    if (std::strcmp(command, "STATUS") == 0)
    {
        print_status();
        return;
    }
    if (std::strcmp(command, "CONFIG") == 0)
    {
        std::printf("PF,CONFIG,imu_zero,%.8f,%.8f,%.8f\n",
                    IMU_ZERO_READING_RPS.x, IMU_ZERO_READING_RPS.y,
                    IMU_ZERO_READING_RPS.z);
        return;
    }
    if (std::strcmp(command, "STOP") == 0 || std::strcmp(command, "DISARM") == 0)
    {
        stream_mode = StreamMode::off;
        stop_and_disarm_magnetorquers();
        std::printf("PF,OK,STOPPED\n");
        return;
    }
    if (std::strcmp(command, "ARM MT") == 0)
    {
        if (!magnetorquer_initialized)
        {
            init_magnetorquer_pwm();
            magnetorquer_initialized = true;
        }
        else
        {
            // STOP/DISARM pulls the driver enable low. Re-arming after a
            // completed test must explicitly restore it even though the PWM
            // pin configuration only needs to be initialized once.
            gpio_put(SAMWISE_ADCS_EN_MAGDRV, 1);
        }
        stop_magnetorquer_pwm();
        magnetorquer_armed = true;
        magnetorquer_arm_deadline_us =
            time_us_64() + kMagnetorquerArmWindowMs * 1000ULL;
        std::printf("PF,OK,MT_ARMED,%u\n", kMagnetorquerArmWindowMs);
        return;
    }

    char mode_name[8]{};
    unsigned int rate_hz = 10;
    if (std::sscanf(command, "STREAM %7s %u", mode_name, &rate_hz) >= 1)
    {
        stream_mode = parse_stream_mode(mode_name);
        rate_hz = std::clamp(rate_hz, 1U, 50U);
        stream_period_ms = 1000U / rate_hz;
        next_stream_us = 0;
        std::printf("PF,OK,STREAM,%s,%u\n", mode_name, rate_hz);
        return;
    }

    char axis = '\0';
    int sign = 0;
    float duty = 0.0f;
    unsigned int duration_ms = 0;
    if (std::sscanf(command, "MT %c %d %f %u", &axis, &sign, &duty,
                    &duration_ms) == 4)
    {
        axis = static_cast<char>(std::toupper(axis));
        run_magnetorquer_pulse(axis, sign, duty, duration_ms);
        return;
    }

    std::printf("PF,ERROR,UNKNOWN_COMMAND,%s\n", command);
}

void service_usb_commands()
{
    static char buffer[96]{};
    static size_t length = 0;

    while (true)
    {
        const int value = getchar_timeout_us(0);
        if (value == PICO_ERROR_TIMEOUT)
        {
            break;
        }
        if (value == '\r' || value == '\n')
        {
            if (length > 0)
            {
                buffer[length] = '\0';
                handle_command(buffer);
                length = 0;
            }
        }
        else if (length + 1 < sizeof(buffer))
        {
            buffer[length++] = static_cast<char>(value);
        }
        else
        {
            length = 0;
            std::printf("PF,ERROR,COMMAND_TOO_LONG\n");
        }
    }
}

void service_stream()
{
    const uint64_t now = time_us_64();
    if (stream_mode == StreamMode::off || now < next_stream_us)
    {
        return;
    }
    next_stream_us = now + stream_period_ms * 1000ULL;

    switch (stream_mode)
    {
        case StreamMode::magnetometer: emit_magnetometer(); break;
        case StreamMode::imu: emit_imu(); break;
        case StreamMode::sun: emit_sun(); break;
        case StreamMode::all:
            emit_magnetometer();
            emit_imu();
            emit_sun();
            break;
        case StreamMode::off: break;
    }
}

}  // namespace

int main()
{
    stdio_init_all();
    init_watchdog_output();
    stop_and_disarm_magnetorquers();
    init_i2c_buses();
    // Leave enough time for USB CDC to enumerate without entering the
    // watchdog's 200 ms high phase immediately before blocking sensor init.
    safe_sleep_ms(500);

    imu_alive = imu_init();
    service_watchdog();
    magnetometer_alive = rm3100_init() == RM3100_OK;
    service_watchdog();
    sun_pyramid_alive = rp2350b_adc_init();
    sun_ads_alive = ads7830_init();

    std::printf("PF,HELLO,%u,samwise-adcs-preflight\n", kProtocolVersion);
    std::printf("PF,CONFIG,imu_zero,%.8f,%.8f,%.8f\n",
                IMU_ZERO_READING_RPS.x, IMU_ZERO_READING_RPS.y,
                IMU_ZERO_READING_RPS.z);
    print_status();

    while (true)
    {
        service_watchdog();
        if (magnetorquer_armed && time_us_64() >= magnetorquer_arm_deadline_us)
        {
            stop_and_disarm_magnetorquers();
            std::printf("PF,OK,MT_ARM_EXPIRED\n");
        }
        service_usb_commands();
        service_stream();
        sleep_ms(1);
    }
}
