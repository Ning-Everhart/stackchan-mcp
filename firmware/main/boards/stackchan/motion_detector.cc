// Ning-Everhart fork (2026-06-12 锡): IMU 抱起/摇晃检测实现.
// 移植自 mo-hantang/Stackchan-HtSz (MIT). 详见 motion_detector.h 头文件注释.
#include "motion_detector.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <math.h>
#include <string.h>

// Bosch BMI270 SDK (espressif/bmi270_sensor component, 0.1.1+).
// 跟 Stackchan-HtSz 一致: bmi270_api.h (component 公开 API) + bmi2.h (Bosch
// SDK 原生头, 含 BMI2_OK / BMI2_ACCEL / bmi2_sens_data 等). 不 include bmi2_defs.h,
// bmi2.h 内部已 include 它了 (重复 include 在 GCC 也 OK, 但少一行少风险).
extern "C" {
#include "bmi270_api.h"
#include "bmi2.h"
}

#define TAG "MotionDetector"

// ===== BMI270 SDK 自定义 I2C 接口 =====
// K151 上 BMI270 实际地址是 0x69, 而 bmi270_sensor_create 硬编码 0x68.
// 我们绕开它, 用自己 I2C handle 实现 read/write/delay 回调, 直接调底层 bmi270_init.

extern "C" {
// SDK 在 .a 里有这些 public 符号但头文件未必都暴露, 显式声明用来绕过 0x68 限制.
int8_t bmi270_init(struct bmi2_dev *dev);
extern const uint8_t bmi270_config_file[];
}

static int8_t Bmi270I2cRead(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    auto dev = (i2c_master_dev_handle_t)intf_ptr;
    return i2c_master_transmit_receive(dev, &reg_addr, 1, data, len, 200) == ESP_OK ? 0 : -1;
}

static int8_t Bmi270I2cWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    // BMI270 config blob 一次最大 ~8KB, 静态分配避免栈上大数组
    static uint8_t big_buf[8300];
    if (len + 1 > sizeof(big_buf)) return -1;
    auto dev = (i2c_master_dev_handle_t)intf_ptr;
    big_buf[0] = reg_addr;
    memcpy(big_buf + 1, data, len);
    return i2c_master_transmit(dev, big_buf, len + 1, 500) == ESP_OK ? 0 : -1;
}

static void Bmi270DelayUs(uint32_t period_us, void * /*intf_ptr*/) {
    if (period_us < 1000) {
        esp_rom_delay_us(period_us);
    } else {
        vTaskDelay(pdMS_TO_TICKS((period_us + 999) / 1000));
    }
}

// ===== MotionDetector 实现 =====

MotionDetector::~MotionDetector() {
    if (task_) {
        vTaskDelete(task_);
        task_ = nullptr;
    }
    if (bmi_dev_impl_) {
        delete static_cast<bmi2_dev*>(bmi_dev_impl_);
        bmi_dev_impl_ = nullptr;
    }
    // i2c_dev_ 不主动 remove, 跟着 bus 走
}

bool MotionDetector::Initialize(i2c_master_bus_handle_t i2c_bus, EventCallback cb, void* user_data) {
    if (!i2c_bus || !cb) {
        ESP_LOGE(TAG, "Initialize: i2c_bus or callback is null");
        return false;
    }
    callback_ = cb;
    user_data_ = user_data;

    // 1. 加 I2C device @ 0x69 (K151 BMI270 实际地址)
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = 0x69;
    dev_cfg.scl_speed_hz = 400000;
    if (i2c_master_bus_add_device(i2c_bus, &dev_cfg, &i2c_dev_) != ESP_OK) {
        ESP_LOGW(TAG, "BMI270: add i2c device failed (no IMU? continuing without)");
        return false;
    }

    // 2. 构造 bmi2_dev: 自定义 read/write 走我们的 i2c_dev_ @ 0x69
    auto* dev = new bmi2_dev();
    bmi_dev_impl_ = dev;
    dev->intf = BMI2_I2C_INTF;
    dev->intf_ptr = i2c_dev_;
    dev->read = Bmi270I2cRead;
    dev->write = Bmi270I2cWrite;
    dev->delay_us = Bmi270DelayUs;
    dev->read_write_len = 256;
    dev->config_file_ptr = bmi270_config_file;
    dev->dummy_byte = 0;

    // 3. 初始化 BMI270 (load config blob + 启动 chip)
    int8_t rslt = bmi270_init(dev);
    if (rslt != BMI2_OK) {
        ESP_LOGW(TAG, "bmi270_init failed: %d (no IMU? continuing without)", rslt);
        delete dev;
        bmi_dev_impl_ = nullptr;
        return false;
    }
    ESP_LOGI(TAG, "BMI270 initialized (custom driver @ 0x69, chip_id=0x%02X)", dev->chip_id);

    bmi_handle_ = dev;

    const uint8_t sens_list[] = {BMI2_ACCEL};
    rslt = bmi270_sensor_enable(sens_list, 1, dev);
    if (rslt != BMI2_OK) {
        ESP_LOGW(TAG, "bmi270_sensor_enable failed: %d", rslt);
        bmi_handle_ = nullptr;
        delete dev;
        bmi_dev_impl_ = nullptr;
        return false;
    }
    ESP_LOGI(TAG, "BMI270 accel enabled");

    // 4. 启 motion task (核心 1, 跟 main app 同核但低 priority)
    xTaskCreatePinnedToCore(TaskFunc, "imu_motion", 4096, this, 1, &task_, 1);
    return true;
}

void MotionDetector::TaskFunc(void* arg) {
    static_cast<MotionDetector*>(arg)->Loop();
    vTaskDelete(nullptr);
}

void MotionDetector::Loop() {
    // 算法 (跟 Stackchan-HtSz 一致):
    //   每 100ms 读 accel; magnitude 偏离 1g 或轴变化大 算"动"
    //   1 秒内 ≥2 尖峰 → shake; 连续 ≥5 样本 → lift
    //   触发后 disarm, 静止 5s re-arm, 5min 全局冷却
    constexpr float MOTION_THRESHOLD = 0.3f;
    constexpr int SHAKE_PEAKS_TO_TRIGGER = 2;
    constexpr int64_t SHAKE_WINDOW_US = 1'000'000;
    constexpr int LIFT_SAMPLES_TO_TRIGGER = 5;
    constexpr int STILL_SAMPLES_TO_REARM = 50;
    constexpr int64_t GLOBAL_COOLDOWN_US = 5LL * 60 * 1'000'000;

    int lift_count = 0;
    int still_count = 0;
    bool armed = true;
    int64_t shake_peak_times[8] = {0};
    int shake_idx = 0;
    float last_ax = 0, last_ay = 0, last_az = 0;
    bool last_valid = false;
    auto* dev = static_cast<bmi2_dev*>(bmi_handle_);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!dev) continue;

        struct bmi2_sens_data accel = {};
        int8_t rd = bmi2_get_sensor_data(&accel, dev);
        if (rd != BMI2_OK) {
            continue;
        }

        // BMI270 默认 ±8g 量程, int16 raw, 1g ≈ 4096
        float ax = (float)accel.acc.x / 4096.0f;
        float ay = (float)accel.acc.y / 4096.0f;
        float az = (float)accel.acc.z / 4096.0f;
        float mag = sqrtf(ax * ax + ay * ay + az * az);

        // 轴变化率 (旋转/摇晃只改各轴分量, 不改 mag)
        float delta = 0.0f;
        if (last_valid) {
            float dx = ax - last_ax;
            float dy = ay - last_ay;
            float dz = az - last_az;
            delta = sqrtf(dx * dx + dy * dy + dz * dz);
        }
        last_ax = ax; last_ay = ay; last_az = az;
        last_valid = true;

        bool moving = (delta > MOTION_THRESHOLD) || (fabsf(mag - 1.0f) > MOTION_THRESHOLD);
        int64_t now = esp_timer_get_time();

        if (!moving) {
            still_count++;
            if (still_count >= STILL_SAMPLES_TO_REARM) armed = true;
            lift_count = 0;
            for (int i = 0; i < 8; i++) shake_peak_times[i] = 0;
            continue;
        }

        still_count = 0;
        if (!armed) continue;
        if (last_trigger_us_ != 0 && (now - last_trigger_us_) < GLOBAL_COOLDOWN_US) continue;

        // 摇晃检测
        shake_peak_times[shake_idx % 8] = now;
        shake_idx++;
        int peak_count = 0;
        for (int i = 0; i < 8; i++) {
            if (shake_peak_times[i] > 0 && (now - shake_peak_times[i]) < SHAKE_WINDOW_US) {
                peak_count++;
            }
        }
        if (peak_count >= SHAKE_PEAKS_TO_TRIGGER) {
            armed = false;
            lift_count = 0;
            last_trigger_us_ = now;
            for (int i = 0; i < 8; i++) shake_peak_times[i] = 0;
            // duration_ms 估算: 取尖峰窗口 (shake 是短促, 约 1 秒内多次)
            if (callback_) callback_("shake", 1000, user_data_);
            continue;
        }

        // 抱起检测
        lift_count++;
        if (lift_count >= LIFT_SAMPLES_TO_TRIGGER) {
            armed = false;
            lift_count = 0;
            last_trigger_us_ = now;
            // duration_ms = 5 样本 × 100ms = 500ms (持续偏离 1g 的时长)
            if (callback_) callback_("lift", 500, user_data_);
        }
    }
}
