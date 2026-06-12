// Ning-Everhart fork (2026-06-12 锡): IMU 抱起/摇晃检测.
//
// 移植自 mo-hantang/Stackchan-HtSz main/boards/m5stack-core-s3/m5stack_core_s3.cc
// (MIT, 同上游). 原项目通过 SendUserMessage(PickRandom(LiftPool)) 喂自家 xiaozhi
// AI; 我们改成 EventCallback, 调用方 (stackchan.cc) 转 Application::SendStackChanEvent
// 上报 MCP notification stackchan/event subtype=lift|shake.
//
// 算法:
//   每 100ms 读 BMI270 accel (±8g, 1g≈4096). MOTION_THRESHOLD 0.3g.
//   - 1 秒内 ≥2 个尖峰 → shake
//   - 连续 ≥5 样本 (500ms) 持续偏离 1g → lift
//   - 触发后 disarm, 静止 5s 后 re-arm
//   - 全局冷却 5 分钟 (防误触发)
//
// 注意: K151 BMI270 实际 I2C 地址 0x69 (不是 SDK 默认 0x68), 用自定义
// read/write callback 绕过 bmi270_sensor_create 硬编码限制.
#pragma once

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>

class MotionDetector {
public:
    // 回调签名: subtype = "lift" | "shake", duration_ms 是触发动作的持续时长估算
    using EventCallback = void(*)(const char* subtype, uint64_t duration_ms, void* user_data);

    MotionDetector() = default;
    ~MotionDetector();

    MotionDetector(const MotionDetector&) = delete;
    MotionDetector& operator=(const MotionDetector&) = delete;

    // i2c_bus 是 board 已初始化的 I2C bus handle.
    // cb / user_data 是事件回调 (motion task 触发 lift/shake 时调).
    // 返 true 表示 BMI270 初始化+task启动成功; false 表示 BMI270 找不到/失败,
    // motion 探测不工作, 但其他功能不受影响.
    bool Initialize(i2c_master_bus_handle_t i2c_bus, EventCallback cb, void* user_data);

private:
    static void TaskFunc(void* arg);
    void Loop();

    i2c_master_dev_handle_t i2c_dev_ = nullptr;
    // bmi2_dev*: 在 cc 里 new, 避免在头里 include Bosch SDK 头污染调用方.
    void* bmi_dev_impl_ = nullptr;
    void* bmi_handle_ = nullptr;
    TaskHandle_t task_ = nullptr;
    int64_t last_trigger_us_ = 0;
    EventCallback callback_ = nullptr;
    void* user_data_ = nullptr;
};
