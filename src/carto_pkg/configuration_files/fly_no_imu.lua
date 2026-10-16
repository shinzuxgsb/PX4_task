-- 2026-10-09：不带 IMU 的 cartographer 配置（飞控没上电 / IMU 不通时用）
--
-- 为什么需要这一份：
--   cartographer 的 sensor bridge 是"多队列按时间排序"的。一旦
--   TRAJECTORY_BUILDER_2D.use_imu_data = true，它会**等 IMU 数据**才处理扫描 ——
--   没有 IMU 时日志会一直刷
--       W ordered_multi_queue.cc:155] Queue waiting for data: (0, imu)
--   而且**一个位姿都不输出**（实测：飞控没上电 → /tracked_pose 完全没有数据）。
--   所以"飞控不在"的时候绝不能用带 IMU 的那份配置。
--
-- 这份文件只做一件事：把 fly.lua 原样加载进来，再把 use_imu_data 关掉。
-- **配置只有一个来源（fly.lua）**，以后改参数只改那一份，别在这里重复写。
-- （这是 cartographer 官方的写法，见 backpack_2d_localization.lua: include + 覆盖 + return options）

include "fly.lua"

TRAJECTORY_BUILDER_2D.use_imu_data = false

return options
