#!/bin/bash
# 强制结束悬停任务进程（放在脚本里执行，避免 pkill 模式匹配到调用者自己的命令行）
if pkill -f "hover_test.py"; then
  echo "已强制结束悬停任务进程"
else
  echo "当前没有正在运行的悬停任务"
fi
