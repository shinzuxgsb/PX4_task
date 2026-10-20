这里放的是 pymavlink 2.4.50 + fastcrc 0.5.0（aarch64/cp310 wheel，直接用 wheel 解出来的）。
因为香橙派上没有 pip，也不该为了这件事动系统包，所以整套用 PYTHONPATH 挂载：
    export PYTHONPATH=/home/orangepi/mavlink_ws/third_party
start_mavlink_ws.sh 里已经自动 export 过了。
