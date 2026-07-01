#!/bin/bash
# 设置i2c速度
NEW_SPEED=100000
sudo bash -c "echo '$NEW_SPEED' > /sys/bus/i2c/devices/i2c-1/bus_clk_rate"

