#!/bin/sh

sudo openocd -f openocd.cfg -c init -c reset -c exit
make program
