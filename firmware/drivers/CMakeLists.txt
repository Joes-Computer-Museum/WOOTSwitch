cmake_minimum_required(VERSION 3.13)

target_sources(hootswitch PRIVATE
		joystick.c keyboard.c mouse.c serial.c
		)
