init_project: 
	pio init --board esp32doit-devkit-v1
generate_commands_json:
	pio run -t compiledb
build:
	pio run
flash:
	pio run --target upload --upload-port /dev/ttyUSB0

