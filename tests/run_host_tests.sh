#!/usr/bin/env bash
# PC-side tests of the REAL firmware sources (Linux/macOS/WSL, needs g++, python3, numpy, ffmpeg for the MP3 test).
# UART is a pty, I2S is a fake that records the PCM. Does not need an ESP32.
set -e
cd "$(dirname "$0")"; FW=../mp3
g++ -std=gnu++11 -O2 -Wall -Wextra -pthread -Ishim -I$FW -o sim_device sim_device.cpp \
    $FW/stream_rx.cpp $FW/stream_proto.cpp $FW/audio_out.cpp $FW/stream_player.cpp $FW/audio_levels.cpp
g++ -std=gnu++11 -O2 -I$FW -o test_proto test_proto.cpp $FW/stream_proto.cpp
g++ -std=gnu++11 -O2 -I$FW -o test_levels test_levels.cpp $FW/audio_levels.cpp
./test_levels s
SIM_SPEED=1 SIM_OUT=/tmp/i2s_tone.raw ./sim_device > /tmp/sim.port 2>/tmp/sim.log & SIM=$!
sleep 1; PORT=$(head -1 /tmp/sim.port)
PYTHONPATH=stubserial python3 ../play_mp3.py --port "$PORT" --test-tone --tone-seconds 4 --boot-wait 0.2 --quiet
kill $SIM
echo "For the full-MP3 bit-exact comparison see README section 'What was verified'."
