#!/bin/bash

start=0
duration=5

for file in $(ls wav_long); do
    ffmpeg -ss $start -i "~/tts/wav_long/$file" -t $duration -c copy "~/tts/wav/$file";

    if [ $? -ne 0]; then
        break
    fi
    
done