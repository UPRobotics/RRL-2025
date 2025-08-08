#!/bin/bash
ffmpeg -hwaccel qsv -rtsp_transport tcp -fflags nobuffer+fastseek -flags low_delay -probesize 32 -analyzeduration 0 \
  -i rtsp://admin:admin@192.168.0.4:8554/profile0 \
  -hwaccel qsv -rtsp_transport tcp -fflags nobuffer+fastseek -flags low_delay -probesize 32 -analyzeduration 0 \
  -i rtsp://admin:admin@192.168.0.5:8554/profile0 \
  -hwaccel qsv -rtsp_transport tcp -fflags nobuffer+fastseek -flags low_delay -probesize 32 -analyzeduration 0 \
  -i rtsp://admin:admin@192.168.0.6:8554/profile0 \
  -c:v hevc_qsv -pix_fmt nv12 \
  -filter_complex "[0:v][1:v][2:v]hstack=inputs=3[v]" \
  -map "[v]" -f rawvideo -pix_fmt yuv420p pipe: | \
ffplay -f rawvideo -pix_fmt yuv420p -s 5760x1080 -fflags nobuffer -an -