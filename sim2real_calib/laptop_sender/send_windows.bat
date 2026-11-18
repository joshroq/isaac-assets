@echo off
REM ============================================================================
REM  Laptop -> Desktop overhead-camera streamer (Windows / C920e)
REM
REM  Grabs the C920e via DirectShow, H.264-encodes with low latency, and pushes
REM  MPEG-TS over UDP to the desktop's Tailscale IP. Run receive.py on the desktop
REM  FIRST (it just listens), then run this. Ctrl+C here stops the stream.
REM
REM  Prereqs on this laptop:
REM    - ffmpeg installed and on PATH   (winget install Gyan.FFmpeg)
REM    - Tailscale up and connected      (ping %DESKTOP_IP% should work)
REM    - Run list_cameras.bat once to confirm CAM matches your webcam's name.
REM ============================================================================

REM ---- edit these if needed --------------------------------------------------
set DESKTOP_IP=100.107.253.10
set PORT=5000
set CAM=Logi Webcam C920e
set RES=1280x720
set FPS=30
set BITRATE=4M
REM pkt_size 1128 (=6*188) stays under Tailscale's ~1280-byte MTU so UDP packets
REM don't fragment and drop. Do not raise this without lowering it back if laggy.
set PKTSIZE=1128
REM ----------------------------------------------------------------------------

echo Streaming "%CAM%" %RES%@%FPS% to udp://%DESKTOP_IP%:%PORT%
echo Press Ctrl+C to stop.

ffmpeg -hide_banner ^
  -f dshow -rtbufsize 100M -framerate %FPS% -video_size %RES% -vcodec mjpeg -i video="%CAM%" ^
  -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p ^
  -g %FPS% -b:v %BITRATE% -maxrate %BITRATE% -bufsize 1M ^
  -f mpegts "udp://%DESKTOP_IP%:%PORT%?pkt_size=%PKTSIZE%"

pause
