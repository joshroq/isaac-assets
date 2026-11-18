@echo off
REM Lists DirectShow video/audio devices so you can copy the C920e's exact name.
REM Look for something like "Logitech Webcam C920e" under "DirectShow video devices".
REM The (spelling/spacing of the) name goes into send_windows.bat as CAM=...
ffmpeg -hide_banner -list_devices true -f dshow -i dummy
pause
