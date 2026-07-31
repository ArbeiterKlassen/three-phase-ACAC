@echo off
start "" /wait /b "D:\MatLab\bin\matlab.exe" -sd "%~dp0" -batch "run_nuedc_candidate_nominal" 1> "%~dp0validation_results\candidate_nominal_launcher.log" 2> "%~dp0validation_results\candidate_nominal_launcher.err.log"
exit /b %ERRORLEVEL%
