@echo off
taskkill /IM chrome.exe /FI "WINDOWTITLE eq PID*
taskkill /IM cmd.exe /FI "WINDOWTITLE eq C:*"
sleep 3
START cmd.exe /k "node TCPClient 1180 8090"
START cmd.exe /k "node TCPClient 1181 8091"
START cmd.exe /k "node TCPClient 1182 8092"
START cmd.exe /k "node TCPClient 1183 8093"
START cmd.exe /k "node TCPClient 1184 8094"
sleep 3
start "webpage name" "http://127.0.0.1:8090"
start "webpage name" "http://127.0.0.1:8091"
start "webpage name" "http://127.0.0.1:8092"
start "webpage name" "http://127.0.0.1:8093"
start "webpage name" "http://127.0.0.1:8094"
