avrdude -v -V -p m1284p -c stk500v2 -P COM3 -b 115200 -U flash:w:"..\Release\IronAHRS.hex"
pause
