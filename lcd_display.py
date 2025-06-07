from RPLCD.i2c import CharLCD

lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=5, cols=16, rows=2, dotsize=8)
lcd.clear()

lcd.write_string('ezBot \n beep boop')
