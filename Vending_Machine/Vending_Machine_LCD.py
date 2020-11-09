import RPi.GPIO as GPIO
import Adafruit_CharLCD as LCD

lcd_rs = 6
lcd_en = 13
lcd_d4 = 19
lcd_d5 = 26
lcd_d6 = 21
lcd_d7 = 20
lcd_backlight = 4 

# Specify a 20x4 LCD.
lcd_columns = 20
lcd_rows    = 4

# Initialize the LCD using the pins above.
lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7,lcd_columns, lcd_rows, lcd_backlight)
lcd.clear()
lcd.message('AUTOMATED VENDING')
lcd.set_cursor(0,2)
lcd.message('MACHINE')
print("Automated Vending Machine")
