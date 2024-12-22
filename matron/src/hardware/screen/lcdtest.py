#sudo apt-get install python3-pil
#sudo apt-get install fonts-dejavu

import spidev
import RPi.GPIO as GPIO
import numpy as np
import time
from PIL import Image, ImageDraw

class ST7789:
    def __init__(self, dc_pin=25, rst_pin=27, spi_bus=0, spi_device=0):
        # Display constants
        self.width = 240
        self.height = 320
        self.rotation = 0
        
        # GPIO setup
        self.dc_pin = dc_pin
        self.rst_pin = rst_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dc_pin, GPIO.OUT)
        GPIO.setup(self.rst_pin, GPIO.OUT)
        
        # SPI setup
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = 40000000
        self.spi.mode = 0
        
        self._init_display()
        # Create image buffer
        self.buffer = Image.new('RGB', (self.width, self.height))
        self.draw = ImageDraw.Draw(self.buffer)

    def _init_display(self):
        # Hardware reset
        GPIO.output(self.rst_pin, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(self.rst_pin, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(self.rst_pin, GPIO.HIGH)
        time.sleep(0.12)
        
        # Init sequence
        self._write_cmd(0x36, [0x00])  # MADCTL
        self._write_cmd(0x3A, [0x05])  # COLMOD
        self._write_cmd(0xB2, [0x0C, 0x0C, 0x00, 0x33, 0x33])  # PORCTRL
        self._write_cmd(0xB7, [0x35])  # GCTRL
        self._write_cmd(0xBB, [0x19])  # VCOMS

    def write_text(self, text, position=(0,0), font_size=20, color=(255,255,255)):
        """Write text to LCD display
        Args:
            text (str): Text to display
            position (tuple): (x,y) position
            font_size (int): Font size in pixels
            color (tuple): RGB color tuple
        """
        from PIL import ImageFont
        
        # Load default font
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", font_size)
        except:
            font = ImageFont.load_default()
            
        # Draw text on buffer
        self.draw.text(position, text, font=font, fill=color)
        
        # Convert buffer to RGB565 and send to display
        self._send_buffer()
        
    def _send_buffer(self):
        """Convert buffer to RGB565 and send to display"""
        # Convert PIL buffer to RGB565 bytes
        pixels = np.array(self.buffer).astype('uint16')
        rgb565 = ((pixels[:,:,0] & 0xF8) << 8) | ((pixels[:,:,1] & 0xFC) << 3) | (pixels[:,:,2] >> 3)
        
        # Set display address window
        self._write_cmd(0x2A, [0x00, 0x00, 0x00, self.width-1])  # Column
        self._write_cmd(0x2B, [0x00, 0x00, 0x00, self.height-1]) # Row
        self._write_cmd(0x2C) # Memory write
        
        # Send data
        GPIO.output(self.dc_pin, GPIO.HIGH)
        self.spi.xfer(rgb565.tobytes())

    def clear(self):
        """Clear display buffer"""
        self.buffer = Image.new('RGB', (self.width, self.height))
        self.draw = ImageDraw.Draw(self.buffer)

# Example usage
lcd = ST7789()
lcd.clear()
lcd.write_text("Hello World!", (10,10), font_size=30, color=(255,0,0))
