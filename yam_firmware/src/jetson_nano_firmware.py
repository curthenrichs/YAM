#!/usr/bin/env python

import time
import socket
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


DEFAULT_LOOP_TIMESTEP = 0.25


class JetsonNanoFirmware:
	
	def __init__(self, loop_time_step=DEFAULT_LOOP_TIMESTEP):	
		
		self._current_ip_address = self._get_ip_address()
		
		self.display_setup()
		self.print_ip_address(self._current_ip_address)

	def _get_ip_address(self):
		ip_address = '';
		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		s.connect(("8.8.8.8",80))
		ip_address = s.getsockname()[0]
		s.close()
		return ip_address
		
	def display_setup(self):
		# 128x32 display with hardware I2C:
		self._disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1) # setting gpio to 1 is hack to avoid platform detection
		
		try:
			# Initialize library.
			self._disp.begin()
			
			# Clear display.
			self._disp.clear()
			self._disp.display()

			# Create blank image for drawing.
			# Make sure to create image with mode '1' for 1-bit color.
			self._width = self._disp.width
			self._height = self._disp.height
			self._image = Image.new('1', (self._width, self._height))

			self._padding = 2
			self._top = self._padding
			self._bottom = self._height - self._padding

			# Get drawing object to draw on image.
			self._draw = ImageDraw.Draw(self._image)

			# Draw a black filled box to clear the image.
			self._draw.rectangle((0,0,self._width,self._height), outline=0, fill=0)

			# Load default font.
			self._font = ImageFont.load_default()
		except:
			pass

	def print_ip_address(self, address):
		try:
			# Draw a black filled box to clear previous text
			self._draw.rectangle((0,0,self._width,self._height/2), outline=0, fill=0)
			
			# Write IP Address
			self._draw.text((self._padding, self._top), 'IP: {0}'.format(address),  font=self._font, fill=255)
			print self._get_ip_address()

			# Display image.
			self._disp.image(self._image)
			self._disp.display()
		except:
			pass
	
	def spin(self):
		try:  
			while True: 
				time.sleep(self._loop_time_step)
				
				# Check IP address
				ip_address = self._get_ip_address()
				if ip_address != self._current_ip_address:
					self._current_ip_address = ip_address
					self.print_ip_address(self._current_ip_address)
		except:
			pass


if __name__ == "__main__":
	node = JetsonNanoFirmware()
	
	node.spin()
