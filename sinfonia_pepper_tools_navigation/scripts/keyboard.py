#!/usr/bin/env python

#========================================================================================
#			TRACEBACK
#
#	14-02-2019:	Keyboard commands to read keyboard inputs.
#
#========================================================================================

#========================================================================================
#			IMPORTS
#========================================================================================

# Importing module to run class as a thread.
from threading import Thread

# Importing module to read keyboard.
from pynput.keyboard import Key, Listener

#========================================================================================
#			CLASSES
#========================================================================================

#----------------------------------------------------------------------------------------
# Class to read keyboard inputs.
#----------------------------------------------------------------------------------------
class Keyreader(Thread):

	#------------------------------------------------------------------------------------
	# Class initialization function.
	# Parameters:	None
	#------------------------------------------------------------------------------------
	def __init__(self):
		Thread.__init__(self)
		self.num = 6

	#------------------------------------------------------------------------------------
	# On press function. Will assign a value if one of the keys is pressed.
	# Parameters:	key		->	Key that was pressed.
	#------------------------------------------------------------------------------------
	def on_press(self, key):
		key = str(key)
		char = key.replace("u'", "")
		char = char.replace("'", "")
		print(char)
		if char == 'w':
			self.num = 0
		elif char == 's':
			self.num = 1
		elif char == 'a':
			self.num = 3
		elif char == 'd':
			self.num = 2
		elif char == 'e':
			self.num = 4
		elif char == 'q':
			self.num = 5

	#------------------------------------------------------------------------------------
	# On released function. Return the num value to its initial state.
	# Parameters:	key		->	Key that was relaesed.
	#------------------------------------------------------------------------------------
	def on_release(self, key):
		if key == Key.esc:
			return False
		else:
			self.num = 6

	#------------------------------------------------------------------------------------
	# Function that return the number that is stored.
	# Parameters:	None
	#------------------------------------------------------------------------------------
	def getNumber(self):
		return self.num

	#------------------------------------------------------------------------------------
	# Thread run fuction. Runs when the thread is started.
	# Parameters:	None
	#------------------------------------------------------------------------------------
	def run(self):
		with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
				listener.join()



#========================================================================================
#			CLASS TEST
# Test script. Keep commented.
#========================================================================================
# key = keyreader()
# key.start()
#
# while True:
#	num = key.getNumber()
#	print(num)
