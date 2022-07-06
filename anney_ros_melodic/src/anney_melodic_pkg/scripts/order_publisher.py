#! /usr/bin/env python3
import rospy
# from std_msgs.msg import String
from anney_melodic_pkg.msg import Order
import re
import speech_recognition as sr
from pynput import keyboard
import pyaudio
import wave
from os import system
import time

CHUNK = 8192
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
WAVE_OUTPUT_FILENAME = "output.wav"
order_list = []
unit_list = []

def text2num(text):
    ''' This method is standardize all numerical text into numbers. (eg. "satu nasi lemak --> 1 nasi lemak)
        This method accept a string and return the standardize string. '''

    text = text.lower()
    num_dict = {"satu": "1", "one": "1","dua": "2", "two": "2","tiga": "3", "three": "3","empat": "4", "four": "4","lima": "5", "five": "5","enam": "6", "six": "6","tujuh": "7", "seven": "7","lapan": "8", "eight": "8","sembilan": "9", "nine": "9",}
    tokens = text.split(" ")
    keys = num_dict.keys()
    for i in range(len(tokens)):
        for key in keys:
            if(tokens[i] == key):
                tokens[i] = num_dict[key]
    
    

    return ' '.join(tokens)

class MyListener(keyboard.Listener):
    def __enter__(self):
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=FORMAT,
                             channels=CHANNELS,
                             rate=RATE,
                             input=True,
                             frames_per_buffer=CHUNK,
                             stream_callback = self.callback,
                             input_device_index=12)
        self.start() # Speech stream starts here, the stream can only be start once
        return self
    def __init__(self):
        super(MyListener, self).__init__(on_press=self.on_press, on_release=self.on_release)
        self.key_pressed = False
        self.complete_tag = False
        self.frames = []

    def on_press(self, key):
        if key == keyboard.Key.space:
            self.key_pressed = True

    def on_release(self, key):
        if key == keyboard.Key.space:
            print('Press and hold space to continue ordering or right-arrow key button if already done')
            self.key_pressed = False
        if key == keyboard.Key.right:
            print('Done recording...')
            self.complete_tag = True

            return False

    def callback(self,in_data, frame_count, time_info, status):
        callback_flag = pyaudio.paContinue
        if self.key_pressed:
            # If user press and hold 'spacebar', it will append the frames data
            self.frames.append(in_data)
        if self.complete_tag:
            # If user press esc button, it will stop the stream and start to create a wavfile of the order recording
            callback_flag = pyaudio.paComplete
        return in_data, callback_flag

    def __exit__(self, exc_type, exc_value, traceback):
        ''' When the stream is stopped, this method will automatically be called to create the wavfile '''

        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        self.stop()
        wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(self.p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(self.frames))
        wf.close()

        ''' Initialize speech recognition object and take the order recording as the audiofile to be recognized '''
        recognizer = sr.Recognizer()
        recognizer.energy_threshold = 50 # Higher than this consider speech, lower consider silent
        audiofile = sr.AudioFile('output.wav')

        ''' Start the recognition process '''
        with audiofile as source:
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.record(source)
    
            try:
                # Using google speech recognition
                result = recognizer.recognize_google(audio_data=audio,language='ms-MY') #chose ms-MY for the Malay use case
                result = text2num(result)
                print("Text: "+ result)

                # The pattern that needs to be in the order to be recognized
                pattern = r'[iI] want to order'
                match = re.search(pattern,result)
                global unit_list
                global order_list
                if(match):
                    # If there is a match, start to process the unit groups and the order(food) groups
                    
                    matches = re.findall("(([0-9]+) ([A-Za-z\s]+))", result)
                    for match in matches:
                        unit_list.append(int(match[1]))
                        order_list.append(match[2].strip().replace(" ", "_"))
                    
                    if(len(unit_list) != 0 or len(order_list) != 0):
                        print(unit_list)
                        print(order_list)
                    else:
                        print('Sorry I didnt get that, can you repeat that clearly?')
                    
                else:
                    print('Sorry I didnt get that, can you repeat that clearly?')
            except:
                print("Sorry, I did not get that")


def talk_to_me():
    # Create a topic for publisher
    pub = rospy.Publisher("order_topic", Order, queue_size=10)

    # Initialize publisher node
    rospy.init_node("publisher_node", anonymous=True)
    rospy.loginfo("Publisher Node Started")
    global order_list
    global unit_list
    while not rospy.is_shutdown():
        # Custom message which contains the table number, array of ordered food and an array of their corresponding quantities
        msg = Order()

        # Customer insert the table number
        table_num = input("Please enter your table number:")
        with MyListener() as listener:
            #anney robot start listening to take order
            print("You may order now.. Hold Spacebar to order")
            listener.join()

        # Store all information needed in custom message
        # If the order OR unit is empty, will not publish the message to subscriber
        if(len(order_list) != 0 or len(unit_list) != 0):
            msg.table_num = str(table_num)
            msg.orders = order_list
            msg.units = unit_list

            # Publish the message
            pub.publish(msg)

            # Clear the previous order and unit list
            order_list = []
            unit_list = []
        time.sleep(3)
        system('clear')
        table_num = None


if __name__ == "__main__":
    try:
        talk_to_me()
    except rospy.ROSInterruptException:
        pass