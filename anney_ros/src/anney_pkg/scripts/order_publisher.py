#!/usr/bin/env
import rospy
# from std_msgs.msg import String
from anney_pkg.msg import Order
import re
import speech_recognition as sr
from pynput import keyboard
import pyaudio
import wave

CHUNK = 8192
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
WAVE_OUTPUT_FILENAME = "output.wav"
order_list = []
unit_list = []

def text2num(text):
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
                             input_device_index=3)
        self.start()
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
            print('Press and hold space to continue ordering or ESC button if already done')
            self.key_pressed = False
        if key == keyboard.Key.esc:
            print('Done recording...')
            self.complete_tag = True

            return False

    def callback(self,in_data, frame_count, time_info, status):
        callback_flag = pyaudio.paContinue
        if self.key_pressed:
            self.frames.append(in_data)
        if self.complete_tag:
            callback_flag = pyaudio.paComplete
        # print(callback_flag)
        return in_data, callback_flag

    def __exit__(self, exc_type, exc_value, traceback):
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

        recognizer = sr.Recognizer()
        recognizer.energy_threshold = 50 # higher than this consider speech, lower consider silent
        audiofile = sr.AudioFile('output.wav')

        with audiofile as source:
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.record(source)
    
            try:
                result = recognizer.recognize_google(audio_data=audio,language='ms-MY')
                result = text2num(result)
                # using google speech recognition
                print("Text: "+ result)

                pattern = r'[iI] want to order'
                match = re.search(pattern,result)
                if(match):
                    matches = re.findall("(([0-9]+) ([A-Za-z\s]+))", result)

                    for match in matches:
                        unit_list.append(int(match[1]))
                        order_list.append(match[2].strip().replace(" ", "_"))

                    print(unit_list)
                    print(order_list)
                    
                else:
                    print('Sorry I didnt get that, can you repeat that clearly?')
            except:
                print("Sorry, I did not get that")


def talk_to_me():
    pub = rospy.Publisher("order_topic", Order, queue_size=10)
    rospy.init_node("publisher_node", anonymous=True)
    rospy.loginfo("Publisher Node Started")

    while not rospy.is_shutdown():
        msg = Order()
        

        table_num = int(input("Please enter your table number:"))
        with MyListener() as listener:
            print("You may order now..")
            listener.join()
        # order = input("Enter your order:")
        # matches = re.findall("(([0-9]+) ([A-Za-z\s]+))", order)

        # for match in matches:
        #     unit_list.append(int(match[1]))
        #     order_list.append(match[2].strip().replace(" ", "_"))

        msg.table_num = table_num
        msg.orders = order_list
        msg.units = unit_list

        pub.publish(msg)


if __name__ == "__main__":
    try:
        talk_to_me()
    except rospy.ROSInterruptException:
        pass
