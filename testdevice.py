import speech_recognition as sr
import time
import re

recognizer = sr.Recognizer()
recognizer.energy_threshold = 300


for index, name in enumerate(sr.Microphone.list_microphone_names()):
    print(f'{index}, {name}')