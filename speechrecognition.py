import speech_recognition as sr
import time
import re

recognizer = sr.Recognizer()
recognizer.energy_threshold = 300


# for index, name in enumerate(sr.Microphone.list_microphone_names()):
#     print(f'{index}, {name}')

with sr.Microphone(17) as source:
    recognizer.adjust_for_ambient_noise(source,duration=0.5)
    # time.sleep(0.5)
    print('please order')
    audio = recognizer.listen(source,phrase_time_limit = 5)
    try:
        detection = recognizer.recognize_google(audio_data=audio, language='ms-MY')
        print(detection)

        pattern = r'[iI] want to order'
        match = re.search(pattern,detection)
        if(match):
            detection_pattern = r'([0-9]+ [A-Za-z\s]+)'
            orders = re.findall(detection_pattern,detection)
            for order in orders:
                print(order)
            # TODO: check the received match and groups
            
        else:
            print('Sorry I didnt get that, can you repeat that clearly?')
    except:
        print('no sound la dey')

    

    


