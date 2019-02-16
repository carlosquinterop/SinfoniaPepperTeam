#!/usr/bin/env python
# -*- coding: utf-8 -*-
from sinfonia_pepper_tools_interaction.srv import *
# from sinfonia_pepper_robot_toolkit.msg import MoveToVector, MoveTowardVector, Wav, T2S, File
from std_msgs.msg import String, Float64MultiArray
import rospy
from gtts import gTTS
# import sounddevice as sd
import os
import wave
import time
from os import path
from pydub import AudioSegment
import soundfile as sf
import numpy as np
import speech_recognition as sr
import sys
import getpass
username = getpass.getuser()
reload(sys)
sys.setdefaultencoding('utf-8')
source = '0'
micData = []
def callback_t2s(req):
    global source
    print ("Returning [%s]"%(req.text_t2s))
    tts = gTTS(text=req.text_t2s, lang='es', slow=False)
    tts.save('/home/'+username+'/good2.mp3')
    sound = AudioSegment.from_mp3('/home/'+username+'/good2.mp3')
    sound.export('/home/'+username+'/output.wav', format="wav")
    source='1'
    if source=='1':
        os.system('mpg321 /home/'+username+'/good2.mp3')
        return speakResponse(True)
    elif source=='2':
        Speakers()


def Speakers():
    while pub_audio.get_num_connections() == 0:
        rate = rospy.Rate(10)
        rate.sleep()
    msg = File()
    path ='/home/'+username+'/output.wav'
    with open(path, "rb") as f:
        msg.data = f.read()
    msg.extension = path.split('.')[-1]
    pub_audio.publish(msg)


def callback_s2t(req):
    global mp3_fp,micData, source
    a=0
    while a==0:
        print ("Returning [%s]"%(req.send))
        tts = gTTS(text=req.send, lang='es')
        tts.save('/home/'+username+'/good2.mp3')
        sound = AudioSegment.from_mp3('/home/'+username+'/good2.mp3')
        sound.export('/home/'+username+'/output.wav', format="wav")
        source='1'
        if source=='1':
            os.system('mpg321 /home/'+username+'/good2.mp3')
        elif source=='2':
            Speakers()

        if source=='1':
            r = sr.Recognizer()
            with sr.Microphone() as source:
                print('say something')
                audio=r.listen(source)
                print('time over')
            try:
                return listenResponse(r.recognize_google(audio,language="es-CO"))
                a=1
            except sr.UnknownValueError:
                print("no se entendio")
            except sr.RequestError as e:
                return listenResponse("Could not request results from Google Speech Recognition service; {0}".format(e))
        elif source=='2':
            time.sleep(5)
            print('say something')
            Mic()
            print('time over')
            sf.write('/home/'+username+'/stereo_file.wav',micData, 16000, 'PCM_24')
            AUDIO_FILE = os.path.join('/home/'+username+'/stereo_file.wav')
            r = sr.Recognizer()
            with sr.AudioFile(AUDIO_FILE) as source:
                audio = r.record(source)  # read the entire audio file
            try:
                return listenResponse(r.recognize_google(audio,language="es-CO"))
                a=1
            except sr.UnknownValueError:
                print("no se entendio")
            except sr.RequestError as e:
                return listenResponse("Could not request results from Google Speech Recognition service; {0}".format(e))


def Mic():
    global micData, pub
    micData = []
    print(micData)
    while pub.get_num_connections() == 0:
    	rate = rospy.Rate(10)
    	rate.sleep()
    pub.publish("sIA_mic_raw.1.ON")
    time.sleep(2)
    pub.publish("sIA_mic_raw.1.OFF")
    time.sleep(2)
    sd.play(np.array(micData), 16000, mapping=1, blocking=True)


def MicCallback(data):
    global micData
    micData += data.data


def callback_options():
    global pub, pub_audio
    rospy.init_node('sIA_s2t_t2s')
    s = rospy.Service('srvSpeak', speak, callback_t2s)
    v = rospy.Service('srvListen', listen, callback_s2t)
    pub = rospy.Publisher("sIA_stream_from", String, queue_size=10)
    # pub_audio = rospy.Publisher("sIA_play_audio", File, queue_size=10)
    rospy.Subscriber("sIA_mic_raw", Float64MultiArray, MicCallback)
    print ("Ready ")
    rospy.spin()

if __name__ == "__main__":
    callback_options()
