#!/usr/bin/env python
import roslib; roslib.load_manifest('wheelchairint')
import pyaudio
import wave
import audioop
from collections import deque 
import os
import urllib2
import time

import rospy
import math
from std_msgs.msg import String


class Recognizer:
    """
    Does speech recognition using Google's speech  recognition service.
    Records sound from microphone until silence is found and save it as WAV and then converts it to FLAC. Finally, the file is sent to Google and the result is returned.
    """
    def __init__(self):
        
        # config
        self.FLAC_CONV = 'flac -f '  # We need a WAV to FLAC converter. you can use synaptic to install flac
        chunk = 256
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        THRESHOLD = 70  # The threshold intensity that defines silence signal (lower than).
        SILENCE_LIMIT = 0.3  # Silence limit in seconds. The max ammount of seconds where only silence is recorded. When this time passes the recording finishes and the file is delivered.
    
        # open self.stream
        self.p = pyaudio.PyAudio()
    
        self.stream = self.p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=chunk)
    
        print "* listening. CTRL+C to finish."
        self.all_m = []
        self.data = ''
        self.rel = RATE / chunk
        self.slid_win = deque(maxlen=SILENCE_LIMIT * self.rel)
        self.started = False
        """ROS publishers"""
        self.rec_out_pub = rospy.Publisher('recognizer/output', String)
        self.rec_out = String()
        
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.data = self.stream.read(chunk)
            self.slid_win.append (abs(audioop.avg(self.data, 2)))
    
            if(True in [ x > THRESHOLD for x in self.slid_win]):
                if(not self.started):
                    print "starting record"
                self.started = True
                self.all_m.append(self.data)
            elif (self.started == True):
                # the limit was reached, finish capture and deliver
                filename = self.save_speech(self.all_m, self.p)
                self.stt_google_wav(filename)
                # reset all
                self.started = False
                self.slid_win = deque(maxlen=SILENCE_LIMIT * self.rel)
                self.all_m = []
                print "listening ..."
    
        print "* done recording"
        self.stream.close()
        self.p.terminate()
        r.sleep()            

    def save_speech(self, data, p):
        filename = 'output_' + str(int(time.time()))
        # write data to WAVE file
        self.data = ''.join(data)
        wf = wave.open(filename + '.wav', 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(16000)
        wf.writeframes(self.data)
        wf.close()
        return filename
    
    
    def stt_google_wav(self, filename):
        # Convert to flac
        os.system(self.FLAC_CONV + filename + '.wav')
        f = open(filename + '.flac', 'rb')
        flac_cont = f.read()
        f.close()
        # post it
        lang_code = 'en-US'  # es-Latn,en-US
        googl_speech_url = 'https://www.google.com/speech-api/v1/recognize?xjerr=1&client=chromium&pfilter=2&lang=%s&maxresults=6' % (lang_code)
        hrs = {"User-Agent": "Mozilla/5.0 (X11; Linux i686) AppleWebKit/535.7 (KHTML, like Gecko) Chrome/16.0.912.63 Safari/535.7", 'Content-type': 'audio/x-flac; rate=16000'}
        req = urllib2.Request(googl_speech_url, data=flac_cont, headers=hrs)
        p = urllib2.urlopen(req)
        res = eval(p.read())['hypotheses']
        map(os.remove, (filename + '.flac', filename + '.wav'))
        if res != []:
            self.rec_out.data = res[0]['utterance']
            print "******* I HEARED = ", self.rec_out.data, "*********"
            self.rec_out_pub.publish(self.rec_out)  # vel_command should be continuosly published
        return res
    
    

if __name__ == "__main__":
    rospy.init_node('google_recognizer')
    try:
        Recognizer()
    except:
        pass
