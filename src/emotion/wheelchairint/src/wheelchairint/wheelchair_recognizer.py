#!/usr/bin/env python
import roslib; 
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
from wheelchairint._keywords_to_command import *
import re #Regular expressions package

class WheelchairRecognizer:
    """This class performs speech recognition using Google's speech  recognition service.
    It only recognizes very specific commands used by our wheelchair.

    It records sound from microphone until silence is found and save it as WAV and then converts
    it to FLAC format. Finally, the file is sent to the Google service and the result is retreived
    and published as a ROS topic.
    """
    def __init__(self):
        """ The Constructor of the class
        """
        self.FLAC_CONV = 'flac -f '
        ## @var self.FLAC_CONV
        # This is a WAV to FLAC converter. In linux you can use synaptic to install the flac package.
        chunk = 256
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        THRESHOLD = 20  #20 for blurtooth mic. The threshold intensity that defines silence signal (lower than).
        SILENCE_LIMIT = 0.5  # Silence limit in seconds. The max ammount of seconds where only silence is recorded. When this time passes the recording finishes and the file is delivered.


        self.p = pyaudio.PyAudio()
        ## @var p
        # The python audio class.

        self.stream = self.p.open(format=FORMAT,channels=CHANNELS,
                        rate=RATE,input=True,frames_per_buffer=chunk)
        ## @var stream
        #  The voice audio stream in python format.

        print "* listening. CTRL+C to finish."
        self.all_m = []
        self.data = ''
        self.rel = RATE / chunk
        self.slid_win = deque(maxlen=SILENCE_LIMIT * self.rel)
        self.started = False
        """ROS publishers"""
        self.rec_out_pub = rospy.Publisher('recognizer/output', String)
        ## @var rec_out_pub
        #  The ros publisher where the recognized String will be advertised.
        self.rec_out = String()
        ## @var rec_out
        #  The recognized vocal command to be published as a ROS String().



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



    def get_command(self, res):
        """ Look for the corresponding wheelchair command from the answer
        given by the google server.

        res is of the following type:

            res = [{'confidence': 0.9303669, 'utterance': 'okay go'},
            {'utterance': 'can you go'}, {'utterance': 'okay to go'},
            {'utterance': 'can I go'}, {'utterance': 'go'},
            {'utterance': 'can go'}]


        """
        print "Confidence"
        res_confidence = res[0]['confidence'] #The confidence in the recognized command
        #print res_confidence
        #print "res"
        #print res
        #Only accept words with a confidence bigger than 0.
        voice_command = ''


        for element in res:
            phrase = element['utterance']
            if len(phrase) <= "25": #Avoid analysing long phrases, i means the user is doing something else
                #Check if it was an stop command
                for brake_word in BEHAVIOUR_DICT['brake']:
                    match = re.search(brake_word, phrase)
                    if match:
                        print "behaviour"
                        voice_command ="brake"
                        break
                #check if there is any command
                for behaviour,synonim_list in BEHAVIOUR_DICT.iteritems():
                    for word in synonim_list:
                        match = re.search(word, phrase)
                        #If there is a known behaviour
                        if match:
                            print "behaviour"
                            voice_command = behaviour #We already know that it was go command
                            break
                #Check the modifier.
                for modifier,mod_syn_list in GO_MODIFIERS_DICT.iteritems():
                    for word in mod_syn_list:
                        match = re.search(word, phrase)
                        #If there is a modifier
                        if match:
                            print "modifier:"
                            voice_command = modifier
                            break
        return voice_command

    def save_speech(self, data, p):
        """ The recorded speech phrase @param data, is saved as a wav file.

        @param data: The data to be saved.
        @param p: The pyaudio member used to do the sound processing.
        """
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
        """ It reads the saved wav file and and converts it to the flac compressed audio format.
        Then it sends the ffroile to the google voice recognition server and gets the answer

        @param filename: "The name of the .wav file containing the sound to be recognized."
        """
        # Convert to flac compressed audio format
        os.system(self.FLAC_CONV + filename + '.wav')
        f = open(filename + '.flac', 'rb')
        flac_cont = f.read()  # If you print it it gives a series of strange symbols
        f.close()
        # post it
        lang_code = 'en-US'  # es-Latn,en-US
        googl_speech_url = 'https://www.google.com/speech-api/v1/recognize?xjerr=1&client=chromium&pfilter=2&lang=%s&maxresults=6' % (lang_code)
        hrs = {"User-Agent": "Mozilla/5.0 (X11; Linux i686) AppleWebKit/535.7 (KHTML, like Gecko) Chrome/16.0.912.63 Safari/535.7", 'Content-type': 'audio/x-flac; rate=16000'}
        req = urllib2.Request(googl_speech_url, data=flac_cont, headers=hrs)
        p = urllib2.urlopen(req)


        """ @var res: This is the complete answer from google voice recognition server
            It gives an array like the following when saying "ok go":
            [{'confidence': 0.9303669, 'utterance': 'okay go'},
            {'utterance': 'can you go'}, {'utterance': 'okay to go'},
            {'utterance': 'can I go'}, {'utterance': 'go'},
            {'utterance': 'can go'}]

        """
        res = eval(p.read())['hypotheses']  #This the complete answer from google voice recognition server
        map(os.remove, (filename + '.flac', filename + '.wav'))
        print res
        if res != []:
            voice_command = self.get_command(res)
            if voice_command != '':
                self.rec_out.data = voice_command
                print "******* RECOGNIZED COMMAND= ", self.rec_out.data, "*********"
                self.rec_out_pub.publish(self.rec_out)  # vel_command should be continuosly published
        return res

if __name__ == "__main__":
    rospy.init_node('wheelchair_recognizer')
    try:
        WheelchairRecognizer()
    except:
        pass
