#!/usr/bin/env python
import roslib
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
import re #Regular expressions package



class GoogleRecognizer:
    """This class performs speech recognition using Google's speech  recognition service.
    It only recognizes very specific commands used by our wheelchair.

    It records sound from microphone until silence is found and save it as WAV and then converts
    it to FLAC format. Finally, the file is sent to the Google service and the result is retreived
    and published as a ROS topic.
    """
    def __init__(self):
        """ The Constructor of the class
        """
        
        """You can get your own google key as explained here: 
        https://developers.google.com/youtube/registering_an_application"""
        self.GOOGLE_KEY = 'AIzaSyAnGnFzW77_jdE-SwilZBQvAVdYkBcilPs'
        ## @var self.FLAC_CONV
        self.FLAC_CONV = 'flac -f '
        # This is a WAV to FLAC converter. In linux you can use synaptic to install the flac package.
        self.LANG_CODE ='es-me'  # es-Latn,en-US, es-me
        chunk = 256
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        THRESHOLD = 70  #20 for bluetooth mic. The threshold intensity that defines silence signal (lower than).
        SILENCE_LIMIT = 0.5  # Silence limit in seconds. The max amount of seconds where only silence is recorded. When this time passes the recording finishes and the file is delivered.
        

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
        self.rec_out_pub = rospy.Publisher('recognizer/output', String, queue_size=10)
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
            res = [{'transcript': 'Ok Go'}, {'transcript': 'okay go'}, 
            {'transcript': 'okay Google'}, {'transcript': 'ok Google'}, 
            {'transcript': 'locate gold'}]
        """
        return res[0]['transcript']

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
        print "stt_google wav"
        os.system(self.FLAC_CONV + filename + '.wav')
        f = open(filename + '.flac', 'rb')
        flac_cont = f.read()  # If you print it it gives a series of strange symbols
        f.close()
        # post it
        print "url"
        
        googl_speech_url = 'https://www.google.com/speech-api/v2/recognize?output=json&lang='+ self.LANG_CODE + '&key=' + self.GOOGLE_KEY 
        hrs = {"User-Agent": "Mozilla/5.0 (X11; Linux i686) AppleWebKit/535.7 (KHTML, like Gecko) Chrome/16.0.912.63 Safari/535.7", 'Content-type': 'audio/x-flac; rate=16000'}
        print "urllib2 request"
        req = urllib2.Request(googl_speech_url, data=flac_cont, headers=hrs)
        print "urllib2 open requestt"
        try:
            p = urllib2.urlopen(req)
        except:
            print "urllib2.urlopen(req) error"
            return
        
        
        print "urllib2 request finished"
        answer_str = p.read()
        print "Answer string:"
        print answer_str
        string_splitted = answer_str.split("\n")
        print "New string:"
        print string_splitted
        
        if (string_splitted[1]==""):
            print "google empty answer"
            return

        
        
        
        
        raw_answer = string_splitted[1] #We only need the second line of this multi-line string
        corrected_answer = raw_answer.replace('true','True')    #The google answer can contain not python commands 
                                                                #that can't be converted into a python expression with eval
        
        #Decode google answer, and put the result in a python structure
        try:  
            print "evaluating expression:"    
            """ @var res: This is the complete answer from google voice recognition server
            It gives an array like the following when saying "ok go":
            [{'confidence': 0.9303669, 'utterance': 'okay go'},
            {'utterance': 'can you go'}, {'utterance': 'okay to go'},
            {'utterance': 'can I go'}, {'utterance': 'go'},
            {'utterance': 'can go'}]

            """                                               
            res = eval(corrected_answer)['result'][0]['alternative']
        except:
            print "Couldn't convert string into a python expression"
        
        #Delete the generated sound file 
        try:
            print "trying to delete the generated wav file: "+'~/'+filename + '.flac'
            os.remove(filename + '.flac')
            os.remove(filename + '.wav')
        except:
            print "Could not erase the file" 
        
        print "file deleted"
        voice_command = self.get_command(res)
        if voice_command != '':
            self.rec_out.data = voice_command
            print "******* RECOGNIZED COMMAND= ", self.rec_out.data, "*********"
            self.rec_out_pub.publish(self.rec_out)  # vel_command should be continuosly published
        return res

if __name__ == "__main__":
    rospy.init_node('google_recognizer')
    try:
        GoogleRecognizer()
    except:
        pass
