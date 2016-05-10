""" Defines a dictionary that maps all the commands accepted by the wheelchair to words normally understood by the recognizers.

        This mapping is to improve the performance of the voice recognition system, for my voice, I tuned this for the google recognizer and
        pocketsphinx.
"""
KEYWORDS_TO_COMMAND = {'brake':    ['brake'],
                        # Wheelchair Behaviors
                        'manual':   ['manual'],

                        'autonomous': ['autonomous'],

                        'voice_and_head': ['voice'],

                        'follow':     ['follow'],

                        # Speed control
                        'slower':     [ 'slower'],

                        'faster':     ['faster'],

                        'go':         ['go'],

                        'back':       ['back'],

                        'turn':       ['turn'],
                        }

OK_SYNONIMS = ['ok', 'okay', 'OK', 'Ok']

MODE_DICT = { 'joystick':   ['joystick', 'Joystick', 'stick', 'Joystiq'],
              'autonomous': ['autonomous','Autonomous', 'phone', 'Oconomowoc', "condoms", "I'll talk"],
              'manual':   ['manual', 'Manual', 'man', 'man on', "man I'll", 'Menards'],
              'face': ['face', 'Face', 'head', 'Head'],
              'follow':     ['follow', 'Follow'],}

BEHAVIOUR_DICT = {'brake':    ['brake', 'Brake', 'stop', 'top', 'up', 'yup', 'map', 'app', 'rap', 'Drake','bake'],
                  'go':         ['go', 'Go', 'move', "Move", "cool", "Petco"],
                  'turn':       ['turn', 'around', 'Turn', 'Around', 'to run', 'locator'],
                  'joystick':   ['joystick', 'Joystick', 'stick', 'Joystiq'],
                  'manual':   ['manual', 'Manual', 'man', 'man on', "man I'll", 'Menards'],
                  'autonomous': ['phone','autonomous','Autonomous','Oklahoma', 'Toyota','coyote', 'tournament','Tournament', 'Oconomowoc', "condoms", "I'll talk"],
                  'face': ['face', 'Face', 'head', 'Head'],
                  'follow':     ['follow', 'Follow'],
                  }


GO_MODIFIERS_DICT = {'back': ['back', 'Back'],
                    'left': ['left', 'Left'],
                    'right': ['right', 'Right'],
                    'slower': ['slower', 'Slower', 'slow', 'Slow'],
                     'faster': ['faster', 'Faster']}


COMMAND_TO_PHRASE = {'brake':'I will stop',
                    # Wheelchair Behaviors
                    'face':'Move your face',
                    'autonomous':'I am autonomous',
                    'manual': 'Use the joystick',
                    'manual': 'Use the joystick',
                    'follow':'I am following you',
                    # Speed control
                    'slower':'Slower',
                    'faster':'Faster',
                    'go':'I am moving',
                    'back':'I am moving back',
                    'left':'I am moving left',
                    'right':'I am moving right',
                    'turn':'I am turning around'
                    }
