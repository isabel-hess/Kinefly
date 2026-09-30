#!/usr/bin/env python
#! coding=latin-1
#
# Kinefly, standalone (no ROS) version.  Tracks a recorded video:
#   - offline:   every frame, as fast as possible (default for .avi, .mp4, etc).
#   - realtime:  frames paced by their timestamps, dropping frames when tracking
#                can't keep up, as with a live camera (default for .ufmf).
#
# Run "python kinefly.py --help" for the options.
#
import argparse
import copy
import cv2
import glob
import numpy as np
import os
import threading
import yaml
from setdict import SetDict
import ui
from fly import Fly
import imageprocessing
import kinefly_runtime as runtime
from io_adapters import open_source, RealtimePlayer, CsvSink

# gImageTime = 0.0
gbShowMasks = False

class Struct:
    pass


# to_builtin()
# Convert numpy scalars/arrays in a params dict to plain Python types, so they save as clean yaml.
#
def to_builtin(value):
    if isinstance(value, dict):
        return {k:to_builtin(v) for (k,v) in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_builtin(v) for v in value]
    if isinstance(value, np.ndarray):
        return to_builtin(value.tolist())
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    return value



###############################################################################
###############################################################################
# class MainWindow()
# 
# This is the main Kinefly window, where we receive images from the camera,
# process them using the Fly class, and then output the results.
# 
def load_yaml(filename):
    with open(filename, 'r') as f:
        return yaml.safe_load(f) or {}


class MainWindow:
    # source:       A FrameSource (see io_adapters.py).
    # args:         The parsed command-line options.
    # bRealtime:    True to pace frames by their timestamps and drop frames when behind; False to track every frame.
    # bStartPaused: True to start paused on the first frame, e.g. to position the handles.
    # filenameCsv:  Where to write the results, or None.
    #
    def __init__(self, source, args, bRealtime, bStartPaused, filenameCsv):
        self.lockParams = threading.Lock()
        self.lockBuffer = threading.Lock()
        self.eventImage = threading.Event() # Set when an image arrives in the buffer.

        self.nodename = 'kinefly'
        self.source = source
        self.bRealtime = bRealtime
        self.bLoop = args.loop
        self.bPaused = bStartPaused

        self.window_name = 'kinefly'

        self.yamlfile = os.path.expanduser(args.gui_yaml)
        with self.lockParams:
            # Load the (non-GUI) parameters.
            self.params = {}
            if (args.params is not None):
                self.params = load_yaml(os.path.expanduser(args.params))

            # Load the GUI parameters yaml file.
            if (os.path.exists(self.yamlfile)):
                try:
                    self.params['gui'] = load_yaml(self.yamlfile)
                except (yaml.YAMLError, IOError) as e:
                    runtime.logwarn('%s.  Using default values.' % e)
            else:
                runtime.logwarn('No GUI parameters file %s yet; using defaults.  Position the handles, and they will be saved there.' % self.yamlfile)

        if (args.display_hz is not None):
            self.params['display_hz'] = args.display_hz
        if (args.scale is not None):
            self.params['scale_image'] = args.scale
        if (args.no_gui):
            self.params['use_gui'] = False

        defaults = {'filenameBackground':os.path.splitext(self.yamlfile)[0]+'.png', # Background image goes next to the GUI params.
                    'n_queue_images':2,
                    'display_hz':30.0,                  # Refresh rate of the display, independent of the tracking rate.
                    'use_gui':True,                     # You can turn off the GUI to speed the framerate.
                    'scale_image':1.0,                  # Reducing the image scale will speed the framerate.
                    'n_edges_max':1,                    # Max number of edges per wing to detect, subject to threshold.
                    'rc_background':1000.0,             # Time constant of the moving average background.
                    'wingbeat_min':180,                 # Bounds for wingbeat frequency measurement.
                    'wingbeat_max':220,
                    'head':   {'tracker':'area',
                               'autozero':True,         # Automatically figure out where is the center of motion.
                               'threshold':0.0,
                               'feathering':0.0,        # How much to feather the edge pixels for motion tracking by area.
                               'saturation_correction':False},        
                    'abdomen':{'tracker':'tip',
                               'autozero':True,
                               'threshold':0.0,
                               'feathering':0.25,        # How much to feather the edge pixels for motion tracking by area.
                               'saturation_correction':False},
                    'left':   {'tracker':'edge',
                               'autozero':True,
                               'threshold':0.0,
                               'feathering':0.25,        # How much to feather the edge pixels for motion tracking by area.
                               'saturation_correction':False},
                    'right':  {'tracker':'edge',
                               'autozero':True,
                               'threshold':0.0,
                               'feathering':0.25,        # How much to feather the edge pixels for motion tracking by area.
                               'saturation_correction':False},
                    'aux':    {'tracker':'intensity',
                               'saturation_correction':False},
                    'gui': {'windows':False,                    # Show the helpful extra windows.
                            'symmetric':True,                   # Forces the UI to remain symmetric.
                            'axis':   {'track':False,           # To track, or not to track.
                                       'pt1':{'x':310,
                                              'y':150},
                                       'pt2':{'x':310,
                                              'y':250}},
                            'head':   {'track':True,            # To track, or not to track.
                                       'subtract_bg':False,     # Use background subtraction?
                                       'stabilize':False,       # Image stabilization of the bodypart.
                                       'hinge':{'x':300,        # Hinge position in image coordinates.
                                                'y':150},
                                       'radius_outer':80,       # Outer radius in pixel units.
                                       'radius_inner':50,       # Inner radius in pixel units.
                                       'angle_hi':0.7854,       # Angle limit in radians.
                                       'angle_lo':-0.7854},     # Angle limit in radians.
                            'abdomen':{'track':True,
                                       'subtract_bg':False,
                                       'stabilize':False,
                                       'hinge':{'x':300,
                                                'y':250},
                                       'radius_outer':120,
                                       'radius_inner':100,
                                       'angle_hi':3.927, 
                                       'angle_lo':2.3562},
                            'left':   {'track':True,
                                       'subtract_bg':False,
                                       'stabilize':False,
                                       'hinge':{'x':250,
                                                'y':200},
                                       'radius_outer':80,
                                       'radius_inner':50,
                                       'angle_hi':-0.7854, 
                                       'angle_lo':-2.3562},
                            'right':  {'track':True,
                                       'subtract_bg':False,
                                       'stabilize':False,
                                       'hinge':{'x':350,
                                                'y':200},
                                       'radius_outer':80,
                                       'radius_inner':50,
                                       'angle_hi':2.3562, 
                                       'angle_lo':0.7854},
                            'aux':    {'track':True,
                                       'subtract_bg':False,
                                       'center':{'x':350,
                                                 'y':150},
                                       'radius1':30,
                                       'radius2':20,
                                       'angle':0.0},
                            }
                    }

        # The default handle layout is for a 640x480 image; without saved handles, fit it to this video.
        if ('gui' not in self.params) and (source.shape is not None):
            self.scale_default_handles(defaults['gui'], source.shape[1]/640.0)

        SetDict().set_dict_with_preserve(self.params, defaults)
        self.params = self.legalizeParams(self.params)

        self.scale = self.params['scale_image']
        self.bMousing = False
        self.bQuitting = False
        if (not self.params['use_gui']):
            self.bPaused = False # No way to unpause without the GUI.
        if (self.params['use_gui']):
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        # Create the fly.
        self.fly = Fly(self.params)
        self.fly.set_params(self.scale_params(self.params, self.scale))

        # Background image.
        self.filenameBackground = os.path.expanduser(self.params['filenameBackground'])
        imgDisk  = cv2.imread(self.filenameBackground, cv2.IMREAD_GRAYSCALE) if (os.path.exists(self.filenameBackground)) else None
        if (imgDisk is not None) and (source.shape is not None) and (imgDisk.shape != tuple(source.shape)):
            runtime.logwarn('Ignoring background image %s: it is %s, but the video is %s.' % (self.filenameBackground, imgDisk.shape, tuple(source.shape)))
            imgDisk = None
        if (imgDisk is not None):
            if (self.scale == 1.0):              
                imgFullBackground = imgDisk
            else:  
                imgFullBackground = cv2.resize(imgDisk, (0,0), fx=self.scale, fy=self.scale)
             
            self.fly.set_background(imgFullBackground)
            self.bHaveBackground = True
        else:
            self.bHaveBackground = False
        
        
        self.nameSelected   = None
        self.uiSelected     = None
        self.stateSelected  = None
        self.fly.update_handle_points()
        self.stampCameraPrev= 0.0
        self.stampCameraDiff= 0.0
        self.stampROSPrev   = 0.0
        self.stampROSDiff   = 0.0
        self.stampMax       = 0.0
        self.dtCamera       = np.inf
        self.hzCameraF      = 0.0
        self.hzCameraSum    = 0.0
        self.hzROSF         = 0.0
        self.hzROSSum       = 0.0
        self.iCountCamera   = 0
        self.iCountROS      = 0
        self.iDroppedFrame  = 0     # Consecutive dropped frames, for the display.
        self.nDroppedTotal  = 0     # All dropped frames.

        self.nQueuePrev     = 0     # Length of the image queue.
        self.dnQueueF       = 0.0   # Rate of change of the image queue length.
        self.aQueue         = 0.0   # Fraction of the image queue in use.
        
        self.bufferImages   = [None]*self.params['n_queue_images'] # Circular buffer for incoming images.
        self.iImgLoading    = 0  # Index of the next slot to load.
        self.iImgWorking    = 0  # Index of the slot to process, i.e. the oldest image in the buffer.
        self.imgUnscaled    = None
        self.imgScaled      = None
        
        self.scaleUI        = 1.0   # Size of the toolbar & text, independent of the image scale.
        self.h_gap          = int(5 * self.scaleUI)
        self.w_gap          = int(10 * self.scaleUI)
        self.scaleText      = 0.4 * self.scaleUI
        self.fontface       = cv2.FONT_HERSHEY_SIMPLEX
        self.buttons        = None
        self.yToolbar       = 0
        self.shapeToolbar   = (0,0)

        self.frame          = None  # The frame being worked on.
        self.bFrameTracked  = False # Has self.frame been through the tracker?
        self.bValidImage    = False
        self.tDisplayPrev   = -np.inf
        self.tStart         = None  # When tracking started, for the stats.
        self.nTracked       = 0
        self.latencies      = []    # Seconds from frame acquisition to tracking results, per tracked frame.
        self.processtimes   = []    # Seconds spent tracking, per tracked frame.
        self.tProgressPrev  = runtime.now()

        # Where results go.
        self.sinks = []
        if (filenameCsv is not None):
            self.sinks.append(CsvSink(filenameCsv, n_edges_max=self.params['n_edges_max']))
            runtime.loginfo('Writing results to %s' % filenameCsv)

        # user callbacks
        if (self.params['use_gui']):
            cv2.setMouseCallback(self.window_name, self.onMouse, param=None)

        # In realtime mode a player thread delivers frames through image_callback(), like a camera would.
        self.player = None
        if (self.bRealtime):
            self.player = RealtimePlayer(self.source, self.image_callback, loop=self.bLoop)


    # Check the given button to see if it extends outside the image, and if so then reposition it to the next line.
    def wrap_button(self, btn, shape):
        if (btn.right >= shape[1]):
            btn.set_pos(pt=[1, btn.bottom+1])

        
    # Create the button bar, with overflow onto more than one line if needed to fit on the image.        
    def create_buttons(self, shape):
        self.shapeToolbar = shape
        
        # UI button specs.
        self.buttons = []
        x = 1
        y = 1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='pushbutton', name='exit', text='exit')
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='pushbutton', name='save_bg', text='saveBG')
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='static', name='track', text='track:', sides=ui.SIDE_LEFT|ui.SIDE_TOP|ui.SIDE_BOTTOM)
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='head', text='H', state=self.params['gui']['head']['track'], sides=ui.SIDE_TOP|ui.SIDE_BOTTOM)
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='abdomen', text='A', state=self.params['gui']['abdomen']['track'], sides=ui.SIDE_TOP|ui.SIDE_BOTTOM)
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='left', text='L', state=self.params['gui']['left']['track'], sides=ui.SIDE_TOP|ui.SIDE_BOTTOM)
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='right', text='R', state=self.params['gui']['right']['track'], sides=ui.SIDE_TOP|ui.SIDE_BOTTOM)
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='aux', text='X', state=self.params['gui']['aux']['track'], sides=ui.SIDE_TOP|ui.SIDE_BOTTOM|ui.SIDE_RIGHT)
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='static', name='subtract', text='subtract:', sides=ui.SIDE_LEFT|ui.SIDE_TOP|ui.SIDE_BOTTOM)
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='subtract_head', text='H', state=self.params['gui']['head']['subtract_bg'], sides=ui.SIDE_TOP|ui.SIDE_BOTTOM)
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='subtract_abdomen', text='A', state=self.params['gui']['abdomen']['subtract_bg'], sides=ui.SIDE_TOP|ui.SIDE_BOTTOM)
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='subtract_lr', text='LR', state=self.params['gui']['right']['subtract_bg'], sides=ui.SIDE_TOP|ui.SIDE_BOTTOM)
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='subtract_aux', text='X', state=self.params['gui']['aux']['subtract_bg'], sides=ui.SIDE_TOP|ui.SIDE_BOTTOM|ui.SIDE_RIGHT)
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        # A button to allow the user to override the automatic invertcolor detector.  A better autodetect algorithm might eliminate the need for this.
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='invertcolor', text='invertcolor', state=self.fly.bInvertColor)
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        self.ibtnInvertColor = len(self.buttons)-1
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='stabilize', text='stabilize', state=self.params['gui']['head']['stabilize'])
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='symmetry', text='symmetric', state=self.params['gui']['symmetric'])
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        x = btn.right+1
        y = btn.top+1
        btn = ui.Button(pt=[x,y], scale=self.scaleUI, type='checkbox', name='windows', text='windows', state=self.params['gui']['windows'])
        self.wrap_button(btn, shape)
        self.buttons.append(btn)
        
        self.yToolbar = btn.bottom + 1


    # scale_default_handles()
    # Scale the positions & sizes in a gui params dict by the given factor.
    #
    def scale_default_handles(self, gui, factor):
        for partname in ['head', 'abdomen', 'left', 'right']:
            gui[partname]['hinge']['x'] *= factor
            gui[partname]['hinge']['y'] *= factor
            gui[partname]['radius_outer'] *= factor
            gui[partname]['radius_inner'] *= factor
        gui['aux']['center']['x'] *= factor
        gui['aux']['center']['y'] *= factor
        gui['aux']['radius1'] *= factor
        gui['aux']['radius2'] *= factor
        for pt in ['pt1', 'pt2']:
            gui['axis'][pt]['x'] *= factor
            gui['axis'][pt]['y'] *= factor


    # legalizeParams()
    # Make sure that all the parameters contain legal values.
    #
    def legalizeParams(self, paramsIn):
        paramsOut = copy.copy(paramsIn)
        for partname in ['head','abdomen','left','right']:
            if (partname in paramsOut):
                if ('hinge' in paramsOut['gui'][partname]):
                    if ('x' in paramsOut['gui'][partname]['hinge']):
                        paramsOut['gui'][partname]['hinge']['x'] = max(0, paramsOut['gui'][partname]['hinge']['x'])
        
                    if ('y' in paramsOut['gui'][partname]['hinge']):
                        paramsOut['gui'][partname]['hinge']['y'] = max(0, paramsOut['gui'][partname]['hinge']['y'])
    
                if ('radius_inner' in paramsOut['gui'][partname]):
                    paramsOut['gui'][partname]['radius_inner'] = max(5*paramsOut['scale_image'], paramsOut['gui'][partname]['radius_inner'])
    
                if ('radius_outer' in paramsOut['gui'][partname]) and ('radius_inner' in paramsOut['gui'][partname]):
                    paramsOut['gui'][partname]['radius_outer'] = max(paramsOut['gui'][partname]['radius_outer'], paramsOut['gui'][partname]['radius_inner']+5*paramsOut['scale_image'])

        return paramsOut
        
        
    # save_params()
    # Save the GUI params (handle positions, checkboxes) to the yaml file.
    #
    def save_params(self):
        with self.lockParams:
            try:
                dirname = os.path.dirname(self.yamlfile)
                if (dirname) and (not os.path.isdir(dirname)):
                    os.makedirs(dirname)
                with open(self.yamlfile, 'w') as f:
                    yaml.safe_dump(to_builtin(self.params['gui']), f, default_flow_style=False)
            except (IOError, OSError) as e:
                runtime.logwarn('Could not save the GUI params to %s: %s' % (self.yamlfile, e))


    # command_callback()
    # Execute a command from the toolbar or keyboard.
    #
    def command_callback(self, command):
        self.command = command

        if (self.command == 'exit'):
            self.bQuitting = True
            self.save_params()


        if (self.command == 'save_background'):
            self.save_background()


        if (self.command == 'pause'):
            self.set_paused(not self.bPaused)


        
    def scale_params(self, paramsIn, scale):
        paramsScaled = copy.deepcopy(paramsIn)

        for partname in ['head', 'abdomen', 'left', 'right']:
            paramsScaled['gui'][partname]['hinge']['x'] = (paramsIn['gui'][partname]['hinge']['x']*scale)  
            paramsScaled['gui'][partname]['hinge']['y'] = (paramsIn['gui'][partname]['hinge']['y']*scale)  
            paramsScaled['gui'][partname]['radius_outer'] = (paramsIn['gui'][partname]['radius_outer']*scale)  
            paramsScaled['gui'][partname]['radius_inner'] = (paramsIn['gui'][partname]['radius_inner']*scale)  
            
        for partname in ['aux']:
            paramsScaled['gui'][partname]['center']['x'] = (paramsIn['gui'][partname]['center']['x']*scale)  
            paramsScaled['gui'][partname]['center']['y'] = (paramsIn['gui'][partname]['center']['y']*scale)  
            paramsScaled['gui'][partname]['radius1'] = (paramsIn['gui'][partname]['radius1']*scale)  
            paramsScaled['gui'][partname]['radius2'] = (paramsIn['gui'][partname]['radius2']*scale)  
            
        return paramsScaled  
    
    
    # Draw user-interface elements on the image.
    def draw_buttons(self, image):
        if (self.buttons is not None):
            for i in range(len(self.buttons)):
                self.buttons[i].draw(image)


    def image_callback(self, frame):
        # Receive the image:
        if (not self.bQuitting):
            with self.lockBuffer:
                # Check for dropped frame.
                if (self.bufferImages[self.iImgLoading] is None):   # There's an empty slot in the buffer.
                    iImgLoadingNext = (self.iImgLoading+1) % len(self.bufferImages)
                    iImgWorkingNext = self.iImgWorking
                    self.iDroppedFrame = 0
                else:                                               # The buffer is full; we'll overwrite the oldest entry.
                    iImgLoadingNext = (self.iImgLoading+1) % len(self.bufferImages)
                    iImgWorkingNext = (self.iImgWorking+1) % len(self.bufferImages)
                    self.iDroppedFrame += 1
                    self.nDroppedTotal += 1

                # Put the image into the queue.
                self.bufferImages[self.iImgLoading] = frame
                self.iImgLoading = iImgLoadingNext
                self.iImgWorking = iImgWorkingNext
                self.eventImage.set()


    # get_frame()
    # Take the oldest frame from the buffer, or None if the buffer is empty.
    #
    def get_frame(self):
        frame = None

        with self.lockBuffer:
            # The image queue length.
            nQueue = (self.iImgLoading - self.iImgWorking) %  len(self.bufferImages)
            if (nQueue==0) and (self.bufferImages[self.iImgLoading] is not None):
                nQueue += len(self.bufferImages)

            # Rate of change of the queue length.
            if (nQueue == len(self.bufferImages)):
                dnQueue = 1.0
            elif (nQueue <= 1):
                dnQueue = -1.0
            else:
                dnQueue = nQueue - self.nQueuePrev

            # Bring the bar back to red, if it's green and we dropped a frame.
            if (self.iDroppedFrame>0) and (self.dnQueueF<0.0):
                self.dnQueueF = 0.1
            else:
                a = 0.001
                self.dnQueueF = (1-a)*self.dnQueueF + a*dnQueue

            self.aQueue = float(nQueue)/float(len(self.bufferImages))
            self.nQueuePrev = nQueue

            # Pull the image from the queue.
            if (self.bufferImages[self.iImgWorking] is not None):
                frame = self.bufferImages[self.iImgWorking]

                # Mark this buffer entry as available for loading.
                self.bufferImages[self.iImgWorking] = None

                # Go to the next image.
                self.iImgWorking = (self.iImgWorking+1) % len(self.bufferImages)

            if (self.bufferImages[self.iImgWorking] is None):
                self.eventImage.clear()

        return frame


    # ingest_frame()
    # Make the given frame the current one:  update the framerate stats, and scale the image.
    #
    def ingest_frame(self, frame):
        # Compute processing times.
        self.stampROS        = runtime.now()
        self.stampROSDiff    = (self.stampROS - self.stampROSPrev)
        self.stampROSPrev    = self.stampROS
        self.dtROS           = max(0, self.stampROSDiff)

        # If time wrapped, then just assume a value.
        if (self.dtROS == 0.0):
            self.dtROS = 1.0

        # Compute system freq.
        hzROS = 1/self.dtROS
        self.iCountROS += 1
        if (self.iCountROS > 100):
            a= 0.04 # Filter the framerate.
            self.hzROSF = (1-a)*self.hzROSF + a*hzROS
        else:
            if (self.iCountROS>20):             # Get past the transient response.
                self.hzROSSum += hzROS
            else:
                self.hzROSSum = hzROS * self.iCountROS

            self.hzROSF = self.hzROSSum / self.iCountROS


        # Compute processing times.
        self.stampCamera     = frame.header.stamp
        self.stampCameraDiff = (self.stampCamera - self.stampCameraPrev)
        self.stampCameraPrev = self.stampCamera
        self.dtCamera        = max(0, self.stampCameraDiff)

        # If the camera is not giving good timestamps, then use our own clock.
        if (self.dtCamera == 0.0):
            self.dtCamera = self.dtROS

        # If time wrapped, then just assume a value.
        if (self.dtCamera == 0.0):
            self.dtCamera = 1.0

        # Compute processing freq.
        hzCamera = 1/self.dtCamera
        self.iCountCamera += 1
        if (self.iCountCamera > 100):
            a= 0.01 # Filter the framerate.
            self.hzCameraF = (1-a)*self.hzCameraF + a*hzCamera
        else:
            if (self.iCountCamera>20):             # Get past the transient response.
                self.hzCameraSum += hzCamera
            else:
                self.hzCameraSum = hzCamera * self.iCountCamera

            self.hzCameraF = self.hzCameraSum / self.iCountCamera


        # Scale the image.
        img = frame.image
        self.imgUnscaled = img
        if (self.scale == 1.0):
            self.imgScaled = self.imgUnscaled
        else:
            self.imgScaled = cv2.resize(img, (0,0), fx=self.scale, fy=self.scale)

        self.shapeImage = self.imgScaled.shape # (height,width)
        self.frame = frame
        self.bFrameTracked = False
        self.bValidImage = True

        # Create the button bar if needed.
        if (self.buttons is None) or (self.imgScaled.shape != self.shapeToolbar):
            self.create_buttons(self.imgScaled.shape)


    # track_frame()
    # Run the tracker on the current frame, and send the results to the sinks.
    #
    def track_frame(self):
        if (self.frame is None) or (self.bFrameTracked) or (self.bMousing):
            return

        tStart = runtime.now()
        if (self.tStart is None):
            self.tStart = tStart

        # Update the fly internals.
        self.fly.update(self.frame.header, self.imgScaled)
        flystate = self.fly.get_flystate()
        tDone = runtime.now()

        self.bFrameTracked = True
        self.nTracked += 1
        self.latencies.append(tDone - self.frame.t_acquired)
        self.processtimes.append(tDone - tStart)

        # Output the results.
        meta = {'frame':         self.frame.header.seq,
                'stamp':         self.frame.header.stamp,
                't_acquired':    self.frame.t_acquired,
                't_start':       tStart,
                't_done':        tDone,
                'dropped_total': self.nDroppedTotal,
                'scale':         self.scale}
        for sink in self.sinks:
            sink.write(flystate, meta)


    # display_due()
    # Is it time to refresh the display?
    #
    def display_due(self):
        return (runtime.now() - self.tDisplayPrev) >= 1.0/max(1.0, self.params['display_hz'])


    # render()
    # Draw the current frame with the tracking overlay and the toolbar, and show it.
    #
    def render(self):
        if (self.imgScaled is None):
            return
        self.tDisplayPrev = runtime.now()

        if (self.params['use_gui']):

            imgOutput = cv2.cvtColor(self.imgScaled, cv2.COLOR_GRAY2RGB)
            self.fly.draw(imgOutput)
        
            x_left   = int(10 * self.scaleUI)
            y_bottom = int(imgOutput.shape[0] - 10 * self.scaleUI)
            x_right  = int(imgOutput.shape[1] - 10 * self.scaleUI)
            x = x_left
            y = y_bottom
            h = 10

            w = 55
            if (not self.bMousing):

                # Output the framerate.
                s = '%5.1fHz [queue:          ]' % (self.hzCameraF)
                if (self.iDroppedFrame>0):
                    s += '    Dropped Frames: %d' % self.iDroppedFrame
                cv2.putText(imgOutput, s, (x, y), self.fontface, self.scaleText, ui.bgra_dict['dark_red'] )
                h_text = int(h * self.scaleUI)
                
                # Draw the gauge bar for the image queue length.
                nBar = int(50*self.scaleUI) # pixels max.
                xBar = int(x+110*self.scaleUI)
                yBar = y-int(h_text/2)+1
                ptBar0 = (xBar,                       yBar)
                ptBar1 = (xBar+int(self.aQueue*nBar), yBar)
                if (self.dnQueueF>0):
                    bgra = ui.bgra_dict['dark_red']
                else:
                    bgra = ui.bgra_dict['dark_green']
                    
                cv2.line(imgOutput, ptBar0, ptBar1, bgra, max(1,int(2*self.scaleUI)))
            

                # Output the aux state.
                if (self.params['gui']['aux']['track']):
                    y -= h_text+self.h_gap
                    s = 'AUX: (%0.3f)' % (self.fly.aux.state.intensity)
                    cv2.putText(imgOutput, s, (x, y), self.fontface, self.scaleText, self.fly.aux.bgra)
                    h_text = int(h * self.scaleUI)
                
                    y -= h_text+self.h_gap
                    s = 'WB Freq: '
                    if (self.fly.aux.state.freq != 0.0):
                        s += '%0.0fhz' % (self.fly.aux.state.freq)
                    cv2.putText(imgOutput, s, (x, y), self.fontface, self.scaleText, self.fly.aux.bgra)
                    h_text = int(h * self.scaleUI)
                

                # Output the wings state.
                if (self.params['gui']['left']['track']) and (self.params['gui']['right']['track']):
                    # L+R
                    y -= h_text+self.h_gap
                    s = 'L+R:'
                    if (len(self.fly.left.state.angles)>0) and (len(self.fly.right.state.angles)>0):
                        leftplusright = self.fly.left.state.angles[0] + self.fly.right.state.angles[0]
                        s += '% 7.4f' % leftplusright
                    cv2.putText(imgOutput, s, (x, y), self.fontface, self.scaleText, ui.bgra_dict['blue'])
                    h_text = int(h * self.scaleUI)
    
                        
                    # L-R
                    y -= h_text+self.h_gap
                    s = 'L-R:'
                    if (len(self.fly.left.state.angles)>0) and (len(self.fly.right.state.angles)>0):
                        leftminusright = self.fly.left.state.angles[0] - self.fly.right.state.angles[0]
                        s += '% 7.4f' % leftminusright
                    cv2.putText(imgOutput, s, (x, y), self.fontface, self.scaleText, ui.bgra_dict['blue'])
                    h_text = int(h * self.scaleUI)
    
                if (self.params['gui']['right']['track']):
                    # Right
                    y -= h_text+self.h_gap
                    s = 'R:'
                    if (len(self.fly.right.state.angles)>0):
                        s += '% 7.4f' % self.fly.right.state.angles[0]
                        #for i in range(1,len(self.fly.right.state.angles)):
                        #    s += ', % 7.4f' % self.fly.right.state.angles[i]
                    cv2.putText(imgOutput, s, (x, y), self.fontface, self.scaleText, self.fly.right.bgra)
                    h_text = int(h * self.scaleUI)
        
                if (self.params['gui']['left']['track']):
                    # Left
                    y -= h_text+self.h_gap
                    s = 'L:'
                    if (len(self.fly.left.state.angles)>0):
                        s += '% 7.4f' % self.fly.left.state.angles[0]
                        #for i in range(1,len(self.fly.left.state.angles)):
                        #    s += ', % 7.4f' % self.fly.left.state.angles[i]
                    cv2.putText(imgOutput, s, (x, y), self.fontface, self.scaleText, self.fly.left.bgra)
                    h_text = int(h * self.scaleUI)
                    
                        


                # Output the abdomen state.
                if (self.params['gui']['abdomen']['track']):
                    y -= h_text+self.h_gap
                    s = 'ABDOMEN:'
                    if (len(self.fly.abdomen.state.angles)>0):
                        s += '% 7.4f' % (self.fly.abdomen.state.angles[0])

                    cv2.putText(imgOutput, s, (x, y), self.fontface, self.scaleText, self.fly.abdomen.bgra)
                    h_text = int(h * self.scaleUI)
                

                # Output the head state.
                if (self.params['gui']['head']['track']):
                    y -= h_text+self.h_gap
                    s = 'HEAD:'
                    if (len(self.fly.head.state.angles)>0):
                        s+= '% 7.4f' % (self.fly.head.state.angles[0])

                    cv2.putText(imgOutput, s, (x, y), self.fontface, self.scaleText, self.fly.head.bgra)
                    h_text = int(h * self.scaleUI)




            self.buttons[self.ibtnInvertColor].state = self.fly.bInvertColor # Set the button state to reflect the fly's bInvertColor flag. 
            self.draw_buttons(imgOutput)

            # Status line, below the toolbar.
            if (self.frame is not None):
                if (self.source.n_frames > 0):
                    s = 'frame %d/%d' % (self.frame.header.seq+1, self.source.n_frames)
                else:
                    s = 'frame %d' % (self.frame.header.seq+1)
                s += '  %s' % ('realtime' if (self.bRealtime) else 'offline')
                if (self.bPaused):
                    s += '  PAUSED (space=run, q=quit)'
                cv2.putText(imgOutput, s, (self.w_gap, int(self.yToolbar + 14*self.scaleUI)), self.fontface, self.scaleText, ui.bgra_dict['yellow'])

            # Display the image.
            cv2.imshow(self.window_name, imgOutput)

    # End render()
            
                
    # save_background()
    # Save the current camera image as the background.
    #
    def save_background(self):
        runtime.logwarn ('Saving new background image %s' % self.filenameBackground)
        cv2.imwrite(self.filenameBackground, self.imgUnscaled)
        self.fly.set_background(self.imgScaled)
        self.bHaveBackground = True
    
    
    # hit_object()
    # Get the nearest handle point or button to the mouse point.
    # ptMouse    = [x,y]
    # Returns the partname, tag, and ui of item the mouse has hit, using the 
    # convention that the name is of the form "tag_partname", e.g. "hinge_left"
    #
    def hit_object(self, ptMouse):
        tagHit  = None
        partnameHit = None
        uiHit = None
        
        # Check for button press.
        iButtonHit = None
        for iButton in range(len(self.buttons)):
            if (self.buttons[iButton].hit_test(ptMouse)):
                iButtonHit = iButton
            
        if (iButtonHit is not None):
            nameNearest = self.buttons[iButtonHit].name
            (tagHit,delim,partnameHit) = nameNearest.partition('_')
            uiHit = self.buttons[iButtonHit].type
        else: # Check for handle hit.
            tag      = [None,None,None,None,None,None]
            partname = [None,None,None,None,None,None]
            (partname[0], tag[0]) = self.fly.left.hit_object(ptMouse)
            (partname[1], tag[1]) = self.fly.right.hit_object(ptMouse)
            (partname[2], tag[2]) = self.fly.head.hit_object(ptMouse)
            (partname[3], tag[3]) = self.fly.abdomen.hit_object(ptMouse)
            (partname[4], tag[4]) = self.fly.aux.hit_object(ptMouse)
            (partname[5], tag[5]) = self.fly.axis.hit_object(ptMouse)
            i = next((i for i in range(len(tag)) if tag[i]!=None), None)
            if (i is not None):
                tagHit  = tag[i]
                partnameHit = partname[i]
                uiHit = 'handle'
    
        
        return (uiHit, tagHit, partnameHit, iButtonHit)
        
        
    # Convert tag and partname strings to a name string:  tag_partname
    def name_from_tagpartname(self, tag, partname):
        if (partname is not None) and (len(partname)>0):
            name = tag+'_'+partname
        else:
            name = tag
            
        return name
    

    def bodypart_from_partname(self, partname):
        if (partname=='left'):
            bodypart = self.fly.left
        elif (partname=='right'):
            bodypart = self.fly.right
        elif (partname=='head'):
            bodypart = self.fly.head
        elif (partname=='abdomen'):
            bodypart = self.fly.abdomen
        elif (partname=='aux'):
            bodypart = self.fly.aux
        elif (partname=='axis'):
            bodypart = self.fly.axis
        else:
            bodypart = None
            
        return bodypart
    
                
    # update_params_from_mouse()
    # Recalculate self.params based on a currently selected handle and mouse location.
    #
    def update_params_from_mouse(self, tagSelected, partnameSelected, ptMouse):
        partnameSlave = {'axis':None, 'head':'abdomen', 'abdomen':'head', 'right':'left', 'left':'right', 'aux':None}[partnameSelected]
        tagThis = tagSelected
        tagOther = 'angle_lo' if (tagSelected=='angle_hi') else 'angle_hi'
        tagSlave = tagOther
        bodypartSelected = self.bodypart_from_partname(partnameSelected)
        bodypartSlave    = self.bodypart_from_partname(partnameSlave)

        # Scale the parameters in order to work on them.
        paramsScaled = self.scale_params(self.params, self.scale) 
        
        # Hinge.
        if (tagSelected=='hinge'): 
            if (partnameSelected=='head') or (partnameSelected=='abdomen'):

                # Get the hinge points pre-move.
                if (paramsScaled['gui']['symmetric']):
                    ptHead = np.array([paramsScaled['gui']['head']['hinge']['x'], paramsScaled['gui']['head']['hinge']['y']])
                    ptAbdomen = np.array([paramsScaled['gui']['abdomen']['hinge']['x'], paramsScaled['gui']['abdomen']['hinge']['y']])
                    ptCenterPre = (ptHead + ptAbdomen) / 2
                    ptBodyPre = ptHead - ptAbdomen
                    angleBodyPre = np.arctan2(ptBodyPre[1], ptBodyPre[0])
                    ptLeft = np.array([paramsScaled['gui']['left']['hinge']['x'], paramsScaled['gui']['left']['hinge']['y']])
                    ptRight = np.array([paramsScaled['gui']['right']['hinge']['x'], paramsScaled['gui']['right']['hinge']['y']])
                    ptLC = ptLeft-ptCenterPre
                    ptRC = ptRight-ptCenterPre
                    rL = np.linalg.norm(ptLC)
                    aL = np.arctan2(ptLC[1], ptLC[0]) - angleBodyPre # angle from body center to hinge in body axis coords.
                    rR = np.linalg.norm(ptRC)
                    aR = np.arctan2(ptRC[1], ptRC[0]) - angleBodyPre

                # Move the selected hinge point.
                pt = ptMouse
                paramsScaled['gui'][partnameSelected]['hinge']['x'] = float(pt[0])
                paramsScaled['gui'][partnameSelected]['hinge']['y'] = float(pt[1])
                
                # Invalidate the masks.
                self.fly.axis.bValidMask    = False
                self.fly.head.bValidMask    = False
                self.fly.abdomen.bValidMask = False
                self.fly.left.bValidMask    = False
                self.fly.right.bValidMask   = False
                
                # Now move the hinge points relative to the new body axis.
                if (paramsScaled['gui']['symmetric']):
                    ptHead = np.array([paramsScaled['gui']['head']['hinge']['x'], paramsScaled['gui']['head']['hinge']['y']])
                    ptAbdomen = np.array([paramsScaled['gui']['abdomen']['hinge']['x'], paramsScaled['gui']['abdomen']['hinge']['y']])
                    ptCenterPost = (ptHead + ptAbdomen) / 2
                    ptBodyPost = ptHead - ptAbdomen
                    angleBodyPost = np.arctan2(ptBodyPost[1], ptBodyPost[0])
                    ptLeft = ptCenterPost + rL * np.array([np.cos(aL+angleBodyPost), np.sin(aL+angleBodyPost)])
                    ptRight = ptCenterPost + rR * np.array([np.cos(aR+angleBodyPost), np.sin(aR+angleBodyPost)])
                    paramsScaled['gui']['left']['hinge']['x'] = float(ptLeft[0])
                    paramsScaled['gui']['left']['hinge']['y'] = float(ptLeft[1])
                    paramsScaled['gui']['right']['hinge']['x'] = float(ptRight[0])
                    paramsScaled['gui']['right']['hinge']['y'] = float(ptRight[1])

                    
            elif (partnameSelected=='left') or (partnameSelected=='right'):
                paramsScaled['gui'][partnameSelected]['hinge']['x'] = float(ptMouse[0])
                paramsScaled['gui'][partnameSelected]['hinge']['y'] = float(ptMouse[1])
                bodypartSelected.bValidMask = False

                if (paramsScaled['gui']['symmetric']):
                    ptSlave = imageprocessing.get_reflection_across_axis(ptMouse, (self.fly.abdomen.ptHinge_i, self.fly.head.ptHinge_i))
                    paramsScaled['gui'][partnameSlave]['hinge']['x'] = float(ptSlave[0])
                    paramsScaled['gui'][partnameSlave]['hinge']['y'] = float(ptSlave[1])
                    bodypartSlave.bValidMask = False



        elif (tagSelected in ['angle_hi','angle_lo']):
            pt = ptMouse - bodypartSelected.ptHinge_i
            if (tagSelected=='angle_lo'): 
                angle_lo_b = float(bodypartSelected.transform_angle_b_from_i(np.arctan2(pt[1], pt[0])))

                if (partnameSelected in ['head','abdomen']) and (paramsScaled['gui']['symmetric']):
                    angle_hi_b = -angle_lo_b
                else:
                    angle_hi_b = paramsScaled['gui'][partnameSelected][tagOther]

            elif (tagSelected=='angle_hi'):
                angle_hi_b = float(bodypartSelected.transform_angle_b_from_i(np.arctan2(pt[1], pt[0])))
                
                if (partnameSelected in ['head','abdomen']) and (paramsScaled['gui']['symmetric']):
                    angle_lo_b = -angle_hi_b
                else:
                    angle_lo_b = paramsScaled['gui'][partnameSelected][tagOther]
            
            paramsScaled['gui'][partnameSelected]['radius_outer'] = float(max(bodypartSelected.params['gui'][partnameSelected]['radius_inner']+2*self.scale, 
                                                                              np.linalg.norm(bodypartSelected.ptHinge_i - ptMouse)))
                
            # Make angles relative to bodypart origin.
            angle_lo_b -= bodypartSelected.angleBodypart_b
            angle_lo_b = (angle_lo_b+np.pi) % (2.0*np.pi) - np.pi
            angle_hi_b -= bodypartSelected.angleBodypart_b
            angle_hi_b = (angle_hi_b+np.pi) % (2.0*np.pi) - np.pi
            
            # Switch to the other handle.
            if (not (angle_lo_b < angle_hi_b)):
                self.tagSelected = tagOther
                
            # Set the order of the two angles
            paramsScaled['gui'][partnameSelected]['angle_lo'] = min(angle_lo_b, angle_hi_b)
            paramsScaled['gui'][partnameSelected]['angle_hi'] = max(angle_lo_b, angle_hi_b)
            
            # Make angles relative to fly origin. 
            paramsScaled['gui'][partnameSelected]['angle_lo'] += bodypartSelected.angleBodypart_b
            paramsScaled['gui'][partnameSelected]['angle_hi'] += bodypartSelected.angleBodypart_b
            paramsScaled['gui'][partnameSelected]['angle_lo'] = (paramsScaled['gui'][partnameSelected]['angle_lo']+np.pi) % (2.0*np.pi) - np.pi
            paramsScaled['gui'][partnameSelected]['angle_hi'] = (paramsScaled['gui'][partnameSelected]['angle_hi']+np.pi) % (2.0*np.pi) - np.pi
            
            
            if (paramsScaled['gui'][partnameSelected]['angle_hi'] < paramsScaled['gui'][partnameSelected]['angle_lo']):
                paramsScaled['gui'][partnameSelected]['angle_hi'] += 2*np.pi
            
            bodypartSelected.bValidMask = False

            if (partnameSelected in ['left','right']):
                if (paramsScaled['gui']['symmetric']):
                    paramsScaled['gui'][partnameSlave][tagSlave]     = -paramsScaled['gui'][partnameSelected][tagSelected]
                    paramsScaled['gui'][partnameSlave][tagSelected]  = -paramsScaled['gui'][partnameSelected][tagSlave]
                    paramsScaled['gui'][partnameSlave]['radius_outer'] = paramsScaled['gui'][partnameSelected]['radius_outer']
                    paramsScaled['gui'][partnameSlave]['radius_inner'] = paramsScaled['gui'][partnameSelected]['radius_inner']
                    bodypartSlave.bValidMask = False
                    
#                 if (paramsScaled['gui'][partnameSlave]['angle_hi'] < 0 < paramsScaled['gui'][partnameSlave]['angle_lo']):
#                     paramsScaled['gui'][partnameSlave]['angle_hi'] += 2*np.pi
 
              
        # Inner radius.
        elif (tagSelected=='radius_inner'): 
            paramsScaled['gui'][partnameSelected]['radius_inner'] = float(min(np.linalg.norm(bodypartSelected.ptHinge_i - ptMouse), 
                                                                              bodypartSelected.params['gui'][partnameSelected]['radius_outer']-2*self.scale))
            
            bodypartSelected.bValidMask = False
            
            if (partnameSelected in ['left','right']) and (paramsScaled['gui']['symmetric']):
                paramsScaled['gui'][partnameSlave]['radius_outer'] = paramsScaled['gui'][partnameSelected]['radius_outer']
                paramsScaled['gui'][partnameSlave]['radius_inner'] = paramsScaled['gui'][partnameSelected]['radius_inner']
                bodypartSlave.bValidMask = False
                
        # Center.
        elif (tagSelected=='center'): 
            if (partnameSelected=='aux'):

                # Move the center point.
                pt = ptMouse
                paramsScaled['gui'][partnameSelected]['center']['x'] = float(pt[0])
                paramsScaled['gui'][partnameSelected]['center']['y'] = float(pt[1])
                bodypartSelected.bValidMask = False
                
        # Axis Points.
        elif (tagSelected in ['pt1','pt2']): 
            if (partnameSelected=='axis'):
                # Move the point.
                pt = ptMouse
                paramsScaled['gui'][partnameSelected][tagSelected]['x'] = float(pt[0])
                paramsScaled['gui'][partnameSelected][tagSelected]['y'] = float(pt[1])
                bodypartSelected.bValidMask = False
                
        # Radius.
        elif (tagSelected=='radius1'): 
            pt = ptMouse - bodypartSelected.ptCenter_i
            paramsScaled['gui'][partnameSelected]['radius1'] = float(np.linalg.norm(pt))
            paramsScaled['gui'][partnameSelected]['angle'] = float(np.arctan2(pt[1], pt[0]))
            bodypartSelected.bValidMask = False        
        elif (tagSelected=='radius2'): 
            pt = bodypartSelected.ptCenter_i - ptMouse
            paramsScaled['gui'][partnameSelected]['radius2'] = float(np.linalg.norm(pt))
            paramsScaled['gui'][partnameSelected]['angle'] = float(np.arctan2(pt[1], pt[0])-np.pi/2.0)
            bodypartSelected.bValidMask = False        
                

        # Unscale the parameters since we're finished adjusting them.
        self.params = self.scale_params(paramsScaled, 1/self.scale)
        
    # End update_params_from_mouse() 


    # onMouse()
    # Handle mouse events.
    #
    def onMouse(self, event, x, y, flags, param):
        if (self.imgScaled is None) or (self.buttons is None): # No image yet.
            return
        ptMouse = np.array([x, y]).clip((0,0), (self.shapeImage[1],self.shapeImage[0]))

        # Keep track of which UI element is selected.
        if (event==cv2.EVENT_LBUTTONDOWN):
            self.bMousing = True
            
            # Get the name and ui nearest the current point.
            (ui, tag, partname, iButtonSelected) = self.hit_object(ptMouse)
            self.tagSelected = tag                                          # hinge, angle_hi, angle_lo, center, radius1, radius2, radius_inner.
            self.partnameSelected = partname                                # left, right, head, abdomen.
            self.uiSelected = ui                                            # pushbutton, checkbox, handle.
            self.iButtonSelected = iButtonSelected                          # Index of the button.
            self.nameSelected = self.name_from_tagpartname(tag,partname)    # tag_partname.
            #rospy.logwarn((ui, tag, partname, iButtonSelected))

            if (self.iButtonSelected is not None):
                self.stateSelected = self.buttons[self.iButtonSelected].state
            
            self.nameSelectedNow = self.nameSelected
            self.uiSelectedNow = self.uiSelected
            

        if (self.uiSelected=='pushbutton') or (self.uiSelected=='checkbox'):
            # Get the partname and ui tag nearest the mouse point.
            (ui, tag, partname, iButtonSelected) = self.hit_object(ptMouse)
            self.nameSelectedNow     = self.name_from_tagpartname(tag,partname)
            self.tagSelectedNow      = tag
            self.partnameSelectedNow = partname
            self.uiSelectedNow       = ui
            self.iButtonSelectedNow  = iButtonSelected


            # Set selected button to 'down', others to 'up'.
            for iButton in range(len(self.buttons)):
                if (self.buttons[iButton].type=='pushbutton'):
                    if (iButton==self.iButtonSelectedNow==self.iButtonSelected) and not (event==cv2.EVENT_LBUTTONUP):
                        self.buttons[iButton].state = True # 'down'
                    else:
                        self.buttons[iButton].state = False # 'up'
                        
            # Set the checkbox.
            if (self.uiSelected=='checkbox'):
                if (self.nameSelected == self.nameSelectedNow):
                    self.buttons[self.iButtonSelected].state = not self.stateSelected # Set it to the other state when we're on the checkbox.
                else:
                    self.buttons[self.iButtonSelected].state = self.stateSelected # Set it to the original state when we're off the checkbox.

        # end if ('pushbutton' or 'checkbox'):

                        
        elif (self.uiSelected=='handle'):
            # Set the new params.
            self.update_params_from_mouse(self.tagSelected, self.partnameSelected, ptMouse.clip((0,self.yToolbar), (self.shapeImage[1],self.shapeImage[0])))
            self.fly.set_params(self.scale_params(self.params, self.scale))
        
        # end if ('handle'):
            

        if (event==cv2.EVENT_LBUTTONUP):
            # If the mouse is on the same button at mouseup, then do the action.
            if (self.uiSelected=='pushbutton'):
                if (self.nameSelected == self.nameSelectedNow == 'save_bg'):
                    self.command_callback('save_background')

                elif (self.nameSelected == self.nameSelectedNow == 'exit'):
                    self.command_callback('exit')
                    
                    
            elif (self.uiSelected=='checkbox'):
                if (self.nameSelected == self.nameSelectedNow):
                    self.buttons[self.iButtonSelected].state = not self.stateSelected
                else:
                    self.buttons[self.iButtonSelected].state = self.stateSelected


                if (self.nameSelected == self.nameSelectedNow == 'symmetry'):
                    self.params['gui']['symmetric'] = self.buttons[self.iButtonSelected].state
                    
                elif (self.nameSelected == self.nameSelectedNow == 'subtract_lr'):
                    if (not self.bHaveBackground):
                        self.buttons[iButtonSelected].state = False
                        runtime.logwarn('No background image.  Cannot use background subtraction for left/right.')

                    self.params['gui']['left']['subtract_bg']  = self.buttons[iButtonSelected].state
                    self.params['gui']['right']['subtract_bg'] = self.buttons[iButtonSelected].state
                    
                elif (self.nameSelected == self.nameSelectedNow == 'subtract_aux'):
                    if (not self.bHaveBackground):
                        self.buttons[iButtonSelected].state = False
                        runtime.logwarn('No background image.  Cannot use background subtraction for aux.')

                    self.params['gui']['aux']['subtract_bg']  = self.buttons[iButtonSelected].state
                    
                elif (self.nameSelected == self.nameSelectedNow == 'subtract_head'):
                    if (not self.bHaveBackground):
                        self.buttons[iButtonSelected].state = False
                        runtime.logwarn('No background image.  Cannot use background subtraction for head.')

                    self.params['gui']['head']['subtract_bg']  = self.buttons[iButtonSelected].state
                    
                elif (self.nameSelected == self.nameSelectedNow == 'subtract_abdomen'):
                    if (not self.bHaveBackground):
                        self.buttons[iButtonSelected].state = False
                        runtime.logwarn('No background image.  Cannot use background subtraction for abdomen.')

                    self.params['gui']['abdomen']['subtract_bg']  = self.buttons[iButtonSelected].state
                    
                elif (self.nameSelected == self.nameSelectedNow == 'head'):
                    self.params['gui']['head']['track'] = self.buttons[iButtonSelected].state
                    
                elif (self.nameSelected == self.nameSelectedNow == 'abdomen'):
                    self.params['gui']['abdomen']['track'] = self.buttons[iButtonSelected].state

                elif (self.nameSelected == self.nameSelectedNow == 'left'):
                    self.params['gui']['left']['track']  = self.buttons[iButtonSelected].state

                elif (self.nameSelected == self.nameSelectedNow == 'right'):
                    self.params['gui']['right']['track'] = self.buttons[iButtonSelected].state

                elif (self.nameSelected == self.nameSelectedNow == 'aux'):
                    self.params['gui']['aux']['track'] = self.buttons[iButtonSelected].state
                    if (self.params['gui']['aux']['track']):
                        self.fly.aux.wingbeat.warn()
                    
                # A button to allow the user to override the automatic invertcolor detector.  A better autodetect algorithm might eliminate the need for this.
                elif (self.nameSelected == self.nameSelectedNow == 'invertcolor'):
                    self.fly.bInvertColor      = self.buttons[iButtonSelected].state
                    self.fly.bInvertColorValid = True
                    self.fly.bInvertColorAuto  = False # If the user clicked this button, then switch to manual mode.

                elif (self.nameSelected == self.nameSelectedNow == 'stabilize'):
                    self.params['gui']['head']['stabilize']    = self.buttons[iButtonSelected].state
                    self.params['gui']['abdomen']['stabilize'] = self.buttons[iButtonSelected].state
                    self.params['gui']['left']['stabilize']    = self.buttons[iButtonSelected].state
                    self.params['gui']['right']['stabilize']   = self.buttons[iButtonSelected].state

                elif (self.nameSelected == self.nameSelectedNow == 'windows'):
                    self.params['gui']['windows'] = self.buttons[iButtonSelected].state


            if (self.uiSelected in ['handle','checkbox']):
                self.fly.set_params(self.scale_params(self.params, self.scale))
                self.fly.create_masks(self.shapeImage)
    
                # Save the results.
                self.save_params()

            # Dump the params to the screen, for debugging.
#             for k,v in self.params.iteritems():
#                 rospy.logwarn((k,type(v)))
#                 if (type(v)==type({})):
#                     for k2,v2 in v.iteritems():
#                         rospy.logwarn('     %s, %s' % (k2,type(v2)))
                

            self.bMousing           = False
            self.nameSelected       = None
            self.nameSelectedNow    = None
            self.uiSelected         = None
            self.uiSelectedNow      = None
            self.iButtonSelected    = None
            self.iButtonSelectedNow = None
        
    # End onMouse()

            


    def set_paused(self, bPaused):
        self.bPaused = bPaused
        if (self.player is not None):
            if (bPaused):
                self.player.pause()
            else:
                self.player.resume()


    # handle_gui()
    # Refresh the display if it's due, and process the keyboard & mouse.
    #
    def handle_gui(self):
        if (not self.params['use_gui']):
            return

        if (self.display_due()):
            self.render()

            # When paused, block here until the next refresh rather than spinning.  When running,
            # just process the pending window events:  even waitKey(1) blocks for ~10-16ms on Windows.
            if (self.bPaused):
                key = cv2.waitKey(max(1, int(1000.0/max(1.0, self.params['display_hz'])))) & 0xFF
            elif (hasattr(cv2, 'pollKey')):
                key = cv2.pollKey() & 0xFF
            else:
                key = cv2.waitKey(1) & 0xFF

            if (key in [ord('q'), ord('Q'), 27]): # Esc
                self.command_callback('exit')
            elif (key == ord(' ')):
                self.command_callback('pause')

            # Closing the window quits.
            try:
                if (cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1):
                    self.command_callback('exit')
            except cv2.error:
                self.command_callback('exit')


    # report_progress()
    # Print progress occasionally, which matters mostly when there's no GUI.
    #
    def report_progress(self, bForce=False):
        t = runtime.now()
        if (bForce) or (t - self.tProgressPrev >= 5.0):
            self.tProgressPrev = t
            if (self.frame is not None) and (self.tStart is not None):
                elapsed = t - self.tStart
                hz = self.nTracked/elapsed if (elapsed > 0) else 0.0
                s = 'frame %d' % (self.frame.header.seq+1)
                if (self.source.n_frames > 0):
                    s += '/%d' % self.source.n_frames
                    if (not self.bRealtime) and (hz > 0):
                        s += ', about %.0fs to go' % ((self.source.n_frames - self.frame.header.seq - 1)/hz)
                s += ', tracking at %.1f frames/s' % hz
                if (self.bRealtime):
                    s += ', %d dropped' % (self.player.nEmitted - self.nTracked)
                runtime.loginfo(s)


    # report_stats()
    # Summarize how tracking kept up.
    #
    def report_stats(self):
        elapsed = (runtime.now() - self.tStart) if (self.tStart is not None) else 0.0
        runtime.loginfo('----- %s' % self.source.path)
        runtime.loginfo('Mode:            %s' % ('realtime' if (self.bRealtime) else 'offline'))
        runtime.loginfo('Frames tracked:  %d in %.1fs (%.1f frames/s)' % (self.nTracked, elapsed, self.nTracked/elapsed if (elapsed>0) else 0.0))
        if (self.bRealtime):
            nEmitted = self.player.nEmitted
            nUntracked = nEmitted - self.nTracked
            runtime.loginfo('Frames delivered: %d at the recording rate; %d (%.1f%%) not tracked' % (nEmitted, nUntracked, 100.0*nUntracked/nEmitted if (nEmitted>0) else 0.0))
            runtime.loginfo('Player lateness: worst %.1fms' % (1000.0*self.player.lagMax))
        if (len(self.processtimes) > 0):
            p = 1000.0*np.array(self.processtimes)
            l = 1000.0*np.array(self.latencies)
            runtime.loginfo('Tracking time:   mean %.2fms, 95th percentile %.2fms, max %.2fms' % (p.mean(), np.percentile(p, 95), p.max()))
            runtime.loginfo('Latency:         mean %.2fms, 95th percentile %.2fms, max %.2fms  (frame arrival to results)' % (l.mean(), np.percentile(l, 95), l.max()))
        for sink in self.sinks:
            if (hasattr(sink, 'path')):
                runtime.loginfo('Results:         %s (%d rows)' % (sink.path, sink.nRows))


    # run()
    # Track the source until it ends or the user quits.
    #
    def run(self):
        if (self.params['gui']['aux']['track']):
            self.fly.aux.wingbeat.warn()

        if (self.player is not None):
            if (self.bPaused):
                # Show the first frame to position the handles on.
                frame = self.source.read()
                self.source.rewind()
                if (frame is not None):
                    self.ingest_frame(frame)
                    self.bFrameTracked = True # Only a preview; the player delivers it again.
            self.player.start(paused=self.bPaused)
        else:
            frame = self.source.read()
            if (frame is not None):
                self.ingest_frame(frame)

        while (not self.bQuitting):
            if (self.bRealtime):
                frame = self.get_frame()
                if (frame is not None):
                    self.ingest_frame(frame)
                    self.track_frame()
                elif (self.player.bFinished.is_set()):
                    break
                elif (not self.params['use_gui']) or (not self.display_due()):
                    # Wait for the next frame, but not past the next display refresh.
                    tWait = 0.05
                    if (self.params['use_gui']):
                        tWait = min(tWait, max(0.0, self.tDisplayPrev + 1.0/max(1.0, self.params['display_hz']) - runtime.now()))
                    self.eventImage.wait(tWait)

            elif (not self.bPaused) and (not self.bMousing):
                if (self.frame is not None) and (not self.bFrameTracked):
                    self.track_frame()
                else:
                    frame = self.source.read()
                    if (frame is None):
                        if (self.bLoop) and (self.source.n_frames > 0):
                            self.source.rewind()
                            continue
                        break
                    self.ingest_frame(frame)
                    self.track_frame()

            self.handle_gui()
            self.report_progress()

        if (self.player is not None):
            self.player.stop()
        for sink in self.sinks:
            sink.close()
        self.save_params()
        self.report_stats()
        if (self.params['use_gui']):
            cv2.destroyAllWindows()
            cv2.waitKey(1)


###############################################################################
###############################################################################
def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Kinefly: track the wings, head and abdomen of a tethered fly in a recorded video.',
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     epilog='''modes:
  offline    Track every frame as fast as possible.  Default for .avi, .mp4 (and other OpenCV-readable videos).
  realtime   Deliver frames at the rate they were recorded, dropping frames whenever tracking falls
             behind, as with a live camera.  Default for .ufmf.

keys (in the window):
  space      Pause / run.  Kinefly starts paused on the first frame so you can position the handles.
  q, Esc     Quit (the handle positions are saved).
''')
    parser.add_argument('videos', nargs='*', help='Video file(s) to track (.avi, .mp4, .ufmf, ...).  Wildcards are allowed.')
    parser.add_argument('--video', action='append', default=[], help='Video file to track (same as the positional argument).')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--realtime', dest='realtime', action='store_true', default=None, help='Force realtime mode.')
    mode.add_argument('--offline', dest='realtime', action='store_false', help='Force offline mode.')
    parser.add_argument('--gui-yaml', default='~/kinefly.yaml', help='Where the handle positions are loaded from and saved to (default: %(default)s).  Use one file per camera setup.')
    parser.add_argument('--params', default=None, help='YAML file of other Kinefly parameters (trackers, thresholds, wingbeat band, ...).')
    parser.add_argument('--scale', type=float, default=None, help='Scale the images by this factor before tracking, e.g. 0.5 for speed.')
    parser.add_argument('--display-hz', type=float, default=None, help='How often to refresh the window (default 30).  Lower it to leave more time for tracking.')
    parser.add_argument('--csv', default=None, help='Results file (default: <video>_kinefly_<mode>.csv next to the video).  Only with a single video.')
    parser.add_argument('--no-csv', action='store_true', help='Do not write results.')
    parser.add_argument('--loop', action='store_true', help='Replay the video in a loop, e.g. while positioning the handles.  No CSV unless --csv is given.')
    parser.add_argument('--no-gui', action='store_true', help='No window; just track and write results.')
    parser.add_argument('--no-pause', action='store_true', help='Start tracking immediately instead of paused on the first frame.')
    args = parser.parse_args(argv)

    paths = []
    for pattern in args.videos + args.video:
        matches = sorted(glob.glob(os.path.expanduser(pattern)))
        if (len(matches) == 0):
            parser.error('No such video: %s' % pattern)
        paths.extend(matches)
    if (len(paths) == 0):
        parser.error('Give at least one video file.')
    if (args.csv is not None) and (len(paths) > 1):
        parser.error('--csv only works with a single video.')
    args.paths = paths
    return args


def main(argv=None):
    args = parse_args(argv)

    runtime.loginfo('**************************************************************************')
    runtime.loginfo('     Kinefly: Camera-based Tethered Insect Kinematics Analyzer')
    runtime.loginfo('         by Steve Safarik, Floris van Breugel (c) 2014')
    runtime.loginfo('**************************************************************************')

    for (iPath, path) in enumerate(args.paths):
        source = open_source(path)
        bRealtime = source.default_realtime if (args.realtime is None) else args.realtime
        runtime.loginfo('%s: %d frames, %dx%d, %.1f fps, %s mode' % (path, source.n_frames, source.shape[1], source.shape[0], source.fps, 'realtime' if (bRealtime) else 'offline'))

        filenameCsv = None
        if (args.csv is not None):
            filenameCsv = os.path.expanduser(args.csv)
        elif (not args.no_csv) and (not args.loop):
            filenameCsv = '%s_kinefly_%s.csv' % (os.path.splitext(path)[0], 'realtime' if (bRealtime) else 'offline')

        # Pause on the first video only, to position the handles.
        bStartPaused = (iPath == 0) and (not args.no_pause) and (not args.no_gui)

        window = MainWindow(source, args, bRealtime, bStartPaused, filenameCsv)
        try:
            window.run()
        finally:
            source.close()

        if (window.bQuitting) and (iPath < len(args.paths)-1):
            runtime.loginfo('Quit requested; skipping the remaining videos.')
            break


if __name__ == '__main__':
    main()
