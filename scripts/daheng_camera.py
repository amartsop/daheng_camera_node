#!/usr/bin/env python
import rospy
import gxipy as gx
import numpy
import cv2

class DahengCamera:
    def __init__(self):

        # Setup camera params
        self.camera_setup()

        # Raw camera data container
        self.raw_images = []

        # Total frames captured
        self.frames_captured = 0

        # Recording status 
        self.recording_on = False
        self.pending_recording_start = True
        self.pending_recording_stop = False

    def __del__(self):
        # close device
        self.cam.close_device()
        cv2.destroyAllWindows()  

    # Get toal number of frames
    def get_total_frames(self):
        return self.frames_captured


    def raw_img_acquisition(self, recording_status):

        if (recording_status and self.pending_recording_start):
            # Open stream
            self.cam.stream_on()

            # Update start recording pending status
            self.pending_recording_start = False

            # Update stop recording pending status
            self.pending_recording_stop = True

        if (recording_status): 
            # get raw image
            self.raw_images.append(self.cam.data_stream[0].get_image())
            if self.raw_images[-1] is None:
                print("Getting image failed.")

        if ((not recording_status) and self.pending_recording_stop):
            # Close stream
            self.cam.stream_off()

            # Total frames captures
            self.frames_captured = self.raw_images[-1].get_frame_id()

            # Update start recording pending status
            self.pending_recording_start = True

            # Update start recording pending status
            self.pending_recording_stop = False

    def play_frames(self):
        for raw_image in self.raw_images:

            # Get RGB image from raw image
            rgb_image = raw_image.convert("RGB")
            if rgb_image is None:
                continue

            # improve image quality
            rgb_image.image_improvement(self.color_correction_param,
                self.contrast_lut, self.gamma_lut)

            # create numpy array with data from raw image
            numpy_image = rgb_image.get_numpy_array()
            if numpy_image is None:
                continue

	        #display image with opencv
            pimg = cv2.cvtColor(numpy.asarray(numpy_image),cv2.COLOR_BGR2RGB)

	        #cv2.imwrite("cat2.jpg", pimg)
            cv2.imshow("Image", pimg)
            cv2.waitKey(10)

            # print height, width, and frame ID of the acquisition image
            print("Frame ID: %d   Height: %d   Width: %d"
                %(raw_image.get_frame_id(), raw_image.get_height(), raw_image.get_width()))

    def camera_setup(self):
        # create a device manager
        self.device_manager = gx.DeviceManager()
        self.dev_num, self.dev_info_list = self.device_manager.update_device_list()

        if self.dev_num is 0:
            print("Number of enumerated devices is 0")
            return
        
        # open the first device
        self.cam = self.device_manager.open_device_by_index(1)

        # set Width
        self.cam.Width.set(1440)

        # set Height
        self.cam.Height.set(1080)

        # offset width
        self.cam.OffsetX.set(0)

        # offset width
        self.cam.OffsetY.set(0)
    
        # set continuous acquisition
        self.cam.TriggerMode.set(gx.GxSwitchEntry.OFF)

        # set exposure
        self.cam.ExposureTime.set(10000.0)

        # set gain
        self.cam.Gain.set(24.0)

        # set auto white balance
        self.cam.BalanceWhiteAuto.set(1)

        # User Set Selector
        self.cam.UserSetSelector.set(1)
        self.cam.UserSetSave.send_command()

        # get param of improving image quality
        if self.cam.GammaParam.is_readable():
            gamma_value = self.cam.GammaParam.get()
            self.gamma_lut = gx.Utility.get_gamma_lut(gamma_value)
        else:
            self.gamma_lut = None

        if self.cam.ContrastParam.is_readable():
            contrast_value = self.cam.ContrastParam.get()
            self.contrast_lut = gx.Utility.get_contrast_lut(contrast_value)
        else:
            self.contrast_lut = None

        if self.cam.ColorCorrectionParam.is_readable():
            self.color_correction_param = self.cam.ColorCorrectionParam.get()
        else:
            self.color_correction_param = 0

        # set the acq buffer count
        self.cam.data_stream[0].set_acquisition_buffer_number(2)