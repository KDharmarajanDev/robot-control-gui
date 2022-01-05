# import the necessary packages
from __future__ import print_function
from PIL import Image
from PIL import ImageTk
import tkinter as tki
import datetime
import imutils
import cv2
import os
import rospy
from control_gui.msg import PoseEndpoints

class GUI:
	def __init__(self):
		# GUI 
		self.stopEvent = None
		self.root = tki.Tk()
		self.root.wm_title("YuMi GUI")
		self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)

		self.button_frame = tki.Frame(self.root)
		self.button_frame.pack(side="top", pady=10)

		self.align_button = tki.Button(self.button_frame, text="Align", command=self.align_select)
		self.align_button.pack(side="left", pady=10, padx=10)

		self.push_button = tki.Button(self.button_frame, text="Push", command=self.push_select)
		self.push_button.pack(side="right", pady=10, padx=10)

		self.canvas = None
		self.idFrame = None
		# Robot Action States
		self.is_placing_align_points = False
		self.is_placing_push_points = False

		self.data = PoseEndpoints()

		# ROS topics
		self.align_pub = rospy.Publisher('/control/align', PoseEndpoints, queue_size=10)
		self.push_pub = rospy.Publisher('/control/push', PoseEndpoints, queue_size=10)

	def update_background_image(self, cv_img):
		try:
			image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
			image = Image.fromarray(image)
			image = ImageTk.PhotoImage(image)

			if self.canvas is None:
				# Setup canvas
				self.canvas = tki.Canvas(self.root, width=cv_img.shape[1], height=cv_img.shape[0])
				self.canvas.pack()
				self.canvas.bind("<Button-1>", self.click_callback)
				self.idFrame = self.canvas.create_image(0, 0, image=image, anchor=tki.NW)
			else:
				self.canvas.itemconfig(self.idFrame, image=image)
				self.canvas.image = image

		except RuntimeError:
			print("[INFO] caught a RuntimeError")

	def onClose(self):
		# set the stop event, cleanup the camera, and allow the rest of
		# the quit process to continue
		print("[INFO] closing...")
		self.root.quit()

	def click_callback(self, event):
		print(event)

	def align_select(self):
		self.is_placing_align_points = not self.is_placing_align_points
		self.is_placing_push_points = False
		if self.is_placing_align_points:
			self.root.config(cursor="tcross")
		else:
			self.root.config(cursor="arrow")

	def push_select(self):
		self.is_placing_push_points = not self.is_placing_push_points
		self.is_placing_align_points = False
		if self.is_placing_push_points:
			self.root.config(cursor="dotbox")
		else:
			self.root.config(cursor="arrow")

