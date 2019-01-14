#!/usr/bin/python
import rospy
import pygame
from pygame.locals import *
import numpy as np

def process_event(event):
  if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
    click_position = event.pos

    print click_position

def start():
  pygame.init()

  gameDisplay = pygame.display.set_mode( (800, 600) )
  pygame.display.set_caption("Interactive Control")

  running = True

  while running:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        running = False

      process_event( event )

  pygame.quit()

def run():
  rospy.init_node("interactive_control", anonymous = True)
  start()

if __name__ == "__main__":
  try:
    run()
  except rospy.ROSInterruptException, e:
    pass
