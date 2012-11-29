#!/usr/bin/python

import matplotlib.pyplot as plt
import pylab
import numpy as np
 
def processLine(line):
  "Process a line in the form of u v g1 m1 g2 m2 ..." 
  
  array = [float(x) for x in line.split()]
   
  u = int(array[0])
  v = int(array[1])

  g = array[2::2]
  m = array[3::2]
   
  return u, v, g, m
  
def plotLine(line):
  "Plots the data from a line in the file"
  
  u, v, g, m = processLine(line)
  
  plt.plot(g, m, 'ro')
  plt.axis([0, 40, 0, 40])
  plt.show() 
  
  return
  
def fitLine(line):
  "fits the data to a polynomial"

  u, v, g, m = processLine(line)
  
  deg = 3
  coeff = np.polyfit(g, m, deg)
  print coeff
    
  g2 = np.arange(0, 4000, 5) #more points for a smoother plot
  m2 = np.polyval(coeff, g2) 
  
  g3 = np.arange(0, 4000, 5) #more points for a smoother plot
  m3 = np.polyval([1,0], g3) 
  
  lbl = '(' + `u` + ', ' + `v` + ') data'
  print lbl
  
  plt.plot(g, m, 'ro', label=lbl) # ro = red points
  plt.plot(g2, m2, label='fit')
  plt.plot(g3, m3, label="linear")
  
  plt.axis([0, 4000, 0, 4000])
  ax = plt.gca()
  ax.set_autoscale_on(False)
  ax.set_aspect('equal')
  plt.legend(loc='lower right')
  plt.show()
  
  return coeff
  
def processFile(filename):
  "Process a data file with pixel calibration values"
  
  file = open("plot/data.txt","r")
  lines = file.readlines()
  file.close()
  
  u,v  = 20, 20
  index = v * 640 + u 
  coeff = fitLine(lines[index])
  
  u,v  = 320, 240
  index = v * 640 + u 
  coeff = fitLine(lines[index])
  
  
  #index = 198700
  #index = 100 * 640 + 220
  #index = 120 * 640 + 100
  

  
  return
  
# main
  
processFile('plot/data')
