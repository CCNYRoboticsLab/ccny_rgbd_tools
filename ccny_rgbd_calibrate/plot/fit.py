#!/usr/bin/python

import matplotlib.pyplot as plt
import pylab
import numpy as np
 
def processLine(line):
  "Process a line in the form of u v c1 c2 c3 g1 m1 g2 m2 ..." 
  
  array = [float(x) for x in line.split()]
   
  u = int(array[0])
  v = int(array[1])
  
  c = array[2:5]
    
  g = array[5::2]
  m = array[6::2]
  
  c = c[::-1]
   
  return u, v, c, g, m
  
def plotLine(line):
  "Plots the data from a line in the file"
  
  u, v, g, m = processLine(line)
  
  plt.plot(g, m, 'ro')
  plt.axis([0, 40, 0, 40])
  plt.show() 
  
  return
  
def fitLine(u, v, lines):
  "fits a data at a fiven pixel to a polynomial"

  w = 5
  
  acc_g = []
  acc_m = []
  
  for uu in range(u-w,u+w):
    for vv in range(v-w,v+w):
      if uu >= 0 and uu < 640 and vv > 0 and vv <= 640:
        idx_n = vv * 640 + uu
        
        line = lines[idx_n]
        nu, nv, nc, ng, nm = processLine(line)
        
        acc_g.extend(ng)
        acc_m.extend(nm)
       
  deg = 2
  coeff = np.polyfit(acc_m, acc_g, deg)
  print coeff
    
  x = np.arange(0, 5000, 5) #more points for a smoother plot
  y2 = np.polyval(coeff, x) 
  
  # as recovered by gsl
  #u, v, c, g, m = processLine(line)
  #y3 = np.polyval(c, x)
  
  #linear
  y4 = np.polyval([1,0], x) 
    
  lbl = '(' + `u` + ', ' + `v` + ') data'
  print lbl
  
  plt.plot(acc_m, acc_g, 'ro', label=lbl) # ro = red points
  plt.plot(x, y2, label='python fit')
  #plt.plot(x, y3, label="gsl fit")
  plt.plot(x, y4, label="linear")

  plt.axis([0, 5000, 0, 5000])
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

  for u in range(10, 630, 40):
    for v in range(10, 470, 40):
      coeff = fitLine(u, v, lines)
   
  return
  
# main
  
processFile('plot/data')
