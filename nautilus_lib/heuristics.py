#!/usr/bin/python

import numpy as np

def manhattan(source, target):
  return np.abs( (source - target) ).sum()

def euclidean(source, target):
  return np.sqrt(((source - target) ** 2).sum())

def euclidean_quad(source, target):
  return np.sqrt( ((source - target) ** 2).sum() ) ** 2

def chebyshev(source, target):
  return np.max( source - target )

def minkowski(source, target, p=10):
  return (np.abs(( source - target )) ** p).sum() ** 1/p
