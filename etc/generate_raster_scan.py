import sys
import os, time
from optparse import OptionParser
try:
    from pkg_resources import require
    require('six')
    require('cothread')
    require('matplotlib')

    require('h5py-swmr')

    import cothread
    from cothread.catools import *
    import numpy
    from numpy import *

    import h5py
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
except:
    pass

def main():
    print('caput -c -w 1 PMAC_TEST:GB1:A:Offset 0.0')
    print('caput -c -w 1 PMAC_TEST:GB1:B:Offset 0.0')
    print('caput -c -w 1 PMAC_TEST:GB1:A:Resolution -0.0005')
    print('caput -c -w 1 PMAC_TEST:GB1:B:Resolution -0.0005')
    print('caput -c -w 1 PMAC_TEST:GB1:A:UseAxis 1')
    print('caput -c -w 1 PMAC_TEST:GB1:B:UseAxis 1')
    print('caput -c -w 1 PMAC_TEST:GB1:C:UseAxis 0')
    print('caput -c -w 1 PMAC_TEST:GB1:U:UseAxis 0')
    print('caput -c -w 1 PMAC_TEST:GB1:V:UseAxis 0')
    print('caput -c -w 1 PMAC_TEST:GB1:W:UseAxis 0')
    print('caput -c -w 1 PMAC_TEST:GB1:X:UseAxis 0')
    print('caput -c -w 1 PMAC_TEST:GB1:Y:UseAxis 0')
    print('caput -c -w 1 PMAC_TEST:GB1:Z:UseAxis 0')

    no_of_rows = 80
    y_val = -4.0
    x_range = 4.0
    x_delta = x_range / 60.0
    x_start = -12.0 - x_delta
    print('caput -a PMAC_TEST:GB1:A:Positions {0}'.format(no_of_rows * 63)),
    for index in range(no_of_rows):
      for pts in range(63):
        if index%2 == 0:
          print('{0}'.format(x_start + x_delta * float(pts))),
        else:
          print('{0}'.format(x_start + x_delta * float(62-pts))),
        
    sys.stdout.flush()
    print('')
        
    print('caput -a PMAC_TEST:GB1:B:Positions {0}'.format(no_of_rows * 63)),
    for index in range(no_of_rows):
      for pts in range(63):
        print('{0}'.format(-4.0 - (float(index)*0.05))),
        
    sys.stdout.flush()
    print('')
        
    print('caput -a PMAC_TEST:GB1:ProfileTimeArray {0}'.format(no_of_rows * 63)),
    print('{0}'.format(3000000)),
    for index in range(no_of_rows):
      for pts in range(62):
        print('{0}'.format(100000)),
      print('{0}'.format(4000000)),
      
    sys.stdout.flush()
    print('')
       
    print('caput -a PMAC_TEST:GB1:VelocityMode {0}'.format(no_of_rows * 63)),
    print('{0}'.format(2)),
    for index in range(no_of_rows):
      for pts in range(61):
        print('{0}'.format(0)),
      print('{0}'.format(1)),
      print('{0}'.format(2)),
      
    sys.stdout.flush()
    print('')
       
    print('caput -c -w 1 PMAC_TEST:GB1:ProfileNumPoints {0}'.format(no_of_rows * 63 * 2))
    print('caput -c -w 1 PMAC_TEST:GB1:ProfilePointsToBuild {0}'.format(no_of_rows * 63))
    print('caput -c -w 1 PMAC_TEST:GB1:ProfileBuild 1')
    print('# NOTE NumPoints is twice PointsToBuild so 1 additional append will complete upload of the scan')
    print('# Once you hace started the scan hit Append Points and you will upload the same set of points again')
    print('# the scan will then complete at 10080 points')
if __name__ == "__main__":
    main()

