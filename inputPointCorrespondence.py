import sys, argparse
import matplotlib.pyplot as plt
import numpy as np
import os.path
import subprocess

parser = argparse.ArgumentParser(description='The program facilitates point-correspondence input for at least 4 non-colinear points, inputed in the same order in a video frame and a aerial photo/ground map, or from the list of corresponding points in the two planes. It also requires a second point for each initial one in the video frame, in order to allow for calibration of the center-of-mass height of the tracked pedestrians.', epilog = '''The point correspondence file contains at least 4 non-colinear point coordinates 
with the following format:
 - the first two lines are the x and y coordinates in the projected space (usually world space)
 - the third and fourth lines are the x and y coordinates in the origin space (usually image space)
 - the last two lines are the x and y points approximately 1.5 meters above the ground-plane, directly verticle to the previous points in the origin space

If providing video and world images, with a number of points to input
and a ration to convert pixels to world distance unit (eg meters per pixel), 
the images will be shown in turn and the user should click 
in the same order the corresponding points in world and image spaces.''', formatter_class=argparse.RawDescriptionHelpFormatter,)

parser.add_argument('-i', dest = 'videoFrameFilename', help = 'filename of the video frame')
parser.add_argument('-w', dest = 'worldFilename', help = 'filename of the aerial photo/ground map')
parser.add_argument('-u', dest = 'unitsPerPixel', help = 'number of units per pixel', default = 1., type = float)
parser.add_argument('-f', dest = 'filename_addon', help = 'point correspondence filename, including extension', default = 'ext-point-correspondence-default.txt', type = str)
parser.add_argument('-e', '--elevationmod', help = 'Whether allow a range of elevations.', choices=[0,1],default = 0, type=int)
args = parser.parse_args()
faddon = args.filename_addon

if args.videoFrameFilename != None and args.worldFilename != None:
	if os.path.isfile(faddon):
		removal = 'rm ' + faddon
		subprocess.check_call(removal, shell=True)
	worldImg = plt.imread(args.worldFilename)
	videoImg = plt.imread(args.videoFrameFilename)
	print('Click on 4 points in the video frame.')
	plt.figure()
	plt.imshow(videoImg)
	videoPts = np.array(plt.ginput(4, timeout=3000))
	if args.elevationmod == 1:
		print('Click on 4 points ~1.5m above the previous 4.')
		plt.figure()
		plt.imshow(videoImg)
		elevPts = np.array(plt.ginput(4, timeout=3000))
	print('Click on 4 points in the world image')
	plt.figure()
	plt.imshow(worldImg)
	worldPts = args.unitsPerPixel*np.array(plt.ginput(4, timeout=3000))
	plt.close('all')
	f = open(faddon, 'a')
	np.savetxt(f, worldPts.T)
	if args.elevationmod == 1:
		np.savetxt(f, elevPts.T)
	np.savetxt(f, videoPts.T)
	f.close()
