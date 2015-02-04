import cv2
import cvutils
import numpy as np
import sqlite3
import math
import csv
import os.path
import subprocess
import random
import ConfigParser

######################################################################################################
#######################################         SETUP          #######################################
######################################################################################################

#Setup files to use
setup_filename = 'setup.ini'
variable_parameters_filename = 'variableParameters.txt' 
static_parameters_filename = 'staticParameters.txt'

# MODIFY THIS FUNCTION IS APPLYING GenTrO TO TRACKERS OTHER THAN TRAFFIC INTELLIGENCE OR URBAN TRACKER
def extract_trajectories(sqlite_filename):
	#Extracts trajectories into an array which GenTrO can interpret, in the format [object_id, frame, x_coord, y_coord]
	#By default, this function is coded for tracks in the PolyTrack format
	import storage

	objects = storage.loadTrajectoriesFromSqlite(sqlite_filename,'object')
	object_positions = []
	a = 0
	for o in objects:
		instant = o.timeInterval[0]
		for p in o.positions:
			object_positions.append([a,instant,p.x,p.y])
			instant += 1
		a += 1

	return object_positions

######################################################################################################
#################################        CODE - DO NOT CHANGE          ###############################
######################################################################################################

def signer():
	#Randomly selects between 1 and -1
	signer = random.uniform(0,1)
	if signer < 0.5:
		return -1
	else:
		return 1

def matchmaking(object_positions,gt_filename,T):
	#Matches Oi to Hi
	conn = sqlite3.connect(gt_filename)
	gt_positionsx = conn.execute('SELECT * FROM positions')

	max_frame = 0
	n_gt_objects = 0
	n_obj = 0

	#Add boolean IsConnected to gt_positions
	gt_positions = []
	for g in gt_positionsx:
		gt_positions.append(g)

	#Compute number of objects and frames in ground truth
	for g in gt_positions:
		if g[0] > n_gt_objects:
			n_gt_objects = g[0]
		if g[1] > max_frame:
			max_frame = g[1]

	for op in object_positions:
		if op[0] > n_obj:
			n_obj = op[0]

	#Sort into easier tables
	sorted_gt_positions = [0]*(n_gt_objects+1)
	sorted_obj_positions = [0]*(n_obj+1)
	for x in range(0,n_gt_objects+1):
		sorted_gt_positions[x] = [[0,0],[0,0]]
	for x in range(0,n_obj+1):
		sorted_obj_positions[x] = [[0,0],[0,0]]
	for g in gt_positions:
		sorted_gt_positions[g[0]].append(g)
	for s in object_positions:
		sorted_obj_positions[s[0]].append(s)
	#Dirty array fixing
	for sgt in sorted_gt_positions:
		del sgt[0]
		sgt[0] = [sgt[1][1],sgt[-1][1]]
	for sop in sorted_obj_positions:
		del sop[0]
		if len(sop) >= 2:
			sop[0] = [sop[1][1],sop[-1][1]]
		else:
			return 0,0,0,0,0,0

	match_table = []
	frame = 0
	prev_match = []

	#Make matches and calculate distance
	while frame <= max_frame:
		for sgt in sorted_gt_positions:
			if frame >= sgt[0][0] and frame <= sgt[0][1]:
				for gt in sgt:
					if gt[1] == frame:
						best_dist = [frame,sgt[1][0],-1,float('inf')]
						for sop in sorted_obj_positions:
							if frame >= sop[0][0] and frame <= sop[0][1]:
								for s in sop:
									if len(s) == 2 or len(gt) == 2:
										continue
									elif s[1] == frame:
										delta = math.sqrt(math.pow((s[2] - gt[2]),2) + math.pow((s[3] - gt[3]),2))
										if delta >= T:
											continue
										else:
											found_old_track = 0
											for pv in prev_match:
												if pv[1] == gt[0] and pv[2] == s[0]:
													best_dist = [frame,gt[0],s[0],delta,1]
													found_old_track = 1
													break
											if found_old_track == 1:
												break
											else:
												if delta < best_dist[3]:
													best_dist = [frame,gt[0],s[0],delta,0]
											break
						match_table.append(best_dist)
						break

		#Generate array of previous matches
		prev_match = []
		for mtb in match_table:
			if mtb[0] == frame:
				prev_match.append(mtb)
		frame += 1


	#Calculate MOTP
	dit = 0
	ct = 0
	mt = 0
	for mtab in match_table:
		if mtab[2] != -1:
			dit += float(mtab[3])/T
			ct += 1
		else:
			mt += 1
	if ct != 0:
		motp = 1 - (dit/ct)
	else:
		return 0,0,0,0,0,0

	#Calculate MOTA
	gt = 0
	for sgt in sorted_gt_positions:
		gt += (len(sgt)-1)

	total_traces = len(object_positions)

	fpt = total_traces - ct

	gtobj = 0
	mme = 0
	while gtobj <= n_gt_objects:
		prev = [0,0,-1,0]
		new_match = 0
		for mtab in match_table:
			if mtab[1] == gtobj:
				if new_match == 0:
					new_match = 1
					mme = mme - 1
				if mtab[2] != prev[2]:
					mme += 1
				prev = mtab
		gtobj += 1

	mota = 1-(float(mt+fpt+mme)/gt)

	print 'MOTP: ' + str(motp)
	print 'MOTA: ' + str(mota)
	return motp, mota, dit, mt, mme,fpt

def trunc(f,n):
	#Truncates floating point values
	slen = len('%.*f' % (n,f))
	return str(f)[:slen]

def ns_helper(varParamArray,x,prevsol,rc,rsolution):
	#Permutations of individual parameters
	currVarList = varParamArray[x]
	currVal = []
	currVal.append(prevsol[x])
	if str(currVarList[5]) == 'prev':
		currVarList[5] = rsolution[-1]
	if str(currVarList[6]) == 'prev':
		currVarList[6] = rsolution[-1]

	if currVarList[2] == 'float':
		if currVarList[3] == 'add':
			currVal.append(float(currVal[-1]) + signer()*random.uniform(0,float(currVarList[7]))*rc)
			if currVal[-1] < float(currVarList[5]):
				currVal.append(currVarList[5])
			elif currVal[-1] > float(currVarList[6]):
				currVal.append(currVarList[6])
		elif currVarList[3] == 'ratio':
			updown = signer()
			if updown == -1:
				currVal.append(float(currVal[-1]) / (random.uniform(1,float(currVarList[7]))*float(rc)))
			elif updown == 1:
				currVal.append(float(currVal[-1]) * (random.uniform(1,float(currVarList[7]))*float(rc)))
			if currVal[-1] < float(currVarList[5]):
				currVal.append(currVarList[5])
			elif currVal[-1] > float(currVarList[6]):
				currVal.append(currVarList[6])
		currVal.append(trunc(float(currVal[-1]),6))
	elif currVarList[2] == 'int':
		currVal.append(int(currVal[-1]) + signer() * random.randint(1,int(currVarList[7])))
		if currVal[-1] < currVarList[5]:
			currVal.append(currVarList[5])
		elif currVal[-1] > currVarList[6]:
			currVal.append(currVarList[6])
	elif currVarList[2] == 'bool':
		if currVal[-1] == currVarList[5]:
			currVal.append(currVarList[6])
		elif currVal[-1] == currVarList[6]:
			currVal.append(currVarList[5])

	changeprinter(currVarList[1],currVal[0],currVal[-1])
	return currVal[-1]

def changeprinter(param,first,second):
	#Prints changes to selected variables
	if first == second:
		pass
	else:
		ttp = param
		ttp += ' changed from ' + str(first) + ' to ' + str(second)
		print ttp

def neighbor_solution(t,t_init,include_homo_altitude_mod,prevsol,varParamArray,prevelev,prevshift,relative_change,max_n_changes,no_homography,shift_gt_homo):
	#Core state-generation function
	potential_changes = len(varParamArray)
	if include_homo_altitude_mod == 1:
		potential_changes += 4
	if shift_gt_homo == 1:
		potential_changes += 1

	#Random number of parameters to permute
	n_changes = random.randint(1,max_n_changes)

	#Selects parameters to change
	u = 0
	values_to_change = []
	while u < n_changes:
		success = 0
		while success == 0:
			add = random.randint(0,potential_changes)
			if add in values_to_change:
				pass
			else:
				values_to_change.append(add)
				success = 1
				u += 1
	#New tracker parameters
	rsolution = []
	for x in range(0,len(varParamArray)):
		if x in values_to_change:
			newvalue = ns_helper(varParamArray,x,prevsol,relative_change,rsolution)
			rsolution.append(newvalue)
		else:
			rsolution.append(prevsol[x])
	#New homography parameters
	newelev = []
	if include_homo_altitude_mod == 1:
		for x in range(len(varParamArray),len(varParamArray)+4):
			if x in values_to_change:
				newelev.append(float(prevelev[x-len(varParamArray)]) + signer()*random.uniform(0,0.1)*relative_change)
				changeprinter(('Elevation ' + str(x-len(varParamArray))), prevelev[x-len(varParamArray)],newelev[-1])
			else:
				newelev.append(prevelev[x-len(varParamArray)])
	#New ground-truth homography parameters
	newshift = []
	if include_homo_altitude_mod == 1 and shift_gt_homo == 1:
		if potential_changes in values_to_change:
			newshift.append(float(prevshift) + signer()*random.uniform(0,0.1)*relative_change)
			changeprinter('GT Elevation',prevshift,newshift[-1])
		else:
			newshift.append(prevshift)
	print rsolution
	if no_homography == 1 or include_homo_altitude_mod == 0:
		return rsolution,0,0
	elif include_homo_altitude_mod == 1 and shift_gt_homo == 0:
		return rsolution,newelev,0
	else:
		return rsolution,newelev,newshift[-1]

def point_corresp_mod(pointcorr_name,current_elevation,homo_filename,elevdiff,gthomo_filename,shift_gt_homo,altmod):
	#User-input point correspondence for new video sequences
	if altmod == 0:
		worldPts, videoPts = cvutils.loadPointCorrespondences(pointcorr_name)
		homography, mask = cv2.findHomography(videoPts, worldPts)
		np.savetxt(homo_filename,homography)
		np.savetxt(gthomo_filename,homography)

	else:

		elevprop = []
		for ce in current_elevation:
			elevprop.append(float(ce)/1.5)

		pct = open(pointcorr_name,'r')
		fullextract = pct.readlines()
		pclines = fullextract[-6::]
		video_lines = []

		worldPts = []
		temp_holder = []

		#Extract latest point correspondences
		for j in range(0,2):
			temp_holder.append(pclines[j].split())

		#Extract world points
		for k in range(0,4):
			worldPts.append([float(temp_holder[0][k]),float(temp_holder[1][k])])

		worldPts2 = np.float32(worldPts)
	
		#Prepare video point arrays
		for x in range(2,6):
			video_lines.append(pclines[x].split())
			for y in range(0,4):
				video_lines[x-2][y] = video_lines[x-2][y].split('e+')

		point_arrays = []
		#each point:
		# [[X0, Y0]
		# [X1, Y1]]
		for a in range (0,4):
			point_arrays.append([[float(video_lines[0][a][0])*(10**float(video_lines[0][a][1])),
				float(video_lines[1][a][0])*(10**float(video_lines[1][a][1]))],
				[float(video_lines[2][a][0])*(10**float(video_lines[2][a][1])),
				float(video_lines[3][a][0])*(10**float(video_lines[3][a][1]))]])

		curr_videoPts = []
		curr_vidlower = []
		for i in range (0,4):
			delta_x = point_arrays[i][1][0] - point_arrays[i][0][0]
			delta_y = point_arrays[i][1][1] - point_arrays[i][0][1]
			a = math.sqrt((float(elevdiff)**2) / (1 + ((delta_x**2)/(delta_y**2))))
			b = a * (delta_x**2)/(delta_y**2)
			curr_videoPts.append([point_arrays[i][0][0] + (delta_x * elevprop[i]),point_arrays[i][0][1] + (delta_y * elevprop[i])])
			curr_vidlower.append([point_arrays[i][0][0] + ((1-b)*delta_x * elevprop[i]),point_arrays[i][0][1] + ((1-a) * delta_y * elevprop[i])])

		curr_videoPts2 = np.float32(curr_videoPts)
		currlower = np.float32(curr_vidlower)
	
		homography, mask = cv2.findHomography(np.array(curr_videoPts2), np.array(worldPts2))
		homography2, fail = cv2.findHomography(np.array(currlower), np.array(worldPts2))
		np.savetxt(homo_filename,homography)
		if shift_gt_homo == 1:
			np.savetxt(gthomo_filename,homography2)
		else:
			np.savetxt(gthomo_filename,homography)

def run_tracker(nrunlines,runline0,runline1,runline2,runline3):
	#Runs the tracker, obviously
	print 'Running tracker...'
	if nrunlines == 1:
		subprocess.check_call(runline0, shell=True)
	elif nrunlines == 2:
		subprocess.check_call(runline0, shell=True)
		subprocess.check_call(runline1, shell=True)
	elif nrunlines == 3:
		subprocess.check_call(runline0, shell=True)
		subprocess.check_call(runline1, shell=True)
		subprocess.check_call(runline2, shell=True)
	elif nrunlines == 4:
		subprocess.check_call(runline0, shell=True)
		subprocess.check_call(runline1, shell=True)
		subprocess.check_call(runline2, shell=True)
		subprocess.check_call(runline3, shell=True)
	print '...done!'

def gt_homography(gtsqlite,homofile,stayInPixels):
	#Projects ground-truth to world-space
	conn = sqlite3.connect(gtsqlite)

	boxes = conn.execute('SELECT * FROM bounding_boxes')
	averaged_table = []

	for row in boxes:
		new_row = [row[0], row[1]]
		new_row.append((row[2] + row[4])/2)
		new_row.append((row[3] + row[5])/2)
		averaged_table.append(new_row)

	pointsToTranslate = []
	for line in averaged_table:
		pointsToTranslate.append([line[0],line[1],line[2],line[3]])
	if stayInPixels == 0:
		a = np.array(pointsToTranslate, dtype='float32')
		a = np.array([a])

		f = open(homofile,'r')
		homoMatrix = []
		for txtline in f.readlines():
			temptxt = txtline[:-1].split()
			homoMatrix.append(temptxt)

		i,j = 0,0
		hh = [[0,0,0],[0,0,0],[0,0,0]]
		for x in homoMatrix:
			for y in x:
				hh[i][j] = float(y)
				j += 1
				if j == 3:
					j = 0
					i += 1

		h = np.array(hh, dtype='float32')

		translatedPoints = []
		#print hh
		for ptt in pointsToTranslate:
			w = hh[2][0]*ptt[2] + hh[2][1]*ptt[3] + hh[2][2]
			if w != 0:
				x = (hh[0][0]*ptt[2] + hh[0][1]*ptt[3] + hh[0][2])/w
				y = (hh[1][0]*ptt[2] + hh[1][1]*ptt[3] + hh[1][2])/w
			else:
				x = 0
				y = 0
			translatedPoints.append([ptt[0],ptt[1],x,y])

		#translatedPoints = cv2.perspectiveTransform(a,h)

		#print translatedPoints

	else:
		translatedPoints = []
		for ptt in pointsToTranslate:
			translatedPoints.append([ptt[0],ptt[1],ptt[2],ptt[3]])

	conn.execute('''DROP TABLE IF EXISTS positions''')

	conn.execute('''CREATE TABLE positions
		(object_id INTEGER,
		frame_number INTEGER,
		x_coord REAL,
		y_coord REAL);''')

	for r in translatedPoints:
		conn.execute('''INSERT INTO positions (object_id, frame_number, x_coord, y_coord) VALUES (?,?,?,?);''', (int(r[0]),int(r[1]),float(r[2]),float(r[3])))

	conn.commit()
	conn.close()

def config_mod(staParamArray, nConf, config0, config1, config2, config3, varParamArray, curr_params):
	#Prints new parameters to the tracker's configuration files
	for x in range(0,len(curr_params)):
		varParamArray[x].append(curr_params[x])
	for x in range(0,nConf):
		if x == 0:
			cfg = open(config0, 'w')
			cfg.write('# Automatically generated configuration file.')
			for spa in staParamArray:
				if int(spa[0]) == 0:
					cfg.write('\n')
					cfg.write(spa[1])
			for vpa in varParamArray:
				if int(vpa[0]) == 0:
					cfg.write('\n')
					cfg.write(vpa[1])
					cfg.write(' = ')
					cfg.write(str(vpa[-1]))
		if x == 1:
			cfg = open(config1, 'w')
			cfg.write('# Automatically generated configuration file.')
			for spa in staParamArray:
				if int(spa[0]) == 1:
					cfg.write('\n')
					cfg.write(spa[1])
			for vpa in varParamArray:
				if int(vpa[0]) == 1:
					cfg.write('\n')
					cfg.write(vpa[1])
					cfg.write(' = ')
					cfg.write(str(vpa[-1]))
		if x == 2:
			cfg = open(config2, 'w')
			cfg.write('# Automatically generated configuration file.')
			for spa in staParamArray:
				if int(spa[0]) == 2:
					cfg.write('\n')
					cfg.write(spa[1])
			for vpa in varParamArray:
				if int(vpa[0]) == 2:
					cfg.write('\n')
					cfg.write(vpa[1])
					cfg.write(' = ')
					cfg.write(str(vpa[-1]))
		if x == 3:
			cfg = open(config3, 'w')
			cfg.write('# Automatically generated configuration file.')
			for spa in staParamArray:
				if int(spa[0]) == 3:
					cfg.write('\n')
					cfg.write(spa[1])
			for vpa in varParamArray:
				if int(vpa[0]) == 3:
					cfg.write('\n')
					cfg.write(vpa[1])
					cfg.write(' = ')
					cfg.write(str(vpa[-1]))

def ConfigSectionMap(section):
	#Helps extract variables from the INI file
	dict1 = {}
	options = config.options(section)
	for option in options:
		try:
			dict1[option] = config.get(section, option)
			if dict1[option] == -1:
				DebugPrint("skip: $s" % option)
		except:
			print('exception on %s' % option)
			dict1[option] = None
	return dict1

#Read Setup file
config = ConfigParser.ConfigParser()
config.read(setup_filename)

nConfigs = int(ConfigSectionMap('ConfigFiles')['nconfigs'])
congif0 = ''
config1 = ''
config2 = ''
config3 = ''
if nConfigs == 1:
	config0 = ConfigSectionMap('ConfigFiles')['config0']
elif nConfigs == 2:
	config0 = ConfigSectionMap('ConfigFiles')['config0']
	config1 = ConfigSectionMap('ConfigFiles')['config1']
elif nConfigs == 3:
	config0 = ConfigSectionMap('ConfigFiles')['config0']
	config1 = ConfigSectionMap('ConfigFiles')['config1']
	config2 = ConfigSectionMap('ConfigFiles')['config2']
elif nConfigs == 4:
	config0 = ConfigSectionMap('ConfigFiles')['config0']
	config1 = ConfigSectionMap('ConfigFiles')['config1']
	config2 = ConfigSectionMap('ConfigFiles')['config2']
	config3 = ConfigSectionMap('ConfigFiles')['config3']
else:
	print 'Error : Invalid number of configuration files. The current maximum is 4.'
	a = 1/0

#Prepare commands required to run the tracker
nrunlines = int(ConfigSectionMap('RunSettings')['nrunlines'])
runline0 = ''
runline1 = ''
runline2 = ''
runline3 = ''
if nrunlines == 1:
	runline0 = ConfigSectionMap('RunSettings')['runline0']
elif nrunlines == 2:
	runline0 = ConfigSectionMap('RunSettings')['runline0']
	runline1 = ConfigSectionMap('RunSettings')['runline1']
elif nrunlines == 3:
	runline0 = ConfigSectionMap('RunSettings')['runline0']
	runline1 = ConfigSectionMap('RunSettings')['runline1']
	runline2 = ConfigSectionMap('RunSettings')['runline2']
elif nrunlines == 4:
	runline0 = ConfigSectionMap('RunSettings')['runline0']
	runline1 = ConfigSectionMap('RunSettings')['runline1']
	runline2 = ConfigSectionMap('RunSettings')['runline2']
	runline3 = ConfigSectionMap('RunSettings')['runline3']
else:
	print 'Error : Invalid number of run-lines files. The current maximum is 4.'
	a = 1/0

#Extract setup variables for GenTrO
no_homography = int(ConfigSectionMap('HomographyOptions')['no_homography'])
include_homo_altitude_mod = int(ConfigSectionMap('HomographyOptions')['include_homo_altitude_mod'])
shift_gt_homo = int(ConfigSectionMap('HomographyOptions')['shift_gt_homo'])
metersperpixel = float(ConfigSectionMap('HomographyOptions')['metersperpixel'])
homo_filename = str(ConfigSectionMap('HomographyOptions')['homo_filename'])
point_corr_filename = str(ConfigSectionMap('HomographyOptions')['point_corr_filename'])
gthomo_filename = str(ConfigSectionMap('HomographyOptions')['gthomo_filename'])
videoframefile = str(ConfigSectionMap('HomographyOptions')['videoframefile'])
worldfile = str(ConfigSectionMap('HomographyOptions')['worldfile'])

weight_mota = float(ConfigSectionMap('GeneralSettings')['weight_mota'])
max_iterations = int(ConfigSectionMap('GeneralSettings')['max_iterations'])
relative_change = float(ConfigSectionMap('GeneralSettings')['relative_change'])
maxDeltaI = int(ConfigSectionMap('GeneralSettings')['max_delta_i'])
max_n_changes = int(ConfigSectionMap('GeneralSettings')['max_n_changes'])
storage_filename = str(ConfigSectionMap('GeneralSettings')['storage_filename'])
video_filename = str(ConfigSectionMap('GeneralSettings')['video_filename'])
ground_truth_sqlite = str(ConfigSectionMap('GeneralSettings')['ground_truth_sqlite'])
sqlite_filename = str(ConfigSectionMap('GeneralSettings')['sqlite_filename'])

probConstant = float(ConfigSectionMap('OptimizationParameters')['prob_constant'])
t_init = float(ConfigSectionMap('OptimizationParameters')['t_init'])
max_match_dist = float(ConfigSectionMap('OptimizationParameters')['max_match_dist'])
lamda = float(ConfigSectionMap('OptimizationParameters')['lamda'])
emax = float(ConfigSectionMap('OptimizationParameters')['emax'])


weight_motp = 1 - weight_mota

print ''
print 'GenTrO : Generalized Tracker Optimizer'
print ''

#Extract Variable Parameters from .txt
varParams = open(variable_parameters_filename,'r')
lines = varParams.readlines()
varParamArray = []
for l in lines:
	temp = l.split(',')
	temp[-1] = temp[-1][:-1]
	varParamArray.append(temp)

#Extract Static Parameters from .txt
staParams = open(static_parameters_filename,'r')
lines = staParams.readlines()
staParamArray = []
for l in lines:
	temp = l.split(',')
	temp[-1] = temp[-1][:-1]
	staParamArray.append(temp)

curr_params = []
cont = 0

#Look for evidence of previous runs
if os.path.isfile(storage_filename):
	good_value = 0
	while good_value == 0:
		var = raw_input('Previous storage file found. Would you like to continue using the previous data? (Y/N)')
		if var == 'y' or var == 'Y':
			cont = 1
			good_value = 1
		elif var == 'n' or var == 'N':
			var2 = raw_input('WARNING! Previous storage file will be deleted if you continue! Do so anyway? (Y/N)')
			if var2 == 'y' or var2 == 'Y':
				removal = 'rm ' + storage_filename
				subprocess.check_call(removal, shell=True)
			else:
				print 'Quitting ineligantly...'
				a = 1/0
			good_value = 1
		else:
			print "Does not compute. Please try again."

good_userSel = 0
while good_userSel == 0:

	print ''
	print 'Please select what to do: '
	print ''
	print '1 - Begin or continue tracker optimization.'
	print '2 - Extract best parameters to config file(s).'
	print ''
	print 'Traffic Intelligence specific commands:'
	print '3 - Display tracks for an input SQL file.'
	print '4 - Create heatmap for an input SQL file.'
	print '5 - OD counts over thresholds.'
	print ''
	print 'q - Quit.'
	print ''

	userSel = raw_input('')
	if userSel == '1': 
		good_userSel = 1
		if cont == 0:
			print 'Initializing with default parameters.'
			#Prepare current parameter array
	
			for vpa in varParamArray:
				if vpa[2] == 'float':
					curr_params.append(float(vpa[4]))
				elif vpa[2] == 'int':
					curr_params.append(int(vpa[4]))
				elif vpa[2] == 'boolean':
					curr_params.append(vpa[4])
				else:
					print 'Error : ' + vpa[1] + ' does not have an acceptable type (float, int or boolean).'
					a = 1/0
			config_mod(staParamArray, nConfigs, config0, config1, config2, config3, varParamArray, curr_params)
			prevelev = [1.2,1.2,1.2,1.2]
			if no_homography != 1:
				pc_run = 'python inputPointCorrespondence.py -i ' + videoframefile + ' -w ' + worldfile + ' -u ' + str(metersperpixel) + ' -f ' + point_corr_filename + ' -e ' + str(include_homo_altitude_mod)
				subprocess.check_call(pc_run, shell=True)
			if os.path.isfile(sqlite_filename):
				removal = 'rm ' + sqlite_filename
				subprocess.check_call(removal, shell=True)
			if no_homography == 0:
				point_corresp_mod(point_corr_filename,prevelev,homo_filename,0,gthomo_filename,shift_gt_homo,include_homo_altitude_mod)
			
			run_tracker(nrunlines,runline0,runline1,runline2,runline3)
			current_traces = extract_trajectories(sqlite_filename)
			gt_homography(ground_truth_sqlite,gthomo_filename,no_homography)

			motp,mota,dit,mt,mme,fpt = matchmaking(current_traces,ground_truth_sqlite,max_match_dist)
			print 'Initial solution found.'

			#Initilizes storage file
			delta_i = 0
			with open(storage_filename, 'wb') as storagefile:
				csvfiller = csv.writer(storagefile,delimiter=' ')
				uid = []
				legends = []
				legends.append('Iteration')
				for cname in varParamArray:
					legends.append(cname[1])
				if include_homo_altitude_mod == 1:
					legends.append('Elev1')
					legends.append('Elev2')
					legends.append('Elev3')
					legends.append('Elev4')
				if shift_gt_homo == 1:
					legends.append('GTShift')
				legends.append('MOTP')
				legends.append('MOTA')
				legends.append('Dit')
				legends.append('mt')
				legends.append('mme')
				legends.append('fpt')
				legends.append('relative_change')
				legends.append('delta_i')
				csvfiller.writerow(legends)
				if include_homo_altitude_mod == 0 and shift_gt_homo == 0:
					firstrow = [0] + curr_params + [motp,mota,dit,mt,mme,fpt] + [relative_change] + [delta_i] + [0]
					csvfiller.writerow(firstrow)
				elif include_homo_altitude_mod == 1 and shift_gt_homo == 0:
					firstrow = [0] + curr_params + prevelev + [motp,mota,dit,mt,mme,fpt] + [relative_change] + [delta_i] + [0]
					csvfiller.writerow(firstrow)
				elif include_homo_altitude_mod == 0 and shift_gt_homo == 1:
					firstrow = [0] + curr_params + [0] + [motp,mota,dit,mt,mme,fpt] + [relative_change] + [delta_i] + [0]
					csvfiller.writerow(firstrow)
				elif include_homo_altitude_mod == 1 and shift_gt_homo == 1:
					firstrow = [0] + curr_params + prevelev + [0] + [motp,mota,dit,mt,mme,fpt] + [relative_change] + [delta_i] + [0]
					csvfiller.writerow(firstrow)

		#In-app array of csv solutions
		solutions = []
		with open(storage_filename, 'rb') as storagefile:
			csvreader = csv.reader(storagefile, delimiter=' ')
			for row in csvreader:
				solutions.append(row)

		#Extract best solution
		prevsol,prevelev,prevshift = [],[],[]
		prevsol = solutions[(int(solutions[-1][-1]))+1][1:(len(varParamArray)+1)]

		if include_homo_altitude_mod == 1:
			prevelev = solutions[(int(solutions[-1][-1]))+1][(len(prevsol)+1):(len(prevsol)+5)]
		if shift_gt_homo == 1:
			prevshift = solutions[(int(solutions[-1][-1]))+1][-8]
		else:
			prevshift = 0

		currbest = solutions[(int(solutions[-1][-1]))+1]
		besti = int(solutions[-1][-1])
		delta_i = int(solutions[-1][-2])
		relative_change = float(solutions[-1][-3])

		i = int(solutions[-1][0])+1
		ebest = weight_mota*(float(solutions[int(solutions[-1][-1])+1][-8])) + weight_motp*float(solutions[int(solutions[-1][-1])+1][-7])
		if ebest == 0:
			ebest -= 10
		eprev = ebest

		while i < max_iterations:
			print 'Iteration : ' + str(i)

			if delta_i >= (maxDeltaI * (t_init - (lamda * math.log(1+i)))):
				print delta_i + ' iterations since last improvement. Changing permutation size from ' + relative_change + ' to ' + relative_change/2 + ' .'
				delta_i = 0
				relative_change = relative_change/2

			if os.path.isfile(sqlite_filename):
				removal = 'rm ' + sqlite_filename
				subprocess.check_call(removal, shell=True)
			t = t_init - (lamda * math.log(1+i))
			print 'Temperature : ' + str(t)
			curr_params,currelev,currshift = neighbor_solution(t,t_init,include_homo_altitude_mod,prevsol,varParamArray,prevelev,prevshift,relative_change,max_n_changes,no_homography,shift_gt_homo)
			if no_homography == 0:
				point_corresp_mod(point_corr_filename,currelev,homo_filename,currshift,gthomo_filename,shift_gt_homo,include_homo_altitude_mod)
			gt_homography(ground_truth_sqlite,gthomo_filename,no_homography)

			config_mod(staParamArray, nConfigs, config0, config1, config2, config3, varParamArray, curr_params)
			run_tracker(nrunlines,runline0,runline1,runline2,runline3)
			current_traces = extract_trajectories(sqlite_filename)

			motp,mota,dit,mt,mme,fpt = matchmaking(current_traces,ground_truth_sqlite,max_match_dist)

			enew = weight_mota*float(mota) + weight_motp*motp
			print 'New energy : ' + str(enew)

			if enew == 0:
				enew -= 10
				print 'Tracker produced no results'

			if enew > ebest and enew != 0:
				print 'NEW BEST!!!'
				delta_i = 0
				besti = i
				ebest = enew
				saved_best_name = sqlite_filename[0:6]+'best.sqlite'
				if os.path.isfile(saved_best_name):
					remove_command = 'rm ' + saved_best_name
					subprocess.check_call(remove_command, shell=True)
				move_command = 'mv ' + sqlite_filename + ' ' + saved_best_name
				subprocess.check_call(move_command,shell=True)
			else:
				delta_i += 1

			if math.exp((t*probConstant*eprev)) != 0:
				initprob = math.exp((t*probConstant*enew)) / math.exp((t*probConstant*eprev))
			else:
				initprob = 1

			if initprob > 1:
				initprob = 1

			print 'Probability to move : ' + str(initprob)
			probcompare = random.uniform(0,1)
			if initprob > probcompare and enew != 0:
				print 'Moved.'
				prevsol = curr_params
				prevelev = currelev
				prevshift = currshift
				eprev = enew

			if include_homo_altitude_mod == 0 and shift_gt_homo == 0:
				row = [i] + curr_params + [motp,mota,dit,mt,mme,fpt] + [relative_change] + [delta_i] + [besti]
				solutions.append(row)
			elif include_homo_altitude_mod == 1 and shift_gt_homo == 0:
				row = [i] + curr_params + currelev + [motp,mota,dit,mt,mme,fpt] + [relative_change] + [delta_i] + [besti]
				solutions.append(row)
			elif include_homo_altitude_mod == 0 and shift_gt_homo == 1:
				row = [i] + curr_params + [currshift] + [motp,mota,dit,mt,mme,fpt] + [relative_change] + [delta_i] + [besti]
				solutions.append(row)
			elif include_homo_altitude_mod == 1 and shift_gt_homo == 1:
				row = [i] + curr_params + currelev + [currshift] + [motp,mota,dit,mt,mme,fpt] + [relative_change] + [delta_i] + [besti]
				solutions.append(row)
			if os.path.isfile(storage_filename):
				removal = 'rm ' + storage_filename
				subprocess.check_call(removal, shell=True)
				with open(storage_filename, 'wb') as storagefile:
					csvfiller = csv.writer(storagefile, delimiter = ' ')
					for sol in solutions:
						csvfiller.writerow(sol)

			i+=1

	elif userSel == '2':
		import ConfigParser

		solutions = []
		with open(storage_filename, 'rb') as storagefile:
			csvreader = csv.reader(storagefile, delimiter=' ')
			for row in csvreader:
				solutions.append(row)

		prevsol,prevelev,prevshift = [],[],[]
		prevsol = solutions[(int(solutions[-1][-1]))+1][1:(len(varParamArray)+1)]
		if include_homo_altitude_mod == 1:
			prevelev = solutions[(int(solutions[-1][-1]))+1][(len(prevsol)+1):(len(prevsol)+5)]
		if shift_gt_homo == 1:
			prevshift = solutions[(int(solutions[-1][-1]))+1][-8]
		else:
			prevshift = 0

		currbest = solutions[(int(solutions[-1][-1]))+1]

		if no_homography == 0:
			print 'Readjusting homography...'
			point_corresp_mod(point_corr_filename,prevsol,homo_filename,prevshift,gthomo_filename,shift_gt_homo,include_homo_altitude_mod)

		print 'Preparing config files...'
		config_mod(staParamArray, nConfigs, config0, config1, config2, config3, varParamArray, prevsol)

		print ' '
		print 'Good to go!'
		print ' '
		print "NOTE : To use the generated parameters on full videos, the target video MUST be changed manually in the config file(s)!"
		print ' '
		good_userSel = 1

	elif userSel == '3':
		import storage
		import matplotlib.pyplot as plt
		from pylab import *

		selecting = 1
		while selecting == 1:
			sqliteToTrack = raw_input('SQLite filename ("quit" to exit) :')
			if os.path.isfile(sqliteToTrack):
				goodchoice = 0
				thingToTrack = ''
				while goodchoice == 0: 
					featOrObj = raw_input('Track (F)eatures or (O)bjects?')
					if featOrObj == 'f' or featOrObj == 'F':
						thingToTrack = 'feature'
						goodchoice = 1
					elif thingToTrack == 'o' or featOrObj == 'O':
						thingToTrack = 'object'
						goodchoice = 1
					else:
						print 'Input does not compute.'
				things = storage.loadTrajectoriesFromSqlite(sqliteToTrack,thingToTrack)
				for t in things:
					t.plot()
				axis('equal')
				selecting = 0
			elif sqliteToTrack == 'quit':
				selecting = 0

		good_userSel = 1
				
	elif userSel == '4':
		import storage
		import matplotlib.pyplot as plt
		from pylab import *

		selecting = 1
		while selecting == 1:
			sqliteToTrack = raw_input('SQLite filename ("quit" to exit) : ')
			if os.path.isfile(sqliteToTrack):
				selecting = 0
				objects = storage.loadTrajectoriesFromSqlite(sqliteToTrack,'object')
				object_positions = []
				a = 0
				for o in objects:
					instant = o.timeInterval[0]
					for p in o.positions:
						object_positions.append([a,instant,p.x,p.y])
						instant += 1
					a += 1

				n_obj = 0
				for op in object_positions:
					if op[0] > n_obj:
						n_obj = op[0]

				sorted_obj_positions = [0]*(n_obj+1)

				for x in range(0,n_obj+1):
					sorted_obj_positions[x] = [[0,0],[0,0]]

				for s in object_positions:
					sorted_obj_positions[s[0]].append(s)

				for sop in sorted_obj_positions:
					del sop[0]
					del sop[0]

				x = []
				y = []
				for sop in sorted_obj_positions:
					for s in sop:
						x.append(s[2])
						y.append(s[3])

				mincnter = int(raw_input('Input minimum number of detections to plot a hex : '))
				gridx = int(raw_input('Input X for gridsize : '))
				gridy = int(raw_input('Input Y for gridsize : '))
				plt.hexbin(x,y, mincnt = mincnter, gridsize=(gridx,gridy),cmap=plt.cm.YlOrRd_r)
				plt.gca().invert_yaxis()
				axis('equal')
				plt.show()
			elif sqliteToTrack == 'quit':
				selecting = 0
			else:	
				print 'File not found. :('
		good_userSel = 1
					
	elif userSel == '5':
		import sys, argparse
		import matplotlib.pyplot as plt
		import time

		worldImg = worldfile
		worldImg = plt.imread(worldImg)
		unitsPerPixel = metersperpixel

		selecting = 1
		while selecting == 1:
			sqliteToTrack = raw_input('SQLite filename : ')
			if os.path.isfile(sqliteToTrack):
				selecting = 0
			else:
				print 'File not found.'

		nThresholds = int(raw_input("Enter the number of thresholds : "))	
		nPoints = nThresholds*2
		print ''
		print 'Remember the order you place them in!'
		print ''
		time.sleep(2)

		print 'Click on the ' + str(nPoints) + ' points delimiting the thresholds.'
		plt.figure()

		plt.imshow(worldImg)
		WorldPts = unitsPerPixel*np.array(plt.ginput(nPoints, timeout=3000))
		c = 0
		thresh = []
		counter = []
		correspMade = []
		emptyCorr = []
		while c < len(WorldPts):
			thresh.append([WorldPts[c].tolist(),WorldPts[c+1].tolist()])
			c += 2
		
		import storage

		objects = storage.loadTrajectoriesFromSqlite(sqliteToTrack, 'object')
		object_positions0 = []
		a = 0
		for o in objects:
			instant = o.timeInterval[0]
			for p in o.positions:
				object_positions0.append([a,instant,p.x,p.y])
				instant += 1
			a += 1
		plt.close('all')
		
		object_positions = []
		maxObject = object_positions0[-1][0]
		for x in range(0,maxObject+1):
			singleTrack = []
			for op in object_positions0:
				if op[0] == x:
					singleTrack.append(op)
			if len(singleTrack) >= 100:
				xDeltas = []
				yDeltas = []
				for a in range (0,5):
					xDeltas.append(singleTrack[a+20][2] - singleTrack[a][2])
					yDeltas.append(singleTrack[a+20][3] - singleTrack[a][3])
				avgXDelta = sum(xDeltas)/5
				avgYDelta = sum(yDeltas)/5
				for p in range(1,8):
					extrapolatedPos = [singleTrack[0][0],singleTrack[0][1]-p,singleTrack[0][2]-(p*avgXDelta),singleTrack[0][3]-(p*avgYDelta)]
					singleTrack.insert(0,extrapolatedPos)

				xDeltas2 = []
				yDeltas2 = []
				for a in range(1,6):
					xDeltas2.append(singleTrack[-a][2] - singleTrack[-a-20][2])
					yDeltas2.append(singleTrack[-a][3] - singleTrack[-a-20][3])
				avgXDelta2 = sum(xDeltas2)/5
				avgYDelta2 = sum(yDeltas2)/5
				for p in range(1,8):
					extrapolatedPos = [singleTrack[-1][0],singleTrack[-1][1]+p,singleTrack[-1][2]+(p*avgXDelta2),singleTrack[-1][3]+(p*avgYDelta2)]
					singleTrack.append(extrapolatedPos)
			for st in singleTrack:
				object_positions.append(st)
		trackedObjects = []
		for x in range(1,len(object_positions)):
			if object_positions[x][0] == object_positions[x-1][0]:
				for y in range(0,len(thresh)):
					if max(object_positions[x-1][2],object_positions[x][2]) < min(thresh[y][0][0],thresh[y][1][0]):
						continue
					if max(thresh[y][0][0],thresh[y][1][0]) < min(object_positions[x-1][2],object_positions[x][2]):
						continue
					if max(object_positions[x-1][3],object_positions[x][3]) < min(thresh[y][0][1],thresh[y][1][1]):
						continue
					if max(thresh[y][0][1],thresh[y][1][1]) < min(object_positions[x-1][3],object_positions[x][3]):
						continue
					if (object_positions[x-1][2] - object_positions[x][2]) == 0 or (thresh[y][0][0] - thresh[y][1][0]) == 0:
						continue
					A1 = (object_positions[x-1][3] - object_positions[x][3])/(object_positions[x-1][2] - object_positions[x][2])
					A2 = (thresh[y][0][1] - thresh[y][1][1])/(thresh[y][0][0] - thresh[y][1][0])
					b1 = object_positions[x-1][3] - (A1 * object_positions[x-1][2])
					b2 = thresh[y][0][1] - (A2 * thresh[y][0][0])

					if A1 == A2:
						continue

					Xa = (b2 - b1) / (A1 - A2)
					
					if Xa < max(min(object_positions[x-1][2],object_positions[x][2]),min(thresh[y][0][0],thresh[y][1][0])):
						continue
					if Xa > min(max(object_positions[x-1][2],object_positions[x][2]),max(thresh[y][0][0],thresh[y][1][0])):
						continue
					else:
						counter.append([object_positions[x-1][0],y])
						if object_positions[x-1][0] in trackedObjects:
							pass
						else:
							trackedObjects.append(object_positions[x-1][0])	

			else:
				correspMade.append(0)		
		finalCount = []			
		for x in range(0,len(thresh)):
			for y in range(0,len(thresh)):
				finalCount.append([x,y,0])

		for to in trackedObjects:
			tempC = []
			for co in counter:
				if co[0] == to:
					if co[1] in tempC:
						pass
					else:
						tempC.append(co[1])
			if len(tempC) == 1:
				for fc in finalCount:
					if fc[0] == tempC[0] and fc[1] == tempC[0]:
						fc[2] += 1
			else:
				for fc in finalCount:
					if fc[0] == tempC[0] and fc[1] == tempC[1]:
						fc[2] += 1

		print finalCount
		good_userSel = 1

	elif userSel == 'q' or userSel == 'Q':
		print 'Goodbye!'
		good_userSel = 1

	else:
		print 'Does not compute, try again.'