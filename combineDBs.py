import sqlite3
import sys
from subprocess import call, Popen, PIPE

tempDB = "temp.db"

p = Popen(["which", "ompl_benchmark_statistics.py"], stdout=PIPE)
output, err = p.communicate()
ompl_benchmark_statistics = output.rstrip()

if ompl_benchmark_statistics == "":
	print "'which ompl_benchmark_statistics.py' failed to return a file"
	sys.exit(1)

call(["python2.7", ompl_benchmark_statistics, sys.argv[2], "-d", tempDB])

srcConnection = sqlite3.connect(tempDB)
srcConnection.row_factory = sqlite3.Row

srcCursor = srcConnection.cursor()

srcCursor.execute("SELECT * FROM experiments")
experimentDataColumns = [description[0] for description in srcCursor.description]
experimentData = srcCursor.fetchone()

srcCursor.execute("SELECT * FROM plannerConfigs")
plannerDataColumns = [description[0] for description in srcCursor.description]
plannerData = srcCursor.fetchone()

srcCursor.execute("SELECT * FROM runs")
resultsDataColumns = [description[0] for description in srcCursor.description]
resultsData = srcCursor.fetchone()

srcConnection.close()

# print experimentDataColumns
# print experimentData

# print plannerDataColumns
# print plannerData

# print resultsDataColumns
# print resultsData

dstConnection = sqlite3.connect(sys.argv[1])
dstConnection.row_factory = sqlite3.Row
dstCursor = dstConnection.cursor()

#lookup the experiment to see if it exists already
#if not add it to the destination DB
experimentName = (experimentData["name"],)
dstCursor.execute("SELECT id FROM experiments WHERE name=?", experimentName)
row = dstCursor.fetchone()
if row == None:
	cols = []
	for index, val in enumerate(experimentDataColumns):
		cols.append(val)
	values = {}
	for index, val in enumerate(experimentData):
		key = experimentDataColumns[index]
		if key != "id":
			values[key] = val

	columns = ', '.join(values.keys())
	placeholders = ', '.join('?' * len(values))
	sql = 'INSERT INTO experiments ({}) VALUES ({})'.format(columns, placeholders)
	dstCursor.execute(sql, values.values())
	dstConnection.commit()
	
	dstCursor.execute("SELECT id FROM experiments WHERE name=?", experimentName)

	row = dstCursor.fetchone()

experimentID = row["id"]

#lookup the experiment to see if it exists already
#if not add it to the destination DB
plannerConfig = (plannerData["name"], plannerData["settings"],)
dstCursor.execute("SELECT id FROM plannerConfigs WHERE name=? AND settings=?", plannerConfig)
row = dstCursor.fetchone()
if row == None:
	cols = []
	for index, val in enumerate(plannerDataColumns):
		cols.append(val)
	values = {}
	for index, val in enumerate(plannerData):
		key = plannerDataColumns[index]
		if key != "id":
			values[key] = val

	columns = ', '.join(values.keys())
	placeholders = ', '.join('?' * len(values))
	sql = 'INSERT INTO plannerConfigs ({}) VALUES ({})'.format(columns, placeholders)
	dstCursor.execute(sql, values.values())
	dstConnection.commit()
	
	dstCursor.execute("SELECT id FROM plannerConfigs WHERE name=? AND settings=?", plannerConfig)

	row = dstCursor.fetchone()

plannerID = row["id"]

#let's assume that we only run this for each experiment run once
#so there's no checking to see if this row already exists
cols = []
for index, val in enumerate(resultsDataColumns):
	cols.append(val)
values = {}
for index, val in enumerate(resultsData):
	key = resultsDataColumns[index]
	if key == "id":
		continue
	else:
		if key == "experimentid":
			values[key] = experimentID
		else:
			if key == "plannerid":
				values[key] = plannerID
			else:
				values[key] = val

columns = ', '.join(values.keys())
placeholders = ', '.join('?' * len(values))
sql = 'INSERT INTO runs ({}) VALUES ({})'.format(columns, placeholders)
dstCursor.execute(sql, values.values())
dstConnection.commit()

dstConnection.close()